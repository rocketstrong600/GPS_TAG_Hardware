//! Zero-allocation **Server-Sent Events (SSE)** client over TLS 1.3 HTTPS / HTTP (`reqwless` +
//! `embedded-tls` on the `embassy-net` PPP stack).
//!
//! Designed for long-lived server-to-device command channels over cellular networks:
//! - Sends `Accept: text/event-stream`, `Cache-Control: no-cache`, any caller headers (e.g.
//!   `Authorization: Bearer ...`), and `Last-Event-ID` automatically on reconnects.
//! - Parses WHATWG EventSource streams (`event:`, multi-line `data:`, `id:`, `retry:`, and
//!   `: comment` keepalives) with zero heap allocation across both chunked and streaming responses.
//! - Handles cellular CGNAT silent TCP drops via [`SseConfig::idle_timeout`] and automatically
//!   waits for PPP recovery (`stack.wait_config_up()`) before reconnecting with backoff.
//!
//! See `examples/modem_sse.rs` for a runnable example.

#![allow(dead_code)]

use core::ops::AsyncFnMut;

use defmt::{info, warn};
use embassy_net::Stack;
use embassy_net::dns::DnsSocket;
use embassy_net::tcp::client::TcpClient;
use embassy_time::{Duration, Timer, WithTimeout};
use embedded_io_async::Read as _;
use heapless::String;
use reqwless::request::RequestBuilder;

use super::{DEFAULT_TIMEOUT, HttpClient, HttpsBuffers, HttpsError, Method, TlsConfig, TlsVerify};

/// Maximum length of a single SSE `event:` name.
pub const MAX_EVENT_NAME: usize = 32;
/// Maximum length of a single SSE `id:` value (`Last-Event-ID`).
pub const MAX_EVENT_ID: usize = 64;
/// Maximum accumulated `data:` payload size for a single SSE event.
pub const MAX_EVENT_DATA: usize = 1024;
/// Maximum single line length inside an SSE stream before truncation.
const MAX_LINE_LEN: usize = 512;
/// Maximum total headers sent on the SSE request (3 built-in + up to 9 user headers).
const MAX_REQ_HEADERS: usize = 12;

/// Default idle read timeout before assuming cellular NAT silently dropped the socket.
pub const DEFAULT_IDLE_TIMEOUT: Duration = Duration::from_secs(90);
/// Default initial reconnect delay after a stream drop (overridden by server `retry:` field).
pub const DEFAULT_RETRY_DELAY: Duration = Duration::from_secs(3);
/// Maximum exponential reconnect backoff delay.
const MAX_BACKOFF: Duration = Duration::from_secs(30);

/// Control decision returned by the caller's SSE event callback.
#[derive(Debug, Copy, Clone, Eq, PartialEq, defmt::Format)]
pub enum SseAction {
    /// Keep reading events from the stream.
    Continue,
    /// Stop listening and close the connection cleanly.
    Stop,
}

/// A single decoded Server-Sent Event passed to the caller's callback.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct SseEvent<'a> {
    /// Event type from `event:` (defaults to `"message"` when omitted).
    pub event: &'a str,
    /// Event payload formed by joining all `data:` lines with `\n`.
    pub data: &'a str,
    /// Most recent non-empty `id:` value received on this stream, if any.
    pub id: Option<&'a str>,
}

/// Configuration for [`listen`] and [`listen_once`].
pub struct SseConfig<'a> {
    /// Full `https://...` or `http://...` SSE endpoint URL.
    pub url: &'a str,
    /// Optional extra HTTP request headers (e.g. `&[("Authorization", "Bearer ...")]`).
    /// `Accept`, `Cache-Control`, and `Last-Event-ID` are added automatically.
    pub headers: &'a [(&'a str, &'a str)],
    /// TLS verification mode (`TlsVerify::Certificate`, `TlsVerify::Psk`, or `TlsVerify::None`).
    pub verify: TlsVerify<'a>,
    /// Timeout for DNS + TCP + TLS handshake + HTTP response headers.
    pub connect_timeout: Duration,
    /// Optional idle read timeout: if no bytes (including `: keepalive` comments) arrive within
    /// this duration, the stream reconnects automatically. Set to `None` to disable idle timeout.
    pub idle_timeout: Option<Duration>,
    /// Initial reconnect delay after disconnect (dynamically updated if the server sends `retry:`).
    pub retry_delay: Duration,
}

impl<'a> SseConfig<'a> {
    /// Creates an [`SseConfig`] with sensible cellular defaults (`30 s` connect timeout,
    /// `90 s` idle keepalive timeout, `3 s` retry delay).
    pub const fn new(url: &'a str, verify: TlsVerify<'a>) -> Self {
        Self {
            url,
            headers: &[],
            verify,
            connect_timeout: DEFAULT_TIMEOUT,
            idle_timeout: Some(DEFAULT_IDLE_TIMEOUT),
            retry_delay: DEFAULT_RETRY_DELAY,
        }
    }
}

/// Persistent SSE state carried across automatic reconnects (`Last-Event-ID` and `retry:`).
pub struct SseState {
    pub last_event_id: String<MAX_EVENT_ID>,
    pub retry_delay: Duration,
}

impl SseState {
    pub const fn new(retry_delay: Duration) -> Self {
        Self {
            last_event_id: String::new(),
            retry_delay,
        }
    }
}

/// Connects to an HTTPS/HTTP SSE stream and automatically reconnects on disconnects, idle
/// timeouts, or PPP drops — sending `Last-Event-ID` on reconnects — until `on_event` returns
/// [`SseAction::Stop`].
///
/// Pass a `next_seed` closure backed by the hardware TRNG (e.g. `|| b.rng.next_u64()`) so every
/// TLS 1.3 reconnection uses fresh cryptographic entropy.
pub async fn listen<S, F>(
    stack: Stack<'static>,
    bufs: &mut HttpsBuffers,
    mut next_seed: S,
    cfg: &SseConfig<'_>,
    mut on_event: F,
) where
    S: FnMut() -> u64,
    F: AsyncFnMut(SseEvent<'_>) -> SseAction,
{
    let mut state = SseState::new(cfg.retry_delay);
    let mut backoff = state.retry_delay;

    loop {
        if !stack.is_config_up() {
            info!("SSE waiting for PPP IPv4 config...");
            stack.wait_config_up().await;
        }

        let seed = next_seed();
        match listen_once(stack, bufs, seed, cfg, &mut state, &mut on_event).await {
            Ok(SseAction::Stop) => {
                info!("SSE listener stopped by callback");
                return;
            }
            Ok(SseAction::Continue) => {
                // Clean server EOF: honor server retry_delay without escalating backoff.
                backoff = state.retry_delay;
                info!(
                    "SSE stream closed by server; reconnecting in {} ms",
                    backoff.as_millis()
                );
            }
            Err(HttpsError::HttpStatus(204)) => {
                info!("SSE server returned 204 No Content; stopping listener");
                return;
            }
            Err(e) => {
                warn!(
                    "SSE stream error ({:?}); reconnecting in {} ms",
                    e,
                    backoff.as_millis()
                );
                Timer::after(backoff).await;
                backoff = Duration::from_millis(
                    backoff
                        .as_millis()
                        .saturating_mul(2)
                        .min(MAX_BACKOFF.as_millis()),
                );
                continue;
            }
        }

        Timer::after(backoff).await;
    }
}

/// Runs a single SSE HTTP/HTTPS connection until EOF, error/timeout, or `on_event` returns
/// [`SseAction::Stop`]. Updates `state.last_event_id` and `state.retry_delay` in place.
pub async fn listen_once<F>(
    stack: Stack<'static>,
    bufs: &mut HttpsBuffers,
    tls_seed: u64,
    cfg: &SseConfig<'_>,
    state: &mut SseState,
    mut on_event: F,
) -> Result<SseAction, HttpsError>
where
    F: AsyncFnMut(SseEvent<'_>) -> SseAction,
{
    let mut tcp = TcpClient::new(stack, &bufs.tcp_state);
    // Use idle_timeout (if configured) at the socket layer so long silent periods between events
    // don't hit the shorter handshake connect_timeout.
    tcp.set_timeout(cfg.idle_timeout);
    let dns = DnsSocket::new(stack);

    let tls = TlsConfig::new(
        tls_seed,
        &mut bufs.tls_read,
        &mut bufs.tls_write,
        clone_tls_verify(&cfg.verify),
    );
    let mut client = HttpClient::new_with_tls(&tcp, &dns, tls);

    let initial_last_id = state.last_event_id.clone();
    let mut req_headers = [("", ""); MAX_REQ_HEADERS];
    let mut hdr_len = 0usize;
    req_headers[hdr_len] = ("Accept", "text/event-stream");
    hdr_len += 1;
    req_headers[hdr_len] = ("Cache-Control", "no-cache");
    hdr_len += 1;
    if !initial_last_id.is_empty() {
        req_headers[hdr_len] = ("Last-Event-ID", initial_last_id.as_str());
        hdr_len += 1;
    }
    for &hdr in cfg.headers {
        if hdr_len >= MAX_REQ_HEADERS {
            break;
        }
        req_headers[hdr_len] = hdr;
        hdr_len += 1;
    }

    let mut req = client
        .request(Method::GET, cfg.url)
        .with_timeout(cfg.connect_timeout)
        .await
        .map_err(|_| HttpsError::Timeout)??;
    req = req.headers(&req_headers[..hdr_len]);

    let response = req
        .send(&mut bufs.header_rx)
        .with_timeout(cfg.connect_timeout)
        .await
        .map_err(|_| HttpsError::Timeout)??;

    let status = response.status.0;
    if status != 200 {
        return Err(HttpsError::HttpStatus(status));
    }

    info!("SSE connected to {} (HTTP 200)", cfg.url);

    let mut reader = response.body().reader();
    let mut chunk = [0u8; 256];
    let mut parser = SseParser::new();

    loop {
        let read_res = match cfg.idle_timeout {
            Some(timeout) => reader
                .read(&mut chunk)
                .with_timeout(timeout)
                .await
                .map_err(|_| HttpsError::Timeout)?,
            None => reader.read(&mut chunk).await,
        };

        let n = match read_res {
            Ok(0) => return Ok(SseAction::Continue),
            Ok(n) => n,
            Err(e) => return Err(HttpsError::from(e)),
        };

        for &b in &chunk[..n] {
            if let Some(dispatch) = parser.feed_byte(b, state) {
                if dispatch {
                    let event_name = if parser.event_type.is_empty() {
                        "message"
                    } else {
                        parser.event_type.as_str()
                    };
                    let id = if state.last_event_id.is_empty() {
                        None
                    } else {
                        Some(state.last_event_id.as_str())
                    };
                    let ev = SseEvent {
                        event: event_name,
                        data: parser.data.as_str(),
                        id,
                    };
                    let action = on_event(ev).await;
                    parser.clear_event();
                    if action == SseAction::Stop {
                        return Ok(SseAction::Stop);
                    }
                }
            }
        }
    }
}

struct SseParser {
    line: [u8; MAX_LINE_LEN],
    line_len: usize,
    prev_was_cr: bool,
    event_type: String<MAX_EVENT_NAME>,
    data: String<MAX_EVENT_DATA>,
    has_data: bool,
}

impl SseParser {
    const fn new() -> Self {
        Self {
            line: [0u8; MAX_LINE_LEN],
            line_len: 0,
            prev_was_cr: false,
            event_type: String::new(),
            data: String::new(),
            has_data: false,
        }
    }

    fn clear_event(&mut self) {
        self.event_type.clear();
        self.data.clear();
        self.has_data = false;
    }

    /// Feeds one byte into the WHATWG SSE line/event parser.
    /// Returns `Some(true)` when a complete event with data should be dispatched to the callback.
    fn feed_byte(&mut self, b: u8, state: &mut SseState) -> Option<bool> {
        if b == b'\n' && self.prev_was_cr {
            self.prev_was_cr = false;
            return None;
        }
        self.prev_was_cr = false;

        if b == b'\r' || b == b'\n' {
            if b == b'\r' {
                self.prev_was_cr = true;
            }
            let len = self.line_len;
            self.line_len = 0;

            if len == 0 {
                // Blank line terminates the current event block.
                if self.has_data {
                    return Some(true);
                }
                self.event_type.clear();
                return None;
            }

            if let Ok(line_str) = core::str::from_utf8(&self.line[..len]) {
                process_line(
                    line_str,
                    &mut self.event_type,
                    &mut self.data,
                    &mut self.has_data,
                    state,
                );
            }
            return None;
        }

        if self.line_len < self.line.len() {
            self.line[self.line_len] = b;
            self.line_len += 1;
        }
        None
    }
}

fn process_line(
    line: &str,
    event_type: &mut String<MAX_EVENT_NAME>,
    data: &mut String<MAX_EVENT_DATA>,
    has_data: &mut bool,
    state: &mut SseState,
) {
    // Lines starting with ':' are SSE comments / keepalive pings.
    if line.starts_with(':') {
        return;
    }

    let (field, raw_val) = match line.split_once(':') {
        Some((f, v)) => (f, v.strip_prefix(' ').unwrap_or(v)),
        None => (line, ""),
    };

    match field {
        "event" => {
            event_type.clear();
            push_truncated(event_type, raw_val);
        }
        "data" => {
            if *has_data {
                let _ = data.push('\n');
            }
            push_truncated(data, raw_val);
            *has_data = true;
        }
        "id" => {
            // Per WHATWG spec, ignore `id:` values containing U+0000 NULL.
            if !raw_val.contains('\0') {
                state.last_event_id.clear();
                push_truncated(&mut state.last_event_id, raw_val);
            }
        }
        "retry" => {
            if let Ok(ms) = raw_val.parse::<u64>() {
                let clamped = ms.clamp(1_000, 120_000);
                state.retry_delay = Duration::from_millis(clamped);
            }
        }
        _ => {}
    }
}

fn push_truncated<const N: usize>(dst: &mut String<N>, src: &str) {
    for ch in src.chars() {
        if dst.push(ch).is_err() {
            break;
        }
    }
}

fn clone_tls_verify<'a>(v: &TlsVerify<'a>) -> TlsVerify<'a> {
    match v {
        TlsVerify::None => TlsVerify::None,
        TlsVerify::Psk { identity, psk } => TlsVerify::Psk { identity, psk },
        TlsVerify::Certificate { ca, cert, key } => TlsVerify::Certificate {
            ca,
            cert: *cert,
            key: *key,
        },
    }
}
