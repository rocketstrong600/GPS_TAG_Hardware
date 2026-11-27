//! TLS 1.3 HTTPS / HTTP client over the `embassy-net` PPP stack (`reqwless` + `embedded-tls`).
//!
//! Uses `embassy-net`'s `TcpClient` + `DnsSocket` over CMUX `DLCI 2`, seeded with hardware TRNG
//! entropy from [`crate::board::Board::rng`] (`b.rng.next_u64()`).
//!
//! ## Buffer Sizing
//! TLS 1.3 allows records up to 16 KiB + framing/MAC overhead, so [`HttpsBuffers`] allocates a
//! 16,640-byte TLS read buffer and 4,096-byte TLS write buffer. Store [`HttpsBuffers`] in a
//! `static_cell::StaticCell` to keep async task stacks compact.
//!
//! ## Verification Modes
//! Pass [`TlsVerify::Certificate`] with a DER-encoded root CA certificate for full X.509 server
//! authentication (plus optional client cert/key for mTLS), [`TlsVerify::Psk`] for pre-shared
//! keys, or [`TlsVerify::None`] for encrypted TLS 1.3 without CA verification during testing.
//!
//! See `examples/modem_https.rs` for a complete runnable example.

#![allow(dead_code)]

pub mod sse;

use defmt::Format;
use embassy_net::Stack;
use embassy_net::dns::DnsSocket;
use embassy_net::tcp::client::{TcpClient, TcpClientState};
use embassy_time::{Duration, WithTimeout};
use embedded_io_async::Read as _;
use reqwless::request::RequestBuilder;

pub use reqwless::client::{HttpClient, TlsConfig, TlsVerify};
pub use reqwless::headers::ContentType;
pub use reqwless::request::Method;

/// TLS 1.3 RX record buffer size (full 16 KiB max TLS record + header/AEAD tag overhead).
pub const TLS_RX_BUF_SIZE: usize = 16_640;
/// TLS 1.3 TX record buffer size.
pub const TLS_TX_BUF_SIZE: usize = 4_096;
/// TCP socket RX buffer size.
pub const TCP_RX_BUF_SIZE: usize = 4_096;
/// TCP socket TX buffer size.
pub const TCP_TX_BUF_SIZE: usize = 2_048;
/// HTTP response header scratch buffer size.
pub const HTTP_HEADER_BUF_SIZE: usize = 2_048;
/// Default end-to-end timeout for DNS + TCP + TLS 1.3 handshake + HTTP exchange.
pub const DEFAULT_TIMEOUT: Duration = Duration::from_secs(30);

/// Reusable socket, TLS 1.3, and HTTP header buffers for [`request`], [`get`], and [`post_json`].
///
/// Initialize in a `StaticCell<HttpsBuffers>` via [`Self::new`] so the ~29 KiB buffers live in
/// static RAM rather than on a task stack.
pub struct HttpsBuffers {
    pub tcp_state: TcpClientState<1, TCP_TX_BUF_SIZE, TCP_RX_BUF_SIZE>,
    pub tls_read: [u8; TLS_RX_BUF_SIZE],
    pub tls_write: [u8; TLS_TX_BUF_SIZE],
    pub header_rx: [u8; HTTP_HEADER_BUF_SIZE],
}

impl HttpsBuffers {
    /// Creates zero-initialized static buffers for a single concurrent HTTPS/HTTP request.
    pub const fn new() -> Self {
        Self {
            tcp_state: TcpClientState::new(),
            tls_read: [0u8; TLS_RX_BUF_SIZE],
            tls_write: [0u8; TLS_TX_BUF_SIZE],
            header_rx: [0u8; HTTP_HEADER_BUF_SIZE],
        }
    }
}

impl Default for HttpsBuffers {
    fn default() -> Self {
        Self::new()
    }
}

/// Errors returned by [`request`], [`get`], and [`post_json`].
#[derive(Debug, Copy, Clone, Eq, PartialEq, Format)]
pub enum HttpsError {
    Dns,
    Network,
    Tls,
    Codec,
    InvalidUrl,
    BufferTooSmall,
    ConnectionAborted,
    Timeout,
    /// Server returned an unexpected non-2xx HTTP status code (e.g. on an SSE stream connect).
    HttpStatus(u16),
    Other,
}

impl From<reqwless::Error> for HttpsError {
    fn from(e: reqwless::Error) -> Self {
        match e {
            reqwless::Error::Dns => Self::Dns,
            reqwless::Error::Network(_) => Self::Network,
            reqwless::Error::Tls(_) => Self::Tls,
            reqwless::Error::Codec => Self::Codec,
            reqwless::Error::InvalidUrl(_) => Self::InvalidUrl,
            reqwless::Error::BufferTooSmall => Self::BufferTooSmall,
            reqwless::Error::ConnectionAborted => Self::ConnectionAborted,
            reqwless::Error::AlreadySent | reqwless::Error::IncorrectBodyWritten => Self::Other,
        }
    }
}

/// Summary of a completed HTTP/HTTPS response whose body was copied into the caller's buffer.
#[derive(Debug, Copy, Clone, Eq, PartialEq, Format)]
pub struct HttpsResponse {
    /// HTTP status code (e.g. `200`, `201`, `404`).
    pub status: u16,
    /// Number of response body bytes written into `out_body`.
    pub body_len: usize,
    /// `true` if the server sent more body bytes than fit in `out_body`.
    pub truncated: bool,
}

impl HttpsResponse {
    /// Interprets the written portion of `out_body` as UTF-8 text.
    pub fn body_str<'a>(&self, out_body: &'a [u8]) -> Result<&'a str, core::str::Utf8Error> {
        core::str::from_utf8(&out_body[..self.body_len])
    }
}

/// Performs an HTTPS or HTTP `GET` request, copying up to `out_body.len()` response bytes.
pub async fn get(
    stack: Stack<'static>,
    bufs: &mut HttpsBuffers,
    tls_seed: u64,
    url: &str,
    headers: &[(&str, &str)],
    verify: TlsVerify<'_>,
    out_body: &mut [u8],
) -> Result<HttpsResponse, HttpsError> {
    request(
        stack,
        bufs,
        tls_seed,
        Method::GET,
        url,
        None,
        headers,
        None,
        verify,
        DEFAULT_TIMEOUT,
        out_body,
    )
    .await
}

/// Performs an HTTPS or HTTP `POST` request with `Content-Type: application/json`, copying up to
/// `out_body.len()` response bytes.
#[allow(clippy::too_many_arguments)]
pub async fn post_json(
    stack: Stack<'static>,
    bufs: &mut HttpsBuffers,
    tls_seed: u64,
    url: &str,
    json_body: &[u8],
    headers: &[(&str, &str)],
    verify: TlsVerify<'_>,
    out_body: &mut [u8],
) -> Result<HttpsResponse, HttpsError> {
    request(
        stack,
        bufs,
        tls_seed,
        Method::POST,
        url,
        Some(ContentType::ApplicationJson),
        headers,
        Some(json_body),
        verify,
        DEFAULT_TIMEOUT,
        out_body,
    )
    .await
}

/// Performs a full HTTPS/HTTP request (`GET`, `POST`, `PUT`, `PATCH`, `DELETE`, etc.) over the
/// `embassy-net` PPP stack with TLS 1.3 support.
#[allow(clippy::too_many_arguments)]
pub async fn request(
    stack: Stack<'static>,
    bufs: &mut HttpsBuffers,
    tls_seed: u64,
    method: Method,
    url: &str,
    content_type: Option<ContentType>,
    headers: &[(&str, &str)],
    body: Option<&[u8]>,
    verify: TlsVerify<'_>,
    timeout: Duration,
    out_body: &mut [u8],
) -> Result<HttpsResponse, HttpsError> {
    let fut = async {
        let mut tcp = TcpClient::new(stack, &bufs.tcp_state);
        tcp.set_timeout(Some(timeout));
        let dns = DnsSocket::new(stack);

        let tls = TlsConfig::new(tls_seed, &mut bufs.tls_read, &mut bufs.tls_write, verify);
        let mut client = HttpClient::new_with_tls(&tcp, &dns, tls);

        let mut req = client.request(method, url).await?;
        if let Some(ct) = content_type {
            req = req.content_type(ct);
        }
        if !headers.is_empty() {
            req = req.headers(headers);
        }

        match body {
            Some(bytes) => {
                let mut req = req.body(bytes);
                let response = req.send(&mut bufs.header_rx).await?;
                read_response(response, out_body).await
            }
            None => {
                let response = req.send(&mut bufs.header_rx).await?;
                read_response(response, out_body).await
            }
        }
    };

    fut.with_timeout(timeout)
        .await
        .map_err(|_| HttpsError::Timeout)?
}

async fn read_response<C: embedded_io_async::Read>(
    response: reqwless::response::Response<'_, '_, C>,
    out_body: &mut [u8],
) -> Result<HttpsResponse, HttpsError> {
    let status = response.status.0;
    let mut reader = response.body().reader();
    let mut body_len = 0usize;
    let mut truncated = false;

    while body_len < out_body.len() {
        match reader.read(&mut out_body[body_len..]).await {
            Ok(0) => break,
            Ok(n) => body_len += n,
            Err(_) if body_len > 0 => break,
            Err(e) => return Err(HttpsError::from(e)),
        }
    }

    if body_len == out_body.len() {
        let mut probe = [0u8; 1];
        if matches!(reader.read(&mut probe).await, Ok(n) if n > 0) {
            truncated = true;
        }
    }

    Ok(HttpsResponse {
        status,
        body_len,
        truncated,
    })
}
