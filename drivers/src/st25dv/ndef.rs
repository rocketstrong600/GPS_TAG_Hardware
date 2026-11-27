//! Zero-allocation `no_std` NFC Forum Type 5 NDEF codec.
//!
//! Provides composable, record-level primitives for Type 5 tags (such as the ST25DV16K):
//! - **Framing:** [`probe_ndef_frame_len`] and [`extract_ndef_message`] handle 4-byte (`0xE1`)
//!   and 8-byte (`0xE2`) Capability Containers plus 1-byte and 3-byte NDEF TLVs.
//! - **Generic decoding:** [`iter_type5_records`] / [`iter_records`] yield [`RawRecord`]s, and
//!   per-record decoders ([`decode_uri_record`], [`decode_text_record`], [`decode_vcard_record`],
//!   [`decode_record`]) parse individual records with zero heap allocation.
//! - **Generic encoding:** [`encode_type5_records`] encodes any slice of [`WriteRecord`]s
//!   (URI, UTF-8 Text, vCard [`ContactCard`], raw vCard, or arbitrary raw TNF/type/payload) into
//!   a 4-byte-aligned Type 5 EEPROM image (`CC + TLV + records + 0xFE`).

use defmt::Format;

/// Maximum Type 5 NDEF frame (CC + TLV + NDEF payload + terminator) handled by the codec.
pub const MAX_FRAME_BYTES: usize = 1024;

/// ST25DV16K 4-byte NFC Forum Type 5 Capability Container:
/// - `0xE1`: NDEF magic number (1-byte block addressing)
/// - `0x40`: Mapping version 1.0, RF read/write allowed
/// - `0xFF`: MLEN = 255 * 8 = 2040 bytes data area (full ST25DV16K capacity)
/// - `0x05`: Supports Special Frame + Read Multiple Blocks
pub const CC_ST25DV16K: [u8; 4] = [0xE1, 0x40, 0xFF, 0x05];

const TLV_NULL: u8 = 0x00;
const TLV_NDEF: u8 = 0x03;
const TLV_TERMINATOR: u8 = 0xFE;

/// NFC Forum Type Name Format: Well-Known Type (`0x01`, e.g. `"U"`, `"T"`).
pub const TNF_WELL_KNOWN: u8 = 0x01;
/// NFC Forum Type Name Format: Media-type / RFC 2046 MIME (`0x02`, e.g. `"text/vcard"`).
pub const TNF_MIME_MEDIA: u8 = 0x02;

/// Standard MIME type for vCard contact records recognized by Android, iOS, and NFC Tools.
pub const MIME_VCARD: &[u8] = b"text/vcard";

const FLAG_MB: u8 = 0x80;
const FLAG_ME: u8 = 0x40;
const FLAG_CF: u8 = 0x20;
const FLAG_SR: u8 = 0x10;
const FLAG_IL: u8 = 0x08;

/// NFC Forum RTD-URI prefix table (codes `0x00..=0x23`).
const URI_PREFIXES: &[&str] = &[
    "",
    "http://www.",
    "https://www.",
    "http://",
    "https://",
    "tel:",
    "mailto:",
    "ftp://anonymous:anonymous@",
    "ftp://ftp.",
    "ftps://",
    "sftp://",
    "smb://",
    "nfs://",
    "ftp://",
    "dav://",
    "news:",
    "telnet://",
    "imap:",
    "rtsp://",
    "urn:",
    "pop:",
    "sip:",
    "sips:",
    "tftp:",
    "btspp://",
    "btl2cap://",
    "btgoep://",
    "tcpobex://",
    "irdaobex://",
    "file://",
    "urn:epc:id:",
    "urn:epc:tag:",
    "urn:epc:pat:",
    "urn:epc:raw:",
    "urn:epc:",
    "urn:nfc:",
];

#[derive(Debug, Copy, Clone, Eq, PartialEq, Format)]
pub enum NdefError {
    /// Capability Container at address `0x0000` is missing or invalid.
    InvalidCc,
    /// TLV framing or NDEF record header is truncated or malformed.
    Malformed,
    /// Provided output buffer is too small for the decoded or encoded data.
    BufferTooSmall,
    /// URI, Text, or vCard payload is not valid UTF-8, uses unsupported UTF-16, or contains
    /// forbidden control characters (such as `\r`/`\n` inside a [`ContactCard`] field).
    InvalidEncoding,
}

/// A single raw NDEF record sliced directly from an NDEF message buffer.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct RawRecord<'a> {
    /// 3-bit Type Name Format field (`0x00..=0x07`, e.g. [`TNF_WELL_KNOWN`]).
    pub tnf: u8,
    /// Record type bytes (e.g. `b"U"`, `b"T"`, or a MIME type).
    pub rec_type: &'a [u8],
    /// Optional record ID bytes (empty if `IL = 0`).
    pub id: &'a [u8],
    /// Raw record payload bytes.
    pub payload: &'a [u8],
}

/// Decoded NFC Forum Well-Known URI record (`TNF = 0x01`, Type = `"U"`).
///
/// The full URI is the concatenation `prefix + rest`, avoiding any mandatory copy.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct UriRecord<'a> {
    pub prefix: &'a str,
    pub rest: &'a str,
}

impl<'a> UriRecord<'a> {
    /// Total UTF-8 byte length of the expanded URI (`prefix.len() + rest.len()`).
    pub fn len(&self) -> usize {
        self.prefix.len() + self.rest.len()
    }

    /// Returns `true` if the expanded URI is empty.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Compares the expanded URI against `expected` without allocating or copying into a buffer.
    pub fn equals(&self, expected: &str) -> bool {
        expected.len() == self.len()
            && expected.starts_with(self.prefix)
            && &expected[self.prefix.len()..] == self.rest
    }

    /// Copies the expanded UTF-8 URI into `out` and returns the number of bytes written.
    pub fn write_into(&self, out: &mut [u8]) -> Result<usize, NdefError> {
        let total = self.len();
        if total > out.len() {
            return Err(NdefError::BufferTooSmall);
        }
        out[..self.prefix.len()].copy_from_slice(self.prefix.as_bytes());
        out[self.prefix.len()..total].copy_from_slice(self.rest.as_bytes());
        Ok(total)
    }
}

/// Decoded NFC Forum Well-Known UTF-8 Text record (`TNF = 0x01`, Type = `"T"`).
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct TextRecord<'a> {
    /// IANA language code (e.g. `"en"`).
    pub lang: &'a str,
    /// UTF-8 text payload.
    pub text: &'a str,
}

/// Structured contact card encoded as a vCard 3.0 MIME record (`TNF = 0x02`, Type =
/// `"text/vcard"`).
///
/// Recognized natively by Android, iOS, and NFC Tools as a contact card. Field values must not
/// contain `\r` or `\n` (encoding returns [`NdefError::InvalidEncoding`]).
///
/// For custom vCard properties (such as `ADR`, `TITLE`, or multiple `TEL` entries), use
/// [`WriteRecord::VCard`] with a pre-formatted vCard string instead.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct ContactCard<'a> {
    /// Full display name (emitted as `FN:` and automatically split into `N:last;first;;;`).
    pub name: &'a str,
    /// Optional organization / company (`ORG:`).
    pub org: Option<&'a str>,
    /// Optional phone number (`TEL:`).
    pub phone: Option<&'a str>,
    /// Optional email address (`EMAIL:`).
    pub email: Option<&'a str>,
    /// Optional website URL (`URL:`).
    pub url: Option<&'a str>,
    /// Optional free-form note (`NOTE:`).
    pub note: Option<&'a str>,
}

impl<'a> ContactCard<'a> {
    /// Creates a minimal contact card with only a display name and all optional fields unset.
    pub const fn new(name: &'a str) -> Self {
        Self {
            name,
            org: None,
            phone: None,
            email: None,
            url: None,
            note: None,
        }
    }

    /// Returns the exact UTF-8 byte length of the serialized vCard 3.0 payload, validating that
    /// no field contains `\r` or `\n`.
    pub fn encoded_len(&self) -> Result<usize, NdefError> {
        validate_vcard_field(self.name)?;
        let (last, first) = split_vcard_name(self.name);

        let mut len = b"BEGIN:VCARD\r\nVERSION:3.0\r\n".len();
        len += b"N:".len() + last.len() + 1 + first.len() + b";;;\r\n".len();
        len += b"FN:".len() + self.name.len() + 2;

        for (prefix, val) in [
            (b"ORG:".as_slice(), self.org),
            (b"TEL:".as_slice(), self.phone),
            (b"EMAIL:".as_slice(), self.email),
            (b"URL:".as_slice(), self.url),
            (b"NOTE:".as_slice(), self.note),
        ] {
            if let Some(v) = val {
                validate_vcard_field(v)?;
                len += prefix.len() + v.len() + 2;
            }
        }

        len += b"END:VCARD\r\n".len();
        Ok(len)
    }

    /// Serializes this contact card as a vCard 3.0 payload into `out` and returns the number of
    /// bytes written.
    pub fn encode(&self, out: &mut [u8]) -> Result<usize, NdefError> {
        let needed = self.encoded_len()?;
        if needed > out.len() {
            return Err(NdefError::BufferTooSmall);
        }

        let (last, first) = split_vcard_name(self.name);
        let mut pos = 0usize;
        append_bytes(out, &mut pos, b"BEGIN:VCARD\r\nVERSION:3.0\r\n");
        append_bytes(out, &mut pos, b"N:");
        append_bytes(out, &mut pos, last.as_bytes());
        append_bytes(out, &mut pos, b";");
        append_bytes(out, &mut pos, first.as_bytes());
        append_bytes(out, &mut pos, b";;;\r\n");
        append_vcard_line(out, &mut pos, b"FN:", self.name);

        if let Some(v) = self.org {
            append_vcard_line(out, &mut pos, b"ORG:", v);
        }
        if let Some(v) = self.phone {
            append_vcard_line(out, &mut pos, b"TEL:", v);
        }
        if let Some(v) = self.email {
            append_vcard_line(out, &mut pos, b"EMAIL:", v);
        }
        if let Some(v) = self.url {
            append_vcard_line(out, &mut pos, b"URL:", v);
        }
        if let Some(v) = self.note {
            append_vcard_line(out, &mut pos, b"NOTE:", v);
        }

        append_bytes(out, &mut pos, b"END:VCARD\r\n");
        Ok(pos)
    }
}

/// Decoded NFC Forum MIME vCard contact record (`TNF = 0x02`, Type = `"text/vcard"` or
/// `"text/x-vcard"`).
///
/// Holds the zero-copy UTF-8 vCard payload in [`Self::raw`] along with zero-allocation helpers
/// for reading common properties (`FN`, `ORG`, `TEL`, `EMAIL`, `URL`, `NOTE`).
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct VCardRecord<'a> {
    /// Raw UTF-8 vCard text (e.g. `BEGIN:VCARD\r\n...END:VCARD\r\n`).
    pub raw: &'a str,
}

impl<'a> VCardRecord<'a> {
    /// Returns the first value matching vCard property `key` (ASCII case-insensitive), ignoring
    /// optional group prefixes (e.g. `item1.EMAIL:...`) and parameters (e.g. `TEL;TYPE=CELL:...`).
    pub fn field(&self, key: &str) -> Option<&'a str> {
        for line in self.raw.split('\n') {
            let line = line.strip_suffix('\r').unwrap_or(line);
            let Some((lhs, value)) = line.split_once(':') else {
                continue;
            };
            let prop = lhs.split(';').next().unwrap_or(lhs);
            let prop = prop.rsplit('.').next().unwrap_or(prop);
            if prop.eq_ignore_ascii_case(key) {
                return Some(value);
            }
        }
        None
    }

    /// Formatted display name (`FN`), if present.
    pub fn name(&self) -> Option<&'a str> {
        self.field("FN")
    }

    /// Organization (`ORG`), if present.
    pub fn org(&self) -> Option<&'a str> {
        self.field("ORG")
    }

    /// First phone number (`TEL`), if present.
    pub fn phone(&self) -> Option<&'a str> {
        self.field("TEL")
    }

    /// First email address (`EMAIL`), if present.
    pub fn email(&self) -> Option<&'a str> {
        self.field("EMAIL")
    }

    /// First URL (`URL`), if present.
    pub fn url(&self) -> Option<&'a str> {
        self.field("URL")
    }

    /// Note (`NOTE`), if present.
    pub fn note(&self) -> Option<&'a str> {
        self.field("NOTE")
    }
}

/// Typed representation of a decoded NDEF record.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum DecodedRecord<'a> {
    Uri(UriRecord<'a>),
    Text(TextRecord<'a>),
    VCard(VCardRecord<'a>),
    Other(RawRecord<'a>),
}

/// Record specification accepted by [`encode_type5_records`].
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum WriteRecord<'a> {
    /// Well-Known URI record (`TNF = 0x01`, Type = `"U"`) with automatic prefix compression.
    Uri(&'a str),
    /// Well-Known UTF-8 Text record (`TNF = 0x01`, Type = `"T"`).
    Text { lang: &'a str, text: &'a str },
    /// Structured vCard 3.0 contact card (`TNF = 0x02`, Type = `"text/vcard"`).
    Contact(ContactCard<'a>),
    /// Pre-formatted raw vCard text (`TNF = 0x02`, Type = `"text/vcard"`).
    VCard(&'a str),
    /// Arbitrary NDEF record with caller-supplied TNF, type, and payload.
    Raw {
        tnf: u8,
        rec_type: &'a [u8],
        payload: &'a [u8],
    },
}

/// Zero-allocation iterator over [`RawRecord`]s inside an NDEF message slice.
#[derive(Debug, Clone)]
pub struct RecordIter<'a> {
    msg: &'a [u8],
    idx: usize,
    done: bool,
}

impl<'a> Iterator for RecordIter<'a> {
    type Item = Result<RawRecord<'a>, NdefError>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.done || self.idx >= self.msg.len() {
            return None;
        }
        match parse_next_record(self.msg, &mut self.idx) {
            Ok((rec, is_last)) => {
                if is_last {
                    self.done = true;
                }
                Some(Ok(rec))
            }
            Err(e) => {
                self.done = true;
                Some(Err(e))
            }
        }
    }
}

/// Inspects the beginning of User EEPROM (at least 8 bytes, typically 32 bytes) to determine
/// how many total bytes from `0x0000` must be read to cover the full NDEF Message TLV.
///
/// Returns `Ok(Some(total_bytes))` when an NDEF TLV is located, or `Ok(None)` if the tag has a
/// valid CC but no NDEF TLV (e.g. empty tag terminated by `0xFE`).
pub fn probe_ndef_frame_len(head: &[u8]) -> Result<Option<usize>, NdefError> {
    let mut pos = cc_len(head)?;
    while pos < head.len() {
        let tag = head[pos];
        if tag == TLV_NULL {
            pos += 1;
            continue;
        }
        if tag == TLV_TERMINATOR {
            return Ok(None);
        }
        if pos + 1 >= head.len() {
            return Ok(Some(MAX_FRAME_BYTES));
        }
        let (hdr_len, val_len) = if head[pos + 1] == 0xFF {
            if pos + 3 >= head.len() {
                return Ok(Some(MAX_FRAME_BYTES));
            }
            (
                4,
                u16::from_be_bytes([head[pos + 2], head[pos + 3]]) as usize,
            )
        } else {
            (2, head[pos + 1] as usize)
        };
        let end = pos.saturating_add(hdr_len).saturating_add(val_len);
        if tag == TLV_NDEF {
            return Ok(Some(end));
        }
        pos = end;
    }
    Ok(Some(MAX_FRAME_BYTES))
}

/// Extracts the raw NDEF Message TLV (`0x03`) payload slice from a Type 5 EEPROM image starting
/// at address `0x0000`.
///
/// Returns `Ok(None)` if the tag has a valid CC but contains no NDEF TLV.
pub fn extract_ndef_message(raw: &[u8]) -> Result<Option<&[u8]>, NdefError> {
    let mut pos = cc_len(raw)?;
    while pos < raw.len() {
        let tag = raw[pos];
        if tag == TLV_NULL {
            pos += 1;
            continue;
        }
        if tag == TLV_TERMINATOR {
            return Ok(None);
        }
        if pos + 1 >= raw.len() {
            return Err(NdefError::Malformed);
        }
        let (hdr_len, val_len) = if raw[pos + 1] == 0xFF {
            if pos + 3 >= raw.len() {
                return Err(NdefError::Malformed);
            }
            (4, u16::from_be_bytes([raw[pos + 2], raw[pos + 3]]) as usize)
        } else {
            (2, raw[pos + 1] as usize)
        };
        let val_start = pos + hdr_len;
        let val_end = val_start.checked_add(val_len).ok_or(NdefError::Malformed)?;
        if val_end > raw.len() {
            return Err(NdefError::Malformed);
        }
        if tag == TLV_NDEF {
            return Ok(Some(&raw[val_start..val_end]));
        }
        pos = val_end;
    }
    Ok(None)
}

/// Returns a [`RecordIter`] over all NDEF records in a raw NDEF message slice.
pub fn iter_records(msg: &[u8]) -> RecordIter<'_> {
    RecordIter {
        msg,
        idx: 0,
        done: false,
    }
}

/// Extracts the NDEF Message TLV from a Type 5 EEPROM image (`0x0000..`) and returns a
/// [`RecordIter`] over its records (yielding an empty iterator if no NDEF TLV is present).
pub fn iter_type5_records(raw: &[u8]) -> Result<RecordIter<'_>, NdefError> {
    let msg = extract_ndef_message(raw)?.unwrap_or(&[]);
    Ok(iter_records(msg))
}

/// Decodes a single [`RawRecord`] as a Well-Known URI record (`TNF = 0x01`, Type = `"U"`).
///
/// Returns `Ok(None)` if `rec` is not a Well-Known URI record.
pub fn decode_uri_record<'a>(rec: &RawRecord<'a>) -> Result<Option<UriRecord<'a>>, NdefError> {
    if rec.tnf != TNF_WELL_KNOWN || rec.rec_type != b"U" {
        return Ok(None);
    }
    if rec.payload.is_empty() {
        return Err(NdefError::Malformed);
    }
    let prefix = URI_PREFIXES
        .get(rec.payload[0] as usize)
        .copied()
        .unwrap_or("");
    let rest = core::str::from_utf8(&rec.payload[1..]).map_err(|_| NdefError::InvalidEncoding)?;
    Ok(Some(UriRecord { prefix, rest }))
}

/// Decodes a single [`RawRecord`] as a Well-Known UTF-8 Text record (`TNF = 0x01`, Type = `"T"`).
///
/// Returns `Ok(None)` if `rec` is not a Well-Known Text record.
pub fn decode_text_record<'a>(rec: &RawRecord<'a>) -> Result<Option<TextRecord<'a>>, NdefError> {
    if rec.tnf != TNF_WELL_KNOWN || rec.rec_type != b"T" {
        return Ok(None);
    }
    if rec.payload.is_empty() {
        return Err(NdefError::Malformed);
    }
    let status = rec.payload[0];
    if (status & 0x80) != 0 {
        // UTF-16 is unsupported; standard NFC apps write UTF-8.
        return Err(NdefError::InvalidEncoding);
    }
    let lang_len = (status & 0x3F) as usize;
    let text_start = 1 + lang_len;
    if text_start > rec.payload.len() {
        return Err(NdefError::Malformed);
    }
    let lang = core::str::from_utf8(&rec.payload[1..text_start])
        .map_err(|_| NdefError::InvalidEncoding)?;
    let text =
        core::str::from_utf8(&rec.payload[text_start..]).map_err(|_| NdefError::InvalidEncoding)?;
    Ok(Some(TextRecord { lang, text }))
}

/// Decodes a single [`RawRecord`] as a MIME vCard contact record (`TNF = 0x02`, Type =
/// `"text/vcard"` or `"text/x-vcard"`, matched ASCII case-insensitively with optional MIME
/// parameters stripped).
///
/// Returns `Ok(None)` if `rec` is not a vCard MIME record.
pub fn decode_vcard_record<'a>(rec: &RawRecord<'a>) -> Result<Option<VCardRecord<'a>>, NdefError> {
    if rec.tnf != TNF_MIME_MEDIA {
        return Ok(None);
    }
    let mime = rec
        .rec_type
        .split(|&b| b == b';')
        .next()
        .unwrap_or(rec.rec_type);
    if !(mime.eq_ignore_ascii_case(MIME_VCARD) || mime.eq_ignore_ascii_case(b"text/x-vcard")) {
        return Ok(None);
    }
    let raw = core::str::from_utf8(rec.payload).map_err(|_| NdefError::InvalidEncoding)?;
    Ok(Some(VCardRecord { raw }))
}

/// Decodes a [`RawRecord`] into [`DecodedRecord::Uri`], [`DecodedRecord::Text`],
/// [`DecodedRecord::VCard`], or [`DecodedRecord::Other`].
pub fn decode_record<'a>(rec: RawRecord<'a>) -> Result<DecodedRecord<'a>, NdefError> {
    if let Some(uri) = decode_uri_record(&rec)? {
        return Ok(DecodedRecord::Uri(uri));
    }
    if let Some(txt) = decode_text_record(&rec)? {
        return Ok(DecodedRecord::Text(txt));
    }
    if let Some(vc) = decode_vcard_record(&rec)? {
        return Ok(DecodedRecord::VCard(vc));
    }
    Ok(DecodedRecord::Other(rec))
}

/// Returns `true` if a decoded [`RawRecord`] semantically matches an expected [`WriteRecord`],
/// ignoring NDEF message framing flags (`MB`/`ME`/`SR`), TLV length, and URI prefix compression
/// differences.
pub fn record_matches(raw: &RawRecord<'_>, expected: &WriteRecord<'_>) -> bool {
    match expected {
        WriteRecord::Uri(expected_url) => matches!(
            decode_uri_record(raw),
            Ok(Some(uri)) if uri.equals(expected_url)
        ),
        WriteRecord::Text { lang, text } => matches!(
            decode_text_record(raw),
            Ok(Some(txt)) if txt.lang == *lang && txt.text == *text
        ),
        WriteRecord::Contact(card) => {
            let Ok(Some(vc)) = decode_vcard_record(raw) else {
                return false;
            };
            let Ok(expected_len) = card.encoded_len() else {
                return false;
            };
            if vc.raw.len() != expected_len || expected_len > MAX_FRAME_BYTES {
                return false;
            }
            let mut buf = [0u8; MAX_FRAME_BYTES];
            card.encode(&mut buf[..expected_len])
                .is_ok_and(|n| vc.raw.as_bytes() == &buf[..n])
        }
        WriteRecord::VCard(expected_vcard) => matches!(
            decode_vcard_record(raw),
            Ok(Some(vc)) if vc.raw == *expected_vcard
        ),
        WriteRecord::Raw {
            tnf,
            rec_type,
            payload,
        } => raw.tnf == (tnf & 0x07) && raw.rec_type == *rec_type && raw.payload == *payload,
    }
}

/// Encodes any slice of [`WriteRecord`]s into a complete NFC Forum Type 5 EEPROM image
/// (`0x0000..`) containing:
/// - 4-byte ST25DV16K Capability Container ([`CC_ST25DV16K`])
/// - NDEF Message TLV (`0x03`, 1-byte or 3-byte length)
/// - Encoded NDEF records with `MB`/`ME`/`SR` flags set automatically
/// - Terminator TLV (`0xFE`), padded to a 4-byte EEPROM block boundary.
///
/// Returns the total encoded byte length written into `out`.
pub fn encode_type5_records(
    records: &[WriteRecord<'_>],
    out: &mut [u8],
) -> Result<usize, NdefError> {
    let mut ndef_len = 0usize;
    for rec in records {
        ndef_len = ndef_len
            .checked_add(write_record_len(rec)?)
            .ok_or(NdefError::BufferTooSmall)?;
    }

    let tlv_hdr_len = if ndef_len <= 0xFE { 2 } else { 4 };
    let unaligned_len = CC_ST25DV16K.len() + tlv_hdr_len + ndef_len + 1; // +1 for 0xFE
    let total_len = (unaligned_len + 3) & !3; // Align to 4-byte ST25DV16K block size

    if total_len > out.len() || ndef_len > u16::MAX as usize {
        return Err(NdefError::BufferTooSmall);
    }

    let mut pos = 0usize;
    out[pos..pos + 4].copy_from_slice(&CC_ST25DV16K);
    pos += 4;

    out[pos] = TLV_NDEF;
    pos += 1;
    if ndef_len <= 0xFE {
        out[pos] = ndef_len as u8;
        pos += 1;
    } else {
        out[pos] = 0xFF;
        let be = (ndef_len as u16).to_be_bytes();
        out[pos + 1] = be[0];
        out[pos + 2] = be[1];
        pos += 3;
    }

    for (i, rec) in records.iter().enumerate() {
        let is_first = i == 0;
        let is_last = i + 1 == records.len();
        pos += encode_single_record(rec, is_first, is_last, &mut out[pos..])?;
    }

    out[pos] = TLV_TERMINATOR;
    pos += 1;

    while pos < total_len {
        out[pos] = 0x00;
        pos += 1;
    }

    Ok(total_len)
}

fn cc_len(raw: &[u8]) -> Result<usize, NdefError> {
    if raw.len() < 4 {
        return Err(NdefError::InvalidCc);
    }
    match raw[0] {
        0xE1 => Ok(4),
        0xE2 if raw.len() >= 8 => Ok(8),
        _ => Err(NdefError::InvalidCc),
    }
}

fn parse_next_record<'a>(
    msg: &'a [u8],
    idx: &mut usize,
) -> Result<(RawRecord<'a>, bool), NdefError> {
    if *idx + 2 > msg.len() {
        return Err(NdefError::Malformed);
    }
    let hdr = msg[*idx];
    let me = (hdr & FLAG_ME) != 0;
    let cf = (hdr & FLAG_CF) != 0;
    let sr = (hdr & FLAG_SR) != 0;
    let il = (hdr & FLAG_IL) != 0;
    let tnf = hdr & 0x07;
    let type_len = msg[*idx + 1] as usize;
    *idx += 2;

    if cf {
        return Err(NdefError::Malformed);
    }

    let payload_len = if sr {
        if *idx >= msg.len() {
            return Err(NdefError::Malformed);
        }
        let l = msg[*idx] as usize;
        *idx += 1;
        l
    } else {
        if *idx + 4 > msg.len() {
            return Err(NdefError::Malformed);
        }
        let l =
            u32::from_be_bytes([msg[*idx], msg[*idx + 1], msg[*idx + 2], msg[*idx + 3]]) as usize;
        *idx += 4;
        l
    };

    let id_len = if il {
        if *idx >= msg.len() {
            return Err(NdefError::Malformed);
        }
        let l = msg[*idx] as usize;
        *idx += 1;
        l
    } else {
        0
    };

    let type_end = idx.checked_add(type_len).ok_or(NdefError::Malformed)?;
    let id_end = type_end.checked_add(id_len).ok_or(NdefError::Malformed)?;
    let payload_end = id_end
        .checked_add(payload_len)
        .ok_or(NdefError::Malformed)?;
    if payload_end > msg.len() {
        return Err(NdefError::Malformed);
    }

    let rec = RawRecord {
        tnf,
        rec_type: &msg[*idx..type_end],
        id: &msg[type_end..id_end],
        payload: &msg[id_end..payload_end],
    };
    *idx = payload_end;
    Ok((rec, me))
}

fn write_record_len(rec: &WriteRecord<'_>) -> Result<usize, NdefError> {
    match rec {
        WriteRecord::Uri(url) => {
            let (_, rest) = match_uri_prefix(url);
            Ok(record_encoded_len(1, 1 + rest.len()))
        }
        WriteRecord::Text { lang, text } => {
            if lang.len() > 0x3F {
                return Err(NdefError::Malformed);
            }
            Ok(record_encoded_len(1, 1 + lang.len() + text.len()))
        }
        WriteRecord::Contact(card) => Ok(record_encoded_len(MIME_VCARD.len(), card.encoded_len()?)),
        WriteRecord::VCard(raw) => Ok(record_encoded_len(MIME_VCARD.len(), raw.len())),
        WriteRecord::Raw {
            rec_type, payload, ..
        } => {
            if rec_type.len() > u8::MAX as usize {
                return Err(NdefError::Malformed);
            }
            Ok(record_encoded_len(rec_type.len(), payload.len()))
        }
    }
}

fn encode_single_record(
    rec: &WriteRecord<'_>,
    is_first: bool,
    is_last: bool,
    out: &mut [u8],
) -> Result<usize, NdefError> {
    let base_flags = if is_first { FLAG_MB } else { 0 } | if is_last { FLAG_ME } else { 0 };

    match rec {
        WriteRecord::Uri(url) => {
            let (prefix_code, rest) = match_uri_prefix(url);
            let payload_len = 1 + rest.len();
            let flags = base_flags | if payload_len <= 0xFF { FLAG_SR } else { 0 } | TNF_WELL_KNOWN;
            let mut pos = write_record_header(out, flags, b"U", payload_len);
            out[pos] = prefix_code;
            pos += 1;
            out[pos..pos + rest.len()].copy_from_slice(rest.as_bytes());
            Ok(pos + rest.len())
        }
        WriteRecord::Text { lang, text } => {
            if lang.len() > 0x3F {
                return Err(NdefError::Malformed);
            }
            let payload_len = 1 + lang.len() + text.len();
            let flags = base_flags | if payload_len <= 0xFF { FLAG_SR } else { 0 } | TNF_WELL_KNOWN;
            let mut pos = write_record_header(out, flags, b"T", payload_len);
            out[pos] = lang.len() as u8; // UTF-8 (bit 7 = 0), language code length
            pos += 1;
            out[pos..pos + lang.len()].copy_from_slice(lang.as_bytes());
            pos += lang.len();
            out[pos..pos + text.len()].copy_from_slice(text.as_bytes());
            Ok(pos + text.len())
        }
        WriteRecord::Contact(card) => {
            let payload_len = card.encoded_len()?;
            let flags = base_flags | if payload_len <= 0xFF { FLAG_SR } else { 0 } | TNF_MIME_MEDIA;
            let pos = write_record_header(out, flags, MIME_VCARD, payload_len);
            let written = card.encode(&mut out[pos..pos + payload_len])?;
            Ok(pos + written)
        }
        WriteRecord::VCard(raw) => {
            let payload = raw.as_bytes();
            let flags =
                base_flags | if payload.len() <= 0xFF { FLAG_SR } else { 0 } | TNF_MIME_MEDIA;
            let pos = write_record_header(out, flags, MIME_VCARD, payload.len());
            out[pos..pos + payload.len()].copy_from_slice(payload);
            Ok(pos + payload.len())
        }
        WriteRecord::Raw {
            tnf,
            rec_type,
            payload,
        } => {
            if rec_type.len() > u8::MAX as usize {
                return Err(NdefError::Malformed);
            }
            let flags = base_flags | if payload.len() <= 0xFF { FLAG_SR } else { 0 } | (tnf & 0x07);
            let pos = write_record_header(out, flags, rec_type, payload.len());
            out[pos..pos + payload.len()].copy_from_slice(payload);
            Ok(pos + payload.len())
        }
    }
}

fn validate_vcard_field(s: &str) -> Result<(), NdefError> {
    if s.bytes().any(|b| b == b'\r' || b == b'\n') {
        return Err(NdefError::InvalidEncoding);
    }
    Ok(())
}

fn split_vcard_name(name: &str) -> (&str, &str) {
    let trimmed = name.trim();
    if let Some(sp) = trimmed.rfind(' ') {
        let first = trimmed[..sp].trim_end();
        let last = &trimmed[sp + 1..];
        (last, first)
    } else {
        ("", trimmed)
    }
}

fn append_bytes(out: &mut [u8], pos: &mut usize, bytes: &[u8]) {
    out[*pos..*pos + bytes.len()].copy_from_slice(bytes);
    *pos += bytes.len();
}

fn append_vcard_line(out: &mut [u8], pos: &mut usize, prefix: &[u8], value: &str) {
    append_bytes(out, pos, prefix);
    append_bytes(out, pos, value.as_bytes());
    append_bytes(out, pos, b"\r\n");
}

fn match_uri_prefix(url: &str) -> (u8, &str) {
    let mut best_code = 0u8;
    let mut best_len = 0usize;
    for (idx, &prefix) in URI_PREFIXES.iter().enumerate().skip(1) {
        if url.starts_with(prefix) && prefix.len() > best_len {
            best_code = idx as u8;
            best_len = prefix.len();
        }
    }
    (best_code, &url[best_len..])
}

fn record_encoded_len(type_len: usize, payload_len: usize) -> usize {
    let len_field = if payload_len <= 0xFF { 1 } else { 4 };
    2 + len_field + type_len + payload_len
}

fn write_record_header(out: &mut [u8], flags: u8, rec_type: &[u8], payload_len: usize) -> usize {
    out[0] = flags;
    out[1] = rec_type.len() as u8;
    let mut pos = 2;
    if (flags & FLAG_SR) != 0 {
        out[pos] = payload_len as u8;
        pos += 1;
    } else {
        out[pos..pos + 4].copy_from_slice(&(payload_len as u32).to_be_bytes());
        pos += 4;
    }
    out[pos..pos + rec_type.len()].copy_from_slice(rec_type);
    pos + rec_type.len()
}
