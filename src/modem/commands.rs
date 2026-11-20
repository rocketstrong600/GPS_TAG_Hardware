#![allow(unused)]
use atat::atat_derive::{AtatCmd, AtatResp, AtatUrc};
use atat::heapless::String;
use atat::heapless_bytes::Bytes;

#[derive(Clone, AtatUrc)]
pub enum Urc {
    #[at_urc("+UMWI")]
    MessageWaitingIndication(MessageWaitingIndication),
    #[at_urc("RDY")]
    Ready,
    #[at_urc("+CME ERROR")]
    CmeError(CmeError),
    #[at_urc("CONNECT")]
    Connect,
}

#[derive(Clone, AtatCmd)]
#[at_cmd("", NoResponse, timeout_ms = 300)]
pub struct AT;

#[derive(Clone, AtatCmd)]
#[at_cmd("E0", NoResponse, value_sep = false, timeout_ms = 300)]
pub struct EchoOff;

/// Enters 3GPP TS 27.010 Basic Option multiplexer mode (`AT+CMUX=0`).
#[derive(Clone, AtatCmd)]
#[at_cmd("+CMUX", NoResponse, timeout_ms = 1000)]
pub struct SetCmux {
    #[at_arg(position = 1)]
    pub mode: u8,
}

/// Sets RAT search mode via `AT+QCFG="nwscanmode",<mode>,<effect>`.
/// `mode`: 0 = Auto, 1 = GSM only, 2 = WCDMA only, 3 = LTE only.
#[derive(Clone, AtatCmd)]
#[at_cmd("+QCFG", NoResponse, timeout_ms = 500)]
pub struct SetNwScanMode {
    #[at_arg(position = 1)]
    pub param: String<12>,
    #[at_arg(position = 2)]
    pub mode: u8,
    #[at_arg(position = 3)]
    pub effect: u8,
}

/// Configures PDP context 1 (`AT+CGDCONT=1,"IP","<apn>"`).
#[derive(Clone, AtatCmd)]
#[at_cmd("+CGDCONT", NoResponse, timeout_ms = 500)]
pub struct SetPdpContext {
    #[at_arg(position = 1)]
    pub cid: u8,
    #[at_arg(position = 2)]
    pub pdp_type: String<8>,
    #[at_arg(position = 3)]
    pub apn: String<32>,
}

#[derive(Clone, Debug, AtatResp)]
pub struct EpsRegStatus {
    #[at_arg(position = 1)]
    pub n: u8,
    /// 0=not searching, 1=home, 2=searching, 3=denied, 5=roaming
    #[at_arg(position = 2)]
    pub stat: u8,
}

#[derive(Clone, AtatCmd)]
#[at_cmd("+CEREG?", EpsRegStatus, timeout_ms = 500)]
pub struct GetEpsRegStatus;

#[derive(Clone, AtatCmd)]
#[at_cmd("+QGPS", NoResponse, timeout_ms = 300)]
pub struct SetGpsPower {
    #[at_arg(position = 1)]
    pub mode: u8, // 1 = ON, 0 = OFF
}

#[derive(Clone, AtatCmd)]
#[at_cmd("+QGPSXTRA", NoResponse, timeout_ms = 500)]
pub struct SetGpsXtra {
    #[at_arg(position = 1)]
    pub mode: u8, // 1 = Enable gpsOneXTRA
}

#[derive(Clone, AtatCmd)]
#[at_cmd("+QGPSCFG", NoResponse, timeout_ms = 500)]
pub struct SetGpsCfg {
    #[at_arg(position = 1)]
    pub param: String<24>,
    #[at_arg(position = 2)]
    pub value: u8,
}

#[derive(Clone, Debug, AtatResp)]
pub struct GpsXtraData {
    #[at_arg(position = 1)]
    pub valid_mins: u32,
    #[at_arg(position = 2)]
    pub injected_time: String<24>,
}

#[derive(Clone, AtatCmd)]
#[at_cmd("+QGPSXTRADATA?", GpsXtraData, timeout_ms = 500)]
pub struct QueryGpsXtraData;

#[derive(Clone, AtatResp)]
pub struct NoResponse;

#[derive(Clone, AtatResp)]
pub struct Ready;

#[derive(Clone, Debug, AtatResp)]
pub struct CmeError {
    #[at_arg(position = 1)]
    pub err: u16,
}

#[derive(Clone, AtatResp)]
pub struct Connect;

#[derive(Clone, AtatResp, Debug)]
pub struct MessageWaitingIndication;

// Response format: +QGPSLOC: <UTC>,<latitude>,<longitude>,<hdop>,<altitude>,<fix>,<cog>,<spkm>,<spkn>,<date>,<nsat>
#[derive(Debug, AtatResp)]
pub struct GpsLocation {
    #[at_arg(position = 1)]
    pub utc: Bytes<10>,
    #[at_arg(position = 2)]
    pub latitude: Bytes<16>,
    #[at_arg(position = 3)]
    pub longitude: Bytes<16>,
    #[at_arg(position = 4)]
    pub hdop: f32,
    #[at_arg(position = 5)]
    pub altitude: f32,
    #[at_arg(position = 6)]
    pub fix: u8,
    #[at_arg(position = 7)]
    pub cog: Bytes<6>,
    #[at_arg(position = 8)]
    pub spkm: f32,
    #[at_arg(position = 9)]
    pub spkn: f32,
    #[at_arg(position = 10)]
    pub date: Bytes<6>,
    #[at_arg(position = 11)]
    pub nsat: Bytes<2>,
}

#[derive(Clone, AtatCmd)]
#[at_cmd("+QGPSLOC", GpsLocation, timeout_ms = 1000)]
pub struct GetGpsLocation {
    #[at_arg(position = 1)]
    pub mode: u8, // 2 = Coordinate format (DD.DDDDD)
}

#[derive(Clone, AtatCmd)]
#[at_cmd("D*99#", NoResponse, value_sep = false)]
pub struct DialPPP;

#[derive(Clone, AtatCmd)]
#[at_cmd("H", NoResponse, value_sep = false)]
pub struct DialHangup;

#[derive(Clone, AtatCmd)]
#[at_cmd(
    "+++",
    NoResponse,
    value_sep = false,
    cmd_prefix = "",
    termination = ""
)]
pub struct EnterCMDMode;
