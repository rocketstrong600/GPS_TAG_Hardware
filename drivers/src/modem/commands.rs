//! `atat` AT command, response, and URC type definitions for the Quectel EC21-AU.

#![allow(unused)]
use atat::AtatCmd;
use atat::atat_derive::{AtatCmd, AtatResp, AtatUrc};
use atat::heapless::String;
use atat::heapless_bytes::Bytes;

#[derive(Clone, AtatUrc)]
pub enum Urc {
    #[at_urc("+UMWI")]
    MessageWaitingIndication(MessageWaitingIndication),
    #[at_urc("RDY")]
    Ready,
    #[at_urc("CONNECT")]
    Connect,
    #[at_urc("+CMTI")]
    NewSms(NewSms),
    #[at_urc("+QHTTPGET")]
    HttpGet(HttpGetResult),
    #[at_urc("+QHTTPREADFILE")]
    HttpReadFile(HttpReadFileResult),
}

#[derive(Clone, Debug, AtatResp)]
pub struct HttpGetResult {
    #[at_arg(position = 1)]
    pub err: u16,
    #[at_arg(position = 2)]
    pub http_status: Option<u16>,
    #[at_arg(position = 3)]
    pub content_len: Option<u32>,
}

#[derive(Clone, Debug, AtatResp)]
pub struct HttpReadFileResult {
    #[at_arg(position = 1)]
    pub err: u16,
}

#[derive(Clone, Debug, AtatResp)]
pub struct NewSms {
    #[at_arg(position = 1)]
    pub storage: String<8>,
    #[at_arg(position = 2)]
    pub index: u16,
}

#[derive(Clone, AtatCmd)]
#[at_cmd("", NoResponse, timeout_ms = 300)]
pub struct AT;

#[derive(Clone, AtatCmd)]
#[at_cmd("E0", NoResponse, value_sep = false, timeout_ms = 300)]
pub struct EchoOff;

#[derive(Clone, AtatCmd)]
#[at_cmd("+CMEE", NoResponse, timeout_ms = 500)]
pub struct SetCmeErrorMode {
    #[at_arg(position = 1)]
    pub mode: u8, // 1 = numeric +CME ERROR codes
}

#[derive(Clone, Debug, AtatResp)]
pub struct FuncMode {
    #[at_arg(position = 1)]
    pub fun: u8,
}

#[derive(Clone, AtatCmd)]
#[at_cmd("+CFUN?", FuncMode, timeout_ms = 1000)]
pub struct GetFuncMode;

/// Sets modem functionality (`0` = minimum / RF+SIM off, `1` = full, `4` = flight mode / RF off).
#[derive(Clone, AtatCmd)]
#[at_cmd("+CFUN", NoResponse, timeout_ms = 5000)]
pub struct SetFuncMode {
    #[at_arg(position = 1)]
    pub fun: u8,
}

#[derive(Clone, Debug, AtatResp)]
pub struct SleepMode {
    #[at_arg(position = 1)]
    pub mode: u8,
}

#[derive(Clone, AtatCmd)]
#[at_cmd("+QSCLK?", SleepMode, timeout_ms = 500)]
pub struct GetSleepMode;

#[derive(Clone, AtatCmd)]
#[at_cmd("+QSCLK", NoResponse, timeout_ms = 500)]
pub struct SetSleepMode {
    #[at_arg(position = 1)]
    pub mode: u8, // 0 = disable sleep, 1 = DTR-controlled sleep
}

#[derive(Clone, Debug, AtatResp)]
pub struct AdcReading {
    #[at_arg(position = 1)]
    pub status: u8, // 1 = success, 0 = fail
    #[at_arg(position = 2)]
    pub voltage_mv: u16,
}

#[derive(Clone, AtatCmd)]
#[at_cmd("+QADC", AdcReading, timeout_ms = 500)]
pub struct ReadAdc {
    #[at_arg(position = 1)]
    pub port: u8, // 0 = ADC0, 1 = ADC1
}

#[derive(Clone, Debug, AtatResp)]
pub struct SignalQuality {
    /// 0..=31 => -113 + 2*rssi dBm; 99 => unknown/not detectable.
    #[at_arg(position = 1)]
    pub rssi: u8,
    #[at_arg(position = 2)]
    pub ber: u8,
}

impl SignalQuality {
    pub fn rssi_dbm(&self) -> Option<i16> {
        if self.rssi <= 31 {
            Some(-113 + 2 * (self.rssi as i16))
        } else {
            None
        }
    }
}

#[derive(Clone, AtatCmd)]
#[at_cmd("+CSQ", SignalQuality, timeout_ms = 500)]
pub struct GetSignalQuality;

#[derive(Clone, Debug, AtatResp)]
pub struct SimStatus {
    #[at_arg(position = 1)]
    pub code: String<16>,
}

#[derive(Clone, AtatCmd)]
#[at_cmd("+CPIN?", SimStatus, timeout_ms = 500)]
pub struct GetSimStatus;

#[derive(Clone, Debug, AtatResp)]
pub struct Iccid {
    #[at_arg(position = 1)]
    pub ccid: String<24>,
}

#[derive(Clone, AtatCmd)]
#[at_cmd("+QCCID", Iccid, timeout_ms = 500)]
pub struct GetIccid;

#[derive(Clone, Debug, AtatResp)]
pub struct BatteryCharge {
    /// Charge status: 0 = not charging, 1 = charging, 2 = charging finished.
    #[at_arg(position = 1)]
    pub bcs: u8,
    /// Battery charge level percentage (0..=100).
    #[at_arg(position = 2)]
    pub bcl: u8,
    /// Battery voltage in millivolts.
    #[at_arg(position = 3)]
    pub voltage_mv: u16,
}

#[derive(Clone, AtatCmd)]
#[at_cmd("+CBC", BatteryCharge, timeout_ms = 500)]
pub struct GetBatteryCharge;

#[derive(Clone, Debug)]
pub struct Imei {
    pub imei: String<20>,
}

impl atat::AtatResp for Imei {}

/// Queries the modem IMEI (`AT+GSN`), which returns bare digits without a `+GSN:` prefix.
#[derive(Clone)]
pub struct GetImei;

impl AtatCmd for GetImei {
    type Response = Imei;
    const MAX_TIMEOUT_MS: u32 = 500;
    const EXPECTS_RESPONSE_CODE: bool = true;

    fn write(&self, buf: &mut [u8]) -> usize {
        let cmd = b"AT+GSN\r";
        buf[..cmd.len()].copy_from_slice(cmd);
        cmd.len()
    }

    fn parse(
        &self,
        resp: Result<&[u8], atat::InternalError>,
    ) -> Result<Self::Response, atat::Error> {
        let raw = resp?;
        let text = core::str::from_utf8(raw)
            .map_err(|_| atat::Error::Parse)?
            .trim();
        let mut imei = String::<20>::new();
        imei.push_str(text).map_err(|_| atat::Error::Parse)?;
        Ok(Imei { imei })
    }
}

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

/// Routes unsolicited result codes (URCs) to a physical port (`AT+QURCCFG="urcport","uart1"`).
#[derive(Clone, AtatCmd)]
#[at_cmd("+QURCCFG", NoResponse, timeout_ms = 500)]
pub struct SetUrcPort {
    #[at_arg(position = 1)]
    pub param: String<12>,
    #[at_arg(position = 2)]
    pub port: String<12>,
}

/// Routes CMUX URCs to a specific CMUX virtual channel (`AT+QCFG="cmux/urcport",<dlci>`).
#[derive(Clone, AtatCmd)]
#[at_cmd("+QCFG", NoResponse, timeout_ms = 500)]
pub struct SetCmuxUrcPort {
    #[at_arg(position = 1)]
    pub param: String<16>,
    #[at_arg(position = 2)]
    pub dlci: u8,
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

#[derive(Clone, Debug, AtatResp)]
pub struct GpsState {
    #[at_arg(position = 1)]
    pub enabled: u8,
}

#[derive(Clone, AtatCmd)]
#[at_cmd("+QGPS?", GpsState, timeout_ms = 500)]
pub struct QueryGpsState;

#[derive(Clone, AtatCmd)]
#[at_cmd("+QGPS", NoResponse, timeout_ms = 300)]
pub struct SetGpsPower {
    #[at_arg(position = 1)]
    pub mode: u8, // 1 = Standalone ON
}

/// Cleanly ends an active GNSS session (`AT+QGPSEND`), saving ephemeris/position for Hot Start.
#[derive(Clone, AtatCmd)]
#[at_cmd("+QGPSEND", NoResponse, timeout_ms = 500)]
pub struct EndGpsSession;

/// Starts GNSS with explicit parameters: `AT+QGPS=<mode>,<fixmaxtime>,<fixmaxdist>,<fixcount>,<fixrate>`.
/// `fixcount = 0` means continuous positioning; `fixrate` is seconds between fixes (`1..=65535`).
#[derive(Clone, AtatCmd)]
#[at_cmd("+QGPS", NoResponse, timeout_ms = 500)]
pub struct SetGpsPowerEx {
    #[at_arg(position = 1)]
    pub mode: u8,
    #[at_arg(position = 2)]
    pub fixmaxtime: u16,
    #[at_arg(position = 3)]
    pub fixmaxdist: u16,
    #[at_arg(position = 4)]
    pub fixcount: u16,
    #[at_arg(position = 5)]
    pub fixrate: u16,
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

#[derive(Clone, AtatCmd)]
#[at_cmd("+QGPSXTRADATA", NoResponse, timeout_ms = 2000)]
pub struct InjectGpsXtraData {
    #[at_arg(position = 1)]
    pub path: String<24>,
}

#[derive(Clone, AtatCmd)]
#[at_cmd("+QGPSXTRATIME", NoResponse, timeout_ms = 1000)]
pub struct InjectGpsXtraTime {
    #[at_arg(position = 1)]
    pub op: u8,
    #[at_arg(position = 2)]
    pub utc: String<24>,
    #[at_arg(position = 3)]
    pub uncertainty: u8,
    #[at_arg(position = 4)]
    pub force: u8,
    #[at_arg(position = 5)]
    pub time_unc_ms: u16,
}

#[derive(Clone, Debug, AtatResp)]
pub struct NetworkTime {
    #[at_arg(position = 1)]
    pub utc: String<32>,
}

#[derive(Clone, AtatCmd)]
#[at_cmd("+QLTS", NetworkTime, timeout_ms = 500)]
pub struct GetNetworkTime {
    #[at_arg(position = 1)]
    pub mode: u8, // 1 = GMT/UTC
}

#[derive(Clone, AtatCmd)]
#[at_cmd("+QHTTPCFG", NoResponse, timeout_ms = 500)]
pub struct SetHttpCtx {
    #[at_arg(position = 1)]
    pub param: String<16>,
    #[at_arg(position = 2)]
    pub value: u8,
}

#[derive(Clone, AtatCmd)]
#[at_cmd("+QIACT", NoResponse, timeout_ms = 5000)]
pub struct ActivatePdp {
    #[at_arg(position = 1)]
    pub cid: u8,
}

#[derive(Clone, AtatCmd)]
#[at_cmd("+QIDEACT", NoResponse, timeout_ms = 5000)]
pub struct DeactivatePdp {
    #[at_arg(position = 1)]
    pub cid: u8,
}

#[derive(Clone, AtatCmd)]
#[at_cmd("+QHTTPGET", NoResponse, timeout_ms = 5000)]
pub struct HttpGet {
    #[at_arg(position = 1)]
    pub timeout_s: u16,
}

#[derive(Clone, AtatCmd)]
#[at_cmd("+QHTTPREADFILE", NoResponse, timeout_ms = 5000)]
pub struct HttpReadFile {
    #[at_arg(position = 1)]
    pub path: String<24>,
}

#[derive(Clone, AtatCmd)]
#[at_cmd("+QFDEL", NoResponse, timeout_ms = 2000)]
pub struct DeleteFile {
    #[at_arg(position = 1)]
    pub path: String<24>,
}

#[derive(Clone, AtatResp)]
pub struct NoResponse;

#[derive(Clone, AtatResp)]
pub struct Ready;

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

#[derive(Clone, AtatCmd)]
#[at_cmd("+CMGF", NoResponse, timeout_ms = 500)]
pub struct SetSmsTextMode {
    #[at_arg(position = 1)]
    pub mode: u8, // 1 = text mode
}

#[derive(Clone, AtatCmd)]
#[at_cmd("+CNMI", NoResponse, timeout_ms = 500)]
pub struct SetSmsIndication {
    #[at_arg(position = 1)]
    pub mode: u8,
    #[at_arg(position = 2)]
    pub mt: u8,
    #[at_arg(position = 3)]
    pub bm: u8,
    #[at_arg(position = 4)]
    pub ds: u8,
    #[at_arg(position = 5)]
    pub bfr: u8,
}

#[derive(Clone, AtatCmd)]
#[at_cmd("+CMGD", NoResponse, timeout_ms = 1000)]
pub struct DeleteSms {
    #[at_arg(position = 1)]
    pub index: u16,
}

/// Deletes messages by flag (`AT+CMGD=<index>,<delflag>`, e.g. `1,4` deletes all stored SMS).
#[derive(Clone, AtatCmd)]
#[at_cmd("+CMGD", NoResponse, timeout_ms = 5000)]
pub struct DeleteSmsWithFlag {
    #[at_arg(position = 1)]
    pub index: u16,
    #[at_arg(position = 2)]
    pub delflag: u8,
}

#[derive(Clone, Debug)]
pub struct OperatorResp {
    pub name: Option<String<32>>,
}

impl atat::AtatResp for OperatorResp {}

/// Queries the currently selected cellular operator (`AT+COPS?`).
#[derive(Clone)]
pub struct GetOperator;

impl AtatCmd for GetOperator {
    type Response = OperatorResp;
    const MAX_TIMEOUT_MS: u32 = 1000;
    const EXPECTS_RESPONSE_CODE: bool = true;

    fn write(&self, buf: &mut [u8]) -> usize {
        let cmd = b"AT+COPS?\r";
        buf[..cmd.len()].copy_from_slice(cmd);
        cmd.len()
    }

    fn parse(
        &self,
        resp: Result<&[u8], atat::InternalError>,
    ) -> Result<Self::Response, atat::Error> {
        let raw = resp?;
        let text = core::str::from_utf8(raw).map_err(|_| atat::Error::Parse)?;
        // Format when registered: +COPS: <mode>,<format>,"<oper>"[,<AcT>]
        // Format when unregistered: +COPS: <mode>
        let name = text.find('"').and_then(|start| {
            let rest = &text[start + 1..];
            let end = rest.find('"')?;
            let oper = &rest[..end];
            if oper.is_empty() {
                return None;
            }
            let mut s = String::<32>::new();
            for ch in oper.chars() {
                if s.push(ch).is_err() {
                    break;
                }
            }
            Some(s)
        });
        Ok(OperatorResp { name })
    }
}
