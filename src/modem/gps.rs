use atat::Error as AtatError;
use atat::asynch::AtatClient;
use defmt::{info, warn};

use super::commands::{GetGpsLocation, GpsLocation, QueryGpsXtraData, SetGpsPower, SetGpsXtra};

/// Enables gpsOneXTRA assistance (`AT+QGPSXTRA=1`) and logs current XTRA validity
/// (must be called while GNSS is OFF).
pub async fn configure_xtra<C: AtatClient>(client: &mut C) {
    if let Err(e) = client.send(&SetGpsXtra { mode: 1 }).await {
        warn!("AT+QGPSXTRA=1 failed: {:?}", e);
    }

    match client.send(&QueryGpsXtraData).await {
        Ok(xtra) => {
            info!(
                "EC21 XTRA data: valid_mins={}, time={}",
                xtra.valid_mins,
                xtra.injected_time.as_str()
            );
        }
        Err(e) => warn!("AT+QGPSXTRADATA? failed: {:?}", e),
    }
}

pub async fn enable<C: AtatClient>(client: &mut C) -> Result<(), AtatError> {
    client.send(&SetGpsPower { mode: 1 }).await?;
    info!("EC21 GNSS enabled");
    Ok(())
}

#[allow(dead_code)]
pub async fn disable<C: AtatClient>(client: &mut C) -> Result<(), AtatError> {
    client.send(&SetGpsPower { mode: 0 }).await?;
    info!("EC21 GNSS disabled");
    Ok(())
}

pub async fn get_location<C: AtatClient>(client: &mut C) -> Result<GpsLocation, AtatError> {
    client.send(&GetGpsLocation { mode: 2 }).await
}
