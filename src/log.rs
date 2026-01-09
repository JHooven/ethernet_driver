#![allow(dead_code)]

#[cfg(feature = "defmt-logging")]
pub fn info(msg: &str) {
    defmt::info!("{=str}", msg);
}

#[cfg(not(feature = "defmt-logging"))]
pub fn info(msg: &str) {
    let _ = cortex_m_semihosting::hprintln!("{}", msg);
}

#[cfg(feature = "defmt-logging")]
pub fn warn(msg: &str) {
    defmt::warn!("{=str}", msg);
}

#[cfg(not(feature = "defmt-logging"))]
pub fn warn(msg: &str) {
    let _ = cortex_m_semihosting::hprintln!("{}", msg);
}
