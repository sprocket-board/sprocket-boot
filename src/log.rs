#[cfg(not(feature = "defmt-log"))]
#[macro_export]
macro_rules! sprkt_log {
    ($level:ident, $($arg:expr),*) => { { $( let _ = $arg; )* } }
}

#[cfg(feature = "defmt-log")]
#[macro_export]
macro_rules! sprkt_log {
    (trace, $($arg:expr),*) => { defmt::trace!($($arg),*); };
    (debug, $($arg:expr),*) => { defmt::debug!($($arg),*); };
    (info, $($arg:expr),*) => { defmt::info!($($arg),*); };
    (warn, $($arg:expr),*) => { defmt::warn!($($arg),*); };
    (error, $($arg:expr),*) => { defmt::error!($($arg),*); };
}
