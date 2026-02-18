//! Debug logging constants shared between kernel and partition code.
//!
//! These constants define the ABI for debug record headers and are used
//! by both the kernel (consumer) and partitions (producers).

// Log levels: ERROR=0 (highest) to TRACE=4 (lowest)
pub const LOG_ERROR: u8 = 0;
pub const LOG_WARN: u8 = 1;
pub const LOG_INFO: u8 = 2;
pub const LOG_DEBUG: u8 = 3;
pub const LOG_TRACE: u8 = 4;

// Record kinds
pub const KIND_TEXT: u8 = 0;
pub const KIND_DEFMT: u8 = 1;
pub const KIND_BINARY: u8 = 2;
pub const KIND_EVENT_ID: u8 = 3;
