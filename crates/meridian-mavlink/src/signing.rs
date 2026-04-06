//! MAVLink v2 signing — HMAC-SHA256 packet authentication.
//!
//! ArduPilot reference: `GCS_Signing.cpp`
//!
//! MAVLink v2 supports optional per-packet signing using SHA-256. When signing
//! is enabled:
//! 1. The incompatibility flag byte has bit 0 set (MAVLINK_IFLAG_SIGNED = 0x01)
//! 2. A 13-byte signature is appended after the CRC:
//!    - link_id: u8 (identifies the link)
//!    - timestamp: [u8; 6] (48-bit microsecond counter, monotonic)
//!    - signature: [u8; 6] (first 6 bytes of HMAC-SHA256)
//!
//! The HMAC key is a 32-byte secret shared between GCS and flight controller.

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// MAVLink incompatibility flag for signed packets.
pub const MAVLINK_IFLAG_SIGNED: u8 = 0x01;

/// Signature block size (link_id + timestamp + truncated HMAC).
pub const SIGNATURE_SIZE: usize = 13;

/// Secret key size (SHA-256 key).
pub const KEY_SIZE: usize = 32;

// ---------------------------------------------------------------------------
// Signing state
// ---------------------------------------------------------------------------

/// MAVLink signing configuration.
#[derive(Clone)]
pub struct SigningConfig {
    /// 32-byte secret key.
    pub key: [u8; KEY_SIZE],
    /// Link ID for this connection.
    pub link_id: u8,
    /// Whether signing is required for incoming packets.
    pub require_signing: bool,
    /// Whether to sign outgoing packets.
    pub sign_outgoing: bool,
    /// Monotonic timestamp counter (48-bit, microseconds).
    pub timestamp: u64,
}

impl Default for SigningConfig {
    fn default() -> Self {
        Self {
            key: [0u8; KEY_SIZE],
            link_id: 0,
            require_signing: false,
            sign_outgoing: false,
            timestamp: 0,
        }
    }
}

/// MAVLink signing state machine.
pub struct MavlinkSigning {
    /// Signing configuration.
    pub config: SigningConfig,
    /// Whether signing has been set up.
    pub enabled: bool,
}

impl MavlinkSigning {
    pub fn new() -> Self {
        Self {
            config: SigningConfig::default(),
            enabled: false,
        }
    }

    /// Set the signing key and enable signing.
    pub fn set_key(&mut self, key: &[u8; KEY_SIZE]) {
        self.config.key = *key;
        self.config.sign_outgoing = true;
        self.config.require_signing = true;
        self.enabled = true;
    }

    /// Generate a signature for an outgoing MAVLink v2 frame.
    ///
    /// `frame`: the complete MAVLink v2 frame (header + payload + CRC).
    /// `out`: 13-byte buffer for the signature block.
    ///
    /// Returns true on success.
    ///
    /// NOTE: This is a stub that generates a placeholder signature.
    /// A full implementation requires SHA-256 (provided by a crypto crate
    /// or hardware accelerator). The HMAC is computed over:
    ///   key(32) || frame_header(10) || payload(n) || CRC(2) || link_id(1) || timestamp(6)
    pub fn sign_frame(&mut self, frame: &[u8], out: &mut [u8; SIGNATURE_SIZE]) -> bool {
        if !self.config.sign_outgoing {
            return false;
        }

        // Increment timestamp
        self.config.timestamp += 1;

        // Link ID
        out[0] = self.config.link_id;

        // Timestamp (48-bit LE)
        let ts_bytes = self.config.timestamp.to_le_bytes();
        out[1..7].copy_from_slice(&ts_bytes[..6]);

        // Signature: first 6 bytes of HMAC-SHA256(key, frame || link_id || timestamp)
        // STUB: Use a simple XOR-based placeholder until SHA-256 is available.
        // This is NOT cryptographically secure — it's a structural placeholder.
        let mut sig = [0u8; 6];
        for (i, &byte) in frame.iter().take(48).enumerate() {
            sig[i % 6] ^= byte ^ self.config.key[i % KEY_SIZE];
        }
        out[7..13].copy_from_slice(&sig);

        true
    }

    /// Verify the signature on an incoming MAVLink v2 frame.
    ///
    /// `frame`: the MAVLink v2 frame (header + payload + CRC, without signature).
    /// `signature`: the 13-byte signature block from the wire.
    ///
    /// Returns true if the signature is valid.
    ///
    /// NOTE: Stub implementation — always returns true when signing is disabled.
    pub fn verify_signature(&self, _frame: &[u8], signature: &[u8; SIGNATURE_SIZE]) -> bool {
        if !self.config.require_signing {
            return true;
        }

        // Check timestamp is monotonically increasing
        let rx_ts = u64::from_le_bytes([
            signature[1], signature[2], signature[3],
            signature[4], signature[5], signature[6],
            0, 0,
        ]);

        // Reject if timestamp is in the past (replay attack protection)
        if rx_ts < self.config.timestamp.saturating_sub(60_000_000) {
            return false;
        }

        // STUB: Actual HMAC verification requires SHA-256
        true
    }

    /// Check if an incoming frame's incompatibility flags indicate it is signed.
    pub fn frame_is_signed(incompat_flags: u8) -> bool {
        incompat_flags & MAVLINK_IFLAG_SIGNED != 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_signing_disabled_by_default() {
        let signing = MavlinkSigning::new();
        assert!(!signing.enabled);
    }

    #[test]
    fn test_set_key_enables_signing() {
        let mut signing = MavlinkSigning::new();
        signing.set_key(&[0x42; KEY_SIZE]);
        assert!(signing.enabled);
        assert!(signing.config.sign_outgoing);
        assert!(signing.config.require_signing);
    }

    #[test]
    fn test_sign_frame_produces_output() {
        let mut signing = MavlinkSigning::new();
        signing.set_key(&[0xAB; KEY_SIZE]);
        let frame = [0xFD, 0x09, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00];
        let mut sig = [0u8; SIGNATURE_SIZE];
        assert!(signing.sign_frame(&frame, &mut sig));
        // Link ID should match
        assert_eq!(sig[0], 0);
        // Timestamp should be 1 (incremented from 0)
        assert_eq!(sig[1], 1);
    }

    #[test]
    fn test_frame_is_signed_detection() {
        assert!(MavlinkSigning::frame_is_signed(0x01));
        assert!(!MavlinkSigning::frame_is_signed(0x00));
    }
}
