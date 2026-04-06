//! AP_Relay equivalent: GPIO set/clear with pulse timer.
//!
//! Source: ArduPilot AP_Relay/AP_Relay.cpp
//!
//! Provides simple relay control for parachute servos, camera triggers,
//! gripper solenoids, and ICEngine ignition. Each relay is a GPIO pin
//! that can be set high, set low, toggled, or pulsed for a duration.
//!
//! Up to 6 relay channels (matching ArduPilot RELAY1_PIN through RELAY6_PIN).

/// Maximum number of relay channels.
pub const MAX_RELAYS: usize = 6;

/// Default pulse duration in milliseconds.
pub const DEFAULT_PULSE_MS: u32 = 500;

/// State of a single relay channel.
#[derive(Debug, Clone, Copy)]
pub struct RelayChannel {
    /// GPIO pin number. 0 = disabled.
    pub pin: u16,
    /// Whether the relay is currently energized (pin high).
    pub active: bool,
    /// Default state on boot (false = off, true = on).
    pub default_state: bool,
    /// Whether this channel is inverted (active = pin LOW).
    pub inverted: bool,
    /// Remaining pulse time in milliseconds. 0 = no pulse in progress.
    pulse_remaining_ms: u32,
}

impl RelayChannel {
    pub const fn disabled() -> Self {
        Self {
            pin: 0,
            active: false,
            default_state: false,
            inverted: false,
            pulse_remaining_ms: 0,
        }
    }

    /// Whether this channel has a valid pin configured.
    pub fn is_configured(&self) -> bool {
        self.pin > 0
    }

    /// Whether a pulse is currently in progress.
    pub fn is_pulsing(&self) -> bool {
        self.pulse_remaining_ms > 0
    }
}

/// AP_Relay: manages up to 6 GPIO relay channels.
///
/// Call `update()` at a regular interval (e.g., 10 Hz) to tick pulse timers.
/// After calling set/clear/toggle/pulse, the caller must apply the actual
/// GPIO writes using the `pending_writes()` iterator.
pub struct Relay {
    channels: [RelayChannel; MAX_RELAYS],
    /// Bitmask of channels that have pending GPIO writes.
    dirty: u8,
}

impl Relay {
    pub const fn new() -> Self {
        Self {
            channels: [RelayChannel::disabled(); MAX_RELAYS],
            dirty: 0,
        }
    }

    /// Configure a relay channel. `index` is 0-based (0..5).
    pub fn configure(&mut self, index: usize, pin: u16, inverted: bool, default_state: bool) {
        if index >= MAX_RELAYS {
            return;
        }
        self.channels[index] = RelayChannel {
            pin,
            active: default_state,
            default_state,
            inverted,
            pulse_remaining_ms: 0,
        };
        self.dirty |= 1 << index;
    }

    /// Set a relay channel ON (energized).
    pub fn set(&mut self, index: usize) {
        if index >= MAX_RELAYS || !self.channels[index].is_configured() {
            return;
        }
        self.channels[index].active = true;
        self.channels[index].pulse_remaining_ms = 0; // cancel any pulse
        self.dirty |= 1 << index;
    }

    /// Set a relay channel OFF (de-energized).
    pub fn clear(&mut self, index: usize) {
        if index >= MAX_RELAYS || !self.channels[index].is_configured() {
            return;
        }
        self.channels[index].active = false;
        self.channels[index].pulse_remaining_ms = 0;
        self.dirty |= 1 << index;
    }

    /// Toggle a relay channel.
    pub fn toggle(&mut self, index: usize) {
        if index >= MAX_RELAYS || !self.channels[index].is_configured() {
            return;
        }
        self.channels[index].active = !self.channels[index].active;
        self.channels[index].pulse_remaining_ms = 0;
        self.dirty |= 1 << index;
    }

    /// Pulse a relay ON for `duration_ms`, then automatically turn OFF.
    /// Used for camera trigger, parachute release, etc.
    pub fn pulse(&mut self, index: usize, duration_ms: u32) {
        if index >= MAX_RELAYS || !self.channels[index].is_configured() {
            return;
        }
        self.channels[index].active = true;
        self.channels[index].pulse_remaining_ms = duration_ms;
        self.dirty |= 1 << index;
    }

    /// Pulse with default duration.
    pub fn pulse_default(&mut self, index: usize) {
        self.pulse(index, DEFAULT_PULSE_MS);
    }

    /// Tick pulse timers. Call at a regular interval.
    /// `elapsed_ms`: milliseconds since last call.
    /// Returns true if any channel state changed (caller should apply GPIO).
    pub fn update(&mut self, elapsed_ms: u32) -> bool {
        let mut changed = false;
        for i in 0..MAX_RELAYS {
            if self.channels[i].pulse_remaining_ms > 0 {
                if self.channels[i].pulse_remaining_ms <= elapsed_ms {
                    // Pulse complete: turn off
                    self.channels[i].pulse_remaining_ms = 0;
                    self.channels[i].active = false;
                    self.dirty |= 1 << i;
                    changed = true;
                } else {
                    self.channels[i].pulse_remaining_ms -= elapsed_ms;
                }
            }
        }
        changed
    }

    /// Get the desired GPIO output value for a channel.
    /// Returns `(pin, value)` where value accounts for inversion.
    /// Returns `None` if the channel is not configured.
    pub fn gpio_state(&self, index: usize) -> Option<(u16, bool)> {
        if index >= MAX_RELAYS || !self.channels[index].is_configured() {
            return None;
        }
        let ch = &self.channels[index];
        let pin_value = if ch.inverted { !ch.active } else { ch.active };
        Some((ch.pin, pin_value))
    }

    /// Iterate over all channels that need GPIO writes applied.
    /// After processing, call `clear_dirty()`.
    pub fn pending_writes(&self) -> impl Iterator<Item = (usize, u16, bool)> + '_ {
        (0..MAX_RELAYS).filter_map(move |i| {
            if self.dirty & (1 << i) != 0 {
                self.gpio_state(i).map(|(pin, val)| (i, pin, val))
            } else {
                None
            }
        })
    }

    /// Clear the dirty bitmask after GPIO writes have been applied.
    pub fn clear_dirty(&mut self) {
        self.dirty = 0;
    }

    /// Get channel state (read-only).
    pub fn channel(&self, index: usize) -> Option<&RelayChannel> {
        if index < MAX_RELAYS {
            Some(&self.channels[index])
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_relay_set_clear() {
        let mut relay = Relay::new();
        relay.configure(0, 42, false, false);

        assert!(!relay.channels[0].active);
        relay.set(0);
        assert!(relay.channels[0].active);
        relay.clear(0);
        assert!(!relay.channels[0].active);
    }

    #[test]
    fn test_relay_toggle() {
        let mut relay = Relay::new();
        relay.configure(0, 10, false, false);

        relay.toggle(0);
        assert!(relay.channels[0].active);
        relay.toggle(0);
        assert!(!relay.channels[0].active);
    }

    #[test]
    fn test_relay_pulse() {
        let mut relay = Relay::new();
        relay.configure(0, 10, false, false);

        relay.pulse(0, 500);
        assert!(relay.channels[0].active);
        assert!(relay.channels[0].is_pulsing());

        // Tick 250ms — still pulsing
        relay.update(250);
        assert!(relay.channels[0].active);
        assert_eq!(relay.channels[0].pulse_remaining_ms, 250);

        // Tick another 300ms — pulse complete, should turn off
        let changed = relay.update(300);
        assert!(changed);
        assert!(!relay.channels[0].active);
        assert!(!relay.channels[0].is_pulsing());
    }

    #[test]
    fn test_relay_inverted() {
        let mut relay = Relay::new();
        relay.configure(0, 10, true, false); // inverted

        // Active=false, inverted=true → pin should be HIGH
        let (_, val) = relay.gpio_state(0).unwrap();
        assert!(val, "Inverted relay off should output high");

        relay.set(0);
        let (_, val) = relay.gpio_state(0).unwrap();
        assert!(!val, "Inverted relay on should output low");
    }

    #[test]
    fn test_relay_disabled_channel_ignored() {
        let mut relay = Relay::new();
        // Channel 0 not configured (pin=0)
        relay.set(0); // should be no-op
        assert!(!relay.channels[0].active);
        assert!(relay.gpio_state(0).is_none());
    }

    #[test]
    fn test_relay_pending_writes() {
        let mut relay = Relay::new();
        relay.configure(0, 10, false, false);
        relay.configure(1, 20, false, false);
        relay.clear_dirty();

        relay.set(0);
        let writes: heapless::Vec<(usize, u16, bool), 6> =
            relay.pending_writes().collect();
        assert_eq!(writes.len(), 1);
        assert_eq!(writes[0], (0, 10, true));
    }
}
