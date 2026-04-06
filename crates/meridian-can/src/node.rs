//! DroneCAN node management.
//!
//! Implements the flight controller's CAN node: broadcasting heartbeats,
//! sending commands, receiving sensor data, and running the Dynamic Node
//! Allocation (DNA) server for peripheral discovery.

use meridian_hal::can::{CanFrame, CanIface};
use crate::frames::{
    encode_frame, encode_multi_frame, decode_frame, encode_can_id,
    TailByte, TransferAssembler,
    PRIORITY_LOW, PRIORITY_HIGH,
};
use crate::messages::*;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Default node ID for the flight controller (matches ArduPilot).
pub const DEFAULT_NODE_ID: u8 = 10;

/// Maximum number of peripheral nodes we track.
pub const MAX_NODES: usize = 128;

/// Heartbeat interval in microseconds (1 Hz).
pub const HEARTBEAT_INTERVAL_US: u64 = 1_000_000;

/// Valid node ID range: 1-125 (126, 127 reserved by protocol).
pub const MIN_NODE_ID: u8 = 1;
pub const MAX_NODE_ID: u8 = 125;

// ---------------------------------------------------------------------------
// Tracked node state
// ---------------------------------------------------------------------------

/// Status of a peripheral node on the bus.
#[derive(Debug, Clone, Copy)]
pub struct PeripheralNode {
    /// Node ID (1-125).
    pub node_id: u8,
    /// Last known health.
    pub health: NodeHealth,
    /// Last known mode.
    pub mode: NodeMode,
    /// Uptime from last NodeStatus.
    pub uptime_sec: u32,
    /// Timestamp (uptime us) when we last heard from this node.
    pub last_seen_us: u64,
    /// Whether GetNodeInfo has been requested.
    pub info_requested: bool,
    /// Whether this node is considered active.
    pub active: bool,
}

impl PeripheralNode {
    const fn empty() -> Self {
        Self {
            node_id: 0,
            health: NodeHealth::Ok,
            mode: NodeMode::Offline,
            uptime_sec: 0,
            last_seen_us: 0,
            info_requested: false,
            active: false,
        }
    }
}

// ---------------------------------------------------------------------------
// DNA Server state
// ---------------------------------------------------------------------------

/// Dynamic Node Allocation server entry.
#[derive(Debug, Clone, Copy)]
pub struct DnaEntry {
    /// 16-byte hardware unique ID (or first 6 bytes used as hash).
    pub unique_id: [u8; 16],
    /// Assigned node ID.
    pub node_id: u8,
    /// Whether this entry is occupied.
    pub occupied: bool,
}

impl DnaEntry {
    const fn empty() -> Self {
        Self {
            unique_id: [0u8; 16],
            node_id: 0,
            occupied: false,
        }
    }
}

/// DNA server — allocates node IDs to peripherals.
pub struct DnaServer {
    entries: [DnaEntry; MAX_NODE_ID as usize],
    next_free_id: u8,
}

impl DnaServer {
    pub const fn new() -> Self {
        Self {
            entries: [DnaEntry::empty(); MAX_NODE_ID as usize],
            next_free_id: MAX_NODE_ID, // allocate downward from 125, matching ArduPilot
        }
    }

    /// Look up a unique ID. Returns the assigned node ID if found.
    pub fn lookup(&self, unique_id: &[u8; 16]) -> Option<u8> {
        for entry in &self.entries {
            if entry.occupied && entry.unique_id == *unique_id {
                return Some(entry.node_id);
            }
        }
        None
    }

    /// Allocate a node ID for a unique ID. Returns the assigned ID.
    /// If already assigned, returns the existing ID.
    /// Allocates downward from MAX_NODE_ID (125) to match ArduPilot's DNA server.
    pub fn allocate(&mut self, unique_id: &[u8; 16]) -> Option<u8> {
        // Check if already assigned
        if let Some(id) = self.lookup(unique_id) {
            return Some(id);
        }

        // Find the next free ID, searching downward from next_free_id
        let start = self.next_free_id;
        let mut id = start;
        loop {
            if id < MIN_NODE_ID {
                id = MAX_NODE_ID;
            }
            let idx = (id - 1) as usize;
            if idx < self.entries.len() && !self.entries[idx].occupied && id != DEFAULT_NODE_ID {
                self.entries[idx] = DnaEntry {
                    unique_id: *unique_id,
                    node_id: id,
                    occupied: true,
                };
                self.next_free_id = if id <= MIN_NODE_ID { MAX_NODE_ID } else { id - 1 };
                return Some(id);
            }
            if id <= MIN_NODE_ID {
                id = MAX_NODE_ID;
            } else {
                id -= 1;
            }
            if id == start {
                break; // wrapped around, no free IDs
            }
        }
        None // bus full
    }

    /// Clear all allocations.
    pub fn clear(&mut self) {
        self.entries = [DnaEntry::empty(); MAX_NODE_ID as usize];
        self.next_free_id = MAX_NODE_ID;
    }

    /// Number of occupied entries.
    pub fn occupied_count(&self) -> usize {
        self.entries.iter().filter(|e| e.occupied).count()
    }

    /// Serialize the DNA table to a byte buffer for flash persistence.
    ///
    /// Format: `[count: u8]` then for each occupied entry:
    ///   `[node_id: u8][unique_id: 16 bytes]` = 17 bytes per entry.
    ///
    /// Returns the number of bytes written, or `None` if the buffer is too small.
    ///
    /// ArduPilot persists DNA mappings to flash so that peripherals get the
    /// same node IDs across reboots (parameter bindings reference node IDs).
    pub fn save_to_bytes(&self, buf: &mut [u8]) -> Option<usize> {
        let count = self.occupied_count();
        let needed = 1 + count * 17;
        if buf.len() < needed {
            return None;
        }
        buf[0] = count as u8;
        let mut offset = 1;
        for entry in &self.entries {
            if entry.occupied {
                buf[offset] = entry.node_id;
                buf[offset + 1..offset + 17].copy_from_slice(&entry.unique_id);
                offset += 17;
            }
        }
        Some(offset)
    }

    /// Restore the DNA table from a previously saved byte buffer.
    ///
    /// Returns `true` if restoration succeeded.
    pub fn restore_from_bytes(&mut self, buf: &[u8]) -> bool {
        if buf.is_empty() {
            return false;
        }
        let count = buf[0] as usize;
        let needed = 1 + count * 17;
        if buf.len() < needed {
            return false;
        }
        // Clear existing state first
        self.entries = [DnaEntry::empty(); MAX_NODE_ID as usize];
        self.next_free_id = MAX_NODE_ID;

        let mut offset = 1;
        let mut min_id = self.next_free_id;
        for _ in 0..count {
            let node_id = buf[offset];
            let mut unique_id = [0u8; 16];
            unique_id.copy_from_slice(&buf[offset + 1..offset + 17]);
            offset += 17;

            if node_id >= MIN_NODE_ID && node_id <= MAX_NODE_ID && node_id != DEFAULT_NODE_ID {
                let idx = (node_id - 1) as usize;
                if idx < self.entries.len() {
                    self.entries[idx] = DnaEntry {
                        unique_id,
                        node_id,
                        occupied: true,
                    };
                    if node_id <= min_id {
                        min_id = if node_id <= MIN_NODE_ID { MAX_NODE_ID } else { node_id - 1 };
                    }
                }
            }
        }
        self.next_free_id = min_id;
        true
    }
}

// ---------------------------------------------------------------------------
// DNA 3-round handshake FSM (S1)
// ---------------------------------------------------------------------------

/// S1: DNA 3-round anonymous handshake FSM.
///
/// UAVCAN v0 Dynamic Node Allocation uses a 3-round exchange:
///   Round 1: Client sends first 6 bytes of unique_id (preferred_node_id=0)
///   Round 2: Client sends bytes 6-11 of unique_id
///   Round 3: Client sends bytes 12-15 of unique_id
///
/// After each round, the server echoes the received bytes. After round 3,
/// the server assigns a node_id. The client confirms by starting to use it.
///
/// Source: UAVCAN v0 specification, section 5.6
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DnaHandshakePhase {
    /// Idle — no handshake in progress.
    Idle,
    /// Waiting for round 1 response (echoed first 6 bytes).
    WaitRound1,
    /// Waiting for round 2 response (echoed bytes 6-11).
    WaitRound2,
    /// Waiting for round 3 response (echoed bytes 12-15 + node_id).
    WaitRound3,
    /// Allocation complete — node_id assigned.
    Complete,
    /// Allocation failed after max retries.
    Failed,
}

/// State for one in-progress DNA handshake (server side).
#[derive(Debug, Clone)]
pub struct DnaHandshake {
    /// Current phase of the handshake.
    pub phase: DnaHandshakePhase,
    /// Accumulated unique_id bytes from the client (filled across 3 rounds).
    pub unique_id: [u8; 16],
    /// How many bytes of unique_id have been received so far.
    pub bytes_received: u8,
    /// The allocated node_id (valid only in Complete phase).
    pub allocated_id: u8,
    /// Timestamp (microseconds) of the last received round. For timeout.
    pub last_rx_us: u64,
    /// Number of retries attempted.
    pub retries: u8,
    /// Maximum retries before failing.
    pub max_retries: u8,
    /// Timeout per round in microseconds (500 ms).
    pub round_timeout_us: u64,
}

impl DnaHandshake {
    pub const fn new() -> Self {
        Self {
            phase: DnaHandshakePhase::Idle,
            unique_id: [0u8; 16],
            bytes_received: 0,
            allocated_id: 0,
            last_rx_us: 0,
            retries: 0,
            max_retries: 5,
            round_timeout_us: 500_000,
        }
    }

    /// Start a new handshake (server received round 1 from an anonymous client).
    pub fn start_round1(&mut self, first_bytes: &[u8], now_us: u64) {
        self.phase = DnaHandshakePhase::WaitRound1;
        self.unique_id = [0u8; 16];
        let len = first_bytes.len().min(6);
        self.unique_id[..len].copy_from_slice(&first_bytes[..len]);
        self.bytes_received = len as u8;
        self.last_rx_us = now_us;
        self.retries = 0;
    }

    /// Process round 2 data (bytes 6-11).
    pub fn receive_round2(&mut self, data: &[u8], now_us: u64) -> bool {
        if self.phase != DnaHandshakePhase::WaitRound1 {
            return false;
        }
        let start = self.bytes_received as usize;
        let len = data.len().min(6).min(16 - start);
        self.unique_id[start..start + len].copy_from_slice(&data[..len]);
        self.bytes_received += len as u8;
        self.phase = DnaHandshakePhase::WaitRound2;
        self.last_rx_us = now_us;
        true
    }

    /// Process round 3 data (bytes 12-15) and allocate.
    pub fn receive_round3(&mut self, data: &[u8], dna: &mut DnaServer, now_us: u64) -> bool {
        if self.phase != DnaHandshakePhase::WaitRound2 {
            return false;
        }
        let start = self.bytes_received as usize;
        let len = data.len().min(4).min(16 - start);
        self.unique_id[start..start + len].copy_from_slice(&data[..len]);
        self.bytes_received += len as u8;
        self.last_rx_us = now_us;

        // All 16 bytes received — allocate
        if let Some(id) = dna.allocate(&self.unique_id) {
            self.allocated_id = id;
            self.phase = DnaHandshakePhase::Complete;
            true
        } else {
            self.phase = DnaHandshakePhase::Failed;
            false
        }
    }

    /// Check for timeout. Call periodically.
    /// Returns true if the handshake timed out and was reset.
    pub fn check_timeout(&mut self, now_us: u64) -> bool {
        if self.phase == DnaHandshakePhase::Idle
            || self.phase == DnaHandshakePhase::Complete
            || self.phase == DnaHandshakePhase::Failed
        {
            return false;
        }
        if now_us.wrapping_sub(self.last_rx_us) > self.round_timeout_us {
            self.retries += 1;
            if self.retries >= self.max_retries {
                self.phase = DnaHandshakePhase::Failed;
            } else {
                // Reset to Idle for retry
                self.phase = DnaHandshakePhase::Idle;
                self.bytes_received = 0;
            }
            return true;
        }
        false
    }

    /// Reset the handshake to idle.
    pub fn reset(&mut self) {
        *self = Self::new();
    }
}

// ---------------------------------------------------------------------------
// CanardNode — our node on the bus
// ---------------------------------------------------------------------------

/// Callback type for received DroneCAN messages.
pub type MessageCallback = fn(type_id: u16, source_node: u8, payload: &[u8]);

/// The flight controller's DroneCAN node.
///
/// Manages:
/// - Heartbeat broadcasting at 1 Hz
/// - Message transmission (broadcast + service requests)
/// - Incoming message dispatch
/// - Dynamic Node Allocation server
/// - Peripheral node tracking
pub struct CanardNode {
    /// Our node ID.
    pub node_id: u8,
    /// Node uptime in seconds (incremented by heartbeat).
    pub uptime_sec: u32,
    /// Current health status.
    pub health: NodeHealth,
    /// Current operating mode.
    pub mode: NodeMode,
    /// Transfer ID counter per data type (wraps at 32).
    transfer_ids: [u8; 64],
    /// Tracked peripheral nodes.
    pub peripherals: [PeripheralNode; MAX_NODES],
    /// DNA server.
    pub dna: DnaServer,
    /// Timestamp of last heartbeat send. `None` = never sent (forces immediate first send).
    last_heartbeat_us: Option<u64>,
    /// Transfer assemblers for multi-frame reception, keyed by source node.
    /// We support up to 8 simultaneous multi-frame reassemblies.
    assemblers: [TransferAssemblerSlot; 8],
}

/// A multi-frame assembler slot bound to a source node.
struct TransferAssemblerSlot {
    source_node: u8,
    type_id: u16,
    assembler: TransferAssembler,
    active: bool,
}

impl TransferAssemblerSlot {
    const fn empty() -> Self {
        Self {
            source_node: 0,
            type_id: 0,
            assembler: TransferAssembler::new(),
            active: false,
        }
    }
}

impl CanardNode {
    /// Create a new CAN node with the given ID.
    pub fn new(node_id: u8) -> Self {
        Self {
            node_id,
            uptime_sec: 0,
            health: NodeHealth::Ok,
            mode: NodeMode::Initialization,
            transfer_ids: [0u8; 64],
            peripherals: [PeripheralNode::empty(); MAX_NODES],
            dna: DnaServer::new(),
            last_heartbeat_us: None,
            assemblers: [
                TransferAssemblerSlot::empty(),
                TransferAssemblerSlot::empty(),
                TransferAssemblerSlot::empty(),
                TransferAssemblerSlot::empty(),
                TransferAssemblerSlot::empty(),
                TransferAssemblerSlot::empty(),
                TransferAssemblerSlot::empty(),
                TransferAssemblerSlot::empty(),
            ],
        }
    }

    /// Create a node with the default FC node ID (10).
    pub fn default_fc() -> Self {
        Self::new(DEFAULT_NODE_ID)
    }

    /// Get the next transfer ID for a data type, advancing the counter.
    fn next_transfer_id(&mut self, type_id: u16) -> u8 {
        let idx = (type_id as usize) % self.transfer_ids.len();
        let tid = self.transfer_ids[idx];
        self.transfer_ids[idx] = (tid + 1) & 0x1F;
        tid
    }

    // ─── Transmit ──────────────────────────────────────────────────────

    /// Broadcast a message on the CAN bus.
    ///
    /// For payloads <= 7 bytes, sends a single frame.
    /// For larger payloads, sends a multi-frame transfer.
    pub fn broadcast(
        &mut self,
        iface: &mut dyn CanIface,
        priority: u8,
        type_id: u16,
        payload: &[u8],
    ) -> bool {
        let tid = self.next_transfer_id(type_id);

        if payload.len() <= 7 {
            let frame = encode_frame(priority, type_id, self.node_id, payload, tid);
            iface.send(&frame, 1000)
        } else {
            let mut frames = [CanFrame::new_extended(0, &[]); 16];
            let n = encode_multi_frame(
                priority, type_id, self.node_id, payload, tid, &mut frames,
            );
            let mut ok = true;
            for i in 0..n {
                if !iface.send(&frames[i], 1000) {
                    ok = false;
                    break;
                }
            }
            ok
        }
    }

    /// Send a service request to a specific node.
    ///
    /// Service requests use the service_not_message bit in the CAN ID.
    pub fn request(
        &mut self,
        iface: &mut dyn CanIface,
        dest_node: u8,
        type_id: u16,
        payload: &[u8],
    ) -> bool {
        let tid = self.next_transfer_id(type_id);
        // Service frame: service_not_message=1, destination encoded in type_id field
        // UAVCAN v0 service ID encoding in the 16-bit field:
        //   bit 15: request_not_response (1=request)
        //   bits 14..8: destination node ID (7 bits)
        //   bits 7..0: service type ID (8 bits)
        let service_id = (1u16 << 15) | ((dest_node as u16 & 0x7F) << 8) | (type_id & 0xFF);
        let can_id = encode_can_id(PRIORITY_HIGH, service_id, true, self.node_id);

        let tail = TailByte::single_frame(tid);
        let mut data = [0u8; 64];
        let len = payload.len().min(7);
        data[..len].copy_from_slice(&payload[..len]);
        data[len] = tail.0;

        let frame = CanFrame {
            id: can_id,
            extended: true,
            data,
            dlc: (len + 1) as u8,
        };
        iface.send(&frame, 1000)
    }

    /// Send heartbeat (NodeStatus) if the interval has elapsed.
    ///
    /// Call this from the main loop. Returns true if a heartbeat was sent.
    pub fn maybe_send_heartbeat(
        &mut self,
        iface: &mut dyn CanIface,
        now_us: u64,
    ) -> bool {
        if let Some(last) = self.last_heartbeat_us {
            if now_us.wrapping_sub(last) < HEARTBEAT_INTERVAL_US {
                return false;
            }
        }

        self.last_heartbeat_us = Some(now_us);
        self.uptime_sec = (now_us / 1_000_000) as u32;

        let status = NodeStatus {
            uptime_sec: self.uptime_sec,
            health: self.health,
            mode: self.mode,
            sub_mode: 0,
            vendor_status: 0,
        };

        let mut buf = [0u8; 7];
        status.encode(&mut buf);
        self.broadcast(iface, PRIORITY_LOW, NODESTATUS_DTID, &buf)
    }

    /// Send ESC RawCommand (high priority).
    pub fn send_esc_command(
        &mut self,
        iface: &mut dyn CanIface,
        commands: &[i16],
    ) -> bool {
        let mut cmd = RawCommand {
            commands: [0i16; 8],
            num_commands: commands.len().min(8) as u8,
        };
        for (i, &c) in commands.iter().take(8).enumerate() {
            cmd.commands[i] = c;
        }

        let mut buf = [0u8; 16];
        let len = cmd.encode(&mut buf);
        // ESC commands use highest priority (PRIORITY_HIGH - 1)
        let priority = PRIORITY_HIGH.saturating_sub(1);
        self.broadcast(iface, priority, RAW_COMMAND_DTID, &buf[..len])
    }

    /// Send safety state.
    pub fn send_safety_state(
        &mut self,
        iface: &mut dyn CanIface,
        safe: bool,
    ) -> bool {
        let state = SafetyState {
            status: if safe { SafetyStatus::SafetyOn } else { SafetyStatus::SafetyOff },
        };
        let mut buf = [0u8; 1];
        state.encode(&mut buf);
        self.broadcast(iface, PRIORITY_LOW, SAFETY_STATE_DTID, &buf)
    }

    /// Send arming status.
    pub fn send_arming_status(
        &mut self,
        iface: &mut dyn CanIface,
        armed: bool,
    ) -> bool {
        let status = ArmingStatus {
            status: if armed { ArmingState::Armed } else { ArmingState::Disarmed },
        };
        let mut buf = [0u8; 1];
        status.encode(&mut buf);
        self.broadcast(iface, PRIORITY_LOW, ARMING_STATUS_DTID, &buf)
    }

    /// Send lights command.
    pub fn send_lights(
        &mut self,
        iface: &mut dyn CanIface,
        lights: &LightsCommand,
    ) -> bool {
        let mut buf = [0u8; 16];
        let len = lights.encode(&mut buf);
        self.broadcast(iface, PRIORITY_LOW, LIGHTS_COMMAND_DTID, &buf[..len])
    }

    // ─── Receive ───────────────────────────────────────────────────────

    /// Process all available CAN frames from the interface.
    ///
    /// Dispatches decoded messages to the provided callback. For multi-frame
    /// transfers, reassembles them before dispatch.
    pub fn process_rx(
        &mut self,
        iface: &mut dyn CanIface,
        now_us: u64,
        callback: MessageCallback,
    ) {
        while let Some(frame) = iface.receive() {
            if !frame.extended {
                continue; // DroneCAN uses only extended frames
            }

            let (priority, type_id, source_node, payload, tail) = decode_frame(&frame);
            let _ = priority; // used for filtering if needed

            // Single-frame transfer
            if tail.start_of_transfer() && tail.end_of_transfer() {
                self.handle_message(type_id, source_node, payload, now_us, callback);
                continue;
            }

            // Multi-frame transfer — find or create assembler slot
            self.handle_multi_frame(type_id, source_node, payload, tail, now_us, callback);
        }
    }

    /// Handle a complete single-frame or reassembled message.
    fn handle_message(
        &mut self,
        type_id: u16,
        source_node: u8,
        payload: &[u8],
        now_us: u64,
        callback: MessageCallback,
    ) {
        // Update peripheral tracking for NodeStatus
        if type_id == NODESTATUS_DTID {
            if let Some(status) = NodeStatus::decode(payload) {
                self.update_peripheral(source_node, &status, now_us);
            }
        }

        // Dispatch to user callback
        callback(type_id, source_node, payload);
    }

    /// Handle a frame that is part of a multi-frame transfer.
    fn handle_multi_frame(
        &mut self,
        type_id: u16,
        source_node: u8,
        payload: &[u8],
        tail: TailByte,
        now_us: u64,
        callback: MessageCallback,
    ) {
        // Find existing assembler for this source+type
        let mut slot_idx = None;
        for (i, slot) in self.assemblers.iter().enumerate() {
            if slot.active && slot.source_node == source_node && slot.type_id == type_id {
                slot_idx = Some(i);
                break;
            }
        }

        // If start of transfer, grab a free or expired slot
        if tail.start_of_transfer() && slot_idx.is_none() {
            for (i, slot) in self.assemblers.iter().enumerate() {
                if !slot.active {
                    slot_idx = Some(i);
                    break;
                }
            }
            // If no free slot, reuse the oldest (slot 0 as fallback)
            if slot_idx.is_none() {
                slot_idx = Some(0);
            }
            if let Some(idx) = slot_idx {
                self.assemblers[idx].source_node = source_node;
                self.assemblers[idx].type_id = type_id;
                self.assemblers[idx].assembler.reset();
                self.assemblers[idx].active = true;
            }
        }

        if let Some(idx) = slot_idx {
            if let Some(_len) = self.assemblers[idx].assembler.push_frame(payload, tail) {
                let assembled_payload = self.assemblers[idx].assembler.payload();
                // Copy out before calling handle_message (borrow issue).
                // Use a stack buffer matching the assembler's capacity.
                // For very large transfers (>2048), only process the first 2048
                // bytes on the stack to avoid stack overflow; the full payload
                // is still validated by the assembler's CRC.
                let mut tmp = [0u8; 2048];
                let plen = assembled_payload.len().min(tmp.len());
                tmp[..plen].copy_from_slice(&assembled_payload[..plen]);
                self.assemblers[idx].active = false;
                self.handle_message(type_id, source_node, &tmp[..plen], now_us, callback);
            }
        }
    }

    /// Update the tracked state of a peripheral node.
    fn update_peripheral(&mut self, node_id: u8, status: &NodeStatus, now_us: u64) {
        let idx = node_id as usize;
        if idx >= MAX_NODES {
            return;
        }
        self.peripherals[idx] = PeripheralNode {
            node_id,
            health: status.health,
            mode: status.mode,
            uptime_sec: status.uptime_sec,
            last_seen_us: now_us,
            info_requested: self.peripherals[idx].info_requested,
            active: true,
        };
    }

    /// Get all active peripheral nodes.
    pub fn active_peripherals(&self) -> impl Iterator<Item = &PeripheralNode> {
        self.peripherals.iter().filter(|p| p.active)
    }

    /// Request GetNodeInfo from a node.
    pub fn request_node_info(
        &mut self,
        iface: &mut dyn CanIface,
        target_node: u8,
    ) -> bool {
        let idx = target_node as usize;
        if idx < MAX_NODES {
            self.peripherals[idx].info_requested = true;
        }
        self.request(iface, target_node, GETNODEINFO_DTID, &[])
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Minimal mock CAN interface for testing.
    struct MockCanIface {
        tx_frames: heapless::Vec<CanFrame, 32>,
        rx_frames: heapless::Vec<CanFrame, 32>,
    }

    impl MockCanIface {
        fn new() -> Self {
            Self {
                tx_frames: heapless::Vec::new(),
                rx_frames: heapless::Vec::new(),
            }
        }

        fn push_rx(&mut self, frame: CanFrame) {
            let _ = self.rx_frames.push(frame);
        }
    }

    impl CanIface for MockCanIface {
        fn init(&mut self, _bitrate: u32) -> bool { true }
        fn send(&mut self, frame: &CanFrame, _timeout_us: u32) -> bool {
            self.tx_frames.push(*frame).is_ok()
        }
        fn receive(&mut self) -> Option<CanFrame> {
            if self.rx_frames.is_empty() {
                None
            } else {
                Some(self.rx_frames.remove(0))
            }
        }
        fn available(&self) -> u16 { self.rx_frames.len() as u16 }
        fn is_healthy(&self) -> bool { true }
        fn get_error_counts(&self) -> (u16, u16) { (0, 0) }
        fn set_filter(&mut self, _id: u32, _mask: u32, _extended: bool) -> bool { true }
        fn set_fd_mode(&mut self, _enable: bool) {}
        fn bus_index(&self) -> u8 { 0 }
    }

    #[test]
    fn test_heartbeat_sends_at_interval() {
        let mut node = CanardNode::default_fc();
        let mut iface = MockCanIface::new();

        // First call should send
        assert!(node.maybe_send_heartbeat(&mut iface, 0));
        assert_eq!(iface.tx_frames.len(), 1);

        // Second call too soon should not send
        assert!(!node.maybe_send_heartbeat(&mut iface, 500_000));
        assert_eq!(iface.tx_frames.len(), 1);

        // After 1 second should send again
        assert!(node.maybe_send_heartbeat(&mut iface, 1_000_001));
        assert_eq!(iface.tx_frames.len(), 2);
    }

    #[test]
    fn test_heartbeat_content() {
        let mut node = CanardNode::new(10);
        node.health = NodeHealth::Ok;
        node.mode = NodeMode::Operational;
        let mut iface = MockCanIface::new();

        node.maybe_send_heartbeat(&mut iface, 5_000_000);

        let frame = &iface.tx_frames[0];
        assert!(frame.extended);
        let (_p, type_id, source, _payload, _tail) = decode_frame(frame);
        assert_eq!(type_id, NODESTATUS_DTID);
        assert_eq!(source, 10);
    }

    #[test]
    fn test_esc_command_sends() {
        let mut node = CanardNode::default_fc();
        let mut iface = MockCanIface::new();

        let commands = [0i16, 4000, 8191, -8192];
        assert!(node.send_esc_command(&mut iface, &commands));
        assert_eq!(iface.tx_frames.len(), 1);

        let frame = &iface.tx_frames[0];
        let (_p, type_id, source, _payload, _tail) = decode_frame(frame);
        assert_eq!(type_id, RAW_COMMAND_DTID);
        assert_eq!(source, DEFAULT_NODE_ID);
    }

    #[test]
    fn test_dna_allocation() {
        let mut dna = DnaServer::new();
        let uid1 = [1u8; 16];
        let uid2 = [2u8; 16];

        let id1 = dna.allocate(&uid1).unwrap();
        assert!(id1 >= MIN_NODE_ID && id1 <= MAX_NODE_ID);
        assert_ne!(id1, DEFAULT_NODE_ID);

        let id2 = dna.allocate(&uid2).unwrap();
        assert_ne!(id1, id2);

        // Re-allocating same UID returns same ID
        let id1_again = dna.allocate(&uid1).unwrap();
        assert_eq!(id1, id1_again);
    }

    #[test]
    fn test_dna_lookup() {
        let mut dna = DnaServer::new();
        let uid = [42u8; 16];

        assert!(dna.lookup(&uid).is_none());

        let id = dna.allocate(&uid).unwrap();
        assert_eq!(dna.lookup(&uid), Some(id));
    }

    #[test]
    fn test_dna_clear() {
        let mut dna = DnaServer::new();
        let uid = [7u8; 16];
        dna.allocate(&uid).unwrap();
        dna.clear();
        assert!(dna.lookup(&uid).is_none());
    }

    #[test]
    fn test_peripheral_tracking() {
        let mut node = CanardNode::default_fc();
        let mut iface = MockCanIface::new();

        // Simulate receiving a NodeStatus from node 20
        let status = NodeStatus {
            uptime_sec: 100,
            health: NodeHealth::Ok,
            mode: NodeMode::Operational,
            sub_mode: 0,
            vendor_status: 0,
        };
        let mut buf = [0u8; 7];
        status.encode(&mut buf);
        let frame = encode_frame(PRIORITY_LOW, NODESTATUS_DTID, 20, &buf, 0);
        iface.push_rx(frame);

        use core::sync::atomic::{AtomicU32, Ordering};
        static MSG_COUNT: AtomicU32 = AtomicU32::new(0);
        MSG_COUNT.store(0, Ordering::SeqCst);
        fn test_cb(_type_id: u16, _source: u8, _payload: &[u8]) {
            MSG_COUNT.fetch_add(1, Ordering::SeqCst);
        }

        node.process_rx(&mut iface, 1_000_000, test_cb);

        assert!(node.peripherals[20].active);
        assert_eq!(node.peripherals[20].uptime_sec, 100);
        assert_eq!(node.peripherals[20].health, NodeHealth::Ok);
        assert_eq!(MSG_COUNT.load(Ordering::SeqCst), 1);
    }
}
