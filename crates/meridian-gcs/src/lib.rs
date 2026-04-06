// meridian-gcs: Web-based ground control station.
//
// Architecture: Rust WASM core + vanilla JS/HTML/CSS + Leaflet maps.
// Connects directly via Meridian native protocol over WebSocket.
// No install. No Java. No Windows dependency. Just open a browser.
//
// WASM core handles:
// - Protocol parsing (MNP frame decode, message dispatch)
// - State management (vehicle state, telemetry history, mission)
// - Map math (coordinate transforms, geofence checks)
//
// JS handles:
// - WebSocket connection management
// - Leaflet map rendering
// - DOM updates (telemetry panels, HUD)
// - User input (click-to-waypoint, parameter editing)
//
// Build: wasm-pack build --target web
// Serve: any static HTTP server (python -m http.server, nginx, etc.)

pub mod state;

pub use state::GcsState;
