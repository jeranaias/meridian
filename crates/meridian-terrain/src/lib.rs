#![no_std]

//! Terrain database: SRTM elevation grids with LRU cache, bilinear interpolation,
//! and MAVLink terrain request/response protocol.
//!
//! Mirrors ArduPilot's AP_Terrain: 32x28 grid blocks, 12-block LRU cache (~22 KB RAM),
//! bilinear height lookup, and TERRAIN_REQUEST / TERRAIN_DATA message types.

/// Grid block dimensions: 32 longitude x 28 latitude samples.
/// An 8x7 arrangement of 4x4 MAVLink sub-grids.
pub const GRID_COLS: usize = 32;
pub const GRID_ROWS: usize = 28;

/// Grid spacing in degrees (1 arc-second ~ 30 m at equator).
pub const GRID_SPACING_DEG: f64 = 1.0 / 3600.0;

/// Number of columns in a MAVLink 4x4 sub-grid.
pub const SUBGRID_SIZE: usize = 4;

/// Number of 4x4 sub-grids per row: 32 / 4 = 8.
pub const SUBGRIDS_PER_ROW: usize = GRID_COLS / SUBGRID_SIZE;

/// Number of 4x4 sub-grid rows: 28 / 4 = 7.
pub const SUBGRIDS_PER_COL: usize = GRID_ROWS / SUBGRID_SIZE;

/// Total number of sub-grids in one grid block: 8 x 7 = 56.
pub const SUBGRIDS_TOTAL: usize = SUBGRIDS_PER_ROW * SUBGRIDS_PER_COL;

/// Invalid height sentinel (ArduPilot uses 0 with a separate valid flag; we use i16::MIN).
pub const INVALID_HEIGHT: i16 = i16::MIN;

/// Maximum number of cached grid blocks (matches AP_Terrain TERRAIN_CACHE_SZ default).
pub const MAX_CACHE_BLOCKS: usize = 12;

/// Maximum number of pending terrain requests queued at once.
pub const MAX_PENDING_REQUESTS: usize = 4;

// ---------------------------------------------------------------------------
// MAVLink protocol types
// ---------------------------------------------------------------------------

/// Terrain request sent to GCS (maps to TERRAIN_REQUEST MAVLink message).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TerrainRequest {
    /// Latitude in degrees * 1e7.
    pub lat: i32,
    /// Longitude in degrees * 1e7.
    pub lon: i32,
    /// Terrain grid spacing in meters.
    pub grid_spacing: u16,
    /// Bitmask of requested 4x4 sub-grids (up to 56 bits used).
    pub mask: u64,
}

/// Terrain data received from GCS (maps to TERRAIN_DATA MAVLink message).
#[derive(Debug, Clone, Copy)]
pub struct TerrainData {
    /// Latitude in degrees * 1e7.
    pub lat: i32,
    /// Longitude in degrees * 1e7.
    pub lon: i32,
    /// Terrain grid spacing in meters.
    pub grid_spacing: u16,
    /// Index of this 4x4 sub-grid within the block (0..55).
    pub gridbit: u8,
    /// 4x4 sub-grid heights in meters AMSL, row-major.
    pub data: [i16; 16],
}

// ---------------------------------------------------------------------------
// Grid block
// ---------------------------------------------------------------------------

/// A single terrain grid block: 32 x 28 elevation samples.
#[derive(Clone)]
pub struct TerrainGrid {
    /// Height samples in meters AMSL. `heights[row][col]` with row 0 at lat_origin.
    pub heights: [[i16; GRID_COLS]; GRID_ROWS],
    /// Latitude of grid[0][0] in degrees.
    pub lat_origin: f64,
    /// Longitude of grid[0][0] in degrees.
    pub lon_origin: f64,
    /// Whether this block contains valid data.
    pub valid: bool,
}

impl TerrainGrid {
    /// Create an empty (invalid) grid block.
    pub const fn empty() -> Self {
        Self {
            heights: [[INVALID_HEIGHT; GRID_COLS]; GRID_ROWS],
            lat_origin: 0.0,
            lon_origin: 0.0,
            valid: false,
        }
    }

    /// Create a grid block with the given origin and heights.
    pub fn new(lat_origin: f64, lon_origin: f64, heights: [[i16; GRID_COLS]; GRID_ROWS]) -> Self {
        Self {
            heights,
            lat_origin,
            lon_origin,
            valid: true,
        }
    }

    /// Check if a lat/lon position (degrees) falls within this grid block.
    fn contains(&self, lat: f64, lon: f64) -> bool {
        if !self.valid {
            return false;
        }
        // Small tolerance for floating-point boundary comparisons
        const EPS: f64 = 1e-10;
        let lat_end = self.lat_origin + (GRID_ROWS - 1) as f64 * GRID_SPACING_DEG;
        let lon_end = self.lon_origin + (GRID_COLS - 1) as f64 * GRID_SPACING_DEG;
        lat >= self.lat_origin - EPS
            && lat <= lat_end + EPS
            && lon >= self.lon_origin - EPS
            && lon <= lon_end + EPS
    }

    /// Bilinear interpolation for a position within this block.
    /// Returns None if out of bounds or block is invalid.
    fn interpolate(&self, lat: f64, lon: f64) -> Option<f32> {
        if !self.valid {
            return None;
        }

        // Fractional row/col position within the grid
        let row_f = (lat - self.lat_origin) / GRID_SPACING_DEG;
        let col_f = (lon - self.lon_origin) / GRID_SPACING_DEG;

        let row_max = (GRID_ROWS - 1) as f64;
        let col_max = (GRID_COLS - 1) as f64;

        // Tolerance for floating-point boundary comparison (< 1/1000 of a grid cell)
        const EPS: f64 = 1e-6;

        // Check bounds with small tolerance, then clamp
        if row_f < -EPS || col_f < -EPS {
            return None;
        }
        if row_f > row_max + EPS || col_f > col_max + EPS {
            return None;
        }

        // Clamp to valid range after tolerance check
        let row_f = if row_f < 0.0 { 0.0 } else if row_f > row_max { row_max } else { row_f };
        let col_f = if col_f < 0.0 { 0.0 } else if col_f > col_max { col_max } else { col_f };

        // Integer indices of the four surrounding samples
        let row0 = libm::floor(row_f) as usize;
        let col0 = libm::floor(col_f) as usize;
        let row1 = if row0 < GRID_ROWS - 1 { row0 + 1 } else { row0 };
        let col1 = if col0 < GRID_COLS - 1 { col0 + 1 } else { col0 };

        // Fractional offsets
        let fy = row_f - row0 as f64;
        let fx = col_f - col0 as f64;

        let h00 = self.heights[row0][col0] as f64;
        let h10 = self.heights[row0][col1] as f64;
        let h01 = self.heights[row1][col0] as f64;
        let h11 = self.heights[row1][col1] as f64;

        // Check for invalid samples
        if self.heights[row0][col0] == INVALID_HEIGHT
            || self.heights[row0][col1] == INVALID_HEIGHT
            || self.heights[row1][col0] == INVALID_HEIGHT
            || self.heights[row1][col1] == INVALID_HEIGHT
        {
            return None;
        }

        // Bilinear interpolation:
        // h = (1-fx)*(1-fy)*h00 + fx*(1-fy)*h10 + (1-fx)*fy*h01 + fx*fy*h11
        let h = (1.0 - fx) * (1.0 - fy) * h00
            + fx * (1.0 - fy) * h10
            + (1.0 - fx) * fy * h01
            + fx * fy * h11;

        Some(h as f32)
    }
}

// ---------------------------------------------------------------------------
// Terrain database with LRU cache
// ---------------------------------------------------------------------------

/// Terrain database: LRU cache of grid blocks with height lookup.
pub struct TerrainDatabase {
    /// Cached grid blocks.
    cache: [TerrainGrid; MAX_CACHE_BLOCKS],
    /// LRU ordering: index 0 = most recently used slot index.
    lru_order: [u8; MAX_CACHE_BLOCKS],
    /// Number of valid blocks currently in cache.
    cache_count: usize,
    /// Pending requests for grid blocks not yet in cache.
    pending_requests: heapless::Vec<TerrainRequest, MAX_PENDING_REQUESTS>,
}

impl TerrainDatabase {
    /// Create an empty terrain database.
    pub fn new() -> Self {
        let mut lru_order = [0u8; MAX_CACHE_BLOCKS];
        // Initialize LRU order: 0, 1, 2, ... so slot 0 is "most recent"
        let mut i = 0;
        while i < MAX_CACHE_BLOCKS {
            lru_order[i] = i as u8;
            i += 1;
        }

        Self {
            cache: [const { TerrainGrid::empty() }; MAX_CACHE_BLOCKS],
            lru_order,
            cache_count: 0,
            pending_requests: heapless::Vec::new(),
        }
    }

    /// Look up the terrain height AMSL at a given lat/lon (degrees) using bilinear
    /// interpolation across cached grid blocks.
    ///
    /// Returns `None` if the required grid block is not in the cache.
    pub fn height_amsl(&mut self, lat: f64, lon: f64) -> Option<f32> {
        // Search cache for a block containing this position
        for lru_idx in 0..self.cache_count {
            let slot = self.lru_order[lru_idx] as usize;
            if self.cache[slot].contains(lat, lon) {
                // Promote this slot to most-recently-used
                if lru_idx > 0 {
                    self.promote_lru(lru_idx);
                }
                return self.cache[slot].interpolate(lat, lon);
            }
        }
        None
    }

    /// Compute height above terrain: `alt_amsl - terrain_height`.
    ///
    /// Returns `None` if terrain data is not available for this position.
    pub fn height_above_terrain(&mut self, lat: f64, lon: f64, alt_amsl: f32) -> Option<f32> {
        self.height_amsl(lat, lon).map(|h| alt_amsl - h)
    }

    /// Returns the next pending terrain request, if any, for the GCS to fulfill.
    pub fn request_needed(&self) -> Option<TerrainRequest> {
        self.pending_requests.first().copied()
    }

    /// Queue a terrain request for a grid block covering the given lat/lon (degrees).
    /// If the block is already cached or a request is already pending, this is a no-op.
    pub fn request_grid_for(&mut self, lat: f64, lon: f64, grid_spacing_m: u16) {
        // Compute the grid block origin for this position
        let lat_origin = grid_origin(lat, GRID_ROWS);
        let lon_origin = grid_origin(lon, GRID_COLS);

        // Check if already cached
        for i in 0..self.cache_count {
            let slot = self.lru_order[i] as usize;
            if self.cache[slot].valid
                && approx_eq(self.cache[slot].lat_origin, lat_origin)
                && approx_eq(self.cache[slot].lon_origin, lon_origin)
            {
                return; // Already cached
            }
        }

        let lat_e7 = (lat_origin * 1e7) as i32;
        let lon_e7 = (lon_origin * 1e7) as i32;

        // Check if request already pending
        for req in self.pending_requests.iter() {
            if req.lat == lat_e7 && req.lon == lon_e7 {
                return; // Already requested
            }
        }

        // All sub-grids needed
        let mask = (1u64 << SUBGRIDS_TOTAL) - 1;

        let _ = self.pending_requests.push(TerrainRequest {
            lat: lat_e7,
            lon: lon_e7,
            grid_spacing: grid_spacing_m,
            mask,
        });
    }

    /// Store a received terrain grid block into the cache.
    /// If the cache is full, the least-recently-used block is evicted.
    pub fn receive_data(&mut self, block: &TerrainGrid) {
        if !block.valid {
            return;
        }

        // Check if we already have this block (update in place)
        for i in 0..self.cache_count {
            let slot = self.lru_order[i] as usize;
            if self.cache[slot].valid
                && approx_eq(self.cache[slot].lat_origin, block.lat_origin)
                && approx_eq(self.cache[slot].lon_origin, block.lon_origin)
            {
                self.cache[slot] = block.clone();
                self.promote_lru(i);
                self.remove_pending_for(block.lat_origin, block.lon_origin);
                return;
            }
        }

        // Find a slot: use next empty slot, or evict LRU
        let target_slot;
        if self.cache_count < MAX_CACHE_BLOCKS {
            // Use the slot at position cache_count in LRU order
            target_slot = self.lru_order[self.cache_count] as usize;
            self.cache_count += 1;
            // Promote this new entry to MRU (it's at index cache_count - 1)
            self.promote_lru(self.cache_count - 1);
        } else {
            // Evict the least-recently-used (last in lru_order)
            target_slot = self.lru_order[MAX_CACHE_BLOCKS - 1] as usize;
            // Move evicted slot to front (MRU)
            self.promote_lru(MAX_CACHE_BLOCKS - 1);
        }

        self.cache[target_slot] = block.clone();
        self.remove_pending_for(block.lat_origin, block.lon_origin);
    }

    /// Receive a single 4x4 sub-grid from a TERRAIN_DATA MAVLink message.
    /// This fills part of a grid block; the block must already exist in cache
    /// or a new one is created.
    pub fn receive_subgrid(&mut self, data: &TerrainData) {
        let lat_origin = data.lat as f64 / 1e7;
        let lon_origin = data.lon as f64 / 1e7;

        // Find existing block or create a new one
        let mut found_idx = None;
        for i in 0..self.cache_count {
            let slot = self.lru_order[i] as usize;
            if approx_eq(self.cache[slot].lat_origin, lat_origin)
                && approx_eq(self.cache[slot].lon_origin, lon_origin)
            {
                found_idx = Some(i);
                break;
            }
        }

        let slot = if let Some(idx) = found_idx {
            self.promote_lru(idx);
            self.lru_order[0] as usize
        } else {
            // Create a new invalid block and place it
            let mut new_block = TerrainGrid::empty();
            new_block.lat_origin = lat_origin;
            new_block.lon_origin = lon_origin;
            new_block.valid = true;
            self.receive_data(&new_block);
            self.lru_order[0] as usize
        };

        // Compute the sub-grid's position within the 32x28 block
        let gridbit = data.gridbit as usize;
        if gridbit >= SUBGRIDS_TOTAL {
            return;
        }
        let sg_row = gridbit / SUBGRIDS_PER_ROW; // 0..6
        let sg_col = gridbit % SUBGRIDS_PER_ROW; // 0..7
        let base_row = sg_row * SUBGRID_SIZE;
        let base_col = sg_col * SUBGRID_SIZE;

        // Fill in the 4x4 sub-grid (row-major in data.data)
        for r in 0..SUBGRID_SIZE {
            for c in 0..SUBGRID_SIZE {
                let row = base_row + r;
                let col = base_col + c;
                if row < GRID_ROWS && col < GRID_COLS {
                    self.cache[slot].heights[row][col] = data.data[r * SUBGRID_SIZE + c];
                }
            }
        }
    }

    /// Number of valid blocks currently in the cache.
    pub fn cached_block_count(&self) -> usize {
        self.cache_count
    }

    /// Clear the entire cache and all pending requests.
    pub fn clear(&mut self) {
        for i in 0..MAX_CACHE_BLOCKS {
            self.cache[i] = TerrainGrid::empty();
            self.lru_order[i] = i as u8;
        }
        self.cache_count = 0;
        self.pending_requests.clear();
    }

    // -- Internal helpers --

    /// Promote the LRU entry at `idx` to position 0 (most recently used).
    fn promote_lru(&mut self, idx: usize) {
        if idx == 0 {
            return;
        }
        let slot = self.lru_order[idx];
        // Shift entries [0..idx) right by one
        let mut i = idx;
        while i > 0 {
            self.lru_order[i] = self.lru_order[i - 1];
            i -= 1;
        }
        self.lru_order[0] = slot;
    }

    /// Remove pending request matching the given origin.
    fn remove_pending_for(&mut self, lat_origin: f64, lon_origin: f64) {
        let lat_e7 = (lat_origin * 1e7) as i32;
        let lon_e7 = (lon_origin * 1e7) as i32;
        self.pending_requests.retain(|r| r.lat != lat_e7 || r.lon != lon_e7);
    }
}

/// Compute the grid block origin (lower-left corner) for a given coordinate.
/// Aligns to the grid block boundary.
fn grid_origin(coord: f64, grid_size: usize) -> f64 {
    let block_span = (grid_size - 1) as f64 * GRID_SPACING_DEG;
    libm::floor(coord / block_span) * block_span
}

/// Approximate f64 equality for grid origins (within ~1 mm in degrees).
fn approx_eq(a: f64, b: f64) -> bool {
    libm::fabs(a - b) < 1e-9
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: create a flat grid block at the given origin with uniform height.
    fn flat_grid(lat_origin: f64, lon_origin: f64, height: i16) -> TerrainGrid {
        TerrainGrid::new(lat_origin, lon_origin, [[height; GRID_COLS]; GRID_ROWS])
    }

    /// Helper: create a grid with a known slope for interpolation testing.
    /// Height = base + row + col (so it varies linearly in both axes).
    fn sloped_grid(lat_origin: f64, lon_origin: f64, base: i16) -> TerrainGrid {
        let mut heights = [[0i16; GRID_COLS]; GRID_ROWS];
        for row in 0..GRID_ROWS {
            for col in 0..GRID_COLS {
                heights[row][col] = base + row as i16 + col as i16;
            }
        }
        TerrainGrid::new(lat_origin, lon_origin, heights)
    }

    // -----------------------------------------------------------------------
    // Store and retrieve
    // -----------------------------------------------------------------------

    #[test]
    fn test_store_and_retrieve() {
        let mut db = TerrainDatabase::new();
        let origin_lat = 35.0;
        let origin_lon = -120.0;
        let grid = flat_grid(origin_lat, origin_lon, 500);

        db.receive_data(&grid);
        assert_eq!(db.cached_block_count(), 1);

        // Query the center of the block
        let center_lat = origin_lat + (GRID_ROWS / 2) as f64 * GRID_SPACING_DEG;
        let center_lon = origin_lon + (GRID_COLS / 2) as f64 * GRID_SPACING_DEG;
        let h = db.height_amsl(center_lat, center_lon);
        assert!(h.is_some());
        let h = h.unwrap();
        assert!((h - 500.0).abs() < 0.01, "Expected 500.0, got {}", h);
    }

    #[test]
    fn test_store_retrieve_exact_corners() {
        let mut db = TerrainDatabase::new();
        let grid = flat_grid(10.0, 20.0, 1234);
        db.receive_data(&grid);

        // Bottom-left corner (origin)
        let h = db.height_amsl(10.0, 20.0).unwrap();
        assert!((h - 1234.0).abs() < 0.01);

        // Top-right corner
        let lat_max = 10.0 + (GRID_ROWS - 1) as f64 * GRID_SPACING_DEG;
        let lon_max = 20.0 + (GRID_COLS - 1) as f64 * GRID_SPACING_DEG;
        let h = db.height_amsl(lat_max, lon_max).unwrap();
        assert!((h - 1234.0).abs() < 0.01);
    }

    // -----------------------------------------------------------------------
    // Bilinear interpolation accuracy
    // -----------------------------------------------------------------------

    #[test]
    fn test_bilinear_interpolation() {
        let mut db = TerrainDatabase::new();
        let origin_lat = 35.0;
        let origin_lon = -120.0;
        let grid = sloped_grid(origin_lat, origin_lon, 100);
        db.receive_data(&grid);

        // Query at exact grid point [5][10] => height = 100 + 5 + 10 = 115
        let lat = origin_lat + 5.0 * GRID_SPACING_DEG;
        let lon = origin_lon + 10.0 * GRID_SPACING_DEG;
        let h = db.height_amsl(lat, lon).unwrap();
        assert!((h - 115.0).abs() < 0.01, "Expected 115.0, got {}", h);

        // Query halfway between [5][10] and [6][11]
        // h00 = 100+5+10 = 115, h10 = 100+5+11 = 116
        // h01 = 100+6+10 = 116, h11 = 100+6+11 = 117
        // At fx=0.5, fy=0.5:
        // h = 0.25*115 + 0.25*116 + 0.25*116 + 0.25*117 = 116.0
        let lat_mid = origin_lat + 5.5 * GRID_SPACING_DEG;
        let lon_mid = origin_lon + 10.5 * GRID_SPACING_DEG;
        let h = db.height_amsl(lat_mid, lon_mid).unwrap();
        assert!((h - 116.0).abs() < 0.01, "Expected 116.0, got {}", h);
    }

    #[test]
    fn test_bilinear_quarter_point() {
        let mut db = TerrainDatabase::new();
        let origin_lat = 0.0;
        let origin_lon = 0.0;

        // Create a grid where corners of cell [0][0] have known heights:
        // h[0][0]=0, h[0][1]=100, h[1][0]=200, h[1][1]=300
        let mut heights = [[0i16; GRID_COLS]; GRID_ROWS];
        heights[0][0] = 0;
        heights[0][1] = 100;
        heights[1][0] = 200;
        heights[1][1] = 300;
        // Fill rest to avoid INVALID_HEIGHT issues
        for r in 0..GRID_ROWS {
            for c in 0..GRID_COLS {
                if (r, c) != (0, 0) && (r, c) != (0, 1) && (r, c) != (1, 0) && (r, c) != (1, 1) {
                    heights[r][c] = 0;
                }
            }
        }
        let grid = TerrainGrid::new(origin_lat, origin_lon, heights);
        db.receive_data(&grid);

        // At fx=0.25, fy=0.75:
        // h = 0.75*0.25*0 + 0.25*0.25*100 + 0.75*0.75*200 + 0.25*0.75*300
        // h = 0 + 6.25 + 112.5 + 56.25 = 175.0
        let lat = origin_lat + 0.75 * GRID_SPACING_DEG;
        let lon = origin_lon + 0.25 * GRID_SPACING_DEG;
        let h = db.height_amsl(lat, lon).unwrap();
        assert!(
            (h - 175.0).abs() < 0.1,
            "Expected 175.0, got {}",
            h
        );
    }

    // -----------------------------------------------------------------------
    // LRU eviction
    // -----------------------------------------------------------------------

    #[test]
    fn test_lru_eviction() {
        let mut db = TerrainDatabase::new();

        // Fill cache with 12 blocks at distinct origins
        for i in 0..MAX_CACHE_BLOCKS {
            let lat = i as f64 * 1.0; // well-separated origins
            let grid = flat_grid(lat, 0.0, (i * 100) as i16);
            db.receive_data(&grid);
        }
        assert_eq!(db.cached_block_count(), MAX_CACHE_BLOCKS);

        // All 12 should be accessible
        for i in 0..MAX_CACHE_BLOCKS {
            let lat = i as f64 * 1.0;
            let h = db.height_amsl(lat, 0.0);
            assert!(h.is_some(), "Block {} should be in cache", i);
            assert!(
                (h.unwrap() - (i * 100) as f32).abs() < 0.01,
                "Block {} height mismatch",
                i
            );
        }

        // The most recently accessed is block 11 (last in the loop above).
        // The least recently used is block 0 (accessed first, then pushed back).
        // Actually, let's be precise: after the loop, the access order was
        // 0, 1, 2, ..., 11. So block 11 is MRU and block 0 is LRU.
        // But then the second loop accessed 0..11 again, making block 0 LRU
        // after being just accessed... let's restart cleanly.

        // Reset and re-fill
        db.clear();
        for i in 0..MAX_CACHE_BLOCKS {
            let lat = i as f64 * 1.0;
            let grid = flat_grid(lat, 0.0, (i * 100) as i16);
            db.receive_data(&grid);
        }

        // Block 0 was inserted first, block 11 last (MRU).
        // Insert a 13th block -- should evict block 0 (LRU).
        let new_grid = flat_grid(99.0, 0.0, 9999);
        db.receive_data(&new_grid);
        assert_eq!(db.cached_block_count(), MAX_CACHE_BLOCKS);

        // Block 0 should be gone
        let h = db.height_amsl(0.0, 0.0);
        assert!(h.is_none(), "Block 0 should have been evicted");

        // New block should be present
        let h = db.height_amsl(99.0, 0.0).unwrap();
        assert!((h - 9999.0).abs() < 0.01);

        // Block 1 (second oldest) should still be present
        let h = db.height_amsl(1.0, 0.0);
        assert!(h.is_some(), "Block 1 should still be in cache");
    }

    #[test]
    fn test_lru_access_refreshes() {
        let mut db = TerrainDatabase::new();

        // Fill cache with 12 blocks
        for i in 0..MAX_CACHE_BLOCKS {
            let lat = i as f64 * 1.0;
            let grid = flat_grid(lat, 0.0, (i * 100) as i16);
            db.receive_data(&grid);
        }

        // Access block 0 (the oldest) to refresh it in the LRU
        let _ = db.height_amsl(0.0, 0.0);

        // Now block 1 should be the LRU. Insert a new block.
        let new_grid = flat_grid(99.0, 0.0, 9999);
        db.receive_data(&new_grid);

        // Block 0 should survive (we just accessed it)
        assert!(
            db.height_amsl(0.0, 0.0).is_some(),
            "Block 0 should survive after LRU refresh"
        );

        // Block 1 should be evicted (it was the least recently used)
        assert!(
            db.height_amsl(1.0, 0.0).is_none(),
            "Block 1 should have been evicted"
        );
    }

    // -----------------------------------------------------------------------
    // Height above terrain
    // -----------------------------------------------------------------------

    #[test]
    fn test_height_above_terrain() {
        let mut db = TerrainDatabase::new();
        let grid = flat_grid(35.0, -120.0, 500);
        db.receive_data(&grid);

        let lat = 35.0 + 5.0 * GRID_SPACING_DEG;
        let lon = -120.0 + 5.0 * GRID_SPACING_DEG;

        // Flying at 650 m AMSL over 500 m terrain => 150 m AGL
        let hat = db.height_above_terrain(lat, lon, 650.0).unwrap();
        assert!((hat - 150.0).abs() < 0.01, "Expected 150.0, got {}", hat);

        // Flying at 400 m AMSL over 500 m terrain => -100 m (below terrain)
        let hat = db.height_above_terrain(lat, lon, 400.0).unwrap();
        assert!((hat - (-100.0)).abs() < 0.01, "Expected -100.0, got {}", hat);
    }

    // -----------------------------------------------------------------------
    // Missing block returns None
    // -----------------------------------------------------------------------

    #[test]
    fn test_missing_block_returns_none() {
        let mut db = TerrainDatabase::new();

        // Empty database - should return None
        assert!(db.height_amsl(35.0, -120.0).is_none());
        assert!(db.height_above_terrain(35.0, -120.0, 100.0).is_none());
    }

    #[test]
    fn test_out_of_bounds_returns_none() {
        let mut db = TerrainDatabase::new();
        let grid = flat_grid(35.0, -120.0, 500);
        db.receive_data(&grid);

        // Query outside the grid block
        assert!(db.height_amsl(34.0, -120.0).is_none());
        assert!(db.height_amsl(35.0, -121.0).is_none());
        assert!(db.height_amsl(36.0, -119.0).is_none());
    }

    // -----------------------------------------------------------------------
    // Pending requests
    // -----------------------------------------------------------------------

    #[test]
    fn test_pending_requests() {
        let mut db = TerrainDatabase::new();

        // No pending requests initially
        assert!(db.request_needed().is_none());

        // Request a grid
        db.request_grid_for(35.0, -120.0, 100);
        assert!(db.request_needed().is_some());

        let req = db.request_needed().unwrap();
        assert_eq!(req.grid_spacing, 100);

        // Duplicate request should not add another
        db.request_grid_for(35.0, -120.0, 100);
        assert_eq!(db.pending_requests.len(), 1);
    }

    #[test]
    fn test_receive_clears_pending() {
        let mut db = TerrainDatabase::new();
        db.request_grid_for(35.0, -120.0, 100);
        assert_eq!(db.pending_requests.len(), 1);

        // Receive the block
        let origin_lat = grid_origin(35.0, GRID_ROWS);
        let origin_lon = grid_origin(-120.0, GRID_COLS);
        let grid = flat_grid(origin_lat, origin_lon, 500);
        db.receive_data(&grid);

        // Pending request should be cleared
        assert!(db.request_needed().is_none());
    }

    // -----------------------------------------------------------------------
    // Sub-grid receive
    // -----------------------------------------------------------------------

    #[test]
    fn test_receive_subgrid() {
        let mut db = TerrainDatabase::new();

        let data = TerrainData {
            lat: 350000000, // 35.0 * 1e7
            lon: -1200000000, // -120.0 * 1e7
            grid_spacing: 100,
            gridbit: 0, // First 4x4 sub-grid (top-left)
            data: [100; 16],
        };

        db.receive_subgrid(&data);
        assert_eq!(db.cached_block_count(), 1);

        // The origin of the block should be at the lat/lon from the data
        let h = db.height_amsl(35.0, -120.0);
        assert!(h.is_some());
        assert!((h.unwrap() - 100.0).abs() < 0.01);
    }

    // -----------------------------------------------------------------------
    // Invalid grid data
    // -----------------------------------------------------------------------

    #[test]
    fn test_invalid_grid_not_stored() {
        let mut db = TerrainDatabase::new();
        let grid = TerrainGrid::empty(); // valid = false
        db.receive_data(&grid);
        assert_eq!(db.cached_block_count(), 0);
    }
}
