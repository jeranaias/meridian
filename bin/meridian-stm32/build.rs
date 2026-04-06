// Build script for meridian-stm32-bin
//
// Tells the linker where to find memory.x (the linker script).
// cortex-m-rt's link.x includes memory.x which defines MEMORY regions.

use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    // Copy memory.x from the platform crate to the OUT_DIR so cortex-m-rt finds it.
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());

    // Use the platform crate's memory.x as the canonical linker script.
    let memory_x = PathBuf::from("../../crates/meridian-platform-stm32/memory.x");
    if memory_x.exists() {
        fs::copy(&memory_x, out.join("memory.x")).unwrap();
    }

    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=../../crates/meridian-platform-stm32/memory.x");
    println!("cargo:rerun-if-changed=build.rs");
}
