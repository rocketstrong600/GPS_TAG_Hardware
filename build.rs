use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

const NRF_APP_BASE: usize = 0x0000_4000;

fn main() {
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    println!("cargo:rustc-env=DEFMT_LOG=info");

    build_nrf_firmware();
}

fn watch_dir(dir: &Path) {
    println!("cargo:rerun-if-changed={}", dir.display());
    if let Ok(entries) = fs::read_dir(dir) {
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                watch_dir(&path);
            } else {
                println!("cargo:rerun-if-changed={}", path.display());
            }
        }
    }
}

fn build_nrf_firmware() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let ble_dir = manifest_dir.join("ble-test");

    for file in [
        "Cargo.toml",
        "Cargo.lock",
        "memory.x",
        "build.rs",
        ".cargo/config.toml",
    ] {
        println!("cargo:rerun-if-changed={}", ble_dir.join(file).display());
    }
    watch_dir(&ble_dir.join("src"));

    let cargo = env::var("CARGO").unwrap_or_else(|_| "cargo".into());
    let ble_target_dir = ble_dir.join("target");

    let mut cmd = Command::new(cargo);
    cmd.current_dir(&ble_dir)
        .args(["build", "--release"])
        .env("CARGO_TARGET_DIR", &ble_target_dir)
        // Strip outer STM32 target / rustflags env vars so ble-test/.cargo/config.toml applies.
        .env_remove("CARGO_ENCODED_RUSTFLAGS")
        .env_remove("RUSTFLAGS")
        .env_remove("CARGO_BUILD_TARGET")
        .env_remove("TARGET");

    let status = cmd
        .status()
        .expect("failed to invoke cargo for ble-test nRF52832 firmware");
    assert!(
        status.success(),
        "building ble-test nRF52832 firmware failed (ensure nix develop / LIBCLANG_PATH is active)"
    );

    let elf_path = ble_target_dir
        .join("thumbv7em-none-eabihf")
        .join("release")
        .join("ble-test");
    let elf_bytes = fs::read(&elf_path).expect("failed to read ble-test ELF");
    let bin_bytes = elf32_to_bin(&elf_bytes, NRF_APP_BASE);

    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let bin_path = out_dir.join("nrf_fw.bin");
    fs::write(&bin_path, &bin_bytes).expect("failed to write nrf_fw.bin");

    println!("cargo:rustc-env=NRF_FW_BIN={}", bin_path.display());
}

/// Flatten an ARM little-endian ELF32's `PT_LOAD` segments into a raw binary image.
fn elf32_to_bin(elf: &[u8], expected_base: usize) -> Vec<u8> {
    assert!(
        elf.len() >= 52 && &elf[0..4] == b"\x7fELF",
        "not an ELF file"
    );
    assert!(
        elf[4] == 1 && elf[5] == 1,
        "expected 32-bit little-endian ELF"
    );

    let e_phoff = u32::from_le_bytes(elf[28..32].try_into().unwrap()) as usize;
    let e_phentsize = u16::from_le_bytes(elf[42..44].try_into().unwrap()) as usize;
    let e_phnum = u16::from_le_bytes(elf[44..46].try_into().unwrap()) as usize;

    let mut segs = Vec::new();
    for i in 0..e_phnum {
        let off = e_phoff + i * e_phentsize;
        let p_type = u32::from_le_bytes(elf[off..off + 4].try_into().unwrap());
        let p_offset = u32::from_le_bytes(elf[off + 4..off + 8].try_into().unwrap()) as usize;
        let p_paddr = u32::from_le_bytes(elf[off + 12..off + 16].try_into().unwrap()) as usize;
        let p_filesz = u32::from_le_bytes(elf[off + 16..off + 20].try_into().unwrap()) as usize;
        if p_type == 1 && p_filesz > 0 {
            segs.push((p_paddr, p_offset, p_filesz));
        }
    }

    let min_paddr = segs.iter().map(|s| s.0).min().expect("no PT_LOAD segments");
    assert_eq!(
        min_paddr, expected_base,
        "ble-test must be linked at {expected_base:#x}, got {min_paddr:#x}"
    );
    let max_end = segs.iter().map(|s| s.0 + s.2).max().unwrap();

    let mut out = vec![0u8; max_end - min_paddr];
    for (p_paddr, p_offset, p_filesz) in segs {
        let dst = p_paddr - min_paddr;
        out[dst..dst + p_filesz].copy_from_slice(&elf[p_offset..p_offset + p_filesz]);
    }
    out
}
