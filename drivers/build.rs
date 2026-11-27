fn main() {
    // Linker flags apply only to runnable examples in `drivers/examples/`,
    // never to downstream crates linking `gps-tag-drivers` as a library.
    println!("cargo:rustc-link-arg-examples=--nmagic");
    println!("cargo:rustc-link-arg-examples=-Tlink.x");
    println!("cargo:rustc-link-arg-examples=-Tdefmt.x");
    println!("cargo:rustc-env=DEFMT_LOG=info");
}
