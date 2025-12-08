{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
    utils.url = "github:numtide/flake-utils";
    rust-overlay.url = "github:oxalica/rust-overlay";
  };

  outputs = { self, nixpkgs, utils, rust-overlay }:
    utils.lib.eachDefaultSystem (system:
      let
        overlays = [ (import rust-overlay) ];
        pkgs = import nixpkgs { inherit system overlays; };
      in
      {
        devShells.default = with pkgs; mkShell {
          buildInputs = [
            (rust-bin.stable.latest.default.override {
              targets = [ "thumbv7em-none-eabihf" ];
              extensions = [ "rust-analyzer" ]
            })
            probe-rs-tools
          ];
          packages = [ ];
          RUST_SRC_PATH = rustPlatform.rustLibSrc;
          shellHook = ''
          '';
        };
      }
    );
}
