{
  description = "IR Pico firmware for Raspberry Pi Pico";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    crane.url = "github:ipetkov/crane";
    flake-utils.url = "github:numtide/flake-utils";
    treefmt-nix = {
      url = "github:numtide/treefmt-nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs =
    {
      self,
      nixpkgs,
      rust-overlay,
      crane,
      flake-utils,
      treefmt-nix,
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ rust-overlay.overlays.default ];
        };

        treefmtEval = treefmt-nix.lib.evalModule pkgs ./treefmt.nix;

        rustToolchain = pkgs.rust-bin.stable.latest.default.override {
          extensions = [
            "clippy"
            "rustfmt"
          ];
          targets = [ "thumbv6m-none-eabi" ];
        };

        craneLib = (crane.mkLib pkgs).overrideToolchain rustToolchain;

        target = "thumbv6m-none-eabi";

        src = pkgs.lib.cleanSourceWith {
          src = ./.;
          filter = path: type: (pkgs.lib.hasSuffix ".x" path) || (craneLib.filterCargoSources path type);
        };

        # flip-link must be on PATH; .cargo/config.toml sets it as the linker.
        commonArgs = {
          inherit src;
          strictDeps = true;
          doCheck = false;
          cargoExtraArgs = "--target ${target}";
          nativeBuildInputs = [ pkgs.flip-link ];
        };

        ir-pico = craneLib.buildPackage (
          commonArgs
          // {
            # cargoArtifacts MUST be null to get craneLib to pick up the required linker script.
            cargoArtifacts = null;
            nativeBuildInputs = commonArgs.nativeBuildInputs ++ [ pkgs.elf2uf2-rs ];

            postBuild = ''
              cargo clippy --target ${target} --all-features -- --deny warnings
            '';

            cargoInstallCommand = "true";
            postInstall = ''
              mkdir -p $out/bin
              cp target/${target}/release/ir-pico $out/bin/ir-pico.elf
              elf2uf2-rs target/${target}/release/ir-pico $out/bin/ir-pico.uf2
            '';
          }
        );

        pythonTests =
          pkgs.runCommand "python-tests"
            {
              nativeBuildInputs = [
                (pkgs.python3.withPackages (ps: [ ps.pytest ]))
              ];
            }
            ''
              cd ${./py}
              pytest tests/
              touch $out
            '';

      in
      {
        packages.default = ir-pico;

        checks = {
          formatting = treefmtEval.config.build.check self;

          python = pythonTests;
        };

        formatter = treefmtEval.config.build.wrapper;

        devShells.default = pkgs.mkShell {
          packages = [
            rustToolchain
            pkgs.flip-link
            pkgs.elf2uf2-rs
            pkgs.probe-rs-tools
          ];
        };
      }
    );
}
