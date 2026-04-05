{ ... }:
{
  projectRootFile = "flake.nix";

  programs.nixfmt.enable = true;
  programs.deadnix.enable = true;
  programs.rustfmt.enable = true;
  programs.ruff-check.enable = true;
  programs.ruff-format.enable = true;
}
