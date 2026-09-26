# nixpkgs with nix-ros-overlay applied. The sources come from flake.lock: as flake inputs
# under `nix build`, or through nix/sources.nix under nix-build and nix-shell.
{ nixpkgs, nix-ros-overlay, system ? builtins.currentSystem }:

import nixpkgs {
  inherit system;
  overlays = [ (import "${nix-ros-overlay}/overlay.nix") ];
}
