# nix-build                      → ./result/bin/regolith (runs outside any shell)
# nix-build --arg withRos false  → the same sim built with no ROS 2 at all
#   (and --arg withCan false for no SocketCAN)
# Both use the revisions pinned in flake.lock, as `nix build` does.
let
  sources = import ./nix/sources.nix;
in
{ pkgs ? import ./nix/pkgs.nix { inherit (sources) nixpkgs nix-ros-overlay; }
, withRos ? true
, withCan ? true
}:

pkgs.rosPackages.jazzy.callPackage ./nix/packages/regolith {
  inherit withRos withCan;
  chrono = pkgs.callPackage ./nix/packages/chrono { };
}
