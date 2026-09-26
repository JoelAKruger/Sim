# nix-shell (or `nix develop`), then:
#   cmake -B build -G Ninja && cmake --build build && cmake --install build --prefix install
#   ros2 launch regolith sim.launch.py description:=/path/to/rover.urdf.xacro
let
  regolith = import ./default.nix { };
  sources = import ./nix/sources.nix;
in
import ./nix/shell.nix {
  pkgs = import ./nix/pkgs.nix { inherit (sources) nixpkgs nix-ros-overlay; };
  inherit regolith;
}
