# The dev shell: everything to build in-tree, plus the ROS tools to run and inspect the
# sim. Used by flake.nix (nix develop) and shell.nix (nix-shell).
{ pkgs, regolith }:

let
  ros = pkgs.rosPackages.jazzy;

  ros-env = ros.buildEnv {
    paths = regolith.ros-deps ++ (with ros; [
      ros-core
      ros2launch
      robot-state-publisher
      xacro
      rviz2
    ]);
  };
in
pkgs.mkShell {
  inputsFrom = [ regolith ];
  packages = [ ros-env pkgs.can-utils pkgs.gdb pkgs.clang-tools ];

  BOX3D_SOURCE_DIR = regolith.box3d-src;
  REGOLITH_FONT_REGULAR = "${regolith.fonts}/JetBrainsMono-Regular.ttf";
  REGOLITH_FONT_BOLD = "${regolith.fonts}/JetBrainsMono-SemiBold.ttf";

  # ./install first, so the in-tree build is what `ros2 run` and `ros2 launch` find.
  shellHook = ''
    export AMENT_PREFIX_PATH="$PWD/install''${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
  '';
}
