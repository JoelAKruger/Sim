# The Regolith package. flake.nix and default.nix call it with the pinned Box3D source.
{ lib
, stdenv
, buildRosPackage
, buildEnv
  # Box3D v0.1.0 source, pinned in flake.lock (the `box3d` input).
, box3d-src
, cmake
, ninja
, makeWrapper
, clang-tools
, raylib
, expat
  # The viewer's font, copied into the binary at build time (not a runtime dependency).
, jetbrains-mono
, rclcpp
, rmw-cyclonedds-cpp
, rmw-fastrtps-cpp
, std-msgs
, sensor-msgs
, nav-msgs
, geometry-msgs
, tf2-msgs
, rosgraph-msgs
  # false builds the sim with no ROS 2 at all (REGOLITH_WITH_ROS=OFF); the ROS packages
  # above are then never evaluated or fetched.
, withRos ? true
  # false builds the sim without the SocketCAN bridge (REGOLITH_WITH_CAN=OFF).
, withCan ? true
}:

let
  # Both middlewares, so the sim follows whatever RMW_IMPLEMENTATION the Nova
  # environment selects (CycloneDDS by default).
  ros-deps = lib.optionals withRos [
    rclcpp
    rmw-cyclonedds-cpp
    rmw-fastrtps-cpp
    std-msgs
    sensor-msgs
    nav-msgs
    geometry-msgs
    tf2-msgs
    rosgraph-msgs
  ];

  # rclcpp dlopens the RMW and message type-support libraries at runtime, finding them
  # through AMENT_PREFIX_PATH and LD_LIBRARY_PATH. This env provides both.
  ros-env = buildEnv {
    paths = ros-deps;
    wrapPrograms = false;
  };

  fonts = "${jetbrains-mono}/share/fonts/truetype";

  root = ../../..;

  # Without ROS there is nothing ROS-specific about the package, so it is a plain
  # derivation (named regolith-*, not ros-jazzy-regolith-*).
  mkPackage = if withRos then buildRosPackage else stdenv.mkDerivation;
in
mkPackage ({
  pname = "regolith";
  version = "0.1.0";

  # Allowlist, so local build trees and editor files never cause a rebuild.
  src = lib.fileset.toSource {
    inherit root;
    fileset = lib.fileset.unions ([
      (root + "/CMakeLists.txt")
      (root + "/package.xml")
      (root + "/can")
      (root + "/core")
      (root + "/render")
      (root + "/ros")
      (root + "/app")
      (root + "/tests")
      (root + "/tools")
      (root + "/config")
      (root + "/robots")
      (root + "/.clang-tidy")
      (root + "/.clang-format")
    ] ++ map lib.fileset.maybeMissing [
      (root + "/launch")
      (root + "/terrain")
    ]);
  };

  nativeBuildInputs = [ cmake ninja makeWrapper ];
  buildInputs = [ raylib expat ];
  propagatedBuildInputs = ros-deps;

  cmakeFlags = [
    "-DBOX3D_SOURCE_DIR=${box3d-src}"
    (lib.cmakeBool "REGOLITH_WITH_ROS" withRos)
    (lib.cmakeBool "REGOLITH_WITH_CAN" withCan)
    "-DREGOLITH_FONT_REGULAR=${fonts}/JetBrainsMono-Regular.ttf"
    "-DREGOLITH_FONT_BOLD=${fonts}/JetBrainsMono-SemiBold.ttf"
  ];

  # Tests, then layout (clang-format) and naming (clang-tidy).
  doCheck = true;
  nativeCheckInputs = [ clang-tools ];
  checkPhase = ''
    runHook preCheck
    ctest --output-on-failure
    (cd .. && bash tools/check_format.sh && bash tools/check_naming.sh build)
    runHook postCheck
  '';

  postFixup = lib.optionalString withRos ''
    wrapProgram "$out/lib/regolith/regolith" \
      --prefix AMENT_PREFIX_PATH : "$out:${ros-env}" \
      --prefix LD_LIBRARY_PATH : "${ros-env}/lib"
  '' + ''
    mkdir -p "$out/bin"
    ln -s "$out/lib/regolith/regolith" "$out/bin/regolith"
  '';

  passthru = { inherit box3d-src ros-deps fonts; };

  meta = {
    description = "Rover simulator on Raylib and Box3D, with ROS 2 Jazzy and SocketCAN";
    mainProgram = "regolith";
    platforms = lib.platforms.linux;
  };
} // lib.optionalAttrs withRos {
  buildType = "cmake";
})
