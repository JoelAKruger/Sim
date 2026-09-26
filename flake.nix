{
  description = "Regolith: rover simulator on Raylib and Box3D, with ROS 2 Jazzy and SocketCAN";

  # Every revision is pinned in flake.lock; `nix flake update` moves them forward.
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    # The nixpkgs the overlay is tested with, which is what ros.cachix.org is built against.
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    box3d = {
      url = "github:erincatto/box3d/v0.1.0";
      flake = false;
    };
  };

  outputs = { self, nixpkgs, nix-ros-overlay, box3d }:
    let
      system = "x86_64-linux";
      pkgs = import ./nix/pkgs.nix { inherit nixpkgs nix-ros-overlay system; };
      regolith = { withRos ? true, withCan ? true }:
        pkgs.rosPackages.jazzy.callPackage ./nix/packages/regolith {
          inherit withRos withCan;
          box3d-src = box3d;
        };
    in
    {
      packages.${system} = {
        default = regolith { };
        without-ros = regolith { withRos = false; }; # the same sim with no ROS 2 at all
        minimal = regolith { withRos = false; withCan = false; }; # neither adapter
      };

      # Building a package runs the tests and the style and naming checks.
      checks.${system} = self.packages.${system};

      devShells.${system}.default = import ./nix/shell.nix {
        inherit pkgs;
        regolith = self.packages.${system}.default;
      };
    };
}
