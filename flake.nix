{
  description = "Regolith: rover simulator on Raylib and Box3D, with ROS 2 Jazzy and SocketCAN";

  # ROS packages come prebuilt from the overlay's cache (nix asks before using it).
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };

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

      # `nix run`: the sim with the bundled settings and Banksia, unless the arguments
      # give their own --config or --urdf.
      app = package:
        let
          share = "${package}/share/regolith";
          script = pkgs.writeShellScript "regolith-run" ''
            config=1 urdf=1
            for arg in "$@"; do
              case "$arg" in
                --config) config=0 urdf=0 ;; # a config names its own robot (or none)
                --urdf) urdf=0 ;;
              esac
            done
            defaults=()
            if [ "$config" = 1 ]; then defaults+=(--config "${share}/config/sim.yaml"); fi
            if [ "$urdf" = 1 ]; then defaults+=(--urdf "${share}/robots/banksia.urdf"); fi
            exec "${package}/bin/regolith" "''${defaults[@]}" "$@"
          '';
        in
        {
          type = "app";
          program = "${script}";
          meta.description = package.meta.description;
        };
    in
    {
      packages.${system} = {
        default = regolith { };
        without-ros = regolith { withRos = false; }; # the same sim with no ROS 2 at all
        minimal = regolith { withRos = false; withCan = false; }; # neither adapter
      };

      apps.${system} = builtins.mapAttrs (name: package: app package) self.packages.${system};

      # Building a package runs the tests and the style and naming checks.
      checks.${system} = self.packages.${system};

      devShells.${system}.default = import ./nix/shell.nix {
        inherit pkgs;
        regolith = self.packages.${system}.default;
      };
    };
}
