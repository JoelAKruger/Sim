# The flake's inputs, fetched from flake.lock without flakes, so default.nix and
# shell.nix build from exactly the same revisions as `nix build`.
let
  lock = builtins.fromJSON (builtins.readFile ../flake.lock);
  root = lock.nodes.${lock.root}.inputs;

  # A github input, fetched by the NAR hash recorded in the lock.
  fetch = node:
    let locked = lock.nodes.${node}.locked;
    in builtins.fetchTarball {
      url = "https://github.com/${locked.owner}/${locked.repo}/archive/${locked.rev}.tar.gz";
      sha256 = locked.narHash;
    };

  # nixpkgs follows nix-ros-overlay's own input, so resolve it through that node.
  nixpkgs-node = lock.nodes.${root.nix-ros-overlay}.inputs.nixpkgs;
in
{
  nixpkgs = fetch nixpkgs-node;
  nix-ros-overlay = fetch root.nix-ros-overlay;
}
