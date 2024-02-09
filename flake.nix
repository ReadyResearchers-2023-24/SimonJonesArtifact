{
  description = "SimonJonesArtifact";
  inputs.nixpkgs.url = "github:nixos/nixpkgs/nixpkgs-unstable";
  outputs = { self, nixpkgs }:
    let
        pkgs = import nixpkgs { system = "x86_64-linux"; };
    in
    {
      devShells.x86_64-linux.default = pkgs.mkShell {
        packages = [
          pkgs.blender
          pkgs.meshlab
          pkgs.sshfs
        ];
      };
  };
}
