# The scenario workspace's environment: the core's devshell plus the Python
# packages the agents import. Both workspaces must be built and run in the same
# environment — a build against another ROS 2 produces different message
# definitions, and nothing is ever delivered between the two.
{
  description = "LOTUSim generic scenario — ROS 2 workspace environment";

  inputs = {
    lotusim.url = "github:naval-group/LOTUSim";
    nixpkgs.follows = "lotusim/nixpkgs";
    flake-utils.follows = "lotusim/flake-utils";
  };

  outputs = { self, lotusim, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
        coreShell = lotusim.devShells.${system}.default;

        # pymavlink drives PX4 offboard patrol; opencv and pillow are only
        # reached when a detection task handles a frame.
        scenarioPython = with pkgs.python3Packages; [
          pymavlink
          numpy
          opencv4
          pillow
        ];
      in {
        devShells.default = pkgs.mkShell {
          inputsFrom = [ coreShell ];
          packages = scenarioPython;
          shellHook = ''
            echo "LOTUSim generic scenario — core devshell plus the agents' Python packages"
          '';
        };
      });
}
