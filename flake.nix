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

        # ultralytics (with torch) runs the YOLO crack and corrosion detection
        # of the fault_inspection and crack_detector tasks, on the GPU. torch
        # is NVIDIA's CUDA build (torch-bin, unfree), which needs CUDA 13. It
        # is put on PYTHONPATH rather than in the shell's packages: as a
        # package, torch's -dev output joins the C++ include path and its
        # bundled protobuf headers break the gz_ros2_bridge build.
        # nixpkgs compiles NCCL and NVSHMEM from source, and no binary cache
        # holds them for this nixpkgs: the build takes an hour and its nvcc
        # jobs exhaust 32 GB of RAM. NVIDIA's prebuilt wheels of the same
        # versions are used instead.
        nvidiaWheel = { pkgs, name, version, hashes }:
          pkgs.stdenv.mkDerivation {
            pname = "${name}-wheel";
            inherit version;
            src = pkgs.fetchurl hashes.${system};
            nativeBuildInputs = [ pkgs.unzip pkgs.autoPatchelfHook ];
            buildInputs = [ pkgs.stdenv.cc.cc.lib ];
            # libcuda comes from the host driver at run time; the multi-node
            # transports (MPI, UCX, InfiniBand) are never loaded here.
            autoPatchelfIgnoreMissingDeps = [ "*" ];
            unpackPhase = "unzip -q $src";
            installPhase = ''
              mkdir -p $out
              cp -r nvidia/${name}/lib $out/lib
              if [ -d nvidia/${name}/include ]; then cp -r nvidia/${name}/include $out/include; fi
            '';
          };
        pkgsCuda = import nixpkgs {
          inherit system;
          config.allowUnfree = true;
          overlays = [
            (final: prev: {
              cudaPackages = final.cudaPackages_13.overrideScope (cfinal: cprev: {
                nccl = nvidiaWheel {
                  pkgs = final;
                  name = "nccl";
                  version = "2.30.7";
                  hashes = {
                    x86_64-linux = {
                      url = "https://files.pythonhosted.org/packages/3f/34/c500f90c7ae641b8e0f98965b36b8a7ac79cc8b296e8d251fe3eb592ee54/nvidia_nccl_cu13-2.30.7-py3-none-manylinux_2_18_x86_64.whl";
                      sha256 = "cefa7fdb9710efd0f39c5f1be1d61ff6fc9a996c451265bd7fbdcf9455ed4b50";
                    };
                    aarch64-linux = {
                      url = "https://files.pythonhosted.org/packages/d1/21/a73174c6157101bdf1ffc22b517f76ff0082613989dd9bc8f43e8034caac/nvidia_nccl_cu13-2.30.7-py3-none-manylinux_2_18_aarch64.whl";
                      sha256 = "ca786ffa5a647c75d4d1f5cc72a6c4f537947e2ba8823d7c8aaf768e7a7b9f77";
                    };
                  };
                };
                libnvshmem = nvidiaWheel {
                  pkgs = final;
                  name = "nvshmem";
                  version = "3.6.5";
                  hashes = {
                    x86_64-linux = {
                      url = "https://files.pythonhosted.org/packages/5d/7b/2ab033584a3339552472ac8d79543c503a0e06dd0d082448b06697e7f716/nvidia_nvshmem_cu13-3.6.5-py3-none-manylinux2014_x86_64.manylinux_2_17_x86_64.whl";
                      sha256 = "4001aabc72ead32ecc3c9add3c6781befcb71adcbe286d7f5956042e68668c70";
                    };
                    aarch64-linux = {
                      url = "https://files.pythonhosted.org/packages/83/6b/4c642d2cce57c8bd32043b4a608b6acc8cd0579c2f53af9a7ef9e1e1ccca/nvidia_nvshmem_cu13-3.6.5-py3-none-manylinux2014_aarch64.manylinux_2_17_aarch64.whl";
                      sha256 = "94aa0149490be658cf95945ce9f16ba90a90edbd6a564366f996ead7496f2f22";
                    };
                  };
                };
              });
            })
          ];
        };
        detectionPython = pkgsCuda.python3.override {
          packageOverrides = self: super: {
            torch = super.torch-bin;
            torchvision = super.torchvision-bin;
          };
        };
        detectionEnv = detectionPython.withPackages (ps: [ ps.ultralytics ]);
        detectionPythonPath = "${detectionEnv}/${detectionPython.sitePackages}";

        # torch loads the NVIDIA driver's libcuda from the host. NixOS has it
        # in /run/opengl-driver/lib; elsewhere only the driver's libraries are
        # linked into a directory of their own, so the rest of the host's
        # /usr/lib never reaches the nix programs.
        hostCudaHook = ''
          if [ -d /run/opengl-driver/lib ]; then
            export LD_LIBRARY_PATH="/run/opengl-driver/lib''${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
          else
            host_cuda="''${XDG_CACHE_HOME:-$HOME/.cache}/lotusim/host-cuda"
            mkdir -p "$host_cuda"
            rm -f "$host_cuda"/*
            for lib in /usr/lib/x86_64-linux-gnu /usr/lib64 /usr/lib/aarch64-linux-gnu; do
              for f in "$lib"/libcuda.so* "$lib"/libnvidia-ml.so* "$lib"/libnvidia-ptxjitcompiler.so* "$lib"/libnvidia-nvvm.so*; do
                if [ -e "$f" ]; then ln -sf "$f" "$host_cuda/"; fi
              done
            done
            export LD_LIBRARY_PATH="$host_cuda''${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
          fi
        '';
      in {
        devShells.default = pkgs.mkShell {
          inputsFrom = [ coreShell ];
          packages = scenarioPython;
          shellHook = ''
            export PYTHONPATH="${detectionPythonPath}''${PYTHONPATH:+:$PYTHONPATH}"
            ${hostCudaHook}
            echo "LOTUSim generic scenario — core devshell plus the agents' Python packages"
          '';
        };
      });
}
