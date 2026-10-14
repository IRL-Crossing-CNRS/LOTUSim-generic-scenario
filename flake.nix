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

        # pymavlink drives PX4 offboard patrol; pillow is only reached when a
        # detection task handles a frame.
        scenarioPython = with pkgs.python3Packages; [
          pymavlink
          numpy
          pillow
        ];

        # The detection tasks open a window per camera (cv2.imshow). nixpkgs
        # builds OpenCV without any GUI backend, and its GTK or Qt variants are
        # in no binary cache, so they would compile on every machine. The
        # opencv-python wheel from PyPI carries its own Qt and is used instead,
        # first on PYTHONPATH so that it also shadows the cv2 ultralytics
        # brings in.
        opencvGui = pkgs.stdenv.mkDerivation {
          pname = "opencv-python-wheel";
          version = "4.14.0.94";
          src = pkgs.fetchurl {
            x86_64-linux = {
              url = "https://files.pythonhosted.org/packages/13/09/c05887c383caa9080ef612febffd08c5da1c283418687a3bea6ef7fb6faa/opencv_python-4.14.0.94-cp37-abi3-manylinux2014_x86_64.manylinux_2_17_x86_64.whl";
              sha256 = "4ae0b5a2acbcfb9813035f1644cfba751edd02797fac994e32180813eeef8420";
            };
            aarch64-linux = {
              url = "https://files.pythonhosted.org/packages/84/ec/5777752567986b99222d52e22306e9616829480cc2d48ab241daa9d7e9ef/opencv_python-4.14.0.94-cp37-abi3-manylinux2014_aarch64.manylinux_2_17_aarch64.whl";
              sha256 = "3198fcbcb8f98fd68063bf2c75d129b0f50d46e555a4e47578a6eb086eb38e1f";
            };
          }.${system};
          nativeBuildInputs = [ pkgs.unzip pkgs.autoPatchelfHook ];
          buildInputs = with pkgs; [
            stdenv.cc.cc.lib
            glib
            libGL
            zlib
            libx11
            libxcb
            libxext
            libice
            libsm
          ];
          unpackPhase = "unzip -q $src";
          installPhase = ''
            site=$out/${pkgs.python3.sitePackages}
            mkdir -p $site
            cp -r cv2 opencv_python.libs $site/
          '';
        };

        # ultralytics (with torch) runs the YOLO crack and corrosion detection
        # of the fault_inspection and crack_detector tasks. It is put on
        # PYTHONPATH rather than in the shell's packages: as a package, torch's
        # -dev output joins the C++ include path and its bundled protobuf
        # headers break the gz_ros2_bridge build.
        #
        # The default devshell carries nixpkgs' CPU torch (about 1 GB). The
        # "gpu" devshell carries NVIDIA's CUDA build (torch-bin, unfree, CUDA
        # 13, about 7 GB); the installer asks which one to use.
        cpuDetectionEnv = pkgs.python3.withPackages (ps: [ ps.ultralytics ]);

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
        gpuDetectionEnv = detectionPython.withPackages (ps: [ ps.ultralytics ]);

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

        scenarioShell = { detectionEnv, hook ? "", banner }: pkgs.mkShell {
          inputsFrom = [ coreShell ];
          # jq: scenario_launch.sh reads the scenario JSON with it.
          packages = scenarioPython ++ [ pkgs.jq ];
          shellHook = ''
            export PYTHONPATH="${opencvGui}/${pkgs.python3.sitePackages}:${detectionEnv}/${pkgs.python3.sitePackages}''${PYTHONPATH:+:$PYTHONPATH}"
            ${hook}
            echo "${banner}"
          '';
        };
      in {
        devShells = {
          default = scenarioShell {
            detectionEnv = cpuDetectionEnv;
            banner = "LOTUSim generic scenario — core devshell plus the agents' Python packages (detection on the CPU)";
          };
        } // pkgs.lib.optionalAttrs pkgs.stdenv.hostPlatform.isLinux {
          # CUDA, and NVIDIA's wheels, exist for Linux only.
          gpu = scenarioShell {
            detectionEnv = gpuDetectionEnv;
            hook = hostCudaHook;
            banner = "LOTUSim generic scenario — core devshell plus the agents' Python packages (detection on the GPU)";
          };
        };
      });
}
