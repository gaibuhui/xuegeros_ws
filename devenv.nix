{ pkgs, lib, config, inputs, ... }:

let
  # 1. Overlay nix-ros-overlay onto nixpkgs
  pkgsWithRos = import inputs.nixpkgs {
    system = "aarch64-darwin"; # Explicitly set for M-series Apple Silicon
    overlays = [
      inputs.nix-ros-overlay.overlays.default
      (final: prev: {
        pythonPackagesExtensions = prev.pythonPackagesExtensions ++ [
          (python-final: python-prev: {
            # colcon-ros mixes two setuptools versions through its plugins.
            colcon-ros = python-prev.colcon-ros.overridePythonAttrs (_: {
              catchConflicts = false;
            });
          })
        ];
      })
    ];
    config.allowUnfree = true;
  };

  # 2. Select your target ROS 2 distro (e.g., jazzy or humble)
  rosDistro = pkgsWithRos.rosPackages.humble;
in
{
  # Set Apple SDK frameworks for macOS building
  apple.sdk = if pkgs.stdenv.isDarwin then pkgs.apple-sdk_15 else null;

  packages = with pkgsWithRos; [
    # Build tooling
    colcon
    cmake
    pkg-config

    # ROS 2 Core Packages
    rosDistro.ros-core
    rosDistro.rclcpp
    rosDistro.rclpy
    rosDistro.std-msgs
    rosDistro.geometry-msgs
    rosDistro.demo-nodes-cpp
    rosDistro.demo-nodes-py
  ] ++ lib.optionals pkgs.stdenv.isDarwin [
    # Add native Apple frameworks if compiling custom C++ nodes
    apple-sdk_15
  ];

  enterShell = ''
    echo "⚡ ROS 2 (Humble) + Nix Devenv activated on M3 Apple Silicon"
    export ROS_DOMAIN_ID=42

    # Auto-source colcon workspace if built locally
    if [ -f install/setup.zsh ]; then
      source install/setup.zsh
    elif [ -f install/setup.bash ]; then
      source install/setup.bash
    fi
  '';
}
