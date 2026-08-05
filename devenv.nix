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
        # gnome-icon-theme does not create share/locale on Darwin, but its
        # nixpkgs postInstall hook still removes it unconditionally.
        gnome-icon-theme = prev.gnome-icon-theme.overrideAttrs (old: {
          postInstall = if final.stdenv.hostPlatform.isDarwin then ''
            rm -rf "$out/share/locale"
          '' else old.postInstall;
          meta = old.meta // { broken = false; };
        });
      })
    ];
    config = {
      allowUnfree = true;
      # rqt_image_view pulls in the data-only tango-icon-theme, which Nix
      # incorrectly restricts to Linux despite being usable on macOS.
      allowUnsupportedSystem = true;
    };
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
    libyaml
    python312Packages.numpy
    python312Packages.opencv-python

    # ROS 2 Core Packages
    rosDistro.ros-core
    rosDistro.rclcpp
    rosDistro.rclpy
    rosDistro.std-msgs
    rosDistro.geometry-msgs
    rosDistro.nav-msgs
    rosDistro.sensor-msgs
    rosDistro.visualization-msgs
    rosDistro.std-srvs
    rosDistro.rmw
    rosDistro.tf2
    rosDistro.tf2-ros
    rosDistro.tf2-geometry-msgs
    rosDistro.demo-nodes-cpp
    rosDistro.demo-nodes-py
    # Required by xuegecar_bringup.launch.py to publish the robot link tree.
    rosDistro.joint-state-publisher
    rosDistro.robot-state-publisher
    # Camera nodes publish `sensor_msgs/CompressedImage`; this package
    # registers image_transport/compressed_{pub,sub} with pluginlib.
    rosDistro.compressed-image-transport
    # rqt_image_view imports rqt_gui at runtime and resolves it through the
    # ament package index, so it must be a direct shell dependency.
    rosDistro.rqt-gui
    rosDistro.rqt-image-view
    qt5.full

    # Native micro-ROS Agent build dependencies.
    rosDistro.fastcdr
    rosDistro.fastrtps
    rosDistro.fastrtps-cmake-module
    rosDistro.rmw-dds-common
    rosDistro.rmw-fastrtps-shared-cpp
    rosDistro.rosidl-typesupport-fastrtps-cpp
  ] ++ lib.optionals pkgs.stdenv.isDarwin [
    # Add native Apple frameworks if compiling custom C++ nodes
    apple-sdk_15
  ];

  enterShell = ''
    echo "⚡ ROS 2 (Humble) + Nix Devenv activated on M3 Apple Silicon"
    export ROS_DOMAIN_ID=0

    # These hardware drivers use Linux-only kernel and pthread APIs.
    if [ "$(uname -s)" = "Darwin" ]; then
      export COLCON_DEFAULTS_FILE="$DEVENV_ROOT/xuegeros_ws/colcon-defaults.yaml"
    fi

    # Auto-source colcon workspace if built locally
    if [ -f install/setup.bash ]; then
      source install/setup.bash
    fi

    # Python ROS packages are not always exposed through the overlay's ament
    # dependency closure. Keep their resources available after the local
    # workspace setup script has established the rest of the overlay paths.
    export AMENT_PREFIX_PATH="${rosDistro.rqt-gui}:${rosDistro.joint-state-publisher}:$AMENT_PREFIX_PATH"
  '';

  scripts.micro-ros-agent = {
    description = "Start the native micro-ROS Agent with UDP4 on port 8888";
    exec = ''
      source "$DEVENV_ROOT/install/setup.bash"
      exec "$DEVENV_ROOT/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent" \
        udp4 --port "''${MICRO_ROS_AGENT_PORT:-8888}" "$@"
    '';
  };
}
