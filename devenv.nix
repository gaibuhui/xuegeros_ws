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
        # nix-ros-overlay still defines this legacy variant by passing a
        # removed `qt6` argument to PCL.  PCL now directly uses Qt5.
        pclWithQt5 = final.pcl;
        rosPackages = prev.rosPackages // {
          humble = prev.rosPackages.humble.overrideScope (rosFinal: rosPrev: {
            # nix-ros-overlay omits the Apple Silicon SDK artifact even though
            # Foxglove publishes one for the SDK version used by Humble.
            foxglove-bridge = rosPrev.foxglove-bridge.overrideAttrs (_: let
              sdk = final.fetchurl {
                url = "https://github.com/foxglove/foxglove-sdk/releases/download/sdk%2Fv0.25.3/foxglove-v0.25.3-cpp-aarch64-apple-darwin.zip";
                hash = "sha256-TYsfJyYOBYu4CxnaVS375tMfLQqZYmNVNTbuw49NdDQ=";
              };
            in {
              postPatch = ''
                substituteInPlace CMakeLists.txt --replace-fail \
                  'else()
  message(FATAL_ERROR "Unsupported platform/architecture combination: ''${CMAKE_SYSTEM_PROCESSOR}-''${CMAKE_SYSTEM_NAME}")' \
                  'elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
  set(FOXGLOVE_SDK_PLATFORM "aarch64-apple-darwin")
  set(FOXGLOVE_SDK_SHA "4d8b1f27260e058bb80b19da552dfbe6d31f2d0a996263553536eec38f4d7434")
else()
  message(FATAL_ERROR "Unsupported platform/architecture combination: ''${CMAKE_SYSTEM_PROCESSOR}-''${CMAKE_SYSTEM_NAME}")'
                substituteInPlace CMakeLists.txt --replace-fail \
                  'URL ''${FOXGLOVE_SDK_URL}' \
                  'URL ${sdk}'
                substituteInPlace src/message_definition_cache.cpp --replace-fail \
                  '#include <fstream>' \
                  '#include <fstream>
#include <map>'
                substituteInPlace src/ros2_foxglove_bridge.cpp --replace-warn '%lu' '%llu'
              '';
            });
            # Nav2 1.1.20 still uses rclcpp's deprecated mutable-message
            # subscription callback.  Newer Humble rclcpp headers turn that
            # diagnostic into a build failure under Nav2's -Werror policy.
            nav2-util = rosPrev.nav2-util.overrideAttrs (old: {
              postPatch = (old.postPatch or "") + ''
                substituteInPlace include/nav2_util/odometry_utils.hpp src/odometry_utils.cpp \
                  --replace-warn 'nav_msgs::msg::Odometry::SharedPtr msg' \
                  'nav_msgs::msg::Odometry::ConstSharedPtr msg'
              '';
            });
            nav2-map-server = rosPrev.nav2-map-server.overrideAttrs (old: {
              postPatch = (old.postPatch or "") + ''
                substituteInPlace src/map_saver/map_saver.cpp \
                  --replace-fail 'nav_msgs::msg::OccupancyGrid::SharedPtr' \
                  'nav_msgs::msg::OccupancyGrid::ConstSharedPtr'
              '';
            });
          });
        };
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
    rosDistro.rmw-fastrtps-cpp
    rosDistro.tf2
    rosDistro.tf2-ros
    rosDistro.tf2-tools
    rosDistro.tf2-geometry-msgs
    rosDistro.demo-nodes-cpp
    rosDistro.demo-nodes-py
    rosDistro.teleop-twist-keyboard
    rosDistro.nav2-map-server
    # xuegecar_cartographer launches cartographer_node and the occupancy-grid
    # node from this package; the core library is a direct build dependency.
    rosDistro.cartographer
    rosDistro.cartographer-ros
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
    # The bridge's XML launcher resolves these through the ament index.
    rosDistro.launch
    rosDistro.launch-xml
    rosDistro.foxglove-bridge
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
    export AMENT_PREFIX_PATH="${rosDistro.launch}:${rosDistro.launch-xml}:${rosDistro.rqt-gui}:${rosDistro.joint-state-publisher}:${rosDistro.teleop-twist-keyboard}:${rosDistro.tf2-tools}:${rosDistro.rmw-fastrtps-cpp}:$AMENT_PREFIX_PATH"
  '';

  scripts.micro-ros-agent = {
    description = "Start the native micro-ROS Agent with UDP4 on port 8888";
    exec = ''
      source "$DEVENV_ROOT/install/setup.bash"
      exec "$DEVENV_ROOT/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent" \
        udp4 --port "''${MICRO_ROS_AGENT_PORT:-8888}" "$@"
    '';
  };

  # devenv does not retain DYLD_* exports from enterShell when it executes a
  # command. Launch ROS through a wrapper so its children inherit libyaml.
  scripts.ros2 = {
    description = "Run ROS 2 with Darwin runtime libraries available";
    exec = ''
      export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      export DYLD_LIBRARY_PATH="${rosDistro."libyaml-vendor"}/lib:${rosDistro.robot-state-publisher}/lib:${rosDistro.rmw-fastrtps-cpp}/lib:${rosDistro.rmw-dds-common}/lib:${rosDistro.rcl-interfaces}/lib:${rosDistro.sensor-msgs}/lib:${rosDistro.rosidl-typesupport-fastrtps-cpp}/lib:${rosDistro.rosidl-typesupport-fastrtps-c}/lib:${rosDistro.fastcdr}/lib:${rosDistro.fastrtps}/lib:''${DYLD_LIBRARY_PATH:-}"
      exec ${rosDistro.ros2cli}/bin/ros2 "$@"
    '';
  };
}
