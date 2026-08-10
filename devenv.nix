{ pkgs, lib, config, inputs, ... }:

let
  # 1. Overlay nix-ros-overlay onto nixpkgs
  pkgsWithRos = import inputs.nixpkgs {
    # Evaluate the ROS environment for the active host, including GitHub's
    # Ubuntu and Apple Silicon runners.
    system = pkgs.stdenv.hostPlatform.system;
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
            ignoreDeprecatedRclcpp = package: package.overrideAttrs (old: {
              NIX_CFLAGS_COMPILE = lib.replaceStrings
                [ "-Wno-error=maybe-uninitialized" ] [ "" ]
                (old.NIX_CFLAGS_COMPILE or "")
                + " -Wno-error=deprecated-declarations -Wno-error=inconsistent-missing-override";
            });
            # The upstream nav2-bringup package propagates the navigation2
            # meta-package, which pulls RViz into this headless workspace.
            # Keep the same launch/configuration package name, but expose only
            # the Nav2 servers and plugins used by xuegecar_navigation2.
            nav2-bringup = rosPrev.buildRosPackage {
              pname = "ros-humble-nav2-bringup";
              version = "1.1.20-r1";
              src = final.fetchurl {
                url = "https://github.com/SteveMacenski/navigation2-release/archive/release/humble/nav2_bringup/1.1.20-1.tar.gz";
                name = "1.1.20-1.tar.gz";
                sha256 = "6a1ce6e390fd10f391b1e2644ae1c3fddd388fd7ae652670fd77e691e6b2500f";
              };
              buildType = "ament_cmake";
              buildInputs = [ rosFinal.ament-cmake ];
              # navigation2 is a meta-package that only pulls RViz into this
              # headless build. nav2-bringup does not link against it.
              postPatch = ''
                substituteInPlace CMakeLists.txt \
                  --replace-fail "find_package(navigation2 REQUIRED)" ""
              '';
              propagatedBuildInputs = [
                rosFinal.launch-ros
                rosFinal.nav2-amcl
                rosFinal.nav2-behavior-tree
                rosFinal.nav2-behaviors
                rosFinal.nav2-bt-navigator
                rosFinal.nav2-controller
                rosFinal.nav2-costmap-2d
                rosFinal.nav2-dwb-controller
                rosFinal.nav2-lifecycle-manager
                rosFinal.nav2-map-server
                rosFinal.nav2-navfn-planner
                rosFinal.nav2-planner
                rosFinal.nav2-smoother
                rosFinal.nav2-velocity-smoother
                rosFinal.nav2-waypoint-follower
              ];
              nativeBuildInputs = [ rosFinal.ament-cmake ];
            };
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
              NIX_CFLAGS_COMPILE = (old.NIX_CFLAGS_COMPILE or "")
                + " -Wno-error=deprecated-declarations -Wno-error=inconsistent-missing-override";
              postPatch = (old.postPatch or "") + ''
                substituteInPlace include/nav2_util/odometry_utils.hpp src/odometry_utils.cpp \
                  --replace-warn 'nav_msgs::msg::Odometry::SharedPtr msg' \
                  'nav_msgs::msg::Odometry::ConstSharedPtr msg'
              '';
            });
            nav2-velocity-smoother = rosPrev.nav2-velocity-smoother.overrideAttrs (old: {
              NIX_CFLAGS_COMPILE = (old.NIX_CFLAGS_COMPILE or "")
                + " -Wno-error=deprecated-declarations -Wno-error=inconsistent-missing-override";
              postPatch = (old.postPatch or "") + ''
                substituteInPlace include/nav2_velocity_smoother/velocity_smoother.hpp src/velocity_smoother.cpp \
                  --replace-warn 'geometry_msgs::msg::Twist::SharedPtr msg' \
                  'geometry_msgs::msg::Twist::ConstSharedPtr msg'
                substituteInPlace src/velocity_smoother.cpp \
                  --replace-warn 'command_ = msg;' \
                  'command_ = std::make_shared<geometry_msgs::msg::Twist>(*msg);'
              '';
            });
            nav2-amcl = rosFinal.ignoreDeprecatedRclcpp rosPrev.nav2-amcl;
            nav2-behavior-tree = rosFinal.ignoreDeprecatedRclcpp rosPrev.nav2-behavior-tree;
            nav2-behaviors = rosFinal.ignoreDeprecatedRclcpp rosPrev.nav2-behaviors;
            nav2-bt-navigator = rosFinal.ignoreDeprecatedRclcpp rosPrev.nav2-bt-navigator;
            nav2-controller = rosFinal.ignoreDeprecatedRclcpp rosPrev.nav2-controller;
            nav2-costmap-2d = rosFinal.ignoreDeprecatedRclcpp rosPrev.nav2-costmap-2d;
            nav2-dwb-controller = rosFinal.ignoreDeprecatedRclcpp rosPrev.nav2-dwb-controller;
            dwb-core = rosFinal.ignoreDeprecatedRclcpp rosPrev.dwb-core;
            dwb-critics = rosFinal.ignoreDeprecatedRclcpp rosPrev.dwb-critics;
            dwb-plugins = rosFinal.ignoreDeprecatedRclcpp rosPrev.dwb-plugins;
            nav2-lifecycle-manager = rosFinal.ignoreDeprecatedRclcpp rosPrev.nav2-lifecycle-manager;
            nav2-map-server = rosPrev.nav2-map-server.overrideAttrs (old: {
              NIX_CFLAGS_COMPILE = (old.NIX_CFLAGS_COMPILE or "")
                + " -Wno-error=deprecated-declarations -Wno-error=inconsistent-missing-override";
              postPatch = (old.postPatch or "") + ''
                substituteInPlace src/map_saver/map_saver.cpp \
                  --replace-fail 'nav_msgs::msg::OccupancyGrid::SharedPtr' \
                  'nav_msgs::msg::OccupancyGrid::ConstSharedPtr'
              '';
            });
            nav2-navfn-planner = rosFinal.ignoreDeprecatedRclcpp rosPrev.nav2-navfn-planner;
            nav2-planner = rosFinal.ignoreDeprecatedRclcpp rosPrev.nav2-planner;
            nav2-smoother = rosFinal.ignoreDeprecatedRclcpp rosPrev.nav2-smoother;
            nav2-waypoint-follower = rosFinal.ignoreDeprecatedRclcpp rosPrev.nav2-waypoint-follower;
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
    # xuegecar_navigation2 includes Nav2's headless bringup launch file.
    rosDistro.nav2-bringup
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
    echo "ROS 2 (Humble) + Nix Devenv activated on $(uname -sm)"
    export ROS_DOMAIN_ID=0

    # These hardware drivers use Linux-only kernel and pthread APIs.
    if [ "$(uname -s)" = "Darwin" ]; then
      export COLCON_DEFAULTS_FILE="$DEVENV_ROOT/colcon-defaults.yaml"
    fi

    # Auto-source a complete local colcon workspace. An interrupted build can
    # leave package metadata behind without its local_setup.bash script.
    if [ -f install/setup.bash ]; then
      if find install -path '*/share/colcon-core/packages/*' -type f \
        -exec sh -c '
          for metadata in "$@"; do
            prefix="''${metadata%/share/colcon-core/packages/*}"
            [ -f "$prefix/local_setup.bash" ] || exit 1
          done
        ' sh {} +; then
        source install/setup.bash
      else
        echo "Loading complete local packages; run colcon build to refresh incomplete ones."
        for package_setup in install/*/share/*/package.bash; do
          package_dir="''${package_setup%/package.bash}"
          if [ -f "$package_setup" ] \
            && { ! grep -q 'local_setup\.sh' "$package_setup" || [ -f "$package_dir/local_setup.sh" ]; } \
            && { ! grep -q 'local_setup\.bash' "$package_setup" || [ -f "$package_dir/local_setup.bash" ]; }; then
            source "$package_setup"
          fi
        done
      fi
    fi

    # Python ROS packages are not always exposed through the overlay's ament
    # dependency closure. Keep their resources available after the local
    # workspace setup script has established the rest of the overlay paths.
    export AMENT_PREFIX_PATH="${rosDistro.launch}:${rosDistro.launch-xml}:${rosDistro.nav2-bringup}:${rosDistro.rqt-gui}:${rosDistro.joint-state-publisher}:${rosDistro.teleop-twist-keyboard}:${rosDistro.tf2-tools}:${rosDistro.rmw-fastrtps-cpp}:$AMENT_PREFIX_PATH"
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

  # `devenv up` runs the hardware-facing foundation only. Mapping, navigation,
  # and obstacle avoidance are intentionally separate because they conflict on
  # algorithm selection and /cmd_vel ownership.
  processes = {
    micro-ros-agent.exec = "micro-ros-agent";
    camera.exec = "ros2 launch xuegecar_camera http_video_publisher.launch.py";
    bringup.exec = "ros2 launch xuegecar_bringup xuegecar_bringup.launch.py";
    scan-timestamp-relay.exec = "ros2 launch scan_timestamp_relay scan_timestamp_relay.launch.py";
    foxglove.exec = "ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765";
  };
}
