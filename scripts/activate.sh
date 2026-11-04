# Sourced by pixi on activation.
# gz-transport multicast discovery fails on macOS ("No route to host"); keep it on loopback.
if [ "$(uname -s)" = "Darwin" ] && [ -z "${GZ_IP:-}" ]; then
  export GZ_IP=127.0.0.1
fi
# Make the colcon install space visible if it exists.
if [ -f "$PIXI_PROJECT_ROOT/install/setup.sh" ]; then
  # shellcheck disable=SC1091
  . "$PIXI_PROJECT_ROOT/install/setup.sh"
fi
