#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/ubuntu/Documents/INF3995-102/Limo/ros-packages/packages/src/xacro"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/ubuntu/Documents/INF3995-102/Limo/ros-packages/packages/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/ubuntu/Documents/INF3995-102/Limo/ros-packages/packages/install/lib/python3/dist-packages:/home/ubuntu/Documents/INF3995-102/Limo/ros-packages/packages/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ubuntu/Documents/INF3995-102/Limo/ros-packages/packages/build" \
    "/usr/bin/python3" \
    "/home/ubuntu/Documents/INF3995-102/Limo/ros-packages/packages/src/xacro/setup.py" \
    egg_info --egg-base /home/ubuntu/Documents/INF3995-102/Limo/ros-packages/packages/build/xacro \
    build --build-base "/home/ubuntu/Documents/INF3995-102/Limo/ros-packages/packages/build/xacro" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/ubuntu/Documents/INF3995-102/Limo/ros-packages/packages/install" --install-scripts="/home/ubuntu/Documents/INF3995-102/Limo/ros-packages/packages/install/bin"
