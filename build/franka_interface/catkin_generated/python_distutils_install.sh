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

echo_and_run cd "/home/seungbeom/franka_ws/src/franka_ros_interface/franka_interface"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/seungbeom/franka_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/seungbeom/franka_ws/install/lib/python2.7/dist-packages:/home/seungbeom/franka_ws/build/franka_interface/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/seungbeom/franka_ws/build/franka_interface" \
    "/usr/bin/python2" \
    "/home/seungbeom/franka_ws/src/franka_ros_interface/franka_interface/setup.py" \
     \
    build --build-base "/home/seungbeom/franka_ws/build/franka_interface" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/seungbeom/franka_ws/install" --install-scripts="/home/seungbeom/franka_ws/install/bin"
