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

echo_and_run cd "/home/administrator/git_ws/AuE893_SP21_KartikLoya/catkin_ws/src/vision_opencv/image_geometry"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/administrator/git_ws/AuE893_SP21_KartikLoya/catkin_ws/install_isolated/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/administrator/git_ws/AuE893_SP21_KartikLoya/catkin_ws/install_isolated/lib/python3/dist-packages:/home/administrator/git_ws/AuE893_SP21_KartikLoya/catkin_ws/build_isolated/image_geometry/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/administrator/git_ws/AuE893_SP21_KartikLoya/catkin_ws/build_isolated/image_geometry" \
    "/usr/bin/python3" \
    "/home/administrator/git_ws/AuE893_SP21_KartikLoya/catkin_ws/src/vision_opencv/image_geometry/setup.py" \
    egg_info --egg-base /home/administrator/git_ws/AuE893_SP21_KartikLoya/catkin_ws/build_isolated/image_geometry \
    build --build-base "/home/administrator/git_ws/AuE893_SP21_KartikLoya/catkin_ws/build_isolated/image_geometry" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/administrator/git_ws/AuE893_SP21_KartikLoya/catkin_ws/install_isolated" --install-scripts="/home/administrator/git_ws/AuE893_SP21_KartikLoya/catkin_ws/install_isolated/bin"