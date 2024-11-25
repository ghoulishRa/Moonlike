docker run --rm -it --network=host --privileged \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --device=/dev/bus/usb:/dev/bus/usb \
  -v $(pwd)/ros2_ws:/root/ros2_ws \
  -v $(pwd)/rosbag:/root/rosbag \
  -v $(pwd)/ros1_bridge:/root/ros1_bridge \
  ros2_humble_updated 
