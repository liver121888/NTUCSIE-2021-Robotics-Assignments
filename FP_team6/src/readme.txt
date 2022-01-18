sudo apt update
sudo apt install ros-melodic-libuvc
sudo apt install python3-opencv
catkin_make --only-pkg-with-deps riki_msgs
catkin_make --only-pkg-with-deps exploration_msgs
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
