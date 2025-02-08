cmake_minimum_required(VERSION 3.10)

project(websockets)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
)

catkin_python_setup()

# 添加安装 setup.py 的步骤
execute_process(
    COMMAND ${PYTHON_EXECUTABLE} setup.py install --prefix ${CATKIN_DEVEL_PREFIX}
)