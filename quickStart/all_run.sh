#!/bin/bash
# 2024.3.13 Created by ldq、zh
# 读取命令行参数
para_s=0
para_r=0
para_k=0
para_m=0
para_q=0
while [[ $# -gt 0 ]];do # $#表示参数个数, -gt表示大于
  key=${1}
  case ${key} in
    -s|--nav_service)
    para_s=1
      shift 1
      ;;
    -r|--real)
    para_r=1
      shift 1
      ;;
    -k|--key_test)
    para_k=1
      shift 1
      ;;
    -m|--edit_map)
    para_m=1
      shift 1
      ;;
    -q|--qingqing)
    para_q=1
      shift 1
      ;;
    *)
      shift
      ;;
  esac
done


# 启动实车环境
if [ ${para_r} -eq 1 ];
then # 启动实车环境
    gnome-terminal --tab --title="real_robot"  -- bash -c "wmctrl -r :ACTIVE: -b toggle,above; 
    source /opt/ros/$ROS_DISTRO/setup.bash;
    source devel/setup.bash;
    roslaunch nav_real_start nav_real_start.launch;exec bash"

    sleep 1.5s
    gnome-terminal --tab --title="relocalization"  -- bash -c "wmctrl -r :ACTIVE: -b toggle,above;
    source /opt/ros/$ROS_DISTRO/setup.bash;
    source devel/setup.bash;
    roslaunch relocalization relocalization.launch;exec bash"
# 启动仿真环境
else
    if [ ${para_m} -eq 1 ];
    then 
      gnome-terminal --tab --title="先验图仿真环境"  -- bash -c "wmctrl -r :ACTIVE: -b toggle,above;
      source /opt/ros/$ROS_DISTRO/setup.bash;
      source devel/setup.bash;
      roslaunch sentry_gazebo startup_robot_map.launch; exec bash"
    else
        if [ ${para_k} -eq 1 ];
        then
          gnome-terminal --tab --title="仿真环境"  -- bash -c "wmctrl -r :ACTIVE: -b toggle,above;
          source /opt/ros/$ROS_DISTRO/setup.bash;
          source devel/setup.bash;
          roslaunch sentry_gazebo startup_robot_rm2024.launch; exec bash"
        else
          gnome-terminal --tab --title="仿真环境"  -- bash -c "wmctrl -r :ACTIVE: -b toggle,above;
          source /opt/ros/$ROS_DISTRO/setup.bash;
          source devel/setup.bash;
          roslaunch sentry_gazebo startup_robot.launch; exec bash"
        fi
    fi
  
    sleep 2s


    gnome-terminal --tab --title="局部规划控制系统"  -- bash -c "
    source /opt/ros/noetic/setup.bash;
    source devel/setup.bash;
    roslaunch waypoint_control waypoint_control.launch; exec bash"

    if [ ${para_s} -eq 1 ];
    then
        gnome-terminal --tab --title="nav_service"  -- bash -c "wmctrl -r :ACTIVE: -b toggle,above;
        source /opt/ros/$ROS_DISTRO/setup.bash;
        source devel/setup.bash;
        roslaunch nav_service nav_service.launch; exec bash"
    fi

fi

echo "done!"