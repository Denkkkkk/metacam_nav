#!/bin/bash
# 2024.3.13 Created by ldq、zh
usage() {
  echo "Usage: ${0} [-s|--nav_service] [-r|--real] [-k|--key_test] [-m|--edit_map] [-q|--qingqing] " 1>&2
  exit 1 
}
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
      usage
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
# 启动仿真环境
else 
    gnome-terminal --tab --title="仿真环境"  -- bash -c "wmctrl -r :ACTIVE: -b toggle,above;
    source /opt/ros/$ROS_DISTRO/setup.bash;
    source devel/setup.bash;
    roslaunch sentry_gazebo startup_robot_map.launch; exec bash"

    sleep 2s

    if [ ${para_s} -eq 1 ];
    then
        gnome-terminal --tab --title="nav_service"  -- bash -c "wmctrl -r :ACTIVE: -b toggle,above;
        source /opt/ros/$ROS_DISTRO/setup.bash;
        source devel/setup.bash;
        roslaunch nav_service nav_service.launch; exec bash"
    fi
fi

echo "done!"