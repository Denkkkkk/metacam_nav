#!/bin/bash
# 2024.3.13 Created by ldq、zh

# 1>&2防止命令的执行结果被重定向
usage() {
  echo "Usage: ${0} [-a|--autoaim] [-f|--close_map] [-r|--real] [-k|--key_test] [-m|--edit_map] [-q|--qingqing] [-h|--huanong] [-t|--mirror]" 1>&2
  exit 1 
}

# 读取命令行参数
para_d=0
para_f=0
para_r=0
para_k=0
para_m=0
para_q=0
para_a=0
para_h=0
para_t=0

while [[ $# -gt 0 ]];do # $#表示参数个数, -gt表示大于
  key=${1}
  case ${key} in
    -d|--decision)
    para_d=1
      shift 1
      ;;
    -f|--close_map)
    para_f=1
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
    -a|--autoaim)
    para_a=1
      shift 1
      ;;
    -h|--huanong)
    para_h=1
      shift 1
      ;;
    -t|--mirror)
    para_t=1
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
        roslaunch sentry_gazebo startup_robot.launch; exec bash"
fi

echo "done!"

