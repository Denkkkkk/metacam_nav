#!/bin/bash
# 1>&2防止命令的执行结果被重定向

succeed=1
usage() {
  echo "Usage: [-s|--nav_service] [-r|--real_robot] [-k|--use_global_planner && rmuc2024] [-m|--use_global_planner && no_gazebo_map] [-q|--relocal] " 1>&2
  succeed=0
}



# 读取命令行参数
para_s=""
para_r=""
para_k=""
para_m=""
para_q=""

while [[ $# -gt 0 ]];do # $#表示参数个数, -gt表示大于
  key=${1}
  case ${key} in
    -s|--nav_service)
    echo "nav_service."
    para_s="-s"
      shift 1
      ;;
    -r|--real)
    para_r="-r"
    echo "real_robot."
      shift 1
      ;;
    -k|--use_global_planner)
    para_k="-k"
    echo "use_global_planner."
      shift 1
      ;;
    -m|--edit_map)
    para_m="-m"
    echo "edit_map."
      shift 1
      ;;
    -q|--relocal)
    para_q="-q"
    echo "relocal."
      shift 1
      ;;
    *)
      usage
      shift
      ;;
  esac
done

if [ ${succeed} -eq 1 ]; then

export ROSCONSOLE_FORMAT='[${node}] [${time:%M:%S}]: ${message}'

if [ -z ${para_r} ]; then
  echo "仿真环境..."
fi

ps aux | grep ros | grep -v microsoft | awk '{print $2}' | xargs kill -9
# ps aux | grep rviz | awk '{print $2}' | xargs kill -9;
echo -n "等待ros相关进程结束"
for j in $(seq 1 2)
do
  echo -n "……"
  sleep 0.1
done
echo "……"

script_path=$(readlink -f "${BASH_SOURCE[0]}")
# echo "script_path: ${script_path}"
parent_path=$(dirname "$script_path")


# ---------------------------
# 把当前终端置顶 wmctrl -r :ACTIVE: -b toggle,above;
gnome-terminal --tab -- bash -c "wmctrl -r :ACTIVE: -b toggle,above;$parent_path/all_run.sh ${para_s} ${para_r} ${para_k} ${para_m} ${para_q}"


fi