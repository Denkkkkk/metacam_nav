#!/bin/bash
# 1>&2防止命令的执行结果被重定向
usage() {
  echo "Usage: ${0} [-s|--nav_service] [-r|--real] [-k|--key_test] [-m|--edit_map] [-q|--qingqing] " 1>&2
  exit 1 
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
    -k|--key_test)
    para_k="-k"
    echo "key_test."
      shift 1
      ;;
    -m|--edit_map)
    para_m="-m"
    echo "edit_map."
      shift 1
      ;;
    -q|--qingqing)
    para_q="-q"
    echo "qingqing_map."
      shift 1
      ;;
    *)
      usage
      shift
      ;;
  esac
done

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

gnome-terminal --tab -- bash -c "./quickStart/all_run.sh ${para_s} ${para_r} ${para_k} ${para_m} ${para_q}"