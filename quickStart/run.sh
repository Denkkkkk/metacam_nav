cm,#!/bin/bash
usage() {
  echo "Usage: ${0} [-a|--autoaim] [-d|--decision] [-q|--qingqing] [-f|--close_map] [-c|--catkin] [-r|--real] [-k|--key_test] [-g|--noRestart] [-m|--edit_map] [-h|--huanong] [-t|--mirror]" 1>&2
  exit 1 
}
export ROSCONSOLE_FORMAT='[${node}] [${time:%M:%S}]: ${message}'
# export ROSCONSOLE_FORMAT='[${node}]: ${message}'
# 读取命令行参数
para_a=""
para_d=""
para_f=""
para_c=0
para_r=""
para_k=""
para_g=0
para_m=""
para_q=""
para_h=""
para_t=""

while [[ $# -gt 0 ]];do # $#表示参数个数, -gt表示大于
  key=${1}
  case ${key} in
    -c|--catkin)
    para_c=1
      shift 1
      ;;
    -a|--autoaim)
    para_a="-a"
    echo "autoaim_with_decision."
      shift 1
      ;;
    -d|--decision)
    para_d="-d"
    echo "decision."
      shift 1
      ;;
    -f|--close_map)
    echo "close_map."
    para_f="-f"
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
    -g|--noRestart)
    para_g=1
    echo "noRestart."
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
    -h|--huanong)
    para_h="-h"
    echo "huanong."
      shift 1
      ;;
    -t|--mirror)
    para_t="-t"
    echo "_t."
      shift 1
      ;;
    *)
      usage
      shift
      ;;
  esac
done

if [ -z ${para_r} ]; then
  echo "仿真环境..."
fi

if [ ${para_g} -eq 0 ]; then
  ps aux | grep ros | grep -v microsoft | awk '{print $2}' | xargs kill -9
  # ps aux | grep rviz | awk '{print $2}' | xargs kill -9;
  echo -n "等待ros相关进程结束"
  for j in $(seq 1 2)
  do
    echo -n "……"
    sleep 0.1
  done
  echo "……"
fi

if [ ${para_c} -eq 1 ]; then
  echo "编译中..."
  check_results=`catkin_make 2>&1 | grep failed`
  echo -e "\033[0;31m$check_results\033[0m"
  if [[ $check_results =~ "failed" ]] 
  then 
      echo "编译失败,请手动使用catkin_make指令检查!"
      sleep 1s
      exit 1
  else
      echo "编译成功!"
  fi
fi

gnome-terminal --tab -- bash -c "./quickStart/all_run.sh ${para_a} ${para_f} ${para_d} ${para_r} ${para_k} ${para_m} ${para_q} ${para_h} ${para_t};"