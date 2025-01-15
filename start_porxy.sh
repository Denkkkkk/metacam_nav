#!/bin/bash
function proxy_on() {
    export http_proxy="http://192.168.31.34:7897"
    export https_proxy="http://192.168.31.34:7897"
    export all_proxy="socks5://192.168.31.34:7897"
    echo "代理已开启"
}

# 关闭代理
function proxy_off() {
    unset http_proxy
    unset https_proxy
    unset all_proxy
    echo "代理已关闭"
}

for parm in "$@"
do
  key=$(echo ${parm%%=*})
  value=$(echo ${parm#*=})
  if [ $key = "--on" ];then
    proxy_on
    continue
  fi
  if [ $key = "--off" ];then
    proxy_off
    continue
  fi
  if [ $key = "--help" ] || [ "$key" = "-h" ];then
    echo "Usage: start_porxy.sh --[options]"
    echo "options:"
    echo "    --on: start proxy"
    echo "    --off: stop proxy"
    continue
  fi
  echo "invalid option: ${key}"
  echo "use 'start_porxy.sh -h' or 'start_porxy.sh --help' to get more information"
  exit 1
done