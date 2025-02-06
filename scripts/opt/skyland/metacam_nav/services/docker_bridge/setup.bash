#!/bin/bash

service_name=$(basename "$(pwd)")

script_dir=$(cd "$(dirname "$0")" && pwd)

install_service() {
  ln -sf ${script_dir}/host_send.sh /usr/local/bin/host_send.sh
  #ln -sf ${script_dir}/completion /etc/bash_completion.d/host_send
  ln -sf ${script_dir}/host_send.service /etc/systemd/system/host_send.service
  pkill rosmaster
  systemctl daemon-reload
  systemctl enable host_send.service
  systemctl restart host_send.service

  echo "install ${service_name} done!"
}

uninstall_service() {
  systemctl disable host_send.service
  systemctl stop host_send.service
  rm -f /usr/local/bin/host_send.sh
  #rm -f /etc/bash_completion.d/metacam
  rm -f /etc/systemd/system/host_send.service
  systemctl daemon-reload

  echo "uninstall ${service_name} done!"
}

for parm in "$@"
do
  key=$(echo ${parm%%=*})
  value=$(echo ${parm#*=})
  if [ $key = "--install" ];then
    install_service
    continue
  fi
  if [ $key = "--uninstall" ];then
    uninstall_service
    continue
  fi
  if [ $key = "--help" ] || [ "$key" = "-h" ];then
    echo "Usage: setup.bash --[options]"
    echo "options:"
    echo "    --install: install service"
    echo "    --uninstall: uninstall service"
    continue
  fi
  echo "invalid option: ${key}"
  echo "use 'setup.bash -h' or 'setup.bash --help' to get more information"
  exit 1
done
