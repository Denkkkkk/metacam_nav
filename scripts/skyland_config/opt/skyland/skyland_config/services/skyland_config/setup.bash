#!/bin/bash

service_name=$(basename "$(pwd)")

script_dir=$(cd "$(dirname "$0")" && pwd)

install_service() {
  ln -sf ${script_dir}/skyland_config.sh /usr/local/bin/skyland_config.sh
  #ln -sf ${script_dir}/completion /etc/bash_completion.d/skyland_config
  ln -sf ${script_dir}/skyland_config.service /etc/systemd/system/skyland_config.service
  pkill rosmaster
  systemctl daemon-reload
  systemctl enable skyland_config.service
  systemctl restart skyland_config.service

  echo "install ${service_name} done!"
}

uninstall_service() {
  systemctl disable skyland_config.service
  systemctl stop skyland_config.service
  rm -f /usr/local/bin/skyland_config.sh
  #rm -f /etc/bash_completion.d/metacam
  rm -f /etc/systemd/system/skyland_config.service
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
