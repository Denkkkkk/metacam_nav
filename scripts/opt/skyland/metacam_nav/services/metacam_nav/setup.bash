#!/bin/bash

service_name=$(basename "$(pwd)")

script_dir=$(cd "$(dirname "$0")" && pwd)

install_service() {
  ln -sf ${script_dir}/metacam_nav.sh /usr/local/bin/metacam_nav.sh
  #ln -sf ${script_dir}/completion /etc/bash_completion.d/metacam_nav
  ln -sf ${script_dir}/metacam_nav.service /etc/systemd/system/metacam_nav.service
  pkill rosmaster
  systemctl daemon-reload
  systemctl enable metacam_nav.service
  systemctl restart metacam_nav.service

  echo "install ${service_name} done!"
}

uninstall_service() {
  systemctl disable metacam_nav.service
  systemctl stop metacam_nav.service
  rm -f /usr/local/bin/metacam_nav.sh
  #rm -f /etc/bash_completion.d/metacam
  rm -f /etc/systemd/system/metacam_nav.service
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
