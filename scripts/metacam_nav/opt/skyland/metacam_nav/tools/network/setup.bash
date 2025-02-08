#!/bin/bash

software_name=$(basename "$(pwd)")
script_dir=$(cd "$(dirname "$0")" && pwd)

install_software() {
  dpkg -i ${script_dir}/nfs-kernel-server_1.3.4-2.1ubuntu5.5_arm64.deb 
  dpkg -i ${script_dir}/sshfs_2.8-1_arm64.deb 

  echo "install \"${software_name}\" done!"
}

for parm in "$@"
do
  key=$(echo ${parm%%=*})
  value=$(echo ${parm#*=})
  if [ $key = "--install" ];then
    install_software
    continue
  fi
  if [ $key = "--help" ] || [ "$key" = "-h" ];then
    echo "Usage: setup.bash --[options]"
    echo "options:"
    echo "    --install: install software"
    continue
  fi
  echo "invalid option: ${key}"
  echo "use 'setup.bash -h' or 'setup.bash --help' to get more information"
  exit 1
done
