#!/bin/bash

software_name=$(basename "$(pwd)")
script_dir=$(cd "$(dirname "$0")" && pwd)

install_software() {
  dpkg-deb -x ${script_dir}/libcaca-dev_0.99.beta19-2_arm64.deb /
  dpkg-deb -x ${script_dir}/libpulse-dev_arm64.deb /
  dpkg-deb -x ${script_dir}/libsdl-image1.2-dev_1.2.12-8_arm64.deb /
  dpkg-deb -x ${script_dir}/libsdl1.2-dev_1.2.15+dfsg2-0.1_arm64.deb /
  dpkg-deb -x ${script_dir}/libsdl1.2debian_1.2.15+dfsg2-0.1_arm64.deb /
  dpkg-deb -x ${script_dir}/libslang2-dev_2.3.1a-3_arm64.deb /

  echo "install \"${software_name}\" done!"
}

uninstall_software() {
  echo "uninstall \"${software_name}\" hasn't realized yet!"
  
  echo "uninstall \"${software_name}\" done!"
}

for parm in "$@"
do
  key=$(echo ${parm%%=*})
  value=$(echo ${parm#*=})
  if [ $key = "--install" ];then
    install_software
    continue
  fi
  if [ $key = "--uninstall" ];then
    uninstall_software
    continue
  fi
  if [ $key = "--help" ] || [ "$key" = "-h" ];then
    echo "Usage: setup.bash --[options]"
    echo "options:"
    echo "    --install: install software"
    echo "    --uninstall: uninstall software"
    continue
  fi
  echo "invalid option: ${key}"
  echo "use 'setup.bash -h' or 'setup.bash --help' to get more information"
  exit 1
done
