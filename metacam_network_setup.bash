#!/bin/bash

software_name=$(basename "$(pwd)")
script_dir=$(cd "$(dirname "$0")" && pwd)
source "$script_dir/../log.bash"

# usage: rename_network "old_name" "new_name"
function rename_network() {
  old_name=$1
  new_name=$2

  connection_id=$(nmcli c show | grep "$old_name" | awk '{print $1}')

  if [ -z "$connection_id" ]; then
    log_error "network '$old_name' not found, fail to rename network"
  else
    nmcli c modify "$connection_id" connection.id "$new_name"
    echo "network '$old_name' renamed to '$new_name' successfully"
  fi
}

# usage: set_hotspot_password "hotspot_name" "password"
function set_hotspot_password() {
  hotspot_name=$1
  password=$2
  connection_id=$(nmcli c show | grep "$hotspot_name" | awk '{print $1}')

  if [ -z "$connection_id" ]; then
    log_error "hotspot '$hotspot_name' not found, fail to set hotspot password"
  else
    if [ -z "$password" ]; then
      # TODO: realize modification a hotspot to no passward is not easy, maybe copy a realization
      # from "/etc/NetworkManager/system-connections" or recreate a hotspot with no password 
      # is more reasonable
      network_config_file=/etc/NetworkManager/system-connections/$connection_id
      sed -i '/^\[wifi-security\]/d' $network_config_file
      sed -i '/^key-mgmt=/d' $network_config_file
      sed -i '/^psk=/d' $network_config_file
      echo "password for hotspot '$hotspot_name' removed successfully"
    else
      nmcli c modify "$connection_id" wifi-sec.key-mgmt wpa-psk
      nmcli c modify "$connection_id" wifi-sec.psk "$password"
      echo "password for hotspot '$hotspot_name' set to '$password' successfully"
    fi
  fi
}

# usage: set_network_autoconnect "network_name" "yes|no"
function set_network_autoconnect() {
  network_name=$1
  autoconnect=$2
  connection_id=$(nmcli c show | grep "$network_name" | awk '{print $1}')

  if [ -z "$connection_id" ]; then
    log_error "network '$network_name' not found, fail to set network autoconnect"
  else
    nmcli c modify "$connection_id" connection.autoconnect $autoconnect
    echo "network '$network_name' set to autoconnect successfully"
  fi
}

# usage: set_network_address "network_name" "address", and "address" has a format like "192.168.19.97/24"
function set_network_address() {
  network_name=$1
  address=$2
  connection_id=$(nmcli c show | grep "$network_name" | awk '{print $1}')

  if [ -z "$connection_id" ]; then
    log_error "network '$network_name' not found, fail to set network address"
  else
    nmcli c modify "$connection_id" ipv4.addresses $address
    echo "network '$network_name' address set to '$address' successfully"
  fi
}

# usage: reboot_network "network_name"
function reboot_network() {
  network_name=$1
  connection_id=$(nmcli c show | grep "$network_name" | awk '{print $1}')

  if [ -z "$connection_id" ]; then
    log_error "network '$network_name' not found, fail to reboot network"
  else
    nmcli c down "$connection_id"
    sleep 1
    nmcli c up "$connection_id"
    echo "reboot network '$network_name' successfully"
  fi
}

install_software() {
  rename_network "LAN-click-eth1" "eth1-dhcp"   # ip: depends on external router
  rename_network "ssh-auto-eth1" "eth1-static"  # ip: 192.168.19.97/24
  set_network_address "eth1-static" "192.168.19.97/24"  # to avoid ip confliction with livox lidar

  if [[ $HOSTNAME == SatLab* ]]; then
    set_network_autoconnect "eth1-dhcp" "yes"
    set_network_autoconnect "eth1-static" "no"
    set_hotspot_password "metacam-hotspot" "cygnuslite"
  elif [[ $HOSTNAME == TITA* ]]; then
    set_network_autoconnect "eth1-dhcp" "no"
    set_network_autoconnect "eth1-static" "yes"
    set_hotspot_password "metacam-hotspot" ""
  else
    set_network_autoconnect "eth1-dhcp" "yes"
    set_network_autoconnect "eth1-static" "no"
    set_hotspot_password "metacam-hotspot" ""
  fi

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
