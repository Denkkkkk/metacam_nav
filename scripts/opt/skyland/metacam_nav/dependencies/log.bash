#!/bin/bash

# echo "green" text
log_info() {
  echo -e "\e[32m[INFO] $1\e[0m"
}

# echo "yellow" text
log_warn() {
  echo -e "\e[33m[WARN] $1\e[0m" >&2
}

# echo "red" text
log_error() {
  echo -e "\e[31m[ERROR] $1\e[0m" >&2
}
