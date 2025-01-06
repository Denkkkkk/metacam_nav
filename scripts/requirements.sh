#!/bin/bash

# enter the folder where the script is
script_dir=$(cd "$(dirname "$0")" && pwd)
cd "$script_dir"

# change pip source
pip3 install --upgrade pip
pip3 config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple

# install self requirements
apt-get install -y ccache
pip3 install argcomplete requests-toolbelt nuitka

#timeout 10 表示如果命令在 10 秒内没有完成，终端将自动终止该命令。
#activate-global-python-argcomplete 会启用全局的 Python 脚本自动补全功能，使得以后所有 Python 脚本（如果它们使用 argcomplete 库）都能在命令行中支持自动补全。
# enable auto complete
export PATH=$PATH:/home/skyland/.local/bin
register-python-argcomplete $script_dir/../project.py
timeout 10 activate-global-python-argcomplete   # Note: it should be interactive with terminal, therefore, use `timeout`
