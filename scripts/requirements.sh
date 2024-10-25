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

# enable auto complete
export PATH=$PATH:/home/skyland/.local/bin
register-python-argcomplete $script_dir/../project.py
timeout 10 activate-global-python-argcomplete   # Note: it should be interactive with terminal, therefore, use `timeout`

# install submodules' requirements
if [[ $script_dir == "/opt/skyland/metacam-runtime" ]]; then
    bash /opt/skyland/metacam-runtime/share/livo/requirements.sh
    bash /opt/skyland/metacam-runtime/share/system_monitor/requirements.sh
else
    bash ../src/livo/requirements.sh
    bash ../src/system_monitor/requirements.sh
fi
