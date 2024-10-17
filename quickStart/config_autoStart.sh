#!/bin/bash
cd $(dirname $0)
# 哨兵运动要求依赖的安装
#鱼香ros配置rosdep
# wget http://fishros.com/install -O fishros && . fishros
sleep 0.5s
source ~/.bashrc

sudo apt -y install  wmctrl # 安装wmctrl令窗口置顶
sudo chmod +x all_kill.sh all_run.sh run.sh

cd ..
dir=$(pwd);

# 指定文件和匹配的模式
file="$HOME/.bashrc"
pattern1="sentry_run"
pattern2="sentry_kill"
pattern3="explorationExp_ws"
# 使用 sed 命令删除匹配的行
sed -i "/$pattern1/d" "$file"
sed -i "/$pattern2/d" "$file"
sed -i "/$pattern3/d" "$file"
sed -i "/ROSCONSOLE_FORMAT/d" "$file"

echo "alias sentry_run='cd ${dir};./quickStart/run.sh'" >> ~/.bashrc
echo "alias sentry_kill='. ${dir}/quickStart/all_kill.sh'" >> ~/.bashrc
echo "source ${dir}/devel/setup.bash" >> ~/.bashrc
echo "export ROSCONSOLE_FORMAT='[\${node}]: \${message}'" >> ~/.bashrc

sleep 0.5s
. $HOME/.bashrc

echo -e "\nConfig done!"
echo -e "Use \033[33m'sentry_run'\033[0m to start the sentry robot with parameters: -d|--decision -f|--far_planner -c|--catkin -r|--real\n  e.g. sentry_run -d -f -c -r"
echo -e "Use \033[33m'sentry_kill'\033[0m to kill all the ros process."
