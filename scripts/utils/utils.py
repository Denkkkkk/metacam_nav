#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""
os: 用于与操作系统进行交互，比如处理文件和目录路径。
re: 用于正则表达式匹配和替换操作。
yaml: 用于读取和写入 YAML 格式的文件。
subprocess: 用于执行系统命令，尤其是外部命令。
datetime: 用于获取和处理日期和时间。
"""
import os
import re
import yaml
import subprocess
from datetime import datetime

passwords = [" ", "skyland2024"]

"""
尝试以不同的密码执行管理员命令 (sudo)，直到找到一个有效密码
每次尝试使用一个密码，先执行 touch 和 rm 命令来测试密码是否有效（这些命令用于测试权限）。
"""
def super_command(cmd, pwd=None):
    if pwd is None:
        global passwords
        for password in passwords: 
            cmds = []
            cmds.append(super_command("touch /test", password))
            cmds.append(super_command("rm /test", password))
            code, _, _ = execute_command(one_command(cmds))
            if code==0: 
                pwd = password
                break
    cmd = f"echo '{pwd}' | sudo -S {cmd}"
    return cmd

"""
将多个命令连接成一个字符串，以便执行
cmds: 命令列表
force: 是否强制执行，即使前面的命令执行失败才执行后面的命令
"""
def one_command(cmds, force=False):
    """
    make commands into a single command
    """
    connection = "; " if force else " && "
    command = connection.join(cmds)
    return command

def execute_command(cmd):
    process = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stdout, stderr = process.communicate()
    returncode, stdout, stderr = process.returncode, stdout.decode().strip(), stderr.decode().strip()
    return returncode, stdout, stderr

def compress(source_abs_path:str, target_abs_path:str, depresstion_level:int, password:str):
    # a: add files
    # -mx=0: copy+no compression
    # -mhe=on: encrypt archive headers
    # -p: set password
    dirname = os.path.dirname(source_abs_path)
    filename = os.path.basename(source_abs_path)
    cmd = f"cd {dirname}; 7z a -mx={str(depresstion_level)} \'{target_abs_path}\' \'{filename}\'"
    if password != "": cmd += " -mhe=on -p\'" + password + "\'"
    print(cmd)
    execute_command(cmd)

def decompress(source_abs_path: str, target_abs_path: str, password: str):
    # x: extract files with full paths
    # -aos: skip overwriting existing files
    # -p: set password
    cmd = f"7z x -aos \'{source_abs_path}\' -o\'{target_abs_path}\'"
    if password != "": cmd += f" -p\'{password}\'"
    print(cmd)
    execute_command(cmd)

begin_time = None
def calc_time_cost_begin():
    import time
    global begin_time
    begin_time = time.time()

def calc_time_cost_end():
    import time
    global begin_time
    end_time = time.time()
    print(f"Time cost: {(int)(end_time - begin_time)} s")

# 读取versions.yaml文件，并进行管理
class VERSION():
    def __init__(self, filepath):
        self._filepath = filepath
        self.major = 0
        self.minor = 0
        self.patch = 0
        self.build = 0
        self.tag = ""

    def load(self):
        with open(self._filepath, "r") as file:
            data = yaml.safe_load(file)
            match = re.match(r"v?(\d+)\.(\d+)\.(\d+)", data[0]["version"])
            if match: 
                self.major = int(match.group(1))
                self.minor = int(match.group(2))
                self.patch = int(match.group(3))
                self.build = int(data[0]["build_count"]) if "build_count" in data[0] else 0
                self.tag = data[0]["tag"] if "tag" in data[0] else ""
            else:
                raise re.error("Regex pattern cannot match the given version string")

    def save(self):
        with open(self._filepath, "r") as file: data = file.read()
        data = re.sub(r'^  build_count:.*', f'  build_count: {self.build}', data, count=1, flags=re.MULTILINE)
        save_time = datetime.now().strftime("%Y.%m.%d %H:%M:%S")
        data = re.sub(r'^  build_time:.*', f'  build_time: {save_time}', data, count=1, flags=re.MULTILINE)
        with open(self._filepath, "w") as file: file.write(data)
