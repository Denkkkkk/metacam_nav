import fileinput
import yaml

# 定义要修改的文件路径
file_path = '/home/skyland/info/.internal/config/mapping_config.yaml'
# 要修改的行
search_string = 'blind:'
new_value = '    blind: 0.25'

# 使用 fileinput 模块逐行读取和修改文件
with fileinput.input(file_path, inplace=True, backup='.bak') as file:
    for line in file:
        # 如果找到需要修改的行，替换为新值
        if search_string in line:
            line = new_value + '\n'
        # 输出（替换原文件）
        print(line, end='')

# 修改后，读取文件进行校验
with open(file_path, 'r') as file:
    content = file.read()
    if new_value in content:
        print("The 'blind' parameter has been successfully updated to 0.28.")
    else:
        print("Failed to update the 'blind' parameter.")
