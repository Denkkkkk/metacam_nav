import json
# 定义文件路径
file_path = '/home/skyland/info/.internal/config/metacam_ros2_config.json'

try:
    # 读取 JSON 文件
    with open(file_path, 'r') as file:
        data = json.load(file)

    # 删除包含 "cmd_vel" 的键
    if 'cmd_vel' in data:
        del data['cmd_vel']

    # 在文件末尾的 } 前追加新语句
    # 注意：在这里，我们将新语句添加到字典中
    data['/TITA/cmd_vel'] = '/TITA/cmd_vel'  # 这里假设你要添加的语句是一个键值对

    # 将修改后的数据写回文件
    with open(file_path, 'w') as file:
        json.dump(data, file, indent=4)

    print("add /TITA/cmd_vel into rosbridge succeed.")

except FileNotFoundError:
    print("metacam_ros2_config.json not found, please check the path")
except json.JSONDecodeError:
    print("The file content is not in a valid JSON format")
except Exception as e:
    print(f"metacam_ros2_config.json config error: {e}")
