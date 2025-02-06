#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
# PYTHON_ARGCOMPLETE_OK

import os
import argparse
from scripts.utils.utils import *

try:
    import argcomplete  # pip3 install argcomplete
except ImportError as e:
    print(">> Don't satisfy requirements, start to download!")
    returncode, _, _ = execute_command(super_command("bash scripts/requirements.sh"))
    if returncode==0: 
        print(">> Download requirements successfully, enjoy it now!")
    else:
        print(">> Fail to download requirements, please check your network!")

def parse_args():
    parser = argparse.ArgumentParser(description="project building toolchain")
    parser.add_argument("--init", action="store_true",
                        help="set version and tag")
    parser.add_argument("--build", action="store_true",
                        help="build the whole project")
    parser.add_argument("--make_deb", action="store_true",
                        help="make *.deb")
    parser.add_argument("--make_deb_debug", action="store_true",
                        help="make *.deb")
    parser.add_argument("--install", action="store_true",
                        help="install the lastest *.deb")
    # parser.add_argument("--upload", action="store_true",
    #                     help="upload the lastest *.metadeb to cloud service")
    parser.add_argument("--all", action="store_true",
                        help="build, make deb and then install")
    parser.add_argument("--clean", action="store_true",
                        help="clean build files")
    parser.add_argument("--with_ros2_interface", action="store_true",
                        help="build the interfaces for ROS2")
    argcomplete.autocomplete(parser)
    args, _ = parser.parse_known_args()
    return args


if __name__ == "__main__":
    calc_time_cost_begin()

    args = parse_args()

    # clean pycache，清理 Python 生成的字节码文件（__pycache__ 文件夹）
    execute_command("find ./ -type d -name '__pycache__' -exec rm -rf {} +")
    script_folder = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_folder)

    # auto download submodules
    if not os.path.exists("src/system_monitor/CMakeLists.txt"):
        anwser = input("The submodules haven't been downloaded, do you want to download automatically? [y/n:default=y]:")
        if anwser not in ["n", "N", "No", "no", "NO"]:
            execute_command("git submodule update --init --recursive")

    os.makedirs("output", exist_ok=True)
    version = VERSION("versions.yaml")
    version.load()
    
    if args.init: 
        print(">> Start to check & download requirements")
        execute_command(super_command("bash scripts/requirements.sh"))
        print(">> Check & Download successfully!")

    if args.build or args.all:
        print(f">> Start to build, please refer to output/build.log")
        code, _, _ = execute_command("catkin_make --force-cmake -DUSE_ROS=ON -DCMAKE_BUILD_TYPE=Release > output/build.log 2>&1")
        if code != 0:
            print(">> Fail to build, plese refer to output/build.log!")
            exit(-1)
        if code == 0:
            print(">> Build successfully!")

    if args.make_deb or args.make_deb_debug or args.all:
        version.build += 1
        ver = f"v{version.major}.{version.minor}.{version.patch}.{version.build}"
        if version.tag: ver = f"{ver}.{version.tag}"
        print(f">> Start to make metacam_nav_{ver}.deb and metacam_nav_{ver}.metadeb!")

        cmds = []
        cmds.append("mkdir -p output/metacam_nav")
        cmds.append("cp -r scripts/opt output/metacam_nav")
        cmds.append("cp -r scripts/DEBIAN output/metacam_nav")
        cmds.append("cp -r scripts/requirements.sh output/metacam_nav/opt/skyland/metacam-nav/requirements.sh")
        cmds.append("awk '/^Version/ {print \"Version: " + f"{version.major}.{version.minor}.{version.patch}-rc{version.build}" +
                    "\"; next} 1' output/metacam_nav/DEBIAN/control > temp.txt && mv temp.txt output/metacam_nav/DEBIAN/control")
        cmds.append(f"catkin_make install -DCMAKE_INSTALL_PREFIX=output/metacam_nav/opt/skyland/metacam-runtime -DUSE_ROS=ON -DENCRYPT_PYTHON={'OFF' if args.make_deb_debug else 'ON'}")
        # ros2
        if os.path.exists("interfaces/ros2/install"):
            print(">> Copying ros2 wrapper...")
            cmds.append("mkdir -p output/metacam_runtime/opt/skyland/metacam-ros2-wrapper")
            cmds.append("cp -r interfaces/ros2/install/* output/metacam_runtime/opt/skyland/metacam-ros2-wrapper")
            cmds.append("cp -r interfaces/ros2/scripts/opt/* output/metacam_runtime/opt/")
        # web
        print(">> Copying web service...")
        cmds.append("rm -rf output/metacam_runtime/opt/skyland/metacam-runtime/web")
        cmds.append("mkdir -p output/metacam_runtime/opt/skyland/metacam-runtime/web")
        cmds.append("mkdir -p output/metacam_runtime/opt/skyland/metacam-runtime/web/backend")
        cmds.append(f"cp -r interfaces/web/output/*.so output/metacam_runtime/opt/skyland/metacam-runtime/web/backend")
        cmds.append(f"cp -r interfaces/web/frontend output/metacam_runtime/opt/skyland/metacam-runtime/web")
        # 
        print(">> DPKG packing...")
        cmds.append(super_command("chmod +x output/metacam_runtime/* -R"))
        cmds.append("cp -r versions.yaml output/metacam_runtime/opt/skyland/metacam-runtime")
        cmds.append("git submodule status > output/metacam_runtime/opt/skyland/metacam-runtime/submodule_version.txt")
        cmds.append(f"dpkg-deb -Zxz -z0 -b output/metacam_runtime output/metacam_runtime_{ver}.deb")    # compression type: xz; compression level: 0
        code, _, stderr = execute_command(one_command(cmds, force=False))
        if code == 0:
            execute_command("mkdir -p /home/skyland/Downloads/firmware")
            compress(f"{script_folder}/output/metacam_runtime_{ver}.deb", f"{script_folder}/output/metacam_runtime_{ver}.metadeb", 0, "skyland2024")
            execute_command(f"cp output/metacam_runtime_{ver}.metadeb /home/skyland/Downloads/firmware")
            print(f">> Make deb & metadeb successfully!")
            version.save()
        else:
            print(f"Fail to make deb and metadeb!")
            print(stderr)
            exit(-1)

    if args.install or args.all:
        ver = f"v{version.major}.{version.minor}.{version.patch}.{version.build}"
        if version.tag: ver = f"{ver}.{version.tag}"
        print(f">> Start to install metacam_runtime_{ver}.deb!")
        code, _, stderr = execute_command(super_command(f"dpkg -i output/metacam_runtime_{ver}.deb"))
        if code == 0:
            print(f"Install successfully!")
            
        else:
            print(f"Fail to install!")
            print(stderr)
            exit(-1)

    # if args.upload:
    #     from scripts.utils.cloud_service import CloudService
    #     cloud_service = CloudService(3)
    #     cloud_service.update_authorization()
        
    #     items = cloud_service.get_items()
    #     appVersion = f"metacam_runtime_v{version.major}.{version.minor}.{version.patch}.metadeb"
    #     for item in items:
    #         if item["appVersion"]==appVersion:
    #             cloud_service.delete_item(item["id"])
        
    #     ossApkUrl = cloud_service.upload_file(os.path.join(script_folder, f"output/metacam_runtime_v{version.major}.{version.minor}.{version.patch}.{version.build}.metadeb"))
    #     cloud_service.create_item(appVersion, ossApkUrl)

    if args.clean:
        execute_command("rm -fr build devel install output .catkin_workspace")
        print(">> Clean successfully!")

    calc_time_cost_end()
