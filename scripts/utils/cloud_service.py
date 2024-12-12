#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import os
import json
import requests
from requests_toolbelt.multipart.encoder import MultipartEncoder    # pip3 install requests-toolbelt

class CloudService:
    authorization = ""
    appType = None

    OFFSHELF = 0
    ONSHELF = 1
    
    def __init__(self, appType):
        self.appType = appType
    
    def update_authorization(self):
        try:
            headers = {
                'Connection': 'keep-alive',
                'Content-Type': 'application/json;charset=UTF-8',
                'tenant-id': '1',
            }

            json_data = {
                'username': 'caiguang1997',
                'password': 'caiguang1997',
            }
            response = requests.post(
                'https://skyland.3disfuture.com/admin-api/system/auth/login',
                headers=headers,
                json=json_data,
            )
            data = json.loads(response.text)
            if data["code"]==0:
                self.authorization = f"Bearer {data['data']['accessToken']}"
                print("Update authorization successfully!")
            else:
                raise "Fail to update authorization!"
        except Exception as e:
            print(str(e))
            raise "Fail to update authorization!"

    def upload_file(self, file_path):
        try: 
            headers = {
                'Authorization': self.authorization,
                'Connection': 'keep-alive',
                'Content-Type': 'multipart/form-data; boundary=----WebKitFormBoundarySD1yn7vOJxQdjA5I',
            }
            encoder = MultipartEncoder(fields={'file': (os.path.basename(file_path), open(file_path, 'rb'), 'application/octet-stream')})
            headers['Content-Type'] = encoder.content_type
            response = requests.post('https://skyland.3disfuture.com/admin-api/pntcld/work/upload',headers=headers,data=encoder)
            data = json.loads(response.text)
            if data["code"]==0:
                print("Upload " + file_path + " successfully!")
                return data["data"]
            else:
                print(data)
                raise "Fail to upload " + file_path + "!"
        except Exception as e:
            print(str(e))
            raise "Fail to upload " + file_path + "!"

    def get_items(self):
        try:
            headers = {
                'Authorization': self.authorization,
                'Connection': 'keep-alive',
                'tenant-id': '1',
            }
            params = {
                'pageNo': '1',
                'pageSize': '10',
                'appType': self.appType,
            }
            response = requests.get(
                'https://skyland.3disfuture.com/admin-api/pntcld/app-version/page',
                params=params,
                headers=headers,
            )
            data = json.loads(response.text)
            if data["code"]==0:
                return data["data"]["list"]
            else:
                print(data)
                raise "Fail to get cloud service items!"
        except Exception as e:
            print(str(e))
            raise "Fail to get cloud service items!"

    def offshelf_items(self, items):
        for item in items:
            if item["versionStatus"]==1:
                self.update_item(item["id"], item["appVersion"], item["ossApkUrl"], self.OFFSHELF)
                print("Off shelf: " + item["ossApkUrl"])

    def create_item(self, appVersion, ossApkUrl, versionStatus=ONSHELF, versionDescription="", remark=""):
        try: 
            headers = {
                'Authorization': self.authorization,
                'Connection': 'keep-alive',
                'tenant-id': '1',
            }
            json_data = {
                'appVersion': appVersion,
                'ossApkUrl': ossApkUrl,
                'versionDescription': versionDescription,
                'versionStatus': versionStatus,
                'remark': remark,
                'appType': self.appType,
            }
            response = requests.post('https://skyland.3disfuture.com/admin-api/pntcld/app-version/create',headers=headers,json=json_data)
            data = json.loads(response.text)
            if data["code"]==0:
                print("Create cloud service item " + appVersion + " successfully!")
                return data["data"]
            else:
                print(data)
                raise "Fail to create cloud service item " + appVersion + "!"
        except Exception as e:
            print(str(e))
            raise "Fail to create cloud service item " + appVersion + "!"
        
    def delete_item(self, id):
        try: 
            headers = {
                'Authorization': self.authorization,
                'Connection': 'keep-alive',
                'tenant-id': '1',
            }
            response = requests.delete('https://skyland.3disfuture.com/admin-api/pntcld/app-version/delete?id='+str(id),headers=headers)
            data = json.loads(response.text)
            if data["code"]==0:
                print("Delete cloud service item " + str(id) + " successfully!")
                return data["data"]
            else:
                print(data)
                raise "Fail to delete cloud service item " + str(id) + "!"
        except Exception as e:
            print(str(e))
            raise "Fail to delete cloud service item " + str(id) + "!"

    def update_item(self, id, appVersion, ossApkUrl, versionStatus, versionDescription="", remark=""):
        try: 
            headers = {
                'Authorization': self.authorization,
                'Connection': 'keep-alive',
                'tenant-id': '1',
            }
            json_data = {
                'id': id,
                'appVersion': appVersion,
                'ossApkUrl': ossApkUrl,
                'versionDescription': versionDescription,
                'versionStatus': versionStatus,
                'remark': remark,
                'appType': self.appType,
            }
            response = requests.put('https://skyland.3disfuture.com/admin-api/pntcld/app-version/update',headers=headers,json=json_data)
            data = json.loads(response.text)
            if data["code"]==0:
                print("Update cloud service item " + appVersion + " successfully!")
                return data["data"]
            else:
                print(data)
                raise "Fail to update cloud service item " + appVersion + "!"
        except Exception as e:
            print(str(e))
            raise "Fail to update cloud service item " + appVersion + "!"
