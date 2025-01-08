- 实时生成的.pcd文件
roslaunch map_create nav_map_create.launch

！手动运行后请注意及时请空生成的pcd文件，避免造成内存紧张

1. relocal*用于重定位的pcd地图，不做地面分割剔除，直接进行点云积分
2. terrain*用于规划避障的pcd地图，剔除地面。

- pcd_temp文件夹
防止在pcd文件实时写入的过程中，因程序异常退出而引发的
