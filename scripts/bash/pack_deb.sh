#!/bin/bash

cp -r scripts/opt output/metacam_nav
cp -r scripts/DEBIAN output/metacam_nav

# dpkg-deb -Zxz -z0 -b output/metacam_nav output/metacam_nav_XT_1.3.5_arm64.deb

# dpkg-deb -Zxz -z0 -b output/metacam_nav output/metacam_nav_jinzhong_1.4.12_arm64.deb
dpkg-deb -Zxz -z0 -b output/metacam_nav output/metacam_nav_1.3.13_arm64.deb
# dpkg-deb -Zxz -z0 -b output/metacam_nav output/metacam_nav_jinzhong_1.4.10_arm64.deb
# dpkg-deb -Zxz -z0 -b output/metacam_nav output/metacam_nav_skyland_1.4.9_arm64.deb
# dpkg-deb -Zxz -z0 -b output/metacam_nav ../metacam_nav_deb/metacam_nav_1.0.1_arm64.deb

# xz 是指定的压缩算法，0 表示不进行压缩（即未压缩），1压缩最快，9压缩最慢，但压缩比最高
# dpkg-deb -Zxz -z0 -b output/metacam_nav ../metacam_nav_deb/metacam_nav_1.0.1_amd64.deb
# dpkg-deb -Zxz -z0 -b output/metacam_nav output/metacam_nav_1.0.1_amd64.deb