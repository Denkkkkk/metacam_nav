#!/bin/bash

cp -r scripts/opt output/metacam_nav
cp -r scripts/DEBIAN output/metacam_nav

# dpkg-deb -Zxz -z0 -b output/metacam_nav output/metacam_nav_1.0.1.deb
dpkg-deb -Zxz -z0 -b output/metacam_nav output/metacam_nav_amd64_1.0.1.deb