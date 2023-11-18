#!/bin/bash
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="hldslidar"' >/etc/udev/rules.d/hldslidar.rules

service udev reload
sleep 1
service udev restart
