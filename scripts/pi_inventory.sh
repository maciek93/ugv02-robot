#!/bin/sh
# Read-only inventory. Do not include Wi-Fi profiles, environment or credentials.
set -u
date -Is
cat /etc/os-release
uname -a
tr -d '\000' < /proc/device-tree/model
printf '\n'
hostname -I
id
vcgencmd get_throttled
vcgencmd measure_temp
lsusb
v4l2-ctl --list-devices
ls -l /dev/v4l/by-id/ /dev/serial* /dev/ttyAMA* 2>/dev/null
pinctrl get 2-3
pinctrl get 14-15
cat /boot/firmware/config.txt
cat /boot/firmware/cmdline.txt
printf '\n'
systemctl show ugv-web -p ActiveState -p SubState -p MainPID -p NRestarts
