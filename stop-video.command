#!/bin/zsh
set -eu
set -o pipefail
cd /Users/maciejbargiel/Documents/Robot
mkdir -p .runtime records
chmod 700 .runtime
print 'Enter the Pi SSH password in this Terminal only. It is not recorded.'
print 'This command stops the robot video service and checks camera release.'
ssh -tt -S "$PWD/.runtime/ssh-control" \
  -o ControlMaster=auto -o ControlPersist=10m \
  -o StrictHostKeyChecking=yes -o ConnectTimeout=8 \
  ugvpi@ugvpi.local \
  'sudo -n systemctl stop ugv-web && systemctl show ugv-web -p ActiveState -p MainPID && if sudo -n fuser /dev/video0 >/dev/null 2>&1; then echo CAMERA_STILL_IN_USE; exit 1; else echo CAMERA_RELEASED; fi' \
  | tee records/camera-stop.txt
print 'Finished. You can close this Terminal window.'
