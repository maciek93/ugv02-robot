#!/bin/zsh
set -eu
cd /Users/maciejbargiel/Documents/Robot
ssh -S .runtime/ssh-control -o BatchMode=yes ugvpi@ugvpi.local 'sudo systemctl start ugv-supervised'
exec .venv/bin/python supervised/mac.py
