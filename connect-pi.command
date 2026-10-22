#!/bin/zsh
set -eu
cd /Users/maciejbargiel/Documents/Robot
mkdir -p .runtime
chmod 700 .runtime
print 'Enter the Pi SSH password here in Terminal, never in chat.'
print 'After authentication this window stays open to share the SSH connection.'
exec ssh -MN -S "$PWD/.runtime/ssh-control" \
  -o StrictHostKeyChecking=yes -o ControlMaster=yes \
  -o ServerAliveInterval=15 -o ServerAliveCountMax=3 \
  ugvpi@ugvpi.local
