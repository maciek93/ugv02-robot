#!/usr/bin/env python3
"""Run as root on verified Pi 5 to persist the tested GPIO UART overlay."""
from pathlib import Path
import shutil

model = Path('/proc/device-tree/model').read_text().rstrip('\0')
if not model.startswith('Raspberry Pi 5 '):
    raise RuntimeError(f'Expected Pi 5, got {model}')
if not Path('/dev/ttyAMA0').exists():
    raise RuntimeError('Load and verify the GPIO UART overlay first')
config = Path('/boot/firmware/config.txt')
backup = Path('/home/ugvpi/robot-records/config-before-uart.txt')
text = config.read_text()
if 'dtoverlay=uart0-pi5' not in text:
    if not backup.exists():
        shutil.copy2(config, backup)
    config.write_text(text.rstrip() + '\n\n[all]\n# UGV02: tested GPIO14/15 UART, distinct from Pi 5 debug UART\ndtoverlay=uart0-pi5\n')
print('GPIO UART boot overlay present; reboot verification still pending')
