#!/bin/sh
# Installs and starts the vision server as a systemd service. RUN THIS YOURSELF ON THE PI:
#     cd ~/delta_robot/pi && sh install_service.sh
# It uses sudo. Read it first -- it only does the steps listed below.
set -e

echo "1) Checking that nothing else is holding the camera (old main_harvester.py)..."
if pgrep -af main_harvester.py; then
  echo "   main_harvester.py is running. Stop it first (kill the PID above), then re-run."
  exit 1
fi
if command -v fuser >/dev/null 2>&1 && fuser /dev/video0 2>/dev/null; then
  echo "   Something has /dev/video0 open (PIDs above). Stop it first, then re-run."
  exit 1
fi

echo "2) Installing the unit file (sudo)..."
sudo cp "$(dirname "$0")/delta-vision.service" /etc/systemd/system/delta-vision.service
sudo systemctl daemon-reload

echo "3) Enabling at boot and starting now (sudo)..."
sudo systemctl enable --now delta-vision.service

echo "Done. Watch it with:  journalctl -u delta-vision -f"
echo "Stop it for calibration with:  sudo systemctl stop delta-vision"
