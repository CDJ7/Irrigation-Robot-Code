#!/bin/sh
# Creates the Python virtual environment for the vision server and installs the ToF SDK into it.
# NO sudo is used and nothing outside ~/Delta_Robot/venv is changed.   Run on the Pi:
#     sh ~/Delta_Robot/pi/tof/setup_venv.sh
set -e
VENV="$HOME/Delta_Robot/venv"

echo "Python: $(python3 --version)"
python3 -m venv --system-site-packages "$VENV"
"$VENV/bin/python" -m pip install --upgrade pip

# Why these pins:
#  * ArducamDepthCamera (the vendor's wheel, self-contained: no apt packages needed for Python) requires numpy < 2.
#  * opencv-python-headless 4.12+ requires numpy >= 2, so we stay below it. "headless" = no GUI libraries.
"$VENV/bin/python" -m pip install "numpy<2" "opencv-python-headless<4.12" pyyaml ArducamDepthCamera

echo
"$VENV/bin/python" -c "import ArducamDepthCamera as ac, numpy, cv2; print('OK: ArducamDepthCamera', ac.__version__, '| numpy', numpy.__version__, '| opencv', cv2.__version__)"
echo
echo "Next: activate it and test the camera:"
echo "    source $VENV/bin/activate"
echo "    python ~/Delta_Robot/pi/tof/tof_check.py"
echo "(YOLO / ultralytics is installed later, in the vision-server step: it is a large download.)"
