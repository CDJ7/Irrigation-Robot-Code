#!/bin/sh
# Compiles the real firmware file against a fake Arduino core and runs the host tests. Needs only g++.
set -e
cd "$(dirname "$0")"
g++ -std=c++17 -O1 -Wall -Wextra -Wno-unused-parameter -Ishim -x c++ test_firmware.cpp -o /tmp/fw_test_$$ 
/tmp/fw_test_$$
rm -f /tmp/fw_test_$$
