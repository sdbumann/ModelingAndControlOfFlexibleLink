#!/bin/bash
cd "$(dirname "$0")"


clang -shared -undefined dynamic_lookup -o PID.so /Users/baumann/Downloads/TP_control_systems/doNotShareThis/PID.c

