#!/bin/bash
#adb="$HOME/Library/Android/sdk/platform-tools/adb"
adb tcpip 5555
adb connect "$@"
