# !/bin/sh

vendor="04b0"
product="0429"

camLeftBus=$1
camRightBus=$2

camLeftPort=$(lsusb | awk  '/Nikon/' | awk "\$2==\"$camLeftBus\"" | awk  ' gsub(":","")' | awk '{print $4}')
camRightPort=$(lsusb | awk  '/Nikon/' | awk "\$2==\"$camRightBus\"" | awk  ' gsub(":","")' | awk '{print $4}')

rosparam set /RH/usbcam_left usb:$camLeftBus,$camLeftPort
rosparam set /RH/usbcam_right usb:$camRightBus,$camRightPort
