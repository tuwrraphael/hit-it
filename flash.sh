source ~/ncs/.venv/bin/activate
nrfutil pkg generate --hw-version 52 --sd-req=0x00 --application ./build/zephyr/zephyr.hex --application-version 1 hit-it-zephyr.zip
nrfutil dfu usb-serial -pkg hit-it-zephyr.zip -p /dev/ttyACM0