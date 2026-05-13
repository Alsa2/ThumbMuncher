tio -b 115200 /dev/serial/by-id/usb-Raspberry_Pi_Pico_E465BC8247320722-if00

mavproxy.py --master=/dev/ttyACM2,57600   --out=udp:127.0.0.1:14550   --out=udp:127.0.0.1:14560

chat
https://chatgpt.com/c/69f266a2-ca4c-83ea-aee8-912acbe82889