 #!/bin/sh
# Bind the USBCAN device
slcand -o -c -f -s8 /dev/ttyUSB* slcan0
sleep 2
ifconfig slcan0 up
ifconfig slcan0 txqueuelen 100
