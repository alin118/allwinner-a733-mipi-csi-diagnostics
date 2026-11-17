#!/bin/bash
T=${1:-60}
dmesg -w > /home/orangepi/kz.log & P=$!
timeout $((T+10)) v4l2-ctl -d /dev/video0 --set-fmt-video=width=1280,height=716,pixelformat=RG10 --stream-mmap --stream-count=100000 --stream-to=/dev/null >/home/orangepi/v4lz.log 2>&1 & V=$!
sleep 2
i2ctransfer -f -y 11 w4@0x10 0x01 0x5a 0x04 0x00
i2ctransfer -f -y 11 w3@0x10 0x01 0x57 0x40
sleep 1
A0=$(grep PATCH393 /home/orangepi/kz.log | tail -1 | sed "s/.*buf0 dma=\(0x[0-9a-f]*\).*/\1/")
echo "addr $A0"
/home/orangepi/live $A0 $T
kill $P $V 2>/dev/null
