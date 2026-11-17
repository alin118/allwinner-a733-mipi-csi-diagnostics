#!/bin/bash
T=${1:-60}
dmesg -w > /home/orangepi/kz.log & P=$!
timeout $((T+10)) v4l2-ctl -d /dev/video0 --set-fmt-video=width=1280,height=716,pixelformat=RG10 --stream-mmap --stream-count=100000 --stream-to=/dev/null >/home/orangepi/v4lz.log 2>&1 & V=$!
sleep 2
i2ctransfer -f -y 11 w4@0x10 0x01 0x5a 0x04 0x00
i2ctransfer -f -y 11 w3@0x10 0x01 0x57 0x40
if [ -n "$FL" ]; then i2ctransfer -f -y 11 w4@0x10 0x01 0x60 $((FL>>8)) $((FL&255)); fi
if [ -n "$E" ]; then i2ctransfer -f -y 11 w4@0x10 0x01 0x5a $((E>>8)) $((E&255)); fi
sleep 1
L=$(grep PATCH393 /home/orangepi/kz.log | tail -1)
A0=$(echo "$L" | sed "s/.*buf0 dma=\(0x[0-9a-f]*\).*/\1/")
A1=$(echo "$L" | sed "s/.*buf1 dma=\(0x[0-9a-f]*\).*/\1/")
echo "addr $A0 $A1"
/home/orangepi/livec $A0 $T ${2:-3} $A1
kill $P $V 2>/dev/null
