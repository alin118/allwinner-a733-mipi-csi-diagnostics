#!/bin/bash
T=${1:-30}
D=/tmp/s2.$$; mkdir -p $D
dd if=/dev/zero of=/dev/fb0 bs=4096 count=600 2>/dev/null
dmesg -C
timeout $((T+15)) v4l2-ctl -d /dev/video0 --set-fmt-video=width=1280,height=716,pixelformat=RG10 --stream-mmap --stream-count=100000 --stream-to=/dev/null >$D/c1.log 2>&1 &
sleep 2
timeout $((T+12)) v4l2-ctl -d /dev/video8 --set-fmt-video=width=1280,height=716,pixelformat=RG10 --stream-mmap --stream-count=100000 --stream-to=/dev/null >$D/c2.log 2>&1 &
sleep 2
for b in 11 9; do
  i2ctransfer -f -y $b w4@0x10 0x01 0x5a 0x04 0x00
  i2ctransfer -f -y $b w3@0x10 0x01 0x57 0x40
  i2ctransfer -f -y $b w4@0x10 0x01 0x60 $((${FL:-900}>>8)) $((${FL:-900}&255))
  i2ctransfer -f -y $b w4@0x10 0x01 0x5a $((${E:-800}>>8)) $((${E:-800}&255))
done
sleep 1
NOCLR=1 DW=512 DH=286 DX=0 DY=157 IBUS=11 /home/orangepi/livec 0xfec00000 $T 3 0xfea00000 >$D/l1.out 2>&1 &
NOCLR=1 DW=512 DH=286 DX=512 DY=157 IBUS=9 /home/orangepi/livec 0xfe000000 $T 3 0xfde00000 >$D/l2.out 2>&1 &
sleep $((T+2))
echo CAM1; head -1 $D/l1.out; echo CAM2; head -1 $D/l2.out
pkill -f "v4l2-ctl -d /dev/video" 2>/dev/null
