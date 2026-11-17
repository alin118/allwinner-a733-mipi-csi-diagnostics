#!/bin/bash
T=${1:-8}
D=/tmp/r2.$$; mkdir -p $D
dmesg -C
timeout $((T+12)) v4l2-ctl -d /dev/video0 --set-fmt-video=width=1280,height=716,pixelformat=RG10 --stream-mmap --stream-count=100000 --stream-to=/dev/null >$D/c1.log 2>&1 &
sleep 2
timeout $((T+8)) v4l2-ctl -d /dev/video8 --set-fmt-video=width=1280,height=716,pixelformat=RG10 --stream-mmap --stream-count=100000 --stream-to=/dev/null >$D/c2.log 2>&1 &
sleep 2
for b in 11 9; do
  i2ctransfer -f -y $b w4@0x10 0x01 0x5a 0x04 0x00
  i2ctransfer -f -y $b w3@0x10 0x01 0x57 0x40
  i2ctransfer -f -y $b w4@0x10 0x01 0x60 $((${FL:-900}>>8)) $((${FL:-900}&255))
  i2ctransfer -f -y $b w4@0x10 0x01 0x5a $((${E:-800}>>8)) $((${E:-800}&255))
done
sleep 1
g(){ dmesg | grep PATCH410 | grep "rx$1:buf$2" | sed 's/.*dma_addr is \(0x[0-9a-f]*\).*/\1/' | tail -1; }
A0=0xfec00000; A1=0xfea00000; B0=0xfe000000; B1=0xfde00000
echo "cam1 $A0 $A1 | cam2 $B0 $B1"
/home/orangepi/arrive $A0 $T 3 $A1 > $D/a1.out 2>&1 &
/home/orangepi/arrive $B0 $T 3 $B1 > $D/a2.out 2>&1 &
sleep $((T+2))
echo CAM1; tail -2 $D/a1.out; echo CAM2; tail -2 $D/a2.out
pkill -f "v4l2-ctl -d /dev/video" 2>/dev/null
