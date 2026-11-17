#!/bin/bash
# Самотест драйвера IMX219: режимы, fps, кроп/биннинг. Запуск: sudo ./selftest.sh [/dev/video0]
V=${1:-/dev/video0}; bad=0
chk(){ [ "$2" = "$3" ] && echo "OK   $1" || { echo "FAIL $1 (получено $2, ожидалось $3)"; bad=1; }; }
cap(){ rm -f /tmp/st.raw; timeout 15 v4l2-ctl -d $V "$@" --stream-mmap --stream-count=$N --stream-to=/tmp/st.raw >/dev/null 2>&1; stat -c %s /tmp/st.raw 2>/dev/null || echo 0; }
N=30
echo "== режимы (BG10, 30 кадров)"
for m in 3280x2464 1920x1080 1640x1232 1280x960 1280x720 640x480; do
  W=${m%x*}; H=${m#*x}
  chk "$m" "$(cap --set-fmt-video=width=$W,height=$H,pixelformat=BG10)" $((W*H*2*N))
done
echo "== 8-битный формат BA81"
chk "1280x720 BA81" "$(cap --set-fmt-video=width=1280,height=720,pixelformat=BA81)" $((1280*720*N))
echo "== кроп и биннинг (окно 1600x1200 из центра)"
S="--set-selection=target=crop,left=840,top=632,width=1600,height=1200"
chk "кроп 1600x1200" "$(cap $S --set-fmt-video=width=1600,height=1200,pixelformat=BG10)" $((1600*1200*2*N))
chk "кроп+bin 800x600" "$(cap $S --set-fmt-video=width=800,height=600,pixelformat=BG10)" $((800*600*2*N))
chk "кроп 3280x2464 (полный)" "$(cap --set-selection=target=crop,left=0,top=0,width=3280,height=2464 --set-fmt-video=width=3280,height=2464,pixelformat=BG10)" $((3280*2464*2*N))
echo "== скорость (1280x720, 60 fps, 5 с)"
s=$(date +%s.%N); timeout 20 v4l2-ctl -d $V --set-selection=target=crop,left=0,top=0,width=0,height=0 --set-fmt-video=width=1280,height=720,pixelformat=BG10 --set-parm=60 --stream-mmap --stream-count=300 >/dev/null 2>&1; e=$(date +%s.%N)
f=$(echo "300 $s $e" | awk '{printf "%d", $1/($3-$2)}'); [ "$f" -ge 52 ] && echo "OK   ${f} fps" || { echo "FAIL ${f} fps"; bad=1; }
[ $bad = 0 ] && echo "ВСЁ В ПОРЯДКЕ" || echo "ЕСТЬ ОШИБКИ"
exit $bad
