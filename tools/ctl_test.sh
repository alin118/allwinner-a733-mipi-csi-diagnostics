#!/bin/bash
D=${D:-/dev/video0}
run() { # $1 label, rest = extra v4l2-ctl args
  L="$1"; shift; rm -f /tmp/t.raw
  v4l2-ctl -d $D --set-fmt-video=width=1280,height=716,pixelformat=RG10 "$@" --stream-mmap --stream-count=90 --stream-to=/tmp/t.raw 2>&1 | tr '\r' '\n' | grep -E "fps|rror|ail" | tail -2 | tr -d '<' > /tmp/t.fps
  python3 - "$L" <<'PY'
import array,sys,os
n=1280*716*2; sz=os.path.getsize('/tmp/t.raw')//n
f=open('/tmp/t.raw','rb')
def mean(i):
    f.seek(n*i); a=array.array('H'); a.frombytes(f.read(n)); return round(sum(a[::8])/len(a[::8]),1)
print("%-20s frames=%d mean=%s  %s"%(sys.argv[1],sz,mean(sz-1),open('/tmp/t.fps').read().replace("\n"," ").strip()))
PY
}
run "default"
run "exposure=3200" --set-ctrl exposure=3200
run "exposure=12800" --set-ctrl exposure=12800
run "gain=64 (4x)" --set-ctrl gain=64
run "gain=16 (1x)" --set-ctrl gain=16
run "gain=400 (25x)" --set-ctrl gain=400
