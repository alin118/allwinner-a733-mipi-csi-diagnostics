import array,sys,os
f,W,H=sys.argv[1],int(sys.argv[2]),int(sys.argv[3]); n=W*H*2
sz=os.path.getsize(f)//n; fh=open(f,'rb'); fh.seek(n*(sz-1)); a=array.array('H'); a.frombytes(fh.read(n))
c=(H//16)*(W//16)
v=sum(abs(a[y*W+x]-a[(y+1)*W+x]) for y in range(0,H-1,16) for x in range(0,W,16))/c
h=sum(abs(a[y*W+x]-a[y*W+x+1]) for y in range(0,H,16) for x in range(0,W-1,16))/c
print("frames",sz,"min",min(a),"max",max(a),"mean",round(sum(a)/len(a),1),"| diff vert",round(v,1),"horiz",round(h,1))
