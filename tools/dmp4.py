import mmap,os,struct,sys
fd=os.open("/dev/mem",os.O_RDONLY|os.O_SYNC)
m=mmap.mmap(fd,0x1000,mmap.MAP_SHARED,mmap.PROT_READ,offset=0x5811000)
def rd(o): return [struct.unpack("<I",m[o+4*i:o+4*i+4])[0] for i in range(256)]
a=rd(0x400); b=rd(0x000)
print(" ".join("%x:%x/%x"%(i*4,a[i],b[i]) for i in range(256) if a[i]!=b[i]))
