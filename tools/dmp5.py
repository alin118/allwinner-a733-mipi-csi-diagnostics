import mmap,os,struct
fd=os.open("/dev/mem",os.O_RDONLY|os.O_SYNC)
m=mmap.mmap(fd,0x2000,mmap.MAP_SHARED,mmap.PROT_READ,offset=0x5820000)
rd=lambda o:[struct.unpack("<I",m[o+4*i:o+4*i+4])[0] for i in range(1024)]
a=rd(0x1000); b=rd(0)
print(" ".join("%x:%x/%x"%(i*4,a[i],b[i]) for i in range(1024) if a[i]!=b[i]))
