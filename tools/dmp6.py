import mmap,os,struct,sys
fd=os.open("/dev/mem",os.O_RDONLY|os.O_SYNC)
m=mmap.mmap(fd,0x1000,mmap.MAP_SHARED,mmap.PROT_READ,offset=0x5800000)
print(" ".join("%x"%struct.unpack("<I",m[4*i:4*i+4])[0] for i in range(256)))
