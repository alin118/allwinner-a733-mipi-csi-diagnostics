import mmap,os,struct
fd=os.open("/dev/mem",os.O_RDONLY|os.O_SYNC)
m=mmap.mmap(fd,0x1000,mmap.MAP_SHARED,mmap.PROT_READ,offset=0x5800000)
print("top@0x800:"," ".join("%x"%struct.unpack("<I",m[0x800+4*i:0x800+4*i+4])[0] for i in range(8)))
