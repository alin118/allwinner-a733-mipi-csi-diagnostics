import mmap,os,struct
fd=os.open("/dev/mem",os.O_RDONLY|os.O_SYNC)
m=mmap.mmap(fd,0x1000,mmap.MAP_SHARED,mmap.PROT_READ,offset=0x5908000)
for a in range(0x100,0x300,0x40):
    print("%x:"%a," ".join("%x"%struct.unpack("<I",m[a+4*i:a+4*i+4])[0] for i in range(16)))
