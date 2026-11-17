import mmap,os,struct
fd=os.open("/dev/mem",os.O_RDONLY|os.O_SYNC)
m=mmap.mmap(fd,0x1000,mmap.MAP_SHARED,mmap.PROT_READ,offset=0x5821000)
print(" ".join("%x:%x"%(4*i,struct.unpack("<I",m[4*i:4*i+4])[0]) for i in range(0x1000//4) if struct.unpack("<I",m[4*i:4*i+4])[0]))
