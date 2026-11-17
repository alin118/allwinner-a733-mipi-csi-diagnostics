import mmap,os,struct,sys
fd=os.open("/dev/mem",os.O_RDONLY|os.O_SYNC)
b=int(sys.argv[1],16)
m=mmap.mmap(fd,0x1000,mmap.MAP_SHARED,mmap.PROT_READ,offset=b&~0xfff)
o=b&0xfff
r=lambda off:struct.unpack("<I",m[o+off:o+off+4])[0]
print(hex(b),"dbg0xf0=%x"%r(0xf0),"freq0x34=%x"%r(0x34),"0x30=%x"%r(0x30))
