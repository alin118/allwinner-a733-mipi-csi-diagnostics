import mmap,os,struct,sys
fd=os.open("/dev/mem",os.O_RDONLY|os.O_SYNC)
def rd(base,n):
    m=mmap.mmap(fd,0x1000,mmap.MAP_SHARED,mmap.PROT_READ,offset=base&~0xfff)
    o=base&0xfff
    return [struct.unpack("<I",m[o+4*i:o+4*i+4])[0] for i in range(n)]
for name,b,n in [("prs",int(sys.argv[1],16),24),("phy",int(sys.argv[2],16),12)]:
    print(name,hex(b)," ".join("%x"%v for v in rd(b,n)))
