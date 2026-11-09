#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#define OW 640
#define OH 358
#define OX 192
#define OY 100
static const uint8_t pat[16]={0xA5,0x5A,0xC3,0x3C,0xA5,0x5A,0xC3,0x3C,0x96,0x69,0xF0,0x0F,0x12,0x34,0x56,0x78};
static int gone(uint8_t*p){volatile uint32_t*q=(volatile uint32_t*)p;const uint32_t*r=(const uint32_t*)pat;return q[0]!=r[0]||q[1]!=r[1]||q[2]!=r[2]||q[3]!=r[3];}
static void mark(uint8_t*p){volatile uint32_t*q=(volatile uint32_t*)p;const uint32_t*r=(const uint32_t*)pat;for(int i=0;i<4;i++)q[i]=r[i];}
static double nowt(void){struct timespec z;clock_gettime(CLOCK_MONOTONIC,&z);return z.tv_sec+z.tv_nsec/1e9;}
static volatile int req=0,r_fl,r_e,r_v; static int ifd_g=-1;
static void*aew(void*x){ for(;;){ if(req){ int fl=r_fl,e=r_e,v=r_v; req=0; uint8_t w[4]; int rr; w[0]=1;w[1]=0x60;w[2]=fl>>8;w[3]=fl&255; rr=write(ifd_g,w,4); w[1]=0x5a;w[2]=e>>8;w[3]=e&255; rr=write(ifd_g,w,4); w[1]=0x57;w[2]=v; rr=write(ifd_g,w,3); (void)rr; } else usleep(5000); } return 0; }
static double gt[40],gd[40],gp[40],ga[40],gc[40]; static long gs[40]; static int ng=0;
int main(int argc,char**argv){
  unsigned long A0=strtoul(argv[1],0,16); double T=atof(argv[2]);
  int pt=argc>3?atoi(argv[3]):3; unsigned long A1=argc>4?strtoul(argv[4],0,16):A0;
  size_t len=282*4096;
  int fd=open("/dev/mem",O_RDWR|O_SYNC); int fb=open("/dev/fb0",O_RDWR);
  uint8_t*mm[2];
  mm[0]=mmap(0,len,PROT_READ|PROT_WRITE,MAP_SHARED,fd,A0);
  mm[1]=mmap(0,len,PROT_READ|PROT_WRITE,MAP_SHARED,fd,A1);
  uint32_t*f=mmap(0,1024*1200*4,PROT_READ|PROT_WRITE,MAP_SHARED,fb,0);
  if(mm[0]==MAP_FAILED||mm[1]==MAP_FAILED||f==MAP_FAILED){perror("mmap");return 1;}
  const size_t MK=768+715*1600;
  static uint8_t buf[282*4096]; static int cx[1280];
  for(int p=0;p<1280;p++) cx[p]=(p/4)*5+(p%4);
  static const int Rp[4]={0,1,2,3},Bp[4]={3,2,1,0},G1[4]={1,0,0,1},G2[4]={2,3,3,2};
  static uint8_t L[256]; for(int i=0;i<256;i++) L[i]=(uint8_t)(255.0*pow(i/255.0,0.45)+0.5);
  double ar=0,ag=0,ab=0, TT=1365;
  int ifd=open("/dev/i2c-11",O_RDWR); if(ifd>=0) ioctl(ifd,0x0706,0x10); ifd_g=ifd; static int lfl=-1,le=-1,lv=-1; if(ifd>=0){pthread_t th; pthread_create(&th,0,aew,0);}
  int noae=getenv("NOAE")!=NULL;
  memset(f,0,1024*600*4); mark(mm[0]+MK); mark(mm[1]+MK);
  struct timespec a,b,q; clock_gettime(CLOCK_MONOTONIC,&a); int n=0; long cnt[2]={0,0}; double lastt=0,tdraw=0,tae=0,tc=0;
  for(;;){
    clock_gettime(CLOCK_MONOTONIC,&b);
    double t=(b.tv_sec-a.tv_sec)+(b.tv_nsec-a.tv_nsec)/1e9;
    if(t>=T){printf("frames %d fps %.1f buf0 %ld buf1 %ld\n",n,n/t,cnt[0],cnt[1]);
      for(int i=0;i<ng;i++) printf("gap t=%.2f dt=%.3f spins=%ld draw=%.3f ae=%.3f copy=%.3f\n",gt[i],gd[i],gs[i],gp[i],ga[i],gc[i]);
      break;}
    int got=-1; long spins=0;
    while(got<0){ for(int i=0;i<2;i++){ if(gone(mm[i]+MK)){got=i;break;} } if(got<0){ usleep(200); if(++spins>20000){got=0;break;} } }
    double c0=nowt(); memcpy(buf,mm[got],len); tc=nowt()-c0; mark(mm[got]+MK); cnt[got]++;
    clock_gettime(CLOCK_MONOTONIC,&q); double tn=q.tv_sec+q.tv_nsec/1e9;
    if(lastt>0 && tn-lastt>0.09 && ng<40){gt[ng]=tn-(a.tv_sec+a.tv_nsec/1e9);gd[ng]=tn-lastt;gs[ng]=spins;gp[ng]=tdraw;ga[ng]=tae;gc[ng]=tc;ng++;}
    lastt=tn;
    #define PX(q,x,y) buf[768+(2*(y)+((q)>>1))*1600+cx[2*(x)+((q)&1)]]
    long sr=0,sg=0,sb=0,cnt2=0;
    for(int y=0;y<OH;y+=8) for(int x=0;x<OW;x+=8){
      sr+=PX(Rp[pt],x,y); sb+=PX(Bp[pt],x,y); sg+=(PX(G1[pt],x,y)+PX(G2[pt],x,y))/2; cnt2++; }
    double bl=16, mr=sr/(double)cnt2-bl, mg=sg/(double)cnt2-bl, mb=sb/(double)cnt2-bl;
    if(mr<1)mr=1; if(mg<1)mg=1; if(mb<1)mb=1;
    double norm=1.5; double a0=nowt();
    if(!noae && n%3==0 && ifd>=0){
      double ratio=60.0/mg; if(ratio>1.4)ratio=1.4; if(ratio<0.7)ratio=0.7;
      if(ratio>0.9&&ratio<1.1)ratio=1; else ratio=sqrt(ratio);
      TT*=ratio; if(TT<20)TT=20; if(TT>12000)TT=12000;
      double ex,g; if(TT<=1500){ex=TT;g=1;} else {ex=1500;g=TT/1500;}
      int v=(int)(256-256/g); if(v>224)v=224; int e=(int)ex; int fl=e+8<768?768:e+8;
      if((fl!=lfl||e!=le||v!=lv) && !req){ r_fl=fl;r_e=e;r_v=v;req=1;lfl=fl;le=e;lv=v; }
    }
    tae=nowt()-a0;
    double tr=mg/mr*norm, tg=norm, tb=mg/mb*norm;
    if(ar==0){ar=tr;ag=tg;ab=tb;}else{ar+=(tr-ar)*0.03;ag+=(tg-ag)*0.03;ab+=(tb-ab)*0.03;}
    double d0=nowt();
    for(int y=0;y<OH;y++){
      uint32_t*o=f+(y+OY)*1024+OX;
      for(int x=0;x<OW;x++){
        int r=(PX(Rp[pt],x,y)-bl)*ar, g=((PX(G1[pt],x,y)+PX(G2[pt],x,y))/2-bl)*ag, bb=(PX(Bp[pt],x,y)-bl)*ab;
        if(r<0)r=0;if(r>255)r=255;if(g<0)g=0;if(g>255)g=255;if(bb<0)bb=0;if(bb>255)bb=255;
        r=L[r];g=L[g];bb=L[bb];
        o[x]=0xFF000000u|(r<<16)|(g<<8)|bb;
      }
    }
    tdraw=nowt()-d0;
    n++;
  }
  return 0;
}
