#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <sys/mman.h>
static const uint8_t pat[16]={0xA5,0x5A,0xC3,0x3C,0xA5,0x5A,0xC3,0x3C,0x96,0x69,0xF0,0x0F,0x12,0x34,0x56,0x78};
static int gone(uint8_t*p){volatile uint32_t*q=(volatile uint32_t*)p;const uint32_t*r=(const uint32_t*)pat;return q[0]!=r[0]||q[1]!=r[1]||q[2]!=r[2]||q[3]!=r[3];}
static void mark(uint8_t*p){volatile uint32_t*q=(volatile uint32_t*)p;const uint32_t*r=(const uint32_t*)pat;for(int i=0;i<4;i++)q[i]=r[i];}
static double nowt(void){struct timespec z;clock_gettime(CLOCK_MONOTONIC,&z);return z.tv_sec+z.tv_nsec/1e9;}
int main(int argc,char**argv){
  unsigned long A0=strtoul(argv[1],0,16); double T=atof(argv[2]);
  int fd=open("/dev/mem",O_RDWR|O_SYNC); size_t len=282*4096;
  uint8_t*m=mmap(0,len,PROT_READ|PROT_WRITE,MAP_SHARED,fd,A0);
  const size_t MK=768+700*1600; mark(m+MK);
  int bins[200]={0}; double t0=nowt(); int n=0;
  while(nowt()-t0<T){ if(gone(m+MK)){ int b=(int)((nowt()-t0)*2); if(b<200)bins[b]++; n++; mark(m+MK);} else usleep(300); }
  printf("total %d avg %.1f fps\nper0.5s:",n,n/T);
  for(int i=0;i<(int)(T*2);i++) printf(" %d",bins[i]);
  printf("\n"); return 0;
}
