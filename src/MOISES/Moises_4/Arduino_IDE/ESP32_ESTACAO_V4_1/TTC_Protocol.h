#ifndef TTC_PROTOCOL_V4_H
#define TTC_PROTOCOL_V4_H
#include <stdint.h>
#include <stddef.h>
#include <string.h>
namespace ttc {
// Radio V5: both endpoints must be updated together. STM/USB stay at version 1.
// POLL[26] acknowledges a specific result state; DOWNLINK[92..95] advertises
// the next globally available transaction ID for this RP2040 boot.
static constexpr uint8_t VERSION=5, MAX_STM=56, MAX_PAYLOAD=96;
static constexpr size_t HEADER=18, RF_MAX=116;
static constexpr uint8_t POLL_SIZE=28, DL_SIZE=96;
enum Kind : uint8_t { POLL=1, DOWNLINK=2 };
enum CmdState : uint8_t { NONE=0,UART_PENDING=1,UART_SENT=2,STM_RESPONSE=3,
 BLOCKED=4,BUSY=5,TARGET_MISMATCH=6,STM_TIMEOUT_UNKNOWN=7,UNKNOWN_ID=8,
 UART_TX_FAILED_UNKNOWN=9,ID_CONFLICT=10,SEQ_QUARANTINED=11 };
enum Phase : uint8_t { QUEUED=0,RF_SENT=1,RP_QUEUED=2,RP_SENT=3,
 REAL_RESPONSE=4,LOCAL_BLOCKED=5,REMOTE_BUSY=6,LINK_UNKNOWN=7,
 STM_UNKNOWN=8,BOOT_UNKNOWN=9,LOCAL_BUSY=10,CONFLICT=11,RESULT_EXPIRED=12 };
inline uint16_t read16(const uint8_t*p){return uint16_t(p[0])|(uint16_t(p[1])<<8);}
inline uint32_t read32(const uint8_t*p){return uint32_t(p[0])|(uint32_t(p[1])<<8)|(uint32_t(p[2])<<16)|(uint32_t(p[3])<<24);}
inline int16_t readI16(const uint8_t*p){return static_cast<int16_t>(read16(p));}
inline int32_t readI32(const uint8_t*p){return static_cast<int32_t>(read32(p));}
inline void write16(uint8_t*p,uint16_t x){p[0]=uint8_t(x);p[1]=uint8_t(x>>8);}
inline void write32(uint8_t*p,uint32_t x){for(uint8_t i=0;i<4;i++)p[i]=uint8_t(x>>(8*i));}
constexpr uint16_t crc16(const uint8_t*p,size_t n){
 uint16_t c=0xFFFF;
 for(size_t i=0;i<n;i++){c^=uint16_t(p[i])<<8;for(uint8_t j=0;j<8;j++)c=(c&0x8000)?uint16_t((c<<1)^0x1021):uint16_t(c<<1);}
 return c;
}
static constexpr uint8_t CRC_CHECK[]={49,50,51,52,53,54,55,56,57};
static_assert(crc16(CRC_CHECK,9)==0x29B1,"CRC16 check vector failed");
static constexpr uint8_t PING_CHECK[]={0x01,0x02,0x00,0x01,0x10};
static_assert(crc16(PING_CHECK,5)==0x7735,"STM PING CRC differs from reference");
inline bool validSTM(const uint8_t*p,size_t n){
 return p && n>=8 && n<=MAX_STM && p[0]==0xA5 && p[1]==0x5A && p[2]==1 &&
 p[5]<=48 && n==size_t(p[5])+8 && crc16(p+2,n-4)==read16(p+n-2);
}
inline bool allowedCommand(const uint8_t*p,size_t n){
 return validSTM(p,n) && n==9 && p[3]==2 && p[5]==1 &&
 (p[6]==0x10 || p[6]==0x02 || p[6]==0x03);
}
inline size_t makeSTM(uint8_t type,uint8_t seq,const uint8_t*p,uint8_t len,uint8_t*out){
 if(!out || len>48 || (len&&!p))return 0;
 out[0]=0xA5;out[1]=0x5A;out[2]=1;out[3]=type;out[4]=seq;out[5]=len;
 if(len)memcpy(out+6,p,len);write16(out+6+len,crc16(out+2,size_t(4+len)));
 return size_t(len)+8;
}
struct Frame {uint8_t kind=0,flags=0,length=0;uint32_t boot=0,uptime=0;
 uint16_t counter=0;uint8_t payload[MAX_PAYLOAD]={};};
inline size_t encode(uint8_t kind,uint32_t boot,uint16_t count,uint32_t uptime,
 const uint8_t*payload,uint8_t len,uint8_t*out){
 if(!out || !payload || !boot || !((kind==POLL&&len==POLL_SIZE)||(kind==DOWNLINK&&len==DL_SIZE)))return 0;
 out[0]=0xFE;out[1]=0x6B;out[2]=0x28;out[3]=0x40;
 out[4]=VERSION;out[5]=kind;out[6]=0;out[7]=len;
 write32(out+8,boot);write16(out+12,count);write32(out+14,uptime);
 memcpy(out+HEADER,payload,len);write16(out+HEADER+len,crc16(out+4,HEADER+len-4));
 return HEADER+len+2;
}
inline bool decode(const uint8_t*p,size_t n,Frame&out){
 if(!p || n<HEADER+2 || n>RF_MAX || p[0]!=0xFE || p[1]!=0x6B || p[2]!=0x28 || p[3]!=0x40 || p[4]!=VERSION || p[6]!=0)return false;
 const uint8_t len=p[7],kind=p[5];
 if(!((kind==POLL&&len==POLL_SIZE)||(kind==DOWNLINK&&len==DL_SIZE)) || n!=HEADER+len+2 || !read32(p+8) || crc16(p+4,n-6)!=read16(p+n-2))return false;
 out.kind=kind;out.flags=p[6];out.length=len;out.boot=read32(p+8);
 out.counter=read16(p+12);out.uptime=read32(p+14);memcpy(out.payload,p+HEADER,len);return true;
}
// Known UART/local record lengths reject a false, plausible LEN immediately.
// validSTM remains the generic CRC/length validator for complete raw frames.
inline bool headerLengthKnown(uint8_t type,uint8_t length){
 switch(type){
 case 0x01:return length==29;
 case 0x02:return length==1;
 case 0x03:case 0x04:return length==2;
 case 0x05:return length==10;
 case 0x06:return length==20 || length==36;
 case 0x07:return length==38;
 case 0x08:return length==5;
 case 0x20:return length==18;
 case 0x21:return length==32;
 case 0x22:return length==14;
 default:return false;
 }
}
// Sliding parser: CRC failure discards one byte, retaining any following sync.
// Hardware FIFO must be drained continuously; no CPU-read-gap timeout is used.
// After a successful push, drain pop() until false: recovery may expose several
// complete frames that were hidden in a rejected candidate.
class StmParser {
 uint8_t buffer[MAX_STM]={};uint8_t used=0;
 void discard(uint8_t n){if(n>=used){used=0;return;}memmove(buffer,buffer+n,used-n);used=uint8_t(used-n);}
public:
 uint32_t valid=0,badCrc=0,resync=0;
 bool pop(uint8_t*out,uint8_t&n){
  n=0;
  for(;;){
   while(used && buffer[0]!=0xA5){discard(1);++resync;}
   if(used<2)return false;
   if(buffer[1]!=0x5A){discard(1);++resync;continue;}
   if(used<6)return false;
   if(buffer[2]!=1 || buffer[5]>48 || !headerLengthKnown(buffer[3],buffer[5])){discard(1);++resync;continue;}
   const uint8_t full=uint8_t(buffer[5]+8);if(used<full)return false;
   if(!validSTM(buffer,full)){++badCrc;discard(1);continue;}
   if(out)memcpy(out,buffer,full);n=full;discard(full);++valid;return true;
  }
 }
 bool push(uint8_t b,uint8_t*out,uint8_t&n){
  if(used==MAX_STM){discard(1);++resync;}buffer[used++]=b;
  return pop(out,n);
 }
};
inline bool selfTest(){
 const uint8_t ping[9]={0xA5,0x5A,1,2,0,1,0x10,0x35,0x77};
 if(!allowedCommand(ping,9))return false;
 uint8_t v[9];const uint8_t vc=1;makeSTM(2,1,&vc,1,v);if(allowedCommand(v,9))return false;
 uint8_t payload[POLL_SIZE]={},wire[RF_MAX]={};Frame f;
 const size_t n=encode(POLL,0x12345678,65535,0xFFFFFFF0,payload,POLL_SIZE,wire);
 if(n!=48 || !decode(wire,n,f) || f.boot!=0x12345678 || f.counter!=65535)return false;
 wire[20]^=1;if(decode(wire,n,f))return false;
 uint8_t downlink[DL_SIZE]={};write32(downlink+92,123456);
 const size_t dl=encode(DOWNLINK,0x12345678,1,25,downlink,DL_SIZE,wire);
 if(dl!=116 || !decode(wire,dl,f) || read32(f.payload+92)!=123456)return false;
 wire[4]=4;if(decode(wire,dl,f))return false;
 StmParser parser;uint8_t out[MAX_STM],got=0;parser.push(0x00,out,got);parser.push(0xA5,out,got);
 bool result=false;for(size_t i=0;i<9;i++)result=parser.push(ping[i],out,got);
 return result && got==9 && memcmp(out,ping,9)==0;
}
} // namespace ttc
#endif
