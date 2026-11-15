/* ESP32 ground station: radio V5 <-> version 1 binary USB MATLAB.
 * RA02 CS5 RST14 DIO0=26 SCK18 MISO19 MOSI23. STM32 unchanged.
 * No V, no fabricated STM reply, no automatic STM re-execution.
 */
#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <esp_random.h>
#include <math.h>
#include "TTC_Protocol.h"
#include "RadioAsync.h"
static constexpr uint32_t POLL_MS=600,DOWNLINK_WAIT_MS=450;
static constexpr uint8_t USB_Q=16,JOURNAL_COUNT=16;
static SX1278 radio=new Module(5,26,14,RADIOLIB_NC,SPI,SPISettings(4000000,MSBFIRST,SPI_MODE0));
static ttc::RadioAsync rf(radio);
static ttc::StmParser usbParser;
static bool protocolOK=false,radioOK=false,haveDownlink=false,waitingDL=false,wasTX=false;
static uint32_t boot=0,spaceBoot=0,lastDLAt=0,nextPollAt=0,waitSince=0,lastInitAt=0,lastLinkReport=0;
static uint16_t rfCounter=0;static uint32_t polls=0,downlinks=0,nextTxn=0;
static uint8_t localSeq=0;
static uint32_t ackGround=0,ackId=0;
static uint8_t ackState=ttc::NONE,journalCursor=0,pollTurn=0;
static bool changingBoot=false;
static uint16_t lastDLCounter=0;static uint32_t lastDLUptime=0;
static bool haveSTM=false,usbOverrun=false;static uint32_t lastSTMUptime=0;static uint8_t lastSTMSeq=0;
static uint8_t lastLink[32]={};
struct Pending {
 bool exists=false,active=false,accepted=false,rfNotified=false,uncertain=false;
 bool terminalReported=false,responseQueued=false;
 uint8_t raw[9]={},state=255;uint32_t id=0,created=0,acceptedAt=0;uint16_t attempts=0;
} pending;
static Pending journal[JOURNAL_COUNT];
// Explicit declarations avoid Arduino-generated prototypes before local types.
static bool eventFor(const Pending& item,uint8_t phase,uint8_t reason);
static bool retire(Pending& item,uint8_t phase,uint8_t reason);
static bool replyMatches(const Pending& item,const uint8_t*p,uint8_t n);
static Pending* nextJournal();
struct UsbFrame{uint8_t n=0,data[ttc::MAX_STM]={};};
static void slot(UsbFrame&dst,bool&ready,const uint8_t*p,uint8_t n);
static UsbFrame critical[USB_Q],tmSlot,gpsSlot,linkSlot,usbOut;
static uint8_t qHead=0,qTail=0,qCount=0,usbPos=0;
static bool tmReady=false,gpsReady=false,linkReady=false;
static bool enqueueCritical(const uint8_t*p,uint8_t n){
 if(n>ttc::MAX_STM || qCount==USB_Q){usbOverrun=true;return false;}
 critical[qTail].n=n;memcpy(critical[qTail].data,p,n);qTail=uint8_t((qTail+1)%USB_Q);++qCount;return true;
}
static bool localEvent(uint32_t id,uint8_t cmd,uint8_t seq,uint8_t phase,uint8_t reason,uint32_t elapsed,uint16_t attempts){
 uint8_t p[14]={},f[22];ttc::write32(p,id);p[4]=cmd;p[5]=seq;p[6]=phase;p[7]=reason;
 ttc::write32(p+8,elapsed);ttc::write16(p+12,attempts);ttc::makeSTM(0x22,seq,p,14,f);return enqueueCritical(f,22);
}
static bool eventFor(const Pending& item,uint8_t phase,uint8_t reason){
 return localEvent(item.id,item.raw[6],item.raw[4],phase,reason,uint32_t(millis()-item.created),item.attempts);
}
static bool retire(Pending& item,uint8_t phase,uint8_t reason){
 if(!eventFor(item,phase,reason))return false;
 item.active=false;
 item.uncertain=(phase==ttc::LINK_UNKNOWN||phase==ttc::STM_UNKNOWN||phase==ttc::BOOT_UNKNOWN);
 return true; // Retired records are queried without ever resending their command.
}
static bool archivePending(){
 if(!pending.exists || !pending.uncertain)return true;
 int8_t index=-1;
 for(uint8_t i=0;i<JOURNAL_COUNT;i++)if(!journal[i].exists){index=int8_t(i);break;}
 if(index<0){
  // Only expire a record whose indeterminate outcome was already reported.
  // Its UART command/SEQ quarantine and the RP global watermark remain intact.
  for(uint8_t i=0;i<JOURNAL_COUNT;i++)if(journal[i].terminalReported &&
      (index<0 || int32_t(journal[i].created-journal[index].created)<0))index=int8_t(i);
  if(index<0 || !eventFor(journal[index],ttc::RESULT_EXPIRED,ttc::UNKNOWN_ID))return false;
 }
 journal[index]=pending;return true;
}
static Pending* nextJournal(){
 for(uint8_t i=0;i<JOURNAL_COUNT;i++){
  const uint8_t index=journalCursor;journalCursor=uint8_t((journalCursor+1)%JOURNAL_COUNT);
  if(journal[index].exists)return &journal[index];
 }
 return nullptr;
}
static void slot(UsbFrame&dst,bool&ready,const uint8_t*p,uint8_t n){dst.n=n;memcpy(dst.data,p,n);ready=true;}
static void pumpUSB(){
 if(usbPos>=usbOut.n){
  usbOut.n=0;usbPos=0;
  if(qCount){usbOut=critical[qHead];qHead=uint8_t((qHead+1)%USB_Q);--qCount;}
  else if(tmReady){usbOut=tmSlot;tmReady=false;}
  else if(gpsReady){usbOut=gpsSlot;gpsReady=false;}
  else if(linkReady){usbOut=linkSlot;linkReady=false;}
 }
 const int free=Serial.availableForWrite();if(free<=0 || usbPos>=usbOut.n)return;
 size_t n=usbOut.n-usbPos;if(n>32)n=32;if(n>size_t(free))n=size_t(free);
 usbPos=uint8_t(usbPos+Serial.write(usbOut.data+usbPos,n));
}
static uint16_t age16(uint32_t age){return age>65535?65535:uint16_t(age);}
static int16_t db10(float x){if(!isfinite(x))return 0;float y=x*10.0f;if(y>32767)y=32767;if(y<-32768)y=-32768;return int16_t(lroundf(y));}
static void reportLink(){
 uint8_t f[40];ttc::write16(lastLink+14,haveDownlink?age16(uint32_t(millis()-lastDLAt)):65535);
 lastLink[23]=uint8_t((spaceBoot?1:0)|(rf.transmitting()?2:0)|(usbOverrun?4:0)|
   (radioOK&&rf.lastRx!=0?8:0)|(!radioOK?16:0));
 ttc::write32(lastLink+24,polls);ttc::write32(lastLink+28,downlinks);
 ttc::makeSTM(0x21,localSeq++,lastLink,32,f);slot(linkSlot,linkReady,f,40);lastLinkReport=millis();
}
static void processCommand(const uint8_t*p,uint8_t n){
 if(n<6 || p[3]!=2)return;
 const uint8_t code=n>=7?p[6]:0,seq=p[4];
 if(!ttc::allowedCommand(p,n)){localEvent(0,code,seq,ttc::LOCAL_BLOCKED,ttc::BLOCKED,0,0);return;}
 if(pending.active){localEvent(0,code,seq,ttc::LOCAL_BUSY,ttc::BUSY,0,0);return;}
 if(!protocolOK || !radioOK || changingBoot || !spaceBoot || !haveDownlink || uint32_t(millis()-lastDLAt)>2000 || nextTxn==0){
  localEvent(0,code,seq,ttc::LOCAL_BLOCKED,ttc::TARGET_MISMATCH,0,0);return;
 }
 if(lastLink[21]&2){localEvent(0,code,seq,ttc::LOCAL_BUSY,ttc::BUSY,0,0);return;}
 // Reserve room for an optional expiry event and the new QUEUED event before
 // moving pending into the journal; a failed enqueue must not archive it twice.
 if(qCount>USB_Q-2 || !archivePending()){localEvent(0,code,seq,ttc::LOCAL_BUSY,ttc::BUSY,0,0);return;}
 pending=Pending();pending.exists=pending.active=true;pending.id=nextTxn++;
 pending.created=millis();memcpy(pending.raw,p,9);eventFor(pending,ttc::QUEUED,ttc::NONE);
}
static void receiveUSB(){
 uint8_t f[ttc::MAX_STM],n=0;
 for(unsigned k=0;k<256&&Serial.available();++k){int b=Serial.read();if(b>=0&&usbParser.push(uint8_t(b),f,n))processCommand(f,n);while(usbParser.pop(f,n))processCommand(f,n);}
 while(usbParser.pop(f,n))processCommand(f,n);
}
static bool replyMatches(const Pending& item,const uint8_t*p,uint8_t n){
 if(!ttc::validSTM(p,n) || p[4]!=item.raw[4])return false;
 if(n==18&&p[3]==5&&p[5]==10)return item.raw[6]==0x10;
 return n==10&&(p[3]==3||p[3]==4)&&p[5]==2&&p[6]==item.raw[6];
}
static void processDownlink(const ttc::Frame&f){
 if(f.kind!=ttc::DOWNLINK || f.length!=ttc::DL_SIZE || ttc::read32(f.payload+56)!=boot)return;
 // Reject stale/out-of-order downlinks within this RP boot (wrap-safe uptime).
 if(haveDownlink&&f.boot==spaceBoot&&int32_t(f.uptime-lastDLUptime)<=0)return;
 if(haveDownlink&&f.boot==spaceBoot&&f.counter==lastDLCounter&&f.uptime==lastDLUptime)return;
 const bool changed=spaceBoot&&spaceBoot!=f.boot;
 if(changed){
  changingBoot=true;
  if(pending.exists){
   if((pending.active || pending.uncertain)&&!eventFor(pending,ttc::BOOT_UNKNOWN,ttc::TARGET_MISMATCH))return;
   pending=Pending();
  }
  for(uint8_t i=0;i<JOURNAL_COUNT;i++)if(journal[i].exists){
   if(!eventFor(journal[i],ttc::BOOT_UNKNOWN,ttc::TARGET_MISMATCH))return;
   journal[i]=Pending();
  }
  ackGround=ackId=0;ackState=ttc::NONE;haveSTM=false;tmReady=gpsReady=false;
 }
 changingBoot=false;
 const uint32_t advertised=ttc::read32(f.payload+92);
 if(!haveDownlink || changed)nextTxn=advertised;
 else if(!advertised)nextTxn=0;
 else if(nextTxn && advertised>nextTxn)nextTxn=advertised;
 spaceBoot=f.boot;lastDLCounter=f.counter;lastDLUptime=f.uptime;
 lastDLAt=millis();haveDownlink=true;waitingDL=false;++downlinks;
 const uint8_t*p=f.payload;
 uint8_t gps[26];ttc::makeSTM(0x20,localSeq++,p+38,18,gps);slot(gpsSlot,gpsReady,gps,26);
 if((p[91]&4)&&p[0]==37&&ttc::validSTM(p+1,37)&&p[4]==1&&p[6]==29){
  const uint32_t sample=ttc::read32(p+7);const uint8_t seq=p[5];
  if(!haveSTM||sample!=lastSTMUptime||seq!=lastSTMSeq){
   slot(tmSlot,tmReady,p+1,37);haveSTM=true;lastSTMUptime=sample;lastSTMSeq=seq;
  }
 }
 memset(lastLink,0,sizeof(lastLink));ttc::write32(lastLink,spaceBoot);ttc::write32(lastLink+4,f.uptime);
 ttc::write16(lastLink+8,f.counter);ttc::write16(lastLink+10,uint16_t(db10(rf.lastRSSI)));
 ttc::write16(lastLink+12,uint16_t(db10(rf.lastSNR)));ttc::write16(lastLink+16,ttc::read16(p+86));
 ttc::write16(lastLink+18,ttc::read16(p+88));lastLink[20]=p[90];lastLink[21]=p[91];lastLink[22]=p[53];reportLink();
 const uint32_t resultId=ttc::read32(p+60);
 Pending* found=(pending.exists&&resultId==pending.id)?&pending:nullptr;
 if(!found)for(uint8_t i=0;i<JOURNAL_COUNT;i++)if(journal[i].exists&&journal[i].id==resultId){found=&journal[i];break;}
 if(!found)return;
 Pending& item=*found;
 const uint8_t state=p[66];
 if((p[64]!=item.raw[6]||p[65]!=item.raw[4])&&state!=ttc::UNKNOWN_ID&&state!=ttc::TARGET_MISMATCH)return;
 if(state==ttc::UART_PENDING||state==ttc::UART_SENT||state==ttc::STM_RESPONSE||state==ttc::STM_TIMEOUT_UNKNOWN||state==ttc::UART_TX_FAILED_UNKNOWN){
  if(!item.accepted)item.acceptedAt=millis();item.accepted=true;
 }
 if(state==ttc::STM_RESPONSE){
  const uint8_t len=p[67];if((len!=10&&len!=18)||!replyMatches(item,p+68,len))return;
  if(!item.responseQueued){
   // Do not acknowledge RP response until safely accepted by bounded USB queue.
   if(qCount>USB_Q-2){usbOverrun=true;return;}
   if(!enqueueCritical(p+68,len))return;
   if(!eventFor(item,ttc::REAL_RESPONSE,state))return;
   item.responseQueued=true;
  }
  ackGround=boot;ackId=item.id;ackState=state;item.active=false;item.uncertain=false;item.state=state;
  if(found!=&pending)item.exists=false;
  return;
 }
 // Repeat acknowledgments even when no new event is needed. A lost POLL must
 // not pin a terminal cache record. The state prevents a timeout ACK from
 // accidentally acknowledging a subsequently arrived real STM reply.
 if(state==item.state){
  if(item.terminalReported&&(state==ttc::STM_TIMEOUT_UNKNOWN||state==ttc::UART_TX_FAILED_UNKNOWN)){
   ackGround=boot;ackId=item.id;ackState=state;
  }
  return;
 }
 bool notified=false;
 switch(state){
 case ttc::UART_PENDING:notified=eventFor(item,ttc::RP_QUEUED,state);break;
 case ttc::UART_SENT:notified=eventFor(item,ttc::RP_SENT,state);break;
 case ttc::BLOCKED:notified=retire(item,ttc::LOCAL_BLOCKED,state);break;
 case ttc::BUSY:notified=retire(item,ttc::REMOTE_BUSY,state);break;
 case ttc::TARGET_MISMATCH:notified=retire(item,ttc::BOOT_UNKNOWN,state);break;
 case ttc::STM_TIMEOUT_UNKNOWN:notified=retire(item,ttc::STM_UNKNOWN,state);break;
 case ttc::UART_TX_FAILED_UNKNOWN:notified=retire(item,ttc::LINK_UNKNOWN,state);break;
 case ttc::ID_CONFLICT:case ttc::SEQ_QUARANTINED:notified=retire(item,ttc::CONFLICT,state);break;
 case ttc::UNKNOWN_ID:
   // The cache no longer has a result. Report uncertainty, never retry UART.
   notified=retire(item,ttc::RESULT_EXPIRED,state);break;
 default:return;
 }
 if(!notified)return;
 item.state=state;
 if(state!=ttc::UART_PENDING&&state!=ttc::UART_SENT){
  item.terminalReported=true;
  if(state==ttc::STM_TIMEOUT_UNKNOWN||state==ttc::UART_TX_FAILED_UNKNOWN){ackGround=boot;ackId=item.id;ackState=state;}
  if(found!=&pending&&!item.uncertain)item.exists=false;
 }
}
// With one poll coordinator, a newer downlink can replace this ACK only after
// RP has received a poll carrying it (ACK is applied before processing query).
// Keep sending the same ACK if no newer result arrives; old downlinks are
// rejected above by RP uptime. No STM command is resent by journal queries.
static uint32_t pollCommandId=0;
static void sendPoll(uint32_t now){
 uint8_t p[ttc::POLL_SIZE]={},wire[ttc::RF_MAX];
 ttc::write32(p,spaceBoot);
 Pending* query=nullptr;
 // New commands retain priority; one in four polls services old uncertain IDs.
 if(pending.exists&&(pending.active||pending.uncertain)){
  if((pollTurn++%4)==3)query=nextJournal();
  if(!query)query=&pending;
 }else query=nextJournal();
 if(query){
  ttc::write32(p+4,query->id);
  if(query==&pending&&pending.active&&!pending.accepted&&spaceBoot){p[8]=9;memcpy(p+9,pending.raw,9);}
 }
 ttc::write32(p+18,ackGround);ttc::write32(p+22,ackId);p[26]=ackState;
 const size_t n=ttc::encode(ttc::POLL,boot,rfCounter,millis(),p,sizeof(p),wire);
 const uint32_t failures=rf.txFailures;
 if(n&&rf.send(wire,n,now)){
  ++rfCounter;++polls;waitingDL=true;wasTX=true;pollCommandId=p[8]?pending.id:0;
  if(p[8]&&pending.attempts!=65535)++pending.attempts;
  nextPollAt=now+POLL_MS;
 }else nextPollAt=now+(rf.txFailures!=failures?POLL_MS:10);
}
static void initRadio(){
 lastInitAt=millis();
 const int16_t rc=radio.begin(433.0,125.0,7,5,0x12,17,8);
 radioOK=(rc==RADIOLIB_ERR_NONE);
 if(radioOK)radioOK=(radio.setCRC(true)==RADIOLIB_ERR_NONE);
 if(radioOK)radioOK=rf.beginRx();
}
void setup(){
 Serial.begin(115200);SPI.begin(18,19,23,5);
 boot=esp_random();if(!boot)boot=1;
 protocolOK=ttc::selfTest();
 if(protocolOK)initRadio();
 nextPollAt=millis()+100;reportLink();
}
void loop(){
 const uint32_t now=millis();pumpUSB();receiveUSB();
 if(protocolOK&&!radioOK&&uint32_t(now-lastInitAt)>=2000)initRadio();
 if(radioOK){
  rf.service(now);
  if(wasTX&&!rf.transmitting()){
   wasTX=false;waitSince=now;
   if(rf.lastTx==0&&pollCommandId&&pending.exists&&pending.id==pollCommandId&&!pending.rfNotified){
     if(eventFor(pending,ttc::RF_SENT,ttc::NONE))pending.rfNotified=true;
   }
  }
  uint8_t wire[ttc::RF_MAX];ttc::Frame packet;
  const size_t n=rf.receive(wire,sizeof(wire));
  if(n&&ttc::decode(wire,n,packet))processDownlink(packet);
  if(waitingDL&&!rf.transmitting()&&uint32_t(now-waitSince)>=DOWNLINK_WAIT_MS)waitingDL=false;
  if(pending.active){
   const uint32_t elapsed=uint32_t(now-pending.created);
    if(!pending.accepted&&elapsed>=5000)retire(pending,ttc::LINK_UNKNOWN,ttc::UNKNOWN_ID);
    else if(pending.accepted&&uint32_t(now-pending.acceptedAt)>=(pending.raw[6]==0x03?125000UL:6500UL))retire(pending,ttc::STM_UNKNOWN,ttc::STM_TIMEOUT_UNKNOWN);
  }
  if(!rf.transmitting()&&!waitingDL&&int32_t(now-nextPollAt)>=0)sendPoll(now);
 }
 
 if(uint32_t(now-lastLinkReport)>=1000)reportLink();
 pumpUSB();
}
