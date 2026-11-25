#ifndef TTC_RADIO_ASYNC_V4_H
#define TTC_RADIO_ASYNC_V4_H
#include <Arduino.h>
#include <RadioLib.h>
#include "TTC_Protocol.h"
namespace ttc {
class RadioAsync {
 SX1278& radio;bool tx=false,rx=false;uint32_t started=0,deadline=0,recoveryAt=0;
public:
 int16_t lastTx=0,lastRx=0,lastRead=0;uint32_t txFailures=0,rxFailures=0;
 float lastRSSI=0,lastSNR=0;
 explicit RadioAsync(SX1278&r):radio(r){}
 bool transmitting()const{return tx;}
 bool ready()const{return rx&&!tx;}
 bool beginRx(){lastRx=radio.startReceive();rx=(lastRx==RADIOLIB_ERR_NONE);recoveryAt=millis();if(!rx)++rxFailures;return rx;}
 bool send(const uint8_t*p,size_t n,uint32_t now){
  if(tx || !p || n>RF_MAX || n==0)return false;
  if(rx && (radio.getIRQFlags()&RADIOLIB_SX127X_CLEAR_IRQ_FLAG_RX_DONE))return false;
  const uint32_t air=uint32_t((radio.getTimeOnAir(n)+999)/1000);
  deadline=air+air/2+30;started=now;rx=false;
  lastTx=radio.startTransmit(p,n);
  if(lastTx!=RADIOLIB_ERR_NONE){++txFailures;beginRx();return false;}
  tx=true;return true;
 }
 void service(uint32_t now){
  if(tx){
   const bool done=(radio.getIRQFlags()&RADIOLIB_SX127X_CLEAR_IRQ_FLAG_TX_DONE)!=0;
   if(done || uint32_t(now-started)>=deadline){
    const int16_t finish=radio.finishTransmit();lastTx=done?finish:RADIOLIB_ERR_TX_TIMEOUT;
    if(lastTx!=RADIOLIB_ERR_NONE)++txFailures;tx=false;beginRx();
   }
  }else if(!rx && uint32_t(now-recoveryAt)>=100)beginRx();
 }
 size_t receive(uint8_t*out,size_t capacity){
  if(!rx || tx || !(radio.getIRQFlags()&RADIOLIB_SX127X_CLEAR_IRQ_FLAG_RX_DONE))return 0;
  const size_t n=radio.getPacketLength();
  if(n<HEADER+2 || n>capacity || n>RF_MAX){lastRead=RADIOLIB_ERR_PACKET_TOO_LONG;beginRx();return 0;}
  lastRSSI=radio.getRSSI();lastSNR=radio.getSNR();lastRead=radio.readData(out,n);beginRx();
  return lastRead==RADIOLIB_ERR_NONE?n:0;
 }
};
} // namespace ttc
#endif