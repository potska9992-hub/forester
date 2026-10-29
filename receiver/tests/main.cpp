// ===== тесты приёмника: независимый "пульт" по описанию протокола §52 =====
static int fails = 0;
#define CHECK(c) do{ if(!(c)){ printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } }while(0)
typedef std::vector<uint8_t> Bytes;
static uint16_t c16(const uint8_t* d, size_t n){ uint16_t c=0xFFFF; for(size_t i=0;i<n;i++){ c^=(uint16_t)d[i]<<8; for(int k=0;k<8;k++) c=(c&0x8000)?(uint16_t)((c<<1)^0x1021):(uint16_t)(c<<1);} return c; }
static void fin(Bytes& b){ uint16_t c=c16(b.data(),b.size()); b.push_back(c&0xFF); b.push_back(c>>8); }
static uint32_t pultBind = 0xA5C3E17B;
static uint16_t pairOf(uint32_t b){ uint16_t p=(uint16_t)(b^(b>>16)); if(p==0||p==0xFFFF) p^=0x5A5A; return p; }
static Bytes H(int type,int flags,uint16_t pair,uint8_t tag){ Bytes b; b.push_back((type<<4)|flags); b.push_back(pair&0xFF); b.push_back(pair>>8); b.push_back(tag); return b; }
static uint8_t seqN=0;
static Bytes mkCtl(int flags,const std::vector<std::pair<int,int>>& v){ Bytes b=H(1,flags,pairOf(pultBind),++seqN); uint8_t m=0; for(auto&p:v) m|=1<<p.first; b.push_back(m); for(int i=0;i<8;i++) for(auto&p:v) if(p.first==i) b.push_back((uint8_t)(int8_t)p.second); fin(b); return b; }
static Bytes mkCfg(uint8_t tag,int op,int prm,int ch,int d,int val,bool save=false){ Bytes b=H(3,save?1:0,pairOf(pultBind),tag); b.push_back(op); b.push_back(prm); b.push_back(ch); b.push_back(d); b.push_back(val&0xFF); b.push_back(val>>8); fin(b); return b; }
static Bytes mkNet(uint8_t tag,int slot,const char* ssid,const char* pass){ Bytes b=H(6,0,pairOf(pultBind),tag); b.push_back(slot); b.push_back(1); b.push_back(strlen(ssid)); for(const char*q=ssid;*q;q++) b.push_back(*q); b.push_back(strlen(pass)); for(const char*q=pass;*q;q++) b.push_back(*q); fin(b); return b; }
// эфир
static std::deque<Bytes> toPult; static bool inflight=false; static Bytes ifb; static unsigned long ifDone=0;
static SamplerState SS;
static void mediumStep(){
  if(fakeRadio.txPending && !inflight){ inflight=true; ifb.assign(fakeRadio.lastTx,fakeRadio.lastTx+fakeRadio.lastTxLen); ifDone=T+2+ifb.size()/2; }
  if(inflight && T>=ifDone){ inflight=false; fakeRadio.txPending=false; toPult.push_back(ifb); if(fakeIsr) fakeIsr(); }
}
static void relayModel(){
  bool on=(pinLvl[21]==0);
  if(relayNoClose) fbV = 0; else if(relayStuck) fbV = busV; else fbV = on ? busV : (fbV>0.3f? fbV*0.97f : 0);
}
static void step(int ms){
  for(int i=0;i<ms;i++){ T++; relayModel(); loop(); mediumStep(); radioStep(T); samplerStep(SS); if(T%2==0) samplerStep(SS); }
}
static void toRx(const Bytes& b){ while(inflight) step(1); memcpy(fakeRadio.rx,b.data(),b.size()); fakeRadio.rxLen=b.size(); if(fakeIsr) fakeIsr(); step(1); }
static bool crcOk(const Bytes& b){ return b.size()>2 && c16(b.data(),b.size()-2)==(uint16_t)(b[b.size()-2]|(b[b.size()-1]<<8)); }
static Bytes lastOf(int type){ Bytes r; for(auto&b:toPult) if(!b.empty() && (b[0]>>4)==type) r=b; return r; }
static int countOf(int type){ int n=0; for(auto&b:toPult) if(!b.empty() && (b[0]>>4)==type) n++; return n; }
static Bytes ask(const Bytes& req,int type,int wait=60){ toPult.clear(); toRx(req); for(int i=0;i<wait && countOf(type)==0;i++) step(1); return lastOf(type); }
static std::vector<std::pair<int,int>> curLev;
static Bytes telem(int extraFlags=0){ return ask(mkCtl(2|extraFlags,curLev),2); }
static Bytes hbTelem(){ return telem(); }
// независимая модель CRC8 по умолчанию (таблица из §52.5)
struct PD{ int scope,def; };
static const PD pdef[15]={{0,20},{0,50},{0,600},{1,200},{1,200},{1,1},{2,150},{2,500},{2,100},{2,0},{2,30},{2,10},{2,0},{0,5},{0,15}};
static uint8_t crc8Defaults(){ uint8_t c=0; for(int p=0;p<15;p++){ int nc=pdef[p].scope==2?1:8, nd=pdef[p].scope==0?2:1; for(int ch=0;ch<nc;ch++) for(int d=0;d<nd;d++){ uint16_t v=pdef[p].def; for(int k=0;k<2;k++){ c^=(uint8_t)(k?(v>>8):(v&0xFF)); for(int j=0;j<8;j++) c=(c&0x80)?(uint8_t)((c<<1)^7):(uint8_t)(c<<1);} } } return c; }
static void allCentered(){ toRx(mkCtl(0,{})); }
static int sendCtlLoop(int ms,int flags,const std::vector<std::pair<int,int>>& v,int period=20){ int n=0; for(int t=0;t<ms;t+=period){ toRx(mkCtl(flags,v)); step(period-1); n++; } return n; }
static void bringUp(){ // связь, реле, выходы
  sendCtlLoop(2500,0,{}); }

int main(){
  pinLvl[0]=1; setup();
  CHECK(fakeTaskN==2);
  CHECK(fakeRadio.beginCalls>=1);
  printf("init pins 21=%d 33=%d 38=%d\n",pinLvl[21],pinLvl[33],pinLvl[38]); CHECK(pinLvl[21]==1 && pinLvl[33]==1 && pinLvl[38]==0);
  // --- 1. телеметрия и связь ---
  step(50);
  Bytes t = telem(); printf("telem size=%zu relay=%d fault=%d link=%d\n",t.size(),(int)relayState,(int)faultCode,0); for(auto x:t) printf("%02x ",x); printf("\n");
  CHECK(t.size()==36 && crcOk(t));
  CHECK(t[0]==0x20 && (t[1]|(t[2]<<8))==pairOf(pultBind));
  CHECK(t[30]==1);                                                   // счётчик перезагрузок после первого старта
  CHECK(t[31]==crc8Defaults());
  t = telem(); CHECK((t[27]&1)==0);                                  // killswitch снят, связь есть
  CHECK(t[28]>=118 && t[28]<=122);                                   // 12 В
  CHECK(t[32]==0);
  // --- 2. включение выходов ---
  bringUp();
  CHECK(pinLvl[21]==0);                                              // реле замкнуто
  CHECK(pinLvl[38]==1 && pinLvl[33]==0);                             // EN и OE включены
  CHECK(faultCode==0);
  // --- 3. рычаг, разгон, стороны ---
  sendCtlLoop(400,0,{{0,50}});
  printf("d0=%f d1=%f v0=%d sDuty=%f,%f pcaDuty0..3=%f %f %f %f\n",pcaDuty[0],pcaDuty[1],ctlSh.v[0],sDuty[0],sDuty[1],pcaDuty[0],pcaDuty[1],pcaDuty[2],pcaDuty[3]);
  CHECK(fabsf(pcaDuty[0]-32.5f)<0.7f && pcaDuty[1]==0);              // старт 5 + (60-5)*0.5
  { bool both=false; auto rev=[&](){ sendCtlLoop(1,0,{{0,-50}},1); }; for(int i=0;i<300;i++){ toRx(mkCtl(0,{{0,-50}})); step(1); if(pcaDuty[0]>0&&pcaDuty[1]>0) both=true; } (void)rev; CHECK(!both); }
  printf("rev: fault=%d vbus=%f vfb=%f bind=%d wifi=%d v0=%d relay=%d act=%d kill=%d/%d armed=%d ovl=%d sCap=%f %f\n",(int)faultCode,vBus,vFb,(int)bindMode,(int)wifiActive(),ctlSh.v[0],(int)relayState,(int)outAct,(int)killNow,(int)killReasonNow,(int)armedRun,(int)ovlLatch[0],sCap[0],sCap[1]);
  CHECK(pcaDuty[0]==0 && fabsf(pcaDuty[1]-32.5f)<0.7f);
  sendCtlLoop(100,0,{{0,100}});                                      // обратно: максимум 60
  { for(int i=0;i<400;i++){ toRx(mkCtl(0,{{0,100}})); step(1);} }
  printf("max: %f %f\n",pcaDuty[0],pcaDuty[1]);
  CHECK(fabsf(pcaDuty[0]-60.0f)<0.7f);
  sendCtlLoop(400,0,{});
  CHECK(pcaDuty[0]==0 && pcaDuty[1]==0);
  // --- 4. E-Stop пульта ---
  sendCtlLoop(100,0,{{1,40}});
  CHECK(pcaDuty[2]>0);
  toRx(mkCtl(1,{}));
  CHECK(pinLvl[33]==1 && pinLvl[38]==0);                             // OE и EN аппаратно сразу
  step(10);
  t=telem(1);
  CHECK((t[27]&1)==1 && ((t[27]>>1)&3)==0);
  CHECK(pcaDuty[2]==0 && pinLvl[21]==1 || relayState!=RS_ON);
  sendCtlLoop(100,0,{});                                             // сняли E-Stop, рычаг в нуле
  t=telem(); CHECK((t[27]&1)==0);
  sendCtlLoop(2500,0,{});
  CHECK(pinLvl[21]==0 && pinLvl[38]==1);
  // --- 5. аварийная кнопка приёмника ---
  pinLvl[34]=1; step(50);
  t=telem(); CHECK((t[27]&1)==1 && ((t[27]>>1)&3)==1);
  CHECK(pinLvl[33]==1 && pinLvl[38]==0);
  pinLvl[34]=0; step(100);
  t=telem(); CHECK((t[27]&1)==1 && ((t[27]>>1)&3)==1);               // отпущена, но держит до нулевого пакета
  allCentered(); step(5);
  t=telem(); CHECK((t[27]&1)==0);
  sendCtlLoop(2500,0,{});
  CHECK(pinLvl[21]==0 && pinLvl[38]==1);
  // --- 6. потеря связи и Safe Start ---
  step(900);
  printf("init pins 21=%d 33=%d 38=%d\n",pinLvl[21],pinLvl[33],pinLvl[38]); CHECK(pinLvl[21]==1 && pinLvl[33]==1 && pinLvl[38]==0);
  CHECK(killNow && killReasonNow==3);
  sendCtlLoop(2500,0,{{0,60}});                                      // связь вернулась с рычагом не в центре
  CHECK(pcaDuty[0]==0);                                              // выходы ждут нуль
  sendCtlLoop(100,0,{});
  sendCtlLoop(400,0,{{0,60}});
  CHECK(pcaDuty[0]>0);
  sendCtlLoop(500,0,{});
  // --- 7. настройки ---
  Bytes a=ask(mkCfg(1,0,0,2,1,35),4);
  CHECK(a.size()==12 && crcOk(a) && a[3]==1 && a[4]==0 && a[5]==0 && a[6]==2 && a[7]==1 && (a[8]|(a[9]<<8))==35);
  a=ask(mkCfg(2,1,0,2,1,0),4); CHECK((a[8]|(a[9]<<8))==35);
  Bytes b=ask(mkCfg(3,2,0,0,0,0),5);                                 // BATCH LIMIT: сохранённые
  CHECK(b.size()==8+32 && crcOk(b) && b[4]==0 && b[5]==16 && (b[6+2*(2*2+1)]|(b[7+2*(2*2+1)]<<8))==20);
  CHECK(telem()[31]==crc8Defaults());
  a=ask(mkCfg(4,0,0,2,1,35,true),4); CHECK(a[4]==0);
  step(50);
  b=ask(mkCfg(5,2,0,0,0,0),5); CHECK((b[6+2*5]|(b[7+2*5]<<8))==35);
  CHECK(telem()[31]!=crc8Defaults());
  a=ask(mkCfg(6,0,0,2,1,500),4); CHECK((a[8]|(a[9]<<8))==100);       // предел
  a=ask(mkCfg(7,0,40,0,0,1),4);  CHECK(a[4]==1);                     // нет такого параметра
  a=ask(mkCfg(8,0,12,0,0,1),4);  CHECK(a[4]==2);                     // CURRENT: нет быстрого АЦП
  a=ask(mkCfg(9,3,2,0,0,0xFF),4); CHECK(a[4]==0);                    // CHMASK
  Bytes a2=ask(mkCfg(9,3,2,0,0,0xFF),4); CHECK(a2==a);               // повтор той же метки
  a=ask(mkCfg(10,3,1,0,0,1),4);  CHECK(a[4]==0);                     // PROTO
  a=ask(mkCfg(11,3,1,0,0,2),4);  CHECK(a[4]==1);
  a=ask(mkCfg(12,3,0x10,0,0,0x1234),4); CHECK(a[4]==1);              // FACTORY: неверный ключ
  { uint8_t bc=bootCount; a=ask(mkCfg(13,3,0x10,0,0,0xA55A),4); CHECK(a[4]==0 && bootCount==bc+1); }
  step(50);
  CHECK(telem()[31]==crc8Defaults() && telem()[30]==bootCount);
  a=ask(mkCfg(14,0,0,2,1,0),4); CHECK(a[4]==0);                      // LIMIT ch2B = 0 -> 5
  // --- 8. ограничение и перегрузка ---
  a=ask(mkCfg(15,0,0,0,0,20),4);
  sendCtlLoop(500,0,{{0,50}});
  float d0=pcaDuty[0];
  adsV[0]=1.3f;                                                      // 2.2 А при пределе 2.0
  sendCtlLoop(600,0,{{0,50}});
  curLev={{0,50}}; t=telem(); CHECK(t[23]&1);
  CHECK(pcaDuty[0]<d0);
  adsV[0]=2.2f;                                                      // 3.7 А > 1.3 x 2.0
  curLev={{0,50}};
  sendCtlLoop(500,0,{{0,50}});
  t=telem(); CHECK(t[24]&1);
  CHECK(pcaDuty[0]==0 && pcaDuty[1]==0);
  adsV[0]=0.2f;
  for(int i=0;i<300;i++){ static bool lo=true,la=true; toRx(mkCtl(0,{{0,50}})); step(1); if(lo!=ovlLatch[0]||la!=outAct){ printf("t=%lu ovl=%d act=%d\n",T,(int)ovlLatch[0],(int)outAct); lo=ovlLatch[0]; la=outAct; } }
  printf("hold: pca=%f ovl=%d v=%d act=%d relay=%d kill=%d\n",pcaDuty[0],(int)ovlLatch[0],ctlSh.v[0],(int)outAct,(int)relayState,(int)killNow);
  CHECK(pcaDuty[0]==0);                                              // держит, пока рычаг не в центре
  curLev={}; sendCtlLoop(100,0,{});
  sendCtlLoop(500,0,{{0,50}});
  CHECK(pcaDuty[0]>0);
  sendCtlLoop(300,0,{});
  adsV[0]=0;
  // --- 9. сбой реле: залипло ---
  relayStuck=true;
  step(3000);                                                        // связь потеряна, реле открыто, напряжение держится: 3 с - ещё только запись
  CHECK(faultCode==0);
  step(3000);                                                        // более 5 с - залипло
  CHECK(pinLvl[33]==1 && pinLvl[38]==0 && pinLvl[21]==1);
  CHECK(faultCode==1);
  t=telem(); CHECK((t[27]&1)==1 && ((t[27]>>1)&3)==2);
  sendCtlLoop(3000,0,{});
  CHECK(faultCode==1 && pinLvl[21]==1);                               // защёлка
  printf("part1 fails=%d\n", fails);
  return fails?1:0;
}
