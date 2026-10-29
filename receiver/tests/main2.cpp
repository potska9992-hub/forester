// helpers
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
  if(relayStuck) fbV=busV; else if(on){ if(relayNoClose) fbV=0; else fbV += (busV-fbV)*0.004f; } else fbV *= 0.99995f;
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


static Bytes mkBind(uint32_t id,uint8_t tag){ Bytes b=H(8,0,0xFFFF,tag); for(int i=0;i<4;i++) b.push_back((id>>(8*i))&0xFF); fin(b); return b; }
int main(int argc,char**argv){
  char sc = argc>1?argv[1][0]:'a';
  pinLvl[0]=1; setup();
  bringUp();
  CHECK(pinLvl[21]==0 && pinLvl[38]==1 && pinLvl[33]==0);
  if(sc=='a'){            // реле не замкнулось
    step(900); CHECK(pinLvl[21]==1);
    fbV=0; relayNoClose=true;
    sendCtlLoop(7000,0,{});
    CHECK(faultCode==2);
    CHECK(pinLvl[33]==1 && pinLvl[38]==0 && pinLvl[21]==1);
    Bytes t=telem(); CHECK((t[27]&1)==1 && ((t[27]>>1)&3)==2);
    relayNoClose=false; sendCtlLoop(3000,0,{});
    CHECK(faultCode==2 && pinLvl[21]==1);
  }
  if(sc=='e'){            // синхронизация без управления дольше таймаута связи
    for(int k=0;k<30;k++){ Bytes a=ask(mkCfg(100+k,1,0,k%8,0,0),4); CHECK(a.size()==12); step(30); }   // ~1.8 с без CONTROL
    CHECK(!killNow); CHECK(pinLvl[21]==0 && pinLvl[38]==1);
    step(3300); CHECK(killNow && killReasonNow==3);                 // без управления связь всё же пропадает
  }
  if(sc=='f'){            // Wi-Fi можно включить при аварии реле
    relayNoClose=true; fbV=0; step(900); sendCtlLoop(7000,0,{});
    CHECK(faultCode==2);
    Bytes a=ask(mkCfg(1,3,0x20,0,0,0),4); CHECK(a.size()==12 && a[4]==0);
    sendCtlLoop(300,0,{}); Bytes t=telem(); CHECK(t[32]==2);
  }
  if(sc=='b'){            // потеряна PCA9685
    pcaPresent=false; sendCtlLoop(300,0,{{0,50}});
    sendCtlLoop(300,0,{});
    sendCtlLoop(300,0,{{0,50}});
    CHECK(faultCode==3);
    Bytes t=telem(); CHECK((t[27]&1)==1 && ((t[27]>>1)&3)==2);
    CHECK(pinLvl[33]==1 && pinLvl[38]==0);
  }
  if(sc=='c'){            // Wi-Fi телефон
    Bytes a=ask(mkCfg(1,3,0x20,0,0,0),4); CHECK(a.size()==12 && a[4]==0);
    sendCtlLoop(300,0,{});
    Bytes t=telem(); CHECK(t[32]==2);
    CHECK(pinLvl[21]==1 && pinLvl[38]==1);                          // выходы отключены на время Wi-Fi
    CHECK(pinLvl[15]==1 && pinLvl[16]==1);                          // оба огня
    toPult.clear(); sendCtlLoop(2000,0,{});
    Bytes in=lastOf(7); CHECK(in.size()>=8 && crcOk(in) && in[4]==2 && in[5]>=6 && in[5]<=23 && in.size()==(size_t)(8+in[5]));
    if(in.size()>8) printf("INFO ssid=%.*s\n",(int)in[5],(const char*)&in[6]);
    a=ask(mkCfg(2,3,0x22,0,0,0),4); CHECK(a[4]==0);
    sendCtlLoop(300,0,{}); t=telem(); CHECK(t[32]==0);
    sendCtlLoop(3000,0,{}); CHECK(pinLvl[21]==0);                   // выходы вернулись
    // E-Stop закрывает Wi-Fi
    a=ask(mkCfg(3,3,0x20,0,0,0),4); sendCtlLoop(200,0,{}); t=telem(); CHECK(t[32]==2);
    sendCtlLoop(100,1,{}); sendCtlLoop(100,0,{}); t=telem(); CHECK(t[32]==0);
    // NET
    a=ask(mkNet(4,0,"HomeNet","pass1234"),4); CHECK(a.size()==12 && a[4]==0 && a[5]==0xE0);
    Bytes a2=ask(mkNet(4,0,"HomeNet","pass1234"),4); CHECK(a2==a);
    a=ask(mkNet(5,9,"x","y"),4); CHECK(a[4]==1);
    // повтор CMD с той же меткой после другой команды не должен снова выполнить: WIFI_PHONE требует ноль на рычаге
    a=ask(mkCfg(6,3,0x20,0,0,0),4); CHECK(a[4]==0);
    a=ask(mkCfg(7,3,0x20,0,0,0),4); CHECK(a[4]==3);                // уже занят
  }
  if(sc=='d'){            // привязка
    bool ok;
    Bytes r=ask(mkBind(0x11223344,1),9,100); CHECK(r.empty());    // не в режиме привязки: молчит
    pinLvl[0]=0; step(3200); pinLvl[0]=1; step(100);
    CHECK(bindMode);
    CHECK(pinLvl[21]==1);                                           // реле выключено на время привязки
    r=ask(mkBind(0x11223344,2),9,200);
    CHECK(r.size()==10 && crcOk(r) && r[0]==0x90 && (r[1]|(r[2]<<8))==pairOf(0x11223344) && r[4]==0x44 && r[7]==0x11);
    CHECK(!bindMode);
    pultBind=0x11223344; Bytes t=telem(); CHECK(t.size()==36);
    Bytes r2=ask(mkBind(0x11223344,3),9,200); CHECK(!r2.empty());   // повтор в окне 4 с
    sendCtlLoop(2500,0,{}); CHECK(pinLvl[21]==0);
    // старая пара не слышится
    pultBind=0xA5C3E17B; toPult.clear(); toRx(mkCtl(2,{})); step(40); CHECK(countOf(2)==0);
  }
  printf("scenario %c fails=%d\n",sc,fails);
  return fails?1:0;
}
