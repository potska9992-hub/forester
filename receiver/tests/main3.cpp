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
  pinLvl[0]=1; setup(); bringUp();
  FILE*f=fopen("/tmp/claude-0/work/pult_tx.txt","r"); char line[400]; int n=0, acked=0, tot=0;
  while(fgets(line,sizeof line,f)){ Bytes b; for(int i=0;line[i]&&line[i]!='\n';i+=2){ unsigned v; sscanf(line+i,"%2x",&v); b.push_back(v);} int type=b[0]>>4; if(type!=3&&type!=6) continue; tot++;
    int want = type==3 ? (b[4]==2?5:4) : 4; Bytes r=ask(b,want,80); if(!r.empty()&&crcOk(r)) acked++; else if(tot<=25) printf("no reply to type %d op %d prm %d (pkt %d)\n",type,b[4],b[5],tot); if(tot>=25) break; }
  printf("CFG/NET replayed %d, answered %d\n",tot,acked); return 0; }
