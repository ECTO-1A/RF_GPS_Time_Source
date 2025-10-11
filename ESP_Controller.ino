/* ESP32-WROOM + 128x64 OLED for Trimble Mini-T
   Bridge Mode + TSIP AB/AC + NMEA (strict echo)
   Includes:
   - ALARM DETAIL page in rotation (press '3' to jump)
   - Right-aligned "ALM!" header badge (no overlap)
   - 2-wire BUTTON (GPIO27 to GND):
       * Quick press  -> next screen
       * Long press   -> toggle BRIDGE MODE
   - 2-wire LED (GPIO25) lights when Fix = YES
*/

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

// ===== Config (ESP32-WROOM) =====
static const long UART_BAUD   = 9600;
static const int  OLED_ADDR   = 0x3C;
static const int  OLED_W      = 128;
static const int  OLED_H      = 64;
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1);
static const unsigned long PAGE_PERIOD_MS = 5000;
const char* BRIDGE_CMD = "BRG";

// I2C pins
static const int I2C_SDA = 21, I2C_SCL = 22;

// GPS UART pins (ESP32 Serial2)
static const int UART_RX_PIN = 16; // ESP32 RX2 <- Mini-T TXD
static const int UART_TX_PIN = 17; // ESP32 TX2 -> Mini-T RXD

// ===== User I/O (button + Fix LED) =====
const int BTN_PIN = 27;  // Button to GND (INPUT_PULLUP)
const int LED_PIN = 25;  // External LED + resistor to GND (active-HIGH)
const bool LED_ACTIVE_LOW = false;

// Debounce + press type thresholds
const unsigned long BTN_DEBOUNCE_MS = 30;
const unsigned long LONG_PRESS_MS   = 700;

static bool btnStable = true;           // stable logical level (pull-up idle HIGH)
static bool btnLastReported = true;     // last debounced read
static unsigned long btnLastChange = 0; // last time we changed stable state
static bool btnWasDown = false;         // track press lifecycle
static unsigned long btnPressStart = 0; // millis at press

static inline void setFixLED(bool on){
  digitalWrite(LED_PIN, LED_ACTIVE_LOW ? !on : on);
}

// ===== State =====
enum Page { PAGE_TIMING=0, PAGE_GNSS=1, PAGE_PACKETS=2, PAGE_ALARM=3, PAGE_MAX=4 };
volatile Page currentPage = PAGE_TIMING;
bool bridgeMode = false;

struct Status {
  uint8_t satsUsed=0, satsView=0; float pdop=NAN;
  bool hasFix=false, timeSet=false, ppsGood=false;
  uint8_t decStat=0xFF; uint16_t minorAlarms=0;
  uint8_t hour=0, minute=0, second=0; uint16_t year=0; uint8_t month=0, day=0;
  uint32_t bytesIn=0, tsipPackets=0, tsipBadFrames=0, nmeaSentences=0;
  int fom=-1;
} st;

// ===== NMEA (strict) =====
String nmeaLine; static const size_t NMEA_MAX_LEN=82;
static uint8_t nmeaChecksumBody(const char* s){ uint8_t c=0; while(*s&&*s!='*') c^=(uint8_t)(*s++); return c; }
static bool allPrintableASCII(const String& s){ for(size_t i=0;i<s.length();++i){char c=s[i]; if(c<32||c>126) return false;} return true; }
static bool isHexChar(char c){ return (c>='0'&&c<='9')||(c>='A'&&c<='F')||(c>='a'&&c<='f'); }
static int toIntSafe(const String& s){ return s.length()? s.toInt():0; }
static float toFloatSafe(const String& s){ return s.length()? s.toFloat():NAN; }
static void parseTimeHHMMSS(const String& s,uint8_t&h,uint8_t&m,uint8_t&sec){ if(s.length()>=6){h=toIntSafe(s.substring(0,2));m=toIntSafe(s.substring(2,4));sec=toIntSafe(s.substring(4,6));}}
static void parseDateDDMMYY(const String& s,uint16_t&y,uint8_t&m,uint8_t&d){ if(s.length()>=6){d=toIntSafe(s.substring(0,2));m=toIntSafe(s.substring(2,4));y=2000+toIntSafe(s.substring(4,6));}}

static bool handleNMEA(const String& line){
  if(!line.startsWith("$")||line.length()<9||!allPrintableASCII(line)||line.length()>NMEA_MAX_LEN+3) return false;
  int star=line.indexOf('*'); if(star<0||star+3>(int)line.length()) return false;
  if(!isHexChar(line[star+1])||!isHexChar(line[star+2])) return false;
  String body=line.substring(1,star);
  uint8_t want=strtoul(line.substring(star+1,star+3).c_str(),NULL,16), got=nmeaChecksumBody(body.c_str());
  if(got!=want) return false;
  int comma=body.indexOf(','); String type=(comma>=0)?body.substring(0,comma):body;
  if(type.length()<5 && !type.startsWith("P")) return false;

  String f[32]; int fcnt=0, idx=(comma>=0)?comma+1:body.length();
  while(idx<=(int)body.length() && fcnt<32){ int next=body.indexOf(',',idx); if(next<0) next=body.length(); f[fcnt++]=body.substring(idx,next); idx=next+1; }

  if(type.endsWith("RMC")){ if(fcnt>=9){ parseTimeHHMMSS(f[0],st.hour,st.minute,st.second); if(f[1]=="A") st.hasFix=true; parseDateDDMMYY(f[8],st.year,st.month,st.day);} }
  else if(type.endsWith("GGA")){ if(fcnt>=7){ parseTimeHHMMSS(f[0],st.hour,st.minute,st.second); if(toIntSafe(f[5])>0) st.hasFix=true; st.satsUsed=(uint8_t)toIntSafe(f[6]); } }
  else if(type.endsWith("GSA")){ if(fcnt>=12) st.pdop=toFloatSafe(f[11]); }
  else if(type.endsWith("GSV")){ if(fcnt>=3) st.satsView=(uint8_t)toIntSafe(f[2]); }

  st.nmeaSentences++; Serial.println(line); return true;
}

// ===== TSIP (AB/AC) =====
static const uint8_t DLE=0x10, ETX=0x03;
enum TsipState { WAIT_DLE, IN_PACKET, AFTER_DLE }; TsipState tsState=WAIT_DLE;
static uint8_t tsBuf[256]; static size_t tsLen=0;
static void tsipReset(){ tsState=WAIT_DLE; tsLen=0; }
static uint16_t be16(const uint8_t* p){ return (uint16_t(p[0])<<8)|p[1]; }
static uint32_t be32(const uint8_t* p){ return (uint32_t(p[0])<<24)|(uint32_t(p[1])<<16)|(uint32_t(p[2])<<8)|p[3]; }

static void tsipHandlePacket(const uint8_t* p, size_t n){
  st.tsipPackets++; if(n<2) return;
  if(p[0]==0x8F){
    if(p[1]==0xAB && n>=18){
      uint8_t flags=p[10]; st.timeSet=((flags&(1<<2))==0);
      st.second=p[11]; st.minute=p[12]; st.hour=p[13];
      st.day=p[14]; st.month=p[15]; st.year=be16(&p[16]);
    } else if(p[1]==0xAC && n>=16){
      st.minorAlarms=be16(&p[11]); st.decStat=p[13]; st.ppsGood=(p[15]==0);
      st.hasFix=(st.decStat==0x00);
    }
  }
}
static void tsipFeed(uint8_t b){
  switch(tsState){
    case WAIT_DLE: if(b==DLE){tsState=IN_PACKET; tsLen=0;} break;
    case IN_PACKET: if(b==DLE){tsState=AFTER_DLE;} else { if(tsLen<sizeof(tsBuf)) tsBuf[tsLen++]=b; } break;
    case AFTER_DLE:
      if(b==DLE){ if(tsLen<sizeof(tsBuf)) tsBuf[tsLen++]=DLE; tsState=IN_PACKET; }
      else if(b==ETX){ tsipHandlePacket(tsBuf,tsLen); tsipReset(); }
      else { st.tsipBadFrames++; tsipReset(); }
      break;
  }
}

// ===== OLED helpers =====
static int textWidth6px(const char* s){ int n=0; while(*s){ n++; s++; } return n*6; }

static void drawHeader(const char* title, const char* rightLabel=nullptr){
  display.fillRect(0,0,128,10,SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK); display.setCursor(2,1); display.setTextSize(1); display.print(title);
  if(rightLabel && rightLabel[0]){
    int w = textWidth6px(rightLabel);
    int x = 128 - 2 - w; if (x < 2) x = 2;
    display.setCursor(x,1); display.print(rightLabel);
  }
  display.setTextColor(SSD1306_WHITE);
}

static void drawStatusLamps(){
  if(st.hasFix) display.fillCircle(122,6,3,SSD1306_WHITE); else display.drawCircle(122,6,3,SSD1306_WHITE);
  if(st.ppsGood) display.fillCircle(112,6,2,SSD1306_WHITE); else display.drawCircle(112,6,2,SSD1306_WHITE);
}

// ----- ALM decoding -----
static bool almBit(uint16_t alm, uint8_t bit){ return (alm & (1u<<bit))!=0; }
static void buildAlmList(char* out, size_t sz, uint16_t a){
  out[0]='\0'; bool first=true;
  auto add=[&](const char* s){ if(!first){ strlcat(out,", ",sz);} strlcat(out,s,sz); first=false; };
  if(almBit(a,1))  add("ANT OPEN");
  if(almBit(a,2))  add("ANT SHORT");
  if(almBit(a,3))  add("NOT TRACK");
  if(almBit(a,4))  add("NO DISCIP");
  if(almBit(a,5))  add("SURVEY");
  if(almBit(a,6))  add("NO POS");
  if(almBit(a,11)) add("ALMANAC");
  if(first) strlcpy(out,"OK",sz);
}

// ===== Pages =====
static void drawTimingPage(){
  const char* badge = st.minorAlarms ? "ALM!" : "";
  drawHeader("TIMING STATUS", badge);
  drawStatusLamps();

  display.setTextSize(2); display.setCursor(0,14);
  char buf[48]; snprintf(buf,sizeof(buf),"%02u:%02u:%02u",st.hour,st.minute,st.second); display.print(buf);

  display.setTextSize(1); display.setCursor(0,36);
  snprintf(buf,sizeof(buf),"%04u-%02u-%02u",st.year,st.month,st.day); display.print(buf);

  display.setCursor(0,48); display.print("Fix:"); display.print(st.hasFix?"YES":"NO ");
  display.setCursor(64,48); display.print("TSET:"); display.print(st.timeSet?"YES":"NO ");

  display.setCursor(0,56); display.print("STAT:"); char sb[4]; snprintf(sb,sizeof(sb),"%02X",st.decStat); display.print(sb);
  display.print("  PPS:"); display.print(st.ppsGood?"OK ":"BAD");
}

static void drawGnssPage(){
  const char* badge = st.minorAlarms ? "ALM!" : "";
  drawHeader("GNSS OVERVIEW", badge);
  drawStatusLamps();

  display.setTextSize(2); display.setCursor(0,14);
  char buf[32]; snprintf(buf,sizeof(buf),"U:%02u V:%02u",st.satsUsed,st.satsView); display.print(buf);

  display.setTextSize(1); display.setCursor(0,36);
  display.print("PDOP: "); if(!isnan(st.pdop)) display.print(st.pdop,1); else display.print("-");

  int bars=min<int>(st.satsUsed,8); for(int i=0;i<bars;i++){ int x=4+i*15; int h=8+(i%5)*2; display.fillRect(x,54-h,10,h,SSD1306_WHITE); }
}

static void drawPacketsPage(){
  const char* badge = st.minorAlarms ? "ALM!" : "";
  drawHeader("LINK / PACKETS", badge);
  drawStatusLamps();

  display.setTextSize(1);
  display.setCursor(0,14); display.print("Bytes:"); display.print(st.bytesIn);
  display.setCursor(0,26); display.print("TSIP ok:"); display.print(st.tsipPackets);
  display.setCursor(0,38); display.print("TSIP bad:"); display.print(st.tsipBadFrames);
  display.setCursor(0,50); display.print("NMEA:"); display.print(st.nmeaSentences);

  display.setCursor(74,14); display.print("ALM:");
  char hb[6]; snprintf(hb,sizeof(hb),"%.4X", st.minorAlarms); display.print(hb);
  display.setCursor(74,26); display.print("PPS:"); display.print(st.ppsGood?"OK ":"BAD");
}

static void drawAlarmPage(){
  drawHeader("ALARM DETAIL", "ALM!");
  drawStatusLamps();
  display.setTextSize(1);

  display.setCursor(0,14);
  display.print("SATs U/V: ");
  char satb[16]; snprintf(satb,sizeof(satb),"%u/%u", st.satsUsed, st.satsView); display.print(satb);
  display.print("  PDOP: ");
  if (!isnan(st.pdop)) display.print(st.pdop,1); else display.print("-");

  int y = 26;
  if(!st.minorAlarms){
    display.setCursor(0,y); display.print("No alarms.");
  } else {
    if(almBit(st.minorAlarms,1)){ display.setCursor(0,y); display.print("ANT OPEN"); y+=10; }
    if(almBit(st.minorAlarms,2)){ display.setCursor(0,y); display.print("ANT SHORT"); y+=10; }
    if(almBit(st.minorAlarms,3)){ display.setCursor(0,y); display.print("NOT TRACKING"); y+=10; }
    if(almBit(st.minorAlarms,4)){ display.setCursor(0,y); display.print("NOT DISCIPLINING"); y+=10; }
    if(almBit(st.minorAlarms,5)){ display.setCursor(0,y); display.print("SELF SURVEY"); y+=10; }
    if(almBit(st.minorAlarms,6)){ display.setCursor(0,y); display.print("NO STORED POSITION"); y+=10; }
    if(almBit(st.minorAlarms,11)){ display.setCursor(0,y); display.print("ALMANAC INCOMPLETE"); y+=10; }
  }
}

// ===== Bridge Page =====
static void drawBridgePage(){
  display.clearDisplay();
  drawHeader("BRIDGE MODE", "");
  display.setTextSize(2); display.setCursor(6,22); display.println("PC <-> Mini-T");
  display.setTextSize(1); display.setCursor(6,46); display.println("Press 'B' to exit");
  display.display();
}

static void render(){
  if(bridgeMode){ drawBridgePage(); return; }
  display.clearDisplay();
  switch(currentPage){
    case PAGE_TIMING:  drawTimingPage();  break;
    case PAGE_GNSS:    drawGnssPage();    break;
    case PAGE_PACKETS: drawPacketsPage(); break;
    case PAGE_ALARM:   drawAlarmPage();   break;
    default:           drawTimingPage();  break;
  }
  display.display();
}

// ===== Setup/Loop =====
void setup(){
  Wire.begin(I2C_SDA,I2C_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)){ for(;;){ delay(1000);} }
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE); display.setTextSize(1);
  display.setCursor(0,0); display.println("Mini-T Display"); display.display();

  Serial.begin(UART_BAUD);
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  Serial.println("Ready. Pages 0/1/2/3. 'B' toggles BRIDGE MODE. Button: short=page, long=bridge");

  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  setFixLED(false); // start off
}

void loop(){
// -------- USB controls --------
static char cmdBuf[8]; static uint8_t cmdLen = 0;
while (Serial.available()) {
  char ch = (char)Serial.read();
  if (ch=='B' || ch=='b') { bridgeMode=!bridgeMode; Serial.println(bridgeMode?"Bridge mode: ON":"Bridge mode: OFF"); render(); continue; }
  if (isalpha((unsigned char)ch)) {
    if (cmdLen < sizeof(cmdBuf)-1) cmdBuf[cmdLen++] = (char)toupper(ch);
  } else if (ch=='\r' || ch=='\n' || cmdLen >= sizeof(cmdBuf)-1) {
    cmdBuf[cmdLen] = '\0';
    if (!strcmp(cmdBuf, BRIDGE_CMD)) { bridgeMode=!bridgeMode; Serial.println(bridgeMode?"Bridge mode: ON":"Bridge mode: OFF"); render(); }
    else if (!bridgeMode && cmdLen==1 && cmdBuf[0]>='0' && cmdBuf[0]<='3') { currentPage=(Page)(cmdBuf[0]-'0'); render(); }
    cmdLen = 0;
  }
}

  // ---- Button: debounce + short/long detection ----
  {
    bool raw = digitalRead(BTN_PIN);
    unsigned long now = millis();

    // Debounce into btnStable
    if (raw != btnStable && (now - btnLastChange) >= BTN_DEBOUNCE_MS) {
      btnStable = raw;
      btnLastChange = now;

      // Edge: pressed (goes LOW)
      if (btnStable == LOW && !btnWasDown) {
        btnWasDown = true;
        btnPressStart = now;
      }

      // Edge: released (goes HIGH)
      else if (btnStable == HIGH && btnWasDown) {
        btnWasDown = false;
        unsigned long dur = now - btnPressStart;

        if (dur >= LONG_PRESS_MS) {
          // Long press: toggle bridge mode (from anywhere)
          bridgeMode = !bridgeMode;
          Serial.print("Bridge mode: "); Serial.println(bridgeMode ? "ON (long press)" : "OFF (long press)");
          render();
        } else {
          // Short press: next page (only in normal mode)
          if (!bridgeMode) {
            currentPage = (Page)((currentPage + 1) % PAGE_MAX);
            Serial.print("Page -> "); Serial.println((int)currentPage);
            render();
          }
        }
      }
    }
  }

  // ---- Update Fix LED ----
  setFixLED(st.hasFix);

  // Bridge mode
  if(bridgeMode){
    while(Serial.available()){
      int b=Serial.read();
      Serial2.write(b);
    }
    while(Serial2.available()) Serial.write(Serial2.read());
    static unsigned long lastBridgeDraw=0; unsigned long now=millis();
    if(now-lastBridgeDraw>500){ render(); lastBridgeDraw=now; }
    return;
  }

  // Normal mode
  Stream& gps = Serial2;

  while(gps.available()){
    int b=gps.read(); if(b<0) break; st.bytesIn++; uint8_t u=(uint8_t)b;

    if(u=='$'){ nmeaLine=""; nmeaLine.reserve(96); nmeaLine+='$'; }
    else if(!nmeaLine.isEmpty()){
      if(nmeaLine.length()<(int)NMEA_MAX_LEN+6) nmeaLine+=(char)u;
      if(u=='\n'||u=='\r'){ String s=nmeaLine; s.trim(); if(!s.isEmpty()) (void)handleNMEA(s); nmeaLine=""; }
    }
    tsipFeed(u);
  }

  static unsigned long lastPageTick=0, lastRender=0, lastPrint=0; unsigned long now=millis();
  if(now-lastPageTick>=PAGE_PERIOD_MS){ currentPage=(Page)((currentPage+1)%PAGE_MAX); lastPageTick=now; }
  if(now-lastRender>100){ render(); lastRender=now; }
  if(now-lastPrint>5000){
    lastPrint=now;
    Serial.print("TSIP: STAT="); char sb[4]; snprintf(sb,sizeof(sb),"%02X",st.decStat); Serial.print(sb);
    Serial.print(" PPS="); Serial.print(st.ppsGood? "OK":"BAD");
    Serial.print(" ALM="); char hb[6]; snprintf(hb,sizeof(hb),"%.4X",st.minorAlarms); Serial.print(hb);
    char out[96];
    out[0]='\0'; bool first=true;
    auto add=[&](const char* s){ if(!first){ strlcat(out,", ",sizeof(out)); } strlcat(out,s,sizeof(out)); first=false; };
    if(almBit(st.minorAlarms,1))  add("ANT OPEN");
    if(almBit(st.minorAlarms,2))  add("ANT SHORT");
    if(almBit(st.minorAlarms,3))  add("NOT TRACK");
    if(almBit(st.minorAlarms,4))  add("NO DISCIP");
    if(almBit(st.minorAlarms,5))  add("SURVEY");
    if(almBit(st.minorAlarms,6))  add("NO POS");
    if(almBit(st.minorAlarms,11)) add("ALMANAC");
    if(first) strlcpy(out,"OK",sizeof(out));
    Serial.print(" ["); Serial.print(out); Serial.print("]");
    Serial.print(" TSET="); Serial.print(st.timeSet? "YES":"NO");
    Serial.print(" UTC="); char tb[24]; snprintf(tb,sizeof(tb),"%04u-%02u-%02u %02u:%02u:%02u",st.year,st.month,st.day,st.hour,st.minute,st.second); Serial.println(tb);
  }
}
