/*
  =========================================================
   KUCHYNSKA MINUTKA - Arduino Nano
  =========================================================

  HARDVER:
   - OLED displej SSD1309 128x64, I2C
       VCC -> 5V (alebo 3.3V podla modulu)
       GND -> GND
       SDA -> A4
       SCL -> A5
   - 4x tlacidlo (jeden koniec na GND, druhy na pin - vyuzivame
     interny pull-up, ziadne externe rezistory netreba)
       BTN_START_STOP -> D4
       BTN_MINUTES    -> D5
       BTN_MODE       -> D6
       BTN_SLEEP      -> D2   (MUSI byt D2 alebo D3 - potrebuje
                               hardverove prerusenie na budenie
                               z hlbokeho spanku)
   - Pasivny buzzer (+ na D9, - na GND)

  POTREBNE KNIZNICE (Library Manager v Arduine IDE):
   - U8g2 (autor: oliver)
   - Bounce2 (autor: Thomas O Fredericks)
   EEPROM, avr/sleep.h, avr/power.h su sucastou AVR jadra,
   nic dalsie netreba instalovat.

  OVLADANIE:
   - START/STOP: kratke = start / pauza / pokracovanie
                 dlhe (drz aspon 2 sekundy) = reset - spusti sa HNED po
                           uplynuti 2s (kym je tlacidlo este stale drzane),
                           netreba ho pustit.
                           V ZIADNOM-REZIME (standardna minutka bez nazvu)
                           tento reset VZDY vynuluje cas na 00:00, nech uz
                           bol pred tym nastaveny akykolvek cas.
                           V POMENOVANYCH REZIMOCH (vajicko, knedlik,
                           pizza) reset naopak nastavi ulozeny preset pre
                           dany rezim - tak ako doteraz, bez zmeny.
   - MINUTY:     kratke = +1 min (funguje len pred spustenim)
                 drzanie = rovnaky "tik" ako predtym pri sekundach
                           (cca kazdych 333 ms pri rychlosti 3.0), ale
                           kazdy tik teraz prida CELU MINUTU namiesto
                           1 sekundy - t.j. cca 3 min pribudnu za kazdu
                           1 sekundu drzania tlacidla
   - MODE:       kratke = dalsi rezim (vratane "ziadny rezim" =
                           standardna minutka bez nazvu)
                 dlhe (drz)  = ulozi aktualne nastavene minuty
                           ako novy cas pre tento rezim (natrvalo,
                           do EEPROM)
   - SLEEP:      kratke = uspi displej / cele Arduino (podla toho
                           ci prave nieco pocitame), znova stlac
                           na zobudenie

  REZIMY: po zapnuti je VZDY aktivna "ziadny rezim" - standardna
  minutka bez nazvu. POZOR: tento rezim si vobec NEPAMATA ulozeny
  preset a VZDY pracuje s casom 00:00 - a to nielen hned po zapnuti
  napajania, ale aj kedykolvek sa nan prepne tlacidlom MODE pocas
  behu programu, aj pri resete tlacidlom START/STOP (dlhe drzanie).
  Ulozeny preset v EEPROM pre tento rezim tak realne uz nema ziadny
  vplyv na zobrazovany cas. Tlacidlom MODE sa da prepnut na
  pomenovane rezimy (vajicko namakko/natvrdo, knedlik, pizza), ktore
  maju vlastnu animaciu behom varenia a ich ulozeny preset sa
  pouziva normalne (bez zmeny oproti povodnemu spravaniu).

  DIAKRITIKA: nazvy rezimov pouzivaju slovenske znaky a preto sa
  vykresluju cez vlastny font u8g2_font_unifont_t_slovak (funkcie
  drawUTF8/getUTF8Width). POZOR - povodne sa tu pouzival hotovy font
  u8g2_font_unifont_t_polish, ale ten obsahuje LEN polske znaky
  (Ą Ć Ę Ł Ń Ó Ź Ż a ich male verzie) - ziadne slovenske znaky ako
  í, č, ä, š, ž, ľ, ť, ď, ô, ŕ, ĺ, ň v nom nie su, takze sa napr.
  "Vajíčko" vykreslilo ako "vajko" (chybajuce znaky sa jednoducho
  preskocia). Preto je nizsie definovany UPLNE VLASTNY font (vid
  pole u8g2_font_unifont_t_slovak[] par riadkov nizsie), vygenerovany
  nastrojom bdfconv priamo z GNU Unifontu tak, aby obsahoval len
  bazicke ASCII (32-127) + presne tie slovenske znaky, ktore su
  potrebne (aj velke aj male pismena c,s,z,l,d,t,r,a,e,i,o,u,y s
  prislusnou diakritikou). Vysledkom je font, ktory je este MENSI
  nez povodny polsky (cca 1,8 KB), ale zobrazi slovencinu spravne.

  ANIMACIE POCAS BEHU:
   - ziadny rezim   -> presypacie hodiny, ktore realne ukazuju
                       zostavajuci cas (mnozstvo piesku v hornej
                       banke = zostavajuci cas)
   - vajicko mekke/tvrde -> nizky hrniec s vodou, 3 vajicka a
                       uskami (drzadlami) po strane, z vody stupa
                       para
   - knedlik / pizza -> povodna animacia s parou

  POZNAMKA: Ak displej po nahrati zostane prazdny alebo bliká
  nezmyselne, skus zmenit konstruktor nizsie z NONAME0 na NONAME2
  (rozne vyrobne serie SSD1309 maju mierne odlisnu inicializaciu).
  =========================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Bounce2.h>
#include <EEPROM.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <stdio.h>
#include <math.h>

// ---------- Piny ----------
#define BTN_START_STOP  4
#define BTN_MINUTES     5
#define BTN_MODE        6
#define BTN_SLEEP       2
#define BUZZER_PIN      9

// ---------- Displej ----------
// Ak nefunguje, skus U8G2_SSD1309_128X64_NONAME2_F_HW_I2C
U8G2_SSD1309_128X64_NONAME0_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// ---------- Tlacidla (debounce) ----------
Bounce bStartStop = Bounce();
Bounce bMinutes   = Bounce();
Bounce bMode      = Bounce();
Bounce bSleep     = Bounce();

// ---------- Stavy a rezimy ----------
enum TimerState : uint8_t { STATE_READY, STATE_RUNNING, STATE_PAUSED, STATE_ALARM };
// MODE_NONE = standardna minutka bez nazvu, vzdy aktivna po zapnuti
enum TimerMode  : uint8_t { MODE_NONE, MODE_EGG_SOFT, MODE_EGG_HARD, MODE_DUMPLING, MODE_PIZZA, MODE_COUNT };

// Prazdny nazov pre MODE_NONE - v UI sa jednoducho nezobrazi ziadny text.
// Retazce su v UTF-8 (Arduino IDE uklada .ino subory v UTF-8) a vykreslujú
// sa cez u8g2.drawUTF8()/getUTF8Width(), NIE cez drawStr()/getStrWidth(),
// lebo tie neviem dekodovat viacbajtove UTF-8 znaky.
const char* modeNames[MODE_COUNT] = { "", "Vajíčko na mäkko", "Vajíčko na tvrdo", "Knedlík", "Pizza" };
const char* modeDisplayNames[MODE_COUNT] = { "", "Vajicko na makko", "Vajicko na tvrdo", "Knedlik", "Pizza" };

// ---------- Vlastny font pre nazvy rezimov (slovenska diakritika) ----------
// Povodny u8g2_font_unifont_t_polish obsahoval LEN polske znaky a
// slovenske znaky (i, c, s, z, l, t, d, o, r s diakritikou) chybali -
// preto je tu vlastny font vygenerovany priamo z GNU Unifontu (rovnakym
// nastrojom - bdfconv - akym su robene vsetky ostatne u8g2 fonty),
// obsahujuci len ASCII (32-127) + presne tie slovenske znaky, ktore
// tento projekt potrebuje. Vysledok je MENSI (1805 B) nez povodny
// polsky font (1561 B pri polovicnom pokryti) aj nez alternativa
// u8g2_font_unifont_t_extended (11292 B, vsetko od A do Z aj s
// nepotrebnymi znakmi navyse).
/*
  Fontname: -gnu-Unifont-Medium-R-Normal-Sans-16-160-75-75-c-80-iso10646-1
  Copyright: Copyright (C) 1998-2024 Roman Czyborra, Paul Hardy, Qianqian Fang, Andrew Miller, Johnnie Weaver, David Corbett, Nils Moskopp, Rebecca Bettencourt, Ho-Seok Ee, et al. License: SIL Open Font License version 1.1 and GPLv2+: GNU GPL version 2 or later <http://gnu.org/licenses/gpl.html> with the GNU Font Embedding Exception.
  Glyphs: 130/57084
  BBX Build Mode: 0
*/
const uint8_t u8g2_font_unifont_t_slovak[1805] U8G2_FONT_SECTION("u8g2_font_unifont_t_slovak") = 
  "\202\0\3\2\5\5\4\5\6\20\20\0\376\12\376\13\377\1\233\3\63\5\310 \6\0\240G\1!\10A"
  "\61DqH\4\42\10\205(F\221\271\5#\17F%D\325\323\60$Q\313\60D=\1$\22G%"
  "D\27\16J\24I\341<FRe\20\63\0%\24G%D\243II)\211\224\64N\23)\211\222\222"
  "\246\0&\22G%D\265\225\262J(&\221\226\210I&M\1'\7\201\60F\61\4(\14\203\355C"
  "\225DI\324[\224\5)\15\203\351C\221EY\324K\224D\0*\15\347dDW\252\264mIS-"
  "\3+\14\347dD\27\327\206!\213k\0,\11\202\254C\241$\12\0-\7$(E\61\4.\7B"
  ",D\61\4/\14F%D[\254\206iXM\1\60\22F%D\245EI\250M\211\22mb\22e"
  "\22\0\61\13E)D\225II\330\247A\62\17F%D\63$\241\230fZXM\207\1\63\20F%"
  "D\63$\241\230Fs*\212\311\220\0\64\20F%D\31jIT\311\222,\31\306\264\2\65\17F%"
  "DqH\253\203\234\246b\62$\0\66\17F%D\65\205i:(\241c\62$\0\67\13F%Dq"
  "-\246\305\264\11\70\20F%D\63$\241\61\31\222\320\61\31\22\0\71\16F%D\63$\241\61\31\324"
  "\306h\2:\11\342lD\61\304C\0;\12\42\355C\61\304J\242\0<\11%)D\231u\355\0="
  "\11\246\244Dq'\16\3>\11%%D\221v\353\10\77\17F%D\63$\241\230\206\325\34L#\0"
  "@\22F%D\65eR\242$K\244DJ$-\361\20A\16F%D\245E-\241\70\14\242c\0"
  "B\16F%D\61(\241qXB\307a\1C\16F%D\63$\241\265\243\230\14\11\0D\16F%"
  "D\61DY\22\372-\31\42\0E\15F%DqH\253\203\222\266\16\3F\14F%DqH\253\203"
  "\222v\5G\16F%D\63$\241\265\64\204\66e\11H\13F%D\21:\16\203\350\61I\13E)"
  "D\61Ha\77\15\2J\16G%D\65\210qOY\224e\33\0K\21F%D\21jIT\311D"
  "\61\311\242Z\22\6L\11F%D\221\366\327aM\15F%D\21\212\323\20-\36\35\3N\20F%"
  "D\21n\233\22)\221\224H\211v\14O\14F%D\63$\241\77&C\2P\15F%D\61(\241"
  "qX\322\256\0Q\26g\345C\63Da\22&a\22&a\22&\211\222H\322\220\13R\20F%D"
  "\61(\241qX\242Z\222%\241\30S\15F%D\63$\241\331QL\206\4T\12G%Dq\310\342"
  "\376\6U\13F%D\21\372\307dH\0V\21G%D\221Z\223,\312\242\254\22&i\234\1W\15"
  "F%D\21zq\231\206h\24\3X\17F%D\21\212I\324&jQK(\6Y\16G%D\221"
  "\252I\26e\225\64\356\6Z\13F%Dq-\366\232\16\3[\12\203\361C\61D\375\323\0\134\14F"
  "%D\221\306\325\70\215\253\1]\12\203\345C\61\365OC\0^\11fdF\245EI\30_\7'\344"
  "Cq\10`\7c\250F\221\25a\16\6%D\63$a\232\14\243MY\2b\16f%D\221\266,"
  "\232\350\270)\13\0c\15\6%D\63$\241\332\61\31\22\0d\14f%D\333\262h\243\67e\11e"
  "\17\6%D\63$\241\70\14j\61\31\22\0f\14e%D'\205\245A\12{\2g\23f\245C\233"
  ",Z\222%Y\264\245C\22\212\311\220\0h\14f%D\221\266,\232\350c\0i\13e)D\25\346"
  "\210\330\247Aj\14\245\245CYG\304>J\221\4k\21f%D\221\266%Q%\23\223,\252%a"
  "\0l\12e)D#\366O\203\0m\22\7%D\261(Q$ER$ER$ER\1n\13\6"
  "%D\221,\232\350c\0o\14\6%D\63$\241\217\311\220\0p\16F\245C\221,\232\350\270)K"
  "\232\2q\14F\245C\263h\243\67eI\13r\13\6%D\221,\232\250v\5s\15\6%D\63$"
  "\241\354\230\14\11\0t\13E%D\25\226\6)\354*u\12\6%D\21\372MY\2v\14\6%D"
  "\21\32\223\250\67Q\2w\21\7%D\221J\221\24I\221\24I\221T\261\0x\17\6%D\21\212I"
  "\224\211Z\224\204b\0y\16F\245C\21zL\42KZ\31\22\0z\12\6%Dq\15{\35\6{"
  "\16\244\251C\245da\26\25kQ\26\12|\7\301\261C\361A}\17\244\251C!fQ\26\226ja"
  "\226H\0~\12g$F\243I\221\246\0\177#\20\242\203\221\364;I\347\244\223\16I\66%a\32%"
  "C\222MI\230NC\62\354\234tN:)\351\7\301\20\306%D'\351\64-j\11\305a\20\35\3"
  "\304\20\306%D\23\265S\264\250%\24\207At\14\311\20\306%D'\351\224aH\253\203\222\266\16\3"
  "\315\15\305)D\245\350\360 \205\375\64\10\323\17\306%D'\351\244!\11\375\61\31\22\0\324\17\306%"
  "D\245E\71>$\241\77&C\2\332\15\306%D'\351\224\320\77&C\2\335\20\307%Dg'\246"
  "j\222EY%\215\273\1\341\20\206%D'\351\244!\11\323d\30m\312\22\344\20\206%D\23\265\343"
  "C\22\246\311\60\332\224%\351\21\206%D'\351\244!\11\305aP\213\311\220\0\355\13\205)D\245\350"
  "\270\330\247A\363\16\206%D'\351\244!\11}L\206\4\364\17\206%D\245E\71>$\241\217\311\220"
  "\0\372\14\206%D'\351\224\320o\312\22\375\20\306\245C'\351\224\320c\22Y\322\312\220\0\0\0\0"
  "\4\377\377\1\14\22\306%D\23e:eHBkG\61\31\22\0\1\15\21\206%D\23e:eH"
  "B\265c\62$\0\1\16\22\306%D\21e:e\210\262$\364[\62D\0\1\17\27g!D\227h"
  "Q\26%]\264(\213\262(\213\262DL\62\0\1\71\14\306%De'\245\375u\30\1:\15\305)"
  "D\245\350\220\330\77\15\2\1=\17F%D\21)Y\222%Q\265\353\60\1>\16f%Dc\352%"
  "K{\33\24\0\1G\24\306%D\23e:\36n\233\22)\221\224H\211v\14\1H\17\206%D\23"
  "e:\236,\232\350c\0\1T\24\306%D'\351\224A\11\215\303\22\325\222,\11\305\0\1U\17\206"
  "%D'\351\224d\321D\265+\0\1`\22\306%D\23e:eHB\263\243\230\14\11\0\1a\21"
  "\206%D\23e:eHB\331\61\31\22\0\1d\17\307%D\23\205:m\30\262\270\277\1\1e\20"
  "\207!D+GYT\13\7\61\356.\1\1}\17\306%D\23e:>\254\305^\323a\1~\17\206"
  "%D\23e:>\254a\257\303\0\0";

// Font pre nazvy rezimov - teraz vlastny font so slovenskou diakritikou
// (namiesto povodneho u8g2_font_unifont_t_polish, ktory ju nemal)
#define MODE_FONT u8g2_font_unifont_t_slovak

// Predvolene casy pri prvom spusteni (potom sa daju upravit a ulozit
// dlhym stlacenim MODE - hodnota knedlika 20 min je len odhad, uprav podla chuti)
const uint8_t defaultPresetMinutes[MODE_COUNT] = { 5, 5, 10, 20, 30 };

// ---------- Casove konstanty ----------
const uint16_t LONG_PRESS_MS        = 600;
// Reset casu podrzanim START/STOP - musi sa drzat aspon takto dlho a
// akcia sa spusti HNED (kym je tlacidlo este drzane), nie az po pusteni
const uint16_t RESET_HOLD_MS        = 2000;
// Plynule pridavanie casu pri drzani MINUTY:
// rychlost = kolko sekund "zaslozeneho" casu pribudne za kazdu 1 sekundu
// drzania tlacidla - ale realne sa pripocitava az po celych minutach
// (60 s naraz), nie po jednotlivych sekundach
const float    MINUTES_HOLD_RATE_SEC_PER_SEC = 3.0;
const uint8_t  MAX_MINUTES          = 99;
const uint8_t  MIN_MINUTES          = 1;
const unsigned long MAX_SECONDS = (unsigned long)MAX_MINUTES * 60UL;
const unsigned long ALARM_AUTO_OFF_MS = 60000UL;  // alarm pipa 1 minutu
const unsigned long ALARM_PERIOD_MS   = 200;       // preryvavy ton - perioda 200 ms (aktivny buzzer)
const unsigned long BUTTON_BEEP_MS    = 150;
const unsigned long READY_SLEEP_MS    = 30000UL;
const unsigned long PAUSE_SLEEP_MS    = 600000UL;
const unsigned long WAKE_IGNORE_MS    = 10000UL;
const unsigned long DIM_DELAY_MS      = 10000UL;
const unsigned long DIM_THRESHOLD_SECONDS = 600UL;
const uint8_t DISPLAY_CONTRAST        = 1;
const uint8_t DIMMED_CONTRAST         = 1;
const unsigned long SAVE_MSG_MS       = 1200;
const unsigned long ANIM_STEP_MS      = 150;

// ---------- EEPROM adresy ----------
#define EE_MAGIC_ADDR    0
#define EE_MAGIC_VAL     0xA5
#define EE_PRESET_ADDR   1                         // MODE_COUNT bajtov

// ---------- Globalny stav ----------
uint8_t presetMinutes[MODE_COUNT];
uint8_t currentMode = MODE_NONE;
// remainingSeconds sluzi ako jediny zdroj pravdy pre cas: v stave READY je
// to "nastaveny cas", v stave RUNNING/PAUSED je to "zostavajuci cas"
unsigned long remainingSeconds = 0;
unsigned long totalSecondsAtStart = 0; // pre vypocet podielu v presypacich hodinach
unsigned long lastSecondTick = 0;

TimerState state = STATE_READY;
bool isDisplaySleeping = false;
bool ignoreSleepRelease = false;
bool waitingForSleepRelease = false;
volatile bool sleepWakeRequested = false;
unsigned long ignoreSleepUntil = 0;

unsigned long alarmStartMillis = 0;
unsigned long lastAlarmToggle = 0;
bool alarmToneOn = false;

unsigned long savedMsgUntil = 0;
unsigned long lastActivityMillis = 0;
unsigned long pauseStartMillis = 0;
unsigned long runningStartMillis = 0;
bool displayDimmed = false;
unsigned long beepUntil = 0;
unsigned long welcomeStartMillis = 0;
bool welcomeActive = false;

uint8_t animFrame = 0;
unsigned long lastAnimStep = 0;

unsigned long minutesPressStart = 0;
unsigned long minutesLastTickMs = 0;
float minutesAccumSeconds = 0.0;
bool minutesRepeating = false;

unsigned long modePressStart = 0;
unsigned long startPressStart = 0;
bool startStopLongActionDone = false; // true = reset uz prebehol pocas tohto drzania

// =========================================================
//  SETUP
// =========================================================
void setup() {
  pinMode(BTN_START_STOP, INPUT_PULLUP);
  pinMode(BTN_MINUTES, INPUT_PULLUP);
  pinMode(BTN_MODE, INPUT_PULLUP);
  pinMode(BTN_SLEEP, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  bStartStop.attach(BTN_START_STOP);
  bStartStop.interval(20);
  bMinutes.attach(BTN_MINUTES);
  bMinutes.interval(20);
  bMode.attach(BTN_MODE);
  bMode.interval(20);
  bSleep.attach(BTN_SLEEP);
  bSleep.interval(20);

  u8g2.begin();

  loadSettings(); // vzdy nastavi currentMode = MODE_NONE (standardna minutka)

  // Ziadny-mod (currentMode == MODE_NONE) po zapnuti VZDY zacina na 00:00 -
  // ulozeny preset pre tento rezim sa ignoruje (loadSettings() vzdy
  // nastavi MODE_NONE, takze v praxi tu vzdy vyjde 0).
  remainingSeconds = presetSecondsFor(currentMode);
  lastActivityMillis = millis();
}

// =========================================================
//  HLAVNA SLUCKA
// =========================================================
void loop() {
  bStartStop.update();
  bMinutes.update();
  bMode.update();
  bSleep.update();

  bool startPressed = bStartStop.fell();
  bool minutesPressed = bMinutes.fell();
  bool modePressed = bMode.fell();
  bool sleepPressed = bSleep.fell();
  bool anyPressed = startPressed || minutesPressed || modePressed || sleepPressed;
  if (sleepWakeRequested) {
    sleepWakeRequested = false;
    wakeDisplay();
    isDisplaySleeping = false;
    waitingForSleepRelease = true;
    lastActivityMillis = millis();
    ignoreSleepUntil = millis() + WAKE_IGNORE_MS;
    welcomeStartMillis = millis();
    welcomeActive = true;
    bSleep.update();
  }
  if (startPressed) buttonBeep();
  if (minutesPressed) buttonBeep();
  if (modePressed) buttonBeep();
  updateButtonBeep();

  // --- displej spi (ale MCU stale bezi a pocita) ---
  if (isDisplaySleeping) {
    if (sleepPressed) {
      wakeDisplay();
      waitingForSleepRelease = true;
      lastActivityMillis = millis();
      ignoreSleepUntil = millis() + WAKE_IGNORE_MS;
      welcomeStartMillis = millis();
      welcomeActive = true;
    }
    updateTimer();
    handleAutomaticDimming();
    if (state == STATE_ALARM) {
      wakeDisplay();
      handleAlarmSound();
      drawScreen();
    }
    return;
  }

  // --- prebieha alarm - hocijake tlacidlo ho stlmi ---
  if (state == STATE_ALARM) {
    if (anyPressed) {
      stopAlarm();
    } else {
      handleAlarmSound();
    }
    updateTimer();
    drawScreen();
    return;
  }

  handleStartStopButton();
  handleMinutesButton();
  handleModeButton();
  handleSleepButton();

  updateTimer();
  updateAnimation();
  handleAutomaticDimming();
  drawScreen();
  handleAutomaticSleep();
}

// =========================================================
//  TLACIDLO START/STOP
// =========================================================
void handleStartStopButton() {
  if (bStartStop.fell()) {
    startPressStart = millis();
    startStopLongActionDone = false;
    lastActivityMillis = millis();
  }

  // Kym je tlacidlo drzane, priebezne sleduj ci uz ubehli 3 sekundy.
  // Ak ano, reset sa spusti HNED (nie az po pusteni tlacidla).
  if (bStartStop.read() == LOW && !startStopLongActionDone) {
    if (millis() - startPressStart >= RESET_HOLD_MS) {
      resetToPreset();
      startStopLongActionDone = true; // aby sa reset nezopakoval znova a znova
    }
  }

  if (bStartStop.rose()) {
    // Ak dlhe drzanie uz spustilo reset, kratke-stlacenie akcia sa NEROBI
    // (inak by sa po pusteni tlacidla hned znova prepol stav start/pauza)
    if (!startStopLongActionDone) {
      switch (state) {
        case STATE_READY:
          if (remainingSeconds > 0) {
            totalSecondsAtStart = remainingSeconds;
            lastSecondTick = millis();
            runningStartMillis = millis();
            pauseStartMillis = 0;
            restoreDisplayBrightness();
            animFrame = 0;
            state = STATE_RUNNING;
          }
          break;
        case STATE_RUNNING:
          state = STATE_PAUSED;
          pauseStartMillis = millis();
          restoreDisplayBrightness();
          break;
        case STATE_PAUSED:
          lastSecondTick = millis();
          pauseStartMillis = 0;
          runningStartMillis = millis();
          state = STATE_RUNNING;
          break;
        default:
          break;
      }
    }
    startStopLongActionDone = false;
  }
}

void resetToPreset() {
  state = STATE_READY;
  // MODE_NONE -> vzdy 00:00 (nech uz bol pred tym nastaveny akykolvek
  // cas); pomenovane rezimy -> ich ulozeny preset, bez zmeny
  remainingSeconds = presetSecondsFor(currentMode);
}

// =========================================================
//  TLACIDLO MINUTY (s auto-opakovanim pri drzani)
// =========================================================
void handleMinutesButton() {
  if (state != STATE_READY) return; // pocas behu sa cas neupravuje

  if (bMinutes.fell()) {
    minutesPressStart = millis();
    minutesRepeating = false;
    lastActivityMillis = millis();
  }

  if (bMinutes.read() == LOW) {
    unsigned long held = millis() - minutesPressStart;
    if (held >= LONG_PRESS_MS) {
      unsigned long now = millis();
      if (!minutesRepeating) {
        // prave sme presli z kratkeho tapu do plynuleho drzania - zacni pocitat odteraz
        minutesRepeating = true;
        minutesLastTickMs = now;
        minutesAccumSeconds = 0.0;
      }
      unsigned long deltaMs = now - minutesLastTickMs;
      minutesLastTickMs = now;
      // rovnaka frekvencia "tikov" ako predtym pri sekundach (kazdych
      // ~333 ms pri rychlosti 3.0), len teraz sa pri kazdom tiku
      // pripocita rovno CELA MINUTA (60 s) namiesto 1 sekundy
      minutesAccumSeconds += (deltaMs / 1000.0) * MINUTES_HOLD_RATE_SEC_PER_SEC;
      while (minutesAccumSeconds >= 1.0) {
        addSeconds(60); // +1 cela minuta na kazdy tik
        minutesAccumSeconds -= 1.0;
      }
    }
  }

  if (bMinutes.rose()) {
    unsigned long heldFor = millis() - minutesPressStart;
    if (!minutesRepeating && heldFor < LONG_PRESS_MS) {
      addSeconds(60); // obycajny kratky tap = +1 min
    }
    minutesRepeating = false;
  }
}

void addSeconds(unsigned long sec) {
  unsigned long newVal = remainingSeconds + sec;
  if (newVal > MAX_SECONDS) newVal = MAX_SECONDS;
  remainingSeconds = newVal;
}

// Vrati cas (v sekundach), na ktory sa ma nastavit dany rezim - pre
// ziadny-rezim (MODE_NONE) je to VZDY 00:00, pre pomenovane rezimy je
// to ich ulozeny preset (bez zmeny oproti povodnemu spravaniu).
// Pouziva sa v setup(), pri prepnuti rezimu, pri resete tlacidlom
// START/STOP aj po skonceni alarmu, aby sa toto pravidlo dodrzalo
// konzistentne na vsetkych miestach.
unsigned long presetSecondsFor(uint8_t mode) {
  if (mode == MODE_NONE) return 0UL;
  return (unsigned long)presetMinutes[mode] * 60UL;
}

// =========================================================
//  TLACIDLO MODE
// =========================================================
void handleModeButton() {
  if (bMode.fell()) {
    modePressStart = millis();
    lastActivityMillis = millis();
  }
  if (bMode.rose()) {
    unsigned long heldFor = millis() - modePressStart;
    if (state != STATE_READY) return; // mod sa meni len pred spustenim

    if (heldFor >= LONG_PRESS_MS) {
      // ulozi aktualny cas (zaokruhleny na cele minuty) ako novy preset pre tento rezim
      unsigned long mins = remainingSeconds / 60UL;
      if (mins < MIN_MINUTES) mins = MIN_MINUTES;
      if (mins > MAX_MINUTES) mins = MAX_MINUTES;
      presetMinutes[currentMode] = (uint8_t)mins;
      saveSettings();
      savedMsgUntil = millis() + SAVE_MSG_MS;
    } else {
      currentMode = (currentMode + 1) % MODE_COUNT;
      remainingSeconds = presetSecondsFor(currentMode);
    }
  }
}

// =========================================================
//  TLACIDLO SLEEP
// =========================================================
void handleSleepButton() {
  if (millis() < ignoreSleepUntil) {
    return;
  }
  if (bSleep.rose()) {
    if (waitingForSleepRelease) {
      waitingForSleepRelease = false;
      ignoreSleepRelease = false;
      return;
    }
    lastActivityMillis = millis();
    if (ignoreSleepRelease) {
      ignoreSleepRelease = false;
      return;
    }
    if (state == STATE_READY) {
      enterDeepSleep(); // ozajstny power-down, nic sa nepocita
    } else {
      u8g2.setPowerSave(1); // len vypni displej, MCU pocita dalej
      isDisplaySleeping = true;
    }
  }
}

void wakeDisplay() {
  u8g2.setPowerSave(0);
  restoreDisplayBrightness();
  isDisplaySleeping = false;
}

void restoreDisplayBrightness() {
  u8g2.setContrast(DISPLAY_CONTRAST);
  displayDimmed = false;
}

void handleAutomaticDimming() {
  if (state == STATE_RUNNING && totalSecondsAtStart > DIM_THRESHOLD_SECONDS &&
      !displayDimmed && millis() - runningStartMillis >= DIM_DELAY_MS) {
    u8g2.setContrast(DIMMED_CONTRAST);
    displayDimmed = true;
  }
}

void wakeISR() {
  sleepWakeRequested = true;
}

void buttonBeep() {
  digitalWrite(BUZZER_PIN, HIGH);
  beepUntil = millis() + BUTTON_BEEP_MS;
}

void updateButtonBeep() {
  if (beepUntil != 0 && millis() >= beepUntil) {
    digitalWrite(BUZZER_PIN, LOW);
    beepUntil = 0;
  }
}

void enterDeepSleep() {
  restoreDisplayBrightness();
  u8g2.setPowerSave(1);
  isDisplaySleeping = true;
  bSleep.update();

  attachInterrupt(digitalPinToInterrupt(BTN_SLEEP), wakeISR, FALLING);

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  noInterrupts();
  sleep_enable();
  interrupts();
  sleep_cpu();               // <-- MCU tu naozaj zastavi

  // sem sa dostaneme az po prebudeni
  sleep_disable();
  detachInterrupt(digitalPinToInterrupt(BTN_SLEEP));

  wakeDisplay();
  waitingForSleepRelease = true;
  lastActivityMillis = millis();
  ignoreSleepUntil = millis() + WAKE_IGNORE_MS;
  welcomeStartMillis = millis();
  welcomeActive = true;
  bSleep.update(); // aby sa budiace stlacenie nezapocitalo znova
}

// =========================================================
//  ODPOCITAVANIE CASU
// =========================================================
void updateTimer() {
  if (state != STATE_RUNNING) return;

  unsigned long now = millis();
  if (now - lastSecondTick >= 1000) {
    lastSecondTick += 1000;
    if (remainingSeconds > 0) remainingSeconds--;
    if (remainingSeconds == 0) {
      state = STATE_ALARM;
      alarmStartMillis = millis();
      lastAlarmToggle = alarmStartMillis;
      alarmToneOn = false;
    }
  }
}

// =========================================================
//  ALARM (nonblocking pipanie)
// =========================================================
void handleAlarmSound() {
  // POZOR: toto je pre AKTIVNY buzzer (ma vlastnu elektroniku a piska
  // sam pri privedeni napajania) - preto sa neriadi cez tone(), len sa
  // strieda HIGH/LOW s pevnou periodou 200 ms.
  unsigned long now = millis();

  if (now - alarmStartMillis >= ALARM_AUTO_OFF_MS) {
    stopAlarm();
    return;
  }

  if (now - lastAlarmToggle >= ALARM_PERIOD_MS) {
    lastAlarmToggle = now;
    alarmToneOn = !alarmToneOn;
    digitalWrite(BUZZER_PIN, alarmToneOn ? HIGH : LOW);
  }
}

void handleAutomaticSleep() {
  if (state == STATE_READY &&
      millis() - lastActivityMillis >= READY_SLEEP_MS) {
    enterDeepSleep();
  } else if (state == STATE_PAUSED &&
             millis() - pauseStartMillis >= PAUSE_SLEEP_MS) {
    enterDeepSleep();
  }
}

void stopAlarm() {
  digitalWrite(BUZZER_PIN, LOW);
  state = STATE_READY;
  remainingSeconds = presetSecondsFor(currentMode);
  lastActivityMillis = millis();
  restoreDisplayBrightness();
}

// =========================================================
//  ANIMACIA - krokovanie frame-ov
// =========================================================
void updateAnimation() {
  if (state != STATE_RUNNING) return;
  if (millis() - lastAnimStep >= ANIM_STEP_MS) {
    lastAnimStep = millis();
    animFrame++;
  }
}

// =========================================================
//  EEPROM
// =========================================================
void loadSettings() {
  if (EEPROM.read(EE_MAGIC_ADDR) != EE_MAGIC_VAL) {
    for (uint8_t i = 0; i < MODE_COUNT; i++) {
      presetMinutes[i] = defaultPresetMinutes[i];
      EEPROM.write(EE_PRESET_ADDR + i, presetMinutes[i]);
    }
    EEPROM.write(EE_MAGIC_ADDR, EE_MAGIC_VAL);
  } else {
    for (uint8_t i = 0; i < MODE_COUNT; i++) {
      uint8_t val = EEPROM.read(EE_PRESET_ADDR + i);
      presetMinutes[i] = (val >= MIN_MINUTES && val <= MAX_MINUTES) ? val : defaultPresetMinutes[i];
    }
  }
  // Po zapnuti je vzdy aktivna standardna minutka bez nazvu rezimu
  currentMode = MODE_NONE;
}

void saveSettings() {
  EEPROM.update(EE_PRESET_ADDR + currentMode, presetMinutes[currentMode]); // .update() setri EEPROM
}

// =========================================================
//  KRESLENIE OBRAZOVKY
// =========================================================
void formatTime(unsigned long totalSeconds, char* buf) {
  unsigned int m = totalSeconds / 60;
  unsigned int s = totalSeconds % 60;
  sprintf(buf, "%02u:%02u", m, s);
}

void drawTimeCentered(const char* timeText, int16_t y) {
  int16_t timeWidth = u8g2.getStrWidth("88:88");
  u8g2.drawStr((128 - timeWidth) / 2, y, timeText);
}

// Vykresli nazov aktualneho rezimu vycentrovany podla skutocnej dlzky
// textu (UTF-8, kvoli diakritike). Ak je aktivny ziadny-mod, nic nekresli.
void drawModeNameCentered(int16_t y, bool compact) {
  if (currentMode == MODE_NONE) return;
  const char* name = compact ? modeDisplayNames[currentMode] : modeNames[currentMode];
  u8g2.setFont(compact ? u8g2_font_6x10_tf : MODE_FONT);
  int w = compact ? u8g2.getStrWidth(name) : u8g2.getUTF8Width(name);
  int16_t x = (128 - w) / 2;
  if (x < 0) x = 0; // poistka pre pripad, ze by bol nazov sirsi ako displej
  if (compact) {
    u8g2.drawStr(x, y, name);

    int16_t accentY = y - 7;
    if (currentMode == MODE_EGG_SOFT || currentMode == MODE_EGG_HARD) {
      int16_t iX = x + 3 * 6;
      int16_t cX = x + 4 * 6;
      u8g2.setDrawColor(0);
      u8g2.drawPixel(iX + 2, y - 7);
      u8g2.setDrawColor(1);
      u8g2.drawLine(iX + 1, accentY, iX + 3, accentY - 2);
      u8g2.drawLine(cX + 1, accentY - 1, cX + 2, accentY);
      u8g2.drawLine(cX + 4, accentY - 1, cX + 3, accentY);
      if (currentMode == MODE_EGG_SOFT) {
        int16_t aX = x + 12 * 6;
        u8g2.drawPixel(aX + 1, accentY - 1);
        u8g2.drawPixel(aX + 3, accentY - 1);
      }
    } else if (currentMode == MODE_DUMPLING) {
      int16_t iX = x + 5 * 6;
      u8g2.drawLine(iX + 1, accentY, iX + 3, accentY - 2);
    }
  } else {
    u8g2.drawUTF8(x, y, name);
  }
}

void drawScreen() {
  u8g2.firstPage();
  do {
    if (welcomeActive) {
      drawWelcomeScreen();
    } else if (state == STATE_ALARM) {
      drawAlarmScreen();
    } else if (state == STATE_RUNNING || state == STATE_PAUSED) {
      drawRunningScreen();
    } else {
      drawReadyScreen();
    }
  } while (u8g2.nextPage());
}

void drawWelcomeScreen() {
  unsigned long elapsed = millis() - welcomeStartMillis;
  if (elapsed >= 2000UL) {
    welcomeActive = false;
    return;
  }

  if (elapsed < 1000UL) {
    char message[32];
    strcpy_P(message, PSTR("Ahoj, čo dnes uvaríme?"));
    u8g2.setFont(MODE_FONT);
    int16_t width = u8g2.getUTF8Width(message);
    u8g2.drawUTF8((128 - width) / 2, 31, message);
    return;
  }

  bool wink = ((elapsed - 1000UL) / 180UL) % 2 == 0;
  u8g2.drawCircle(64, 32, 20);
  u8g2.drawDisc(56, 27, 2);
  if (wink) {
    u8g2.drawHLine(69, 27, 7);
  } else {
    u8g2.drawDisc(72, 27, 2);
  }
  u8g2.drawLine(54, 40, 58, 43);
  u8g2.drawLine(58, 43, 64, 45);
  u8g2.drawLine(64, 45, 70, 43);
  u8g2.drawLine(70, 43, 74, 40);
}

void drawReadyScreen() {
  // Nazov rezimu (vyssi Unifont font kvoli diakritike - preto y o kusok
  // nizsie ako povodnych 10 px, aby sa cely zmestil od horneho okraja)
  drawModeNameCentered(14, false);

  if (millis() < savedMsgUntil) {
    const char* msg = "ULOZENE!";
    u8g2.setFont(u8g2_font_7x14B_tr);
    int mw = u8g2.getStrWidth(msg);
    u8g2.drawStr((128 - mw) / 2, 40, msg);
    return;
  }

  char buf[6];
  formatTime(remainingSeconds, buf);
  if (currentMode == MODE_NONE) {
    char prompt[12];
    strcpy_P(prompt, PSTR("Nastav čas"));
    u8g2.setFont(MODE_FONT);
    int16_t promptWidth = u8g2.getUTF8Width(prompt);
    u8g2.drawUTF8((128 - promptWidth) / 2, 16, prompt);
  }
  u8g2.setFont(u8g2_font_logisoso32_tn);
  drawTimeCentered(buf, 50);
}

void drawRunningScreen() {
  bool hasName = (currentMode != MODE_NONE);

  if (hasName) {
    drawModeNameCentered(10, true);
  }

  // Poloha digitalneho casu: font zvacseny z logisoso24 na logisoso26
  // (dalsi dostupny krok bitmapoveho fontu smerom hore). Y pozicie
  // prepocitane tak, aby v ZIADNOM-REZIME zostal vrch cislic 1px od
  // vrchneho okraja a spodok cislic 3px od zaciatku viecka presypacich
  // hodin (ktore zacina na animTopY + 2).
  int16_t timeY = hasName ? 36 : 33;

  char buf[6];
  formatTime(remainingSeconds, buf);
  u8g2.setFont(hasName ? u8g2_font_logisoso24_tn : u8g2_font_logisoso32_tn);
  drawTimeCentered(buf, timeY);

  if (state == STATE_PAUSED) {
    if ((millis() / 400) % 2 == 0) {
      u8g2.setFont(u8g2_font_5x7_tf);
      const char* pauseMsg = "PAUZA";
      int pw = u8g2.getStrWidth(pauseMsg);
      u8g2.drawStr(0, 63, pauseMsg);
      u8g2.drawStr(128 - pw, 63, pauseMsg);
    }
  }

  int16_t animCx = 64;
  // Zaciatok animacie hned pod casom (resp. pod PAUZA napisom) - ak nie je
  // nazov, je vyssie polozeny cas uvolni viac priestoru pre animaciu.
  // POZOR: 28 (nie povodnych 26) - kvoli vacsiemu fontu casu (logisoso26)
  // sa cislice o dalsie 2px "natiahli" nizsie, tak sa hodiny posunuli o
  // 2px nizsie, aby medzi nimi a casom ostala presne 3px medzera.
  int16_t animTopY = hasName ? 40 : 38;

  switch (currentMode) {
    case MODE_NONE:
      // bottomY = 63 (spodok displeja) - vysledna vyska hodin ostava
      // rovnaka (~33px, tj. tu istu ~15% redukciu) ako v predchadzajucich
      // verziach, len sa cele o kusok posunuli nizsie kvoli vacsiemu
      // fontu casu, a teraz vychadza presne na spodny okraj displeja.
      drawHourglassAnimation(animCx, 42, 61);
      drawSteamPotAnimation(38, 61, 0, false);
      drawSteamPotAnimation(90, 61, 9, true);
      break;
    case MODE_EGG_SOFT:
    case MODE_EGG_HARD:
      drawEggPotAnimation(animCx, 63);
      break;
    case MODE_DUMPLING:
      drawSteamAnimation(animCx, 63, false);
      break;
    case MODE_PIZZA:
      drawSteamAnimation(animCx, 63, true);
      break;
  }
}

void drawAlarmScreen() {
  // Nazov rezimu (vyssi Unifont font kvoli diakritike)
  drawModeNameCentered(14, false);

  if ((millis() / 300) % 2 == 0) {
    u8g2.setFont(currentMode == MODE_NONE ? u8g2_font_logisoso32_tn : u8g2_font_logisoso24_tn);
    const char* msg = "00:00";
    int tw = u8g2.getStrWidth(msg);
    // Posunute o kusok nizsie (bolo 40), aby sa nedotykalo vyssieho
    // Unifont nazvu rezimu navrchu.
    u8g2.drawStr((128 - tw) / 2, 42, msg);
  }

  u8g2.setFont(u8g2_font_7x14B_tr);
  const char* done = "HOTOVO!";
  int dw = u8g2.getStrWidth(done);
  u8g2.drawStr((128 - dw) / 2, 60, done);
}

// =========================================================
//  ANIMACIE
// =========================================================

// Nizky siroky hrniec s vodou, 3 vajickami a uskami (drzadlami) po
// stranach - uska su umiestnene mimo tela hrnca, aby sa neprekryvali
// so samotnymi vajickami. Z hladiny vody stupa para (vyparovanie).
void drawEggPotAnimation(int16_t cx, int16_t baseY) {
  const int16_t potHalfW = 30;
  const int16_t potH     = 12;
  const int16_t potTopY  = baseY - potH;

  // telo hrnca
  u8g2.drawFrame(cx - potHalfW, potTopY, potHalfW * 2, potH);
  u8g2.drawHLine(cx - potHalfW, baseY, potHalfW * 2 + 1);

  // uska (drzadla) - mimo tela hrnca, po stranach, aby nezasahovali do vajicok
  u8g2.drawCircle(cx - potHalfW - 3, potTopY + potH / 2, 3, U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_LOWER_LEFT);
  u8g2.drawCircle(cx + potHalfW + 3, potTopY + potH / 2, 3, U8G2_DRAW_UPPER_RIGHT | U8G2_DRAW_LOWER_RIGHT);

  // hladina vody - jemne "vlnenie" tesne pod hornym okrajom hrnca
  int16_t waterY = potTopY + 3;
  for (int16_t x = cx - potHalfW + 2; x <= cx + potHalfW - 2; x += 3) {
    int8_t wave = (((x / 3) + animFrame) % 2 == 0) ? 0 : 1;
    u8g2.drawPixel(x, waterY + wave);
  }

  // 3 vajicka vedla seba, nad hladinou vody, rozostupene aby sa neprekryvali
  // ani s uskami, ani s okrajom hrnca
  const int16_t eggW = 5, eggH = 6;
  const int16_t eggCy = waterY - 1;
  const int16_t spacing = 15;
  for (int8_t i = -1; i <= 1; i++) {
    int16_t eggCx = cx + i * spacing;
    u8g2.drawEllipse(eggCx, eggCy, eggW, eggH, U8G2_DRAW_ALL);
  }

  // vyparovanie - stupajuca para nad hrncom (3 vlniace sa prude)
  for (uint8_t i = 0; i < 3; i++) {
    int16_t sx = cx - 14 + i * 14;
    uint8_t localFrame = (animFrame * 2 + i * 6) % 20;
    int16_t sy = potTopY - 3 - localFrame;
    if (sy < potTopY - 20) continue;
    int8_t wig = ((localFrame / 3) % 2 == 0) ? -2 : 2;
    u8g2.drawLine(sx, sy, sx + wig, sy - 4);
  }
}

void drawSteamPotAnimation(int16_t cx, int16_t baseY, uint8_t phaseOffset, bool mirror) {
  const int16_t potTopY = baseY - 18;
  const int16_t halfW = 8;
  uint8_t phase = (animFrame + phaseOffset) % 24;
  int16_t lidLift = (phase < 5) ? (int16_t)(5 - phase) / 2 : 0;
  int16_t lidY = potTopY - 1;
  int16_t outerX = mirror ? cx + halfW + 1 : cx - halfW - 1;
  int16_t innerX = mirror ? cx - halfW : cx + halfW;
  int16_t handleY = potTopY + 5;

  u8g2.drawFrame(cx - halfW, potTopY + 2, halfW * 2 + 1, 16);
  u8g2.drawHLine(cx - halfW - 1, baseY, halfW * 2 + 3);
  u8g2.drawHLine(cx - halfW - 3, handleY, 3);
  u8g2.drawHLine(cx + halfW + 1, handleY, 3);
  u8g2.drawCircle(cx - halfW - 4, handleY, 2, U8G2_DRAW_ALL);
  u8g2.drawCircle(cx + halfW + 4, handleY, 2, U8G2_DRAW_ALL);

  int16_t waterY = potTopY + 4;
  for (int16_t x = cx - halfW + 1; x <= cx + halfW - 1; x += 3) {
    int8_t wave = (((x / 3) + animFrame + phaseOffset) % 2 == 0) ? 0 : 1;
    u8g2.drawPixel(x, waterY + wave);
  }
  u8g2.drawCircle(cx - 4, waterY + 5, 1, U8G2_DRAW_ALL);
  u8g2.drawCircle(cx + 3, waterY + 8, 1, U8G2_DRAW_ALL);
  u8g2.drawPixel(cx + 1, waterY + 11);

  u8g2.drawLine(innerX, lidY, outerX, lidY - lidLift - 2);
  u8g2.drawLine(innerX + (mirror ? -1 : 1), lidY + 1,
                outerX + (mirror ? -1 : 1), lidY - lidLift - 1);
  int16_t knobX = (innerX + outerX) / 2;
  int16_t knobY = (lidY + lidY - lidLift - 2) / 2 - 2;
  u8g2.drawLine(knobX - 2, knobY + 2, knobX + 2, knobY + 2);
  u8g2.drawCircle(knobX, knobY, 2, U8G2_DRAW_ALL);

  uint8_t steamPhase = phase % 18;
  int16_t sx = outerX;
  int16_t sy = lidY - lidLift - 3 - steamPhase;
  int8_t wig = ((steamPhase / 3) % 2 == 0) ? -1 : 1;
  if (mirror) wig = -wig;
  u8g2.drawLine(sx, sy, sx + wig, sy - 4);
  u8g2.drawLine(sx + wig, sy - 4, sx, sy - 8);
}

// Presypacie hodiny so zaobleny obrysom (nie rovne trojuholnikove steny
// ako predtym) - napodobnuju klasicku ikonu presypacich hodin: hore a dole
// rovne "viecko", banky su plne/zaoblene a zuzuju sa az tesne pri hrdle.
// Mnozstvo piesku realne zodpoveda zostavajucemu / uplynutemu casu (horna
// banka = zostava, dolna banka = uplynulo) - to zostalo rovnake ako predtym,
// zmenil sa iba tvar (aj obrysu, aj samotneho piesku, aby kopiroval steny).
void drawHourglassAnimation(int16_t cx, int16_t topY, int16_t bottomY) {
  // mala rezerva zhora - "viecko" je teraz hrubsie (2px) a bez tejto
  // rezervy by mohlo zasahovat do casu (cislic) zobrazenych nad hodinami,
  // presne ako sa stalo predtym (navyse pixel tesne pod cislicou "3")
  topY += 2;

  const int16_t halfW = 7;           // o 25% mensie v pomere,
                                      // na ziadost pouzivatela zmensit
                                      // presypacie hodiny v pomere k
                                      // zvacsenemu digitalnemu casu
  const int16_t capW  = halfW;       // viecko ma ROVNAKU sirku ako stena banky
                                      // v mieste, kde na ňu nadväzuje (y = topY /
                                      // y = bottomY, teda t = 1 vo vzorci nizsie,
                                      // co dava hw = halfW) - predtym tu bolo
                                      // halfW + 3, cim viecko trcalo 3px na kazdu
                                      // stranu navyse a vznikal viditelny zarez
                                      // presne v rohoch, kde sa viecko stretava
                                      // so stenou (vid fotka)
  const int16_t neckY = (topY + bottomY) / 2;
  const int16_t topBulbH    = neckY - topY;
  const int16_t bottomBulbH = bottomY - neckY;

  // horne a dolne rovne "viecko" - teraz hrubsie (2px miesto 1px). Horne
  // rastie SMEROM DOLE (topY, topY+1) a dolne SMEROM HORE (bottomY-1,
  // bottomY) - teda oboje "dnu" do tela hodin, nie von. Vdaka tomu sa
  // celkovy obrys hodin nezvacsi navonok/nahor a nemoze zasahovat do casu
  // zobrazeneho nad hodinami.
  u8g2.drawHLine(cx - capW, topY,         capW * 2 + 1);
  u8g2.drawHLine(cx - capW, topY + 1,     capW * 2 + 1);
  u8g2.drawHLine(cx - capW, bottomY - 1,  capW * 2 + 1);
  u8g2.drawHLine(cx - capW, bottomY,      capW * 2 + 1);

  // Zaobleny obrys oboch baniek - polsirka sa k hrdlu zuzuje podla
  // odmocninovej krivky (sqrt), vdaka comu je banka pri vieczku pekne
  // plna/zaoblena a zuzi sa az prudko tesne pri hrdle - presne ako na
  // referencnej ikone presypacich hodin.
  for (int16_t y = topY; y <= neckY; y++) {
    float t = (topBulbH > 0) ? (float)(neckY - y) / (float)topBulbH : 0.0;
    int16_t hw = (int16_t)(halfW * sqrtf(t));
    u8g2.drawPixel(cx - hw, y);
    u8g2.drawPixel(cx + hw, y);
  }
  for (int16_t y = neckY; y <= bottomY; y++) {
    float t = (bottomBulbH > 0) ? (float)(y - neckY) / (float)bottomBulbH : 0.0;
    int16_t hw = (int16_t)(halfW * sqrtf(t));
    u8g2.drawPixel(cx - hw, y);
    u8g2.drawPixel(cx + hw, y);
  }

  float fractionRemaining = 0.0;
  if (totalSecondsAtStart > 0) {
    fractionRemaining = (float)remainingSeconds / (float)totalSecondsAtStart;
  }
  if (fractionRemaining < 0.0) fractionRemaining = 0.0;
  if (fractionRemaining > 1.0) fractionRemaining = 1.0;
  float fractionElapsed = 1.0 - fractionRemaining;

  // medzera medzi sklom (obrysom) a pieskom - piesok je v kazdom riadku
  // o SAND_GAP pixelov uzsi nez stena banky. POZOR: 1px medzera je na
  // malom OLED displeji (128x64, navyse cez fotoaparat) prakticky
  // neviditelna - jasne susedne pixely na OLED opticky "prekvitaju"
  // (bloom) do seba, takze 1 tmavy pixel medzi nimi splynie. Preto 2px.
  // TOTO PLATI ROVNAKO AJ ZVISLE (nie len vodorovne pri bokoch) - preto
  // piesok pri hornom aj dolnom viecku musi nechat volne CELE 2 riadky
  // (nie iba 1), inak sa vizualne "zlepi" s viečkom a medzera nie je
  // vidno vobec (presne to sa stalo, ked bol rezervovany len 1 riadok).
  const int16_t SAND_GAP = 2;

  // piesok v hornej banke - jeho vyska (od hrdla nahor) klesa s casom,
  // sirka v kazdom riadku kopiruje zaobleny obrys banky (znizenu o
  // medzeru). POZOR: vyska sa pocita zaokruhlenim NAHOR (ceil), nie
  // orezanim nadol - inak by pri malej vyske banky (len niekolko pixelov)
  // posledny pixel piesku zmizol predcasne, este ked zostavalo nieco cez
  // 10 sekund, a hodiny by tak pocas poslednych sekund uz vyzerali
  // "prazdne" hoci cas este nie je 0. Vdaka ceil() zostane v hornej
  // banke aspon 1px piesku, kym skutocne nezostava 0 sekund.
  // maxSandTopH je znizena o SAND_GAP, aby piesok nikdy nezasiahol do
  // poslednych SAND_GAP riadkov pri hornom viecku (topY..topY+SAND_GAP-1).
  const int16_t maxSandTopH = (topBulbH > SAND_GAP) ? (topBulbH - SAND_GAP) : 0;
  int16_t sandTopH = 0;
  if (fractionRemaining > 0.0) {
    sandTopH = (int16_t)ceilf(maxSandTopH * fractionRemaining);
    if (sandTopH > maxSandTopH) sandTopH = maxSandTopH;
  }
  for (int16_t y = neckY; y > neckY - sandTopH; y--) {
    float t = (topBulbH > 0) ? (float)(neckY - y) / (float)topBulbH : 0.0;
    int16_t hw = (int16_t)(halfW * sqrtf(t)) - SAND_GAP;
    if (hw > 0) u8g2.drawHLine(cx - hw, y, hw * 2 + 1);
  }

  // piesok v dolnej banke - pribuda odspodu priamo umerne uplynutemu casu,
  // opat podla rovnakej zaoblenej krivky banky (znizenej o medzeru od skla).
  //
  // POZOR - dolezity rozdiel oproti hornemu pieskocu vyssie: horny piesok
  // ma kotvu (neckY) DALEKO od viecka, takze uz len znizenie maxSandTopH o
  // SAND_GAP samo od seba necha prazdny riadok pri vieccku. Dolny piesok
  // ma ale kotvu priamo PRI vieccku (bottomY) - ak by sme kotvu posunuli
  // len o SAND_GAP, piesok by skoncil hned VEDLA viecka (ktore je hrube
  // 2 riadky: bottomY-1 a bottomY) bez akehokolvek prazdneho riadku medzi
  // nimi. Preto sa kotva posuva o CAP_THICKNESS (hrubka viecka) + SAND_GAP
  // - 1, cim vznikne rovnaky 1-riadkovy prazdny "vzduch" medzi pieskom a
  // vieckom, aky prirodzene vznika aj hore.
  const int16_t CAP_THICKNESS = 2;  // viecko = 2 riadky (viz drawHLine nizsie)
  const int16_t bottomAnchorOffset = SAND_GAP + CAP_THICKNESS - 1;
  const int16_t maxSandBottomH = (bottomBulbH > bottomAnchorOffset) ? (bottomBulbH - bottomAnchorOffset) : 0;
  int16_t sandBottomH = (int16_t)(maxSandBottomH * fractionElapsed);
  for (int16_t y = bottomY - bottomAnchorOffset; y > bottomY - bottomAnchorOffset - sandBottomH; y--) {
    float t = (bottomBulbH > 0) ? (float)(y - neckY) / (float)bottomBulbH : 0.0;
    int16_t hw = (int16_t)(halfW * sqrtf(t)) - SAND_GAP;
    if (hw > 0) u8g2.drawHLine(cx - hw, y, hw * 2 + 1);
  }

  // padajuce zrnka piesku v hrdle, kym este nieco zostava - 2 zrnka za
  // sebou pre plynulejsi dojem prudu piesku, kazde teraz 2px hrubke
  // (vodorovna usecka dlzky 2, nie jeden osamely pixel ako predtym)
  if (fractionRemaining > 0.01 && fractionRemaining < 0.99) {
    uint8_t fallPhase = animFrame % 4;
    u8g2.drawHLine(cx - 1, neckY - 3 + fallPhase, 2);
    u8g2.drawHLine(cx - 1, neckY - 1 + fallPhase, 2);
  }
}

// Knedlik / pizza so stupajucou parou (loop)
void drawSteamAnimation(int16_t cx, int16_t baseY, bool isPizza) {
  if (isPizza) {
    // -------------------------------------------------------
    // PIZZA - styl jednoduchého plátku ako na referencii
    // -------------------------------------------------------
    // Špička smeruje doprava. Telo je vyplnené bielou a
    // ingrediencie sú čierne, takže na OLED vznikne výrazná
    // ikonka aj pri malom rozlíšení.
    const int16_t backX = cx - 22;
    const int16_t topY  = baseY - 27;
    const int16_t topBotY = baseY - 3;
    const int16_t botY  = baseY;
    const int16_t tipX  = cx + 22;
    const int16_t tipY  = baseY - 12;
    const int16_t crustMidY = topY + 13;

    // Bočná vrstva pod spodnou hranou dáva plátku tretí rozmer.
    for (int16_t x = backX + 6; x <= tipX; x++) {
      int16_t edgeY = topBotY - (int16_t)((float)(x - (backX + 6)) * (topBotY - tipY) / (float)(tipX - backX - 6));
      u8g2.drawVLine(x, edgeY, 4);
    }
    u8g2.drawLine(backX + 6, topBotY + 3, tipX, tipY + 3);

    // Zaobleny biely zadny okrajok, podobny hrubej kor ke na referencii.
    u8g2.drawFilledEllipse(backX + 6, crustMidY, 6, 13, U8G2_DRAW_ALL);

    // Vyplnenie hornej plochy trojuholnika scanline metodou. drawTriangle()
    // samotné telo iba obkreslí, preto ho tu vypĺňame riadok po riadku.
    for (int16_t y = topY; y <= topBotY; y++) {
      int16_t distanceFromMiddle = abs(y - crustMidY);
      int16_t leftX = backX + (distanceFromMiddle * 6) / 13;
      int16_t rightX;
      // Horná hrana: backX,topY -> tipX,tipY
      // Dolná hrana: backX,botY -> tipX,tipY
      if (y <= tipY) {
        rightX = backX + 6 + (int16_t)((float)(y - topY) * (tipX - backX - 6) / (float)(tipY - topY));
      } else {
        rightX = backX + 6 + (int16_t)((float)(topBotY - y) * (tipX - backX - 6) / (float)(topBotY - tipY));
      }
      u8g2.drawHLine(leftX, y, rightX - leftX + 1);
    }

    // Obrys hornej a spodnej hrany plátku.
    u8g2.drawLine(backX + 6, topY, tipX, tipY);
    u8g2.drawLine(backX + 6, topBotY, tipX, tipY);

    // Cierna deliaca ciara oddeluje hornu plochu od 3D bočnej vrstvy.
    u8g2.setDrawColor(0);
    for (int16_t y = topY; y <= topBotY; y++) {
      int16_t distanceFromMiddle = abs(y - crustMidY);
      int16_t outerX = backX + (distanceFromMiddle * 6) / 13;
      u8g2.drawHLine(outerX + 5, y, 2);
    }
    u8g2.drawLine(backX + 6, topBotY, tipX, tipY);
    u8g2.setDrawColor(1);

    const int8_t dx[6] = { -12, -3, 7, -8, 3, 12 };
    const int8_t dy[6] = { -17, -18, -15, -9, -9, -11 };
    const uint8_t radius[6] = { 2, 2, 2, 2, 2, 2 };

    for (uint8_t i = 0; i < 6; i++) {
      int16_t tx = cx + dx[i];
      int16_t ty = baseY + dy[i];
      u8g2.setDrawColor(0);
      u8g2.drawDisc(tx, ty, radius[i], U8G2_DRAW_ALL);
      u8g2.setDrawColor(1);
    }

    // Malý čierny detail pri špičke, podobný otvoru/okraju na referenčnej ikone.
    u8g2.setDrawColor(0);
    u8g2.drawPixel(tipX - 5, tipY);
    u8g2.setDrawColor(1);

    // Para ostáva ako samostatná animácia nad pizzou.
  } else {
    u8g2.drawFilledEllipse(cx - 10, baseY - 6, 9, 6, U8G2_DRAW_ALL);
    u8g2.drawFilledEllipse(cx + 10, baseY - 6, 9, 6, U8G2_DRAW_ALL);
    u8g2.drawHLine(cx - 22, baseY, 44);
  }

  const int16_t steamTopY = baseY - 35;
  const int16_t steamBottomY = isPizza ? baseY - 28 : baseY - 20;

  for (uint8_t i = 0; i < 3; i++) {
    int16_t sx = cx - 10 + i * 10;
    uint8_t localFrame = (animFrame * 2 + i * 6) % 30;
    int16_t sy = steamBottomY - localFrame;
    if (sy < steamTopY) continue;

    int8_t wig = ((localFrame / 3) % 2 == 0) ? -2 : 2;
    u8g2.drawLine(sx, sy, sx + wig, sy - 5);
    u8g2.drawLine(sx + wig, sy - 5, sx, sy - 10);
  }
}
