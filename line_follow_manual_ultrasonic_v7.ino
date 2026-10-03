/*
 * ESP32 4WD Line Follower + Manual Control + Ultrasonic Obstacle Stop (v4)
 * -------------------------------------------------------------------------
 * Motor driver: L298N board with INA / INB / INC / IND inputs (NO enable pins)
 *   INA + INB  -> Left  side motors  (OUT1/OUT2)
 *   INC + IND  -> Right side motors  (OUT3/OUT4)
 * Sensor: 8 channel IR array (digital out)
 * NEW in v4: HC-SR04 ultrasonic obstacle detection + buzzer alert
 *
 * WiFi AP:  LineBot_ESP32 / 12345678   ->  http://192.168.4.1
 * Requires ESP32 Arduino core 3.x
 */

#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <MFRC522.h>

/* ============== AURA STATE MACHINE (enum defined early to avoid
   Arduino's auto-prototype-generation ordering issues) ============== */
enum RobotState {
  STATE_STANDBY,
  STATE_GOING_TO_X,
  STATE_AT_X,
  STATE_GOING_TO_Y,
  STATE_AT_Y,
  STATE_RETURNING_TO_BASE
};

/* ================= WIFI ================= */
const char* AP_SSID = "LineBot_ESP32";
const char* AP_PASS = "12345678";
WebServer server(80);

/* ============== IR SENSOR PINS (Left -> Right) ============== */
const int irPin[8] = {13, 15, 14, 27, 26, 25, 33, 32};
int irVal[8];

/* Black line on white floor -> true ; White line on black floor -> false */
bool BLACK_LINE = true;

/* ============== MOTOR DRIVER PINS ==============
   PWM applied directly on input pins (no ENA/ENB on this board) */
#define INA 22   // Left  forward
#define INB 23   // Left  backward
#define INC 18   // Right forward
#define IND 19   // Right backward

#define PWM_FREQ 5000
#define PWM_RES  8      // 0-255

/* If one side spins the wrong way, flip its flag to true */
bool INVERT_LEFT  = false;
bool INVERT_RIGHT = false;

/* ============== RFID (RC522) PINS + UIDs ============== */
#define RFID_SCK  21
#define RFID_MOSI 12
#define RFID_MISO 34
#define RFID_SDA  2     // apnar existing wiring onujayi (GPIO2 e ageo SDA lagano chilo)
#define RFID_RST  UINT8_MAX   // RST tied directly to 3.3V, no GPIO used

MFRC522 rfid(RFID_SDA, RFID_RST);

// Confirmed UIDs from scanning (byte length = 4 for these tags)
byte UID_J1[] = {0xBC, 0x74, 0xC1, 0x01};   // Key ring -> Junction J1
byte UID_X[]  = {0xAC, 0x71, 0x8F, 0x02};   // Card 1   -> Sender X
byte UID_Y[]  = {0x09, 0xB2, 0x30, 0x03};   // Card 2   -> Receiver Y

String lastRfidLocation = "NONE";
unsigned long lastRfidTime = 0;
const unsigned long RFID_STOP_MS = 3000;   // test-mode only: stay stopped this long after X/Y scan
bool rfidStopActive = false;               // test-mode only flag (when autoModeActive is false)

// All known office locations for the App dropdown. Only "X" and "Y" have a
// real RFID tag + working route right now; the rest are placeholders for
// when the physical track is expanded with more junctions/tags.
const char* ALL_LOCATIONS[] = {"A","B","C","D","E","F","X","Y","Z"};
const int   NUM_LOCATIONS = 9;
String currentSenderName   = "X";   // which location is currently the sender
String currentReceiverName = "Y";   // which location is currently the receiver

bool uidMatches(byte *scanned, byte size, byte *known, byte knownSize) {
  if (size != knownSize) return false;
  for (byte i = 0; i < size; i++) if (scanned[i] != known[i]) return false;
  return true;
}

/* ============== AURA STATE MACHINE ============== */
RobotState robotState = STATE_STANDBY;
bool autoModeActive = false;   // true after App "Call Robot", until robot is back at base

// Junction (J1) turn settings - route map:
//   Base -> J1(Left) -> X -> [180 turn] -> J1(Left) -> Y -> [180 turn] -> J1(Straight) -> Base
char J1_TURN_TO_X = 'L';
char J1_TURN_TO_Y = 'L';
char J1_TURN_RETURNING = 'S';   // straight through, no turn needed

bool isTurningAtJunction = false;
unsigned long turnStartTime = 0;
const unsigned long JUNCTION_TURN_MS = 500;   // 90-degree-ish pivot duration, tune on real track
char currentTurnDir = 'S';

// 180-degree turn (after Item Loaded at X, and after Received at Y)
bool isDoingUTurn = false;
unsigned long uTurnStartTime = 0;
const unsigned long UTURN_MS = 1000;   // ~180 degree pivot duration, tune on real track
RobotState stateAfterUTurn = STATE_STANDBY;

void startJunctionTurn(char dir) {
  isTurningAtJunction = true;
  turnStartTime = millis();
  currentTurnDir = dir;
  Serial.print("[J1] Turning: "); Serial.println(dir);
}

void startUTurn(RobotState nextState) {
  isDoingUTurn = true;
  uTurnStartTime = millis();
  stateAfterUTurn = nextState;
  Serial.println("[UTURN] Starting 180-degree turn.");
}

// Simple "wide black stop marker" detect for base: all 8 IR sensors seeing black at once
unsigned long baseMarkerStart = 0;
bool baseMarkerTiming = false;
const unsigned long BASE_MARKER_MS = 150;   // must hold for this long to count as base marker

bool checkBaseMarker() {
  bool allBlack = true;
  for (int i = 0; i < 8; i++) if (irVal[i] == 0) { allBlack = false; break; }

  if (allBlack) {
    if (!baseMarkerTiming) { baseMarkerTiming = true; baseMarkerStart = millis(); }
    if (millis() - baseMarkerStart >= BASE_MARKER_MS) return true;
  } else {
    baseMarkerTiming = false;
  }
  return false;
}

/* ============== ULTRASONIC + BUZZER PINS ============== */
#define TRIG_PIN 4      // <-- age chilo 2, RFID SDA GPIO2 e thakay ekhane sorano hoyeche
#define ECHO_PIN 35     // age chilo 4, RFID SDA er jonno free kora hoyeche
#define BUZZER_PIN 5

const float OBSTACLE_DISTANCE_CM = 15.0;   // fixed stop distance
const unsigned long US_READ_INTERVAL = 60; // ms between ultrasonic reads (non-blocking)
const unsigned long BEEP_REPEAT_MS   = 10000; // re-beep every 10 sec if obstacle persists

float distanceCM = 999;
bool  obstacleDetected = false;

unsigned long lastUSRead   = 0;
unsigned long lastBeepTime = 0;   // when the beep-beep pattern last started

/* Non-blocking beep-beep state machine */
enum BeepState { BEEP_IDLE, BEEP_ON1, BEEP_OFF1, BEEP_ON2, BEEP_OFF2 };
BeepState beepState = BEEP_IDLE;
unsigned long beepTimer = 0;
const unsigned long BEEP_ON_MS  = 120;
const unsigned long BEEP_OFF_MS = 120;

/* ============== CONTROL VARIABLES ============== */
String mode = "STOP";          // LINE / MANUAL / STOP
String manualCmd = "S";        // F B L R S
int baseSpeed   = 150;
int manualSpeed = 180;

float Kp = 25.0, Ki = 0.0, Kd = 15.0;
float lastError = 0, integral = 0;
float lastKnownError = 0;

/* ================= MOTOR HELPERS ================= */
void leftMotor(int spd) {                  // -255 .. 255
  spd = constrain(spd, -255, 255);
  if (INVERT_LEFT) spd = -spd;
  if (spd >= 0) { ledcWrite(INA, spd);   ledcWrite(INB, 0); }
  else          { ledcWrite(INA, 0);     ledcWrite(INB, -spd); }
}

void rightMotor(int spd) {
  spd = constrain(spd, -255, 255);
  if (INVERT_RIGHT) spd = -spd;
  if (spd >= 0) { ledcWrite(INC, spd);   ledcWrite(IND, 0); }
  else          { ledcWrite(INC, 0);     ledcWrite(IND, -spd); }
}

void driveStop() { leftMotor(0); rightMotor(0); }

/* ================= RFID ================= */
void checkRFID() {
  if (!rfid.PICC_IsNewCardPresent()) return;
  if (!rfid.PICC_ReadCardSerial())   return;

  if (uidMatches(rfid.uid.uidByte, rfid.uid.size, UID_J1, sizeof(UID_J1))) {
    lastRfidLocation = "J1";
    if (autoModeActive && !isTurningAtJunction) {
      if (robotState == STATE_GOING_TO_X)        startJunctionTurn(J1_TURN_TO_X);
      else if (robotState == STATE_GOING_TO_Y)   startJunctionTurn(J1_TURN_TO_Y);
      else if (robotState == STATE_RETURNING_TO_BASE) startJunctionTurn(J1_TURN_RETURNING);
      else Serial.println("[RFID] J1 detected but not in a driving state, ignoring turn.");
    } else {
      Serial.println("[RFID] J1 (Junction) detected! (test mode, no turn)");
    }
  }
  else if (uidMatches(rfid.uid.uidByte, rfid.uid.size, UID_X, sizeof(UID_X))) {
    lastRfidLocation = "X";
    if (autoModeActive && robotState == STATE_GOING_TO_X) {
      Serial.println("[RFID] X (Sender) reached! Waiting for 'Item Loaded' confirmation.");
      robotState = STATE_AT_X;
    } else if (!autoModeActive) {
      Serial.println("[RFID] X (Sender) detected! (test mode, stopping briefly)");
      rfidStopActive = true;
      lastRfidTime = millis();
    }
  }
  else if (uidMatches(rfid.uid.uidByte, rfid.uid.size, UID_Y, sizeof(UID_Y))) {
    lastRfidLocation = "Y";
    if (autoModeActive && robotState == STATE_GOING_TO_Y) {
      Serial.println("[RFID] Y (Receiver) reached! Waiting for 'Received' confirmation.");
      robotState = STATE_AT_Y;
    } else if (!autoModeActive) {
      Serial.println("[RFID] Y (Receiver) detected! (test mode, stopping briefly)");
      rfidStopActive = true;
      lastRfidTime = millis();
    }
  }
  else {
    Serial.print("[RFID] Unknown tag scanned, UID: ");
    for (byte i = 0; i < rfid.uid.size; i++) {
      Serial.print(rfid.uid.uidByte[i], HEX); Serial.print(" ");
    }
    Serial.println();
  }

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

/* ================= ULTRASONIC ================= */
float readUltrasonicCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 25000UL); // 25ms timeout (~4m range)
  if (duration == 0) return 999;                    // no echo -> treat as "clear"

  float cm = duration * 0.0343 / 2.0;
  return cm;
}

void updateUltrasonic() {
  unsigned long now = millis();
  if (now - lastUSRead < US_READ_INTERVAL) return;
  lastUSRead = now;

  distanceCM = readUltrasonicCM();
  obstacleDetected = (distanceCM > 0 && distanceCM <= OBSTACLE_DISTANCE_CM);
}

/* ================= BUZZER (non-blocking beep-beep) ================= */
void updateBuzzer() {
  unsigned long now = millis();

  if (obstacleDetected) {
    // Start a new beep-beep pattern if idle and either first time or 10s passed
    if (beepState == BEEP_IDLE && (now - lastBeepTime >= BEEP_REPEAT_MS || lastBeepTime == 0)) {
      beepState  = BEEP_ON1;
      beepTimer  = now;
      lastBeepTime = now;
      digitalWrite(BUZZER_PIN, HIGH);
    }
  } else {
    // obstacle gone -> reset pattern & silence buzzer
    beepState  = BEEP_IDLE;
    lastBeepTime = 0;
    digitalWrite(BUZZER_PIN, LOW);
    return;
  }

  // Step through beep-beep state machine
  switch (beepState) {
    case BEEP_ON1:
      if (now - beepTimer >= BEEP_ON_MS) { digitalWrite(BUZZER_PIN, LOW); beepTimer = now; beepState = BEEP_OFF1; }
      break;
    case BEEP_OFF1:
      if (now - beepTimer >= BEEP_OFF_MS) { digitalWrite(BUZZER_PIN, HIGH); beepTimer = now; beepState = BEEP_ON2; }
      break;
    case BEEP_ON2:
      if (now - beepTimer >= BEEP_ON_MS) { digitalWrite(BUZZER_PIN, LOW); beepTimer = now; beepState = BEEP_OFF2; }
      break;
    case BEEP_OFF2:
      if (now - beepTimer >= BEEP_OFF_MS) { beepState = BEEP_IDLE; }  // pattern done, wait for next 10s cycle
      break;
    default: break;
  }
}

/* ================= SENSOR READ ================= */
const float weight[8] = {-7, -5, -3, -1, 1, 3, 5, 7};

bool readSensors(float &error) {
  float sum = 0; int count = 0;
  for (int i = 0; i < 8; i++) {
    int raw = digitalRead(irPin[i]);
    irVal[i] = BLACK_LINE ? (raw == LOW ? 1 : 0) : (raw == HIGH ? 1 : 0);
    if (irVal[i]) { sum += weight[i]; count++; }
  }
  if (count == 0) return false;            // line lost
  error = sum / count;
  return true;
}

/* ================= LINE FOLLOW ================= */
void lineFollow() {
  float error;
  if (!readSensors(error)) {
    int s = baseSpeed * 0.8;               // line hariye gele khuje
    if (lastKnownError < 0) { leftMotor(-s); rightMotor(s); }
    else                    { leftMotor(s);  rightMotor(-s); }
    return;
  }
  lastKnownError = error;

  integral += error;
  integral = constrain(integral, -50, 50);
  float derivative = error - lastError;
  float correction = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  leftMotor(baseSpeed + correction);
  rightMotor(baseSpeed - correction);
}

/* ================= MANUAL DRIVE ================= */
void manualDrive() {
  int s = manualSpeed;
  if      (manualCmd == "F") { leftMotor(s);  rightMotor(s);  }
  else if (manualCmd == "B") { leftMotor(-s); rightMotor(-s); }
  else if (manualCmd == "L") { leftMotor(s);  rightMotor(-s); }
  else if (manualCmd == "R") { leftMotor(-s); rightMotor(s);  }
  else driveStop();
}

/* ================= WEB PAGE ================= */
const char PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>ESP32 LineBot</title>
<style>
 *{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
 body{margin:0;font-family:system-ui,Arial;background:#101418;color:#e8eef2}
 header{padding:14px;text-align:center;background:#161d24;font-weight:600;letter-spacing:1px}
 .tabs{display:flex}
 .tab{flex:1;padding:14px;text-align:center;background:#1b242d;cursor:pointer;font-weight:600}
 .tab.on{background:#0aa06e;color:#fff}
 .panel{display:none;padding:18px}
 .panel.on{display:block}
 button{border:0;border-radius:14px;font-size:17px;font-weight:700;color:#fff;padding:18px;width:100%}
 .go{background:#0aa06e}.stop{background:#d0342c}.dir{background:#2b6cb0;height:78px}
 .pad{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;max-width:340px;margin:auto}
 .sp{margin:18px 0}
 input[type=range]{width:100%}
 .sens{display:flex;gap:6px;justify-content:center;margin-top:16px}
 .dot{width:26px;height:26px;border-radius:6px;background:#2a3540}
 .dot.hit{background:#0aa06e}
 .st{text-align:center;margin-top:12px;font-size:14px;color:#8fa3b0}
 .dist{text-align:center;margin-top:8px;font-size:15px}
 .dist.warn{color:#ff5b5b;font-weight:700}
</style></head><body>
<header>ESP32 4WD LINE BOT</header>
<div class="tabs">
  <div class="tab on" id="t1" onclick="tab(1)">LINE FOLLOW</div>
  <div class="tab" id="t2" onclick="tab(2)">MANUAL</div>
  <div class="tab" id="t3" onclick="tab(3)">DELIVERY</div>
</div>

<div class="panel on" id="p1">
  <button class="go" onclick="send('/mode?m=LINE')">START LINE FOLLOWING</button>
  <div class="sp"></div>
  <button class="stop" onclick="send('/mode?m=STOP')">STOP</button>
  <div class="sp">Base Speed: <b id="bsv">150</b>
    <input type="range" min="60" max="255" value="150" oninput="bsv.innerText=this.value" onchange="send('/set?base='+this.value)">
  </div>
  <div class="sp">Kp: <b id="kpv">25</b>
    <input type="range" min="0" max="80" value="25" oninput="kpv.innerText=this.value" onchange="send('/set?kp='+this.value)">
  </div>
  <div class="sp">Kd: <b id="kdv">15</b>
    <input type="range" min="0" max="80" value="15" oninput="kdv.innerText=this.value" onchange="send('/set?kd='+this.value)">
  </div>
  <div class="sens" id="sens"></div>
  <div class="st" id="st">mode: --</div>
  <div class="dist" id="dist">distance: -- cm</div>
</div>

<div class="panel" id="p2">
  <div class="pad">
    <div></div>
    <button class="dir" onmousedown="mv('F')" ontouchstart="mv('F')" onmouseup="mv('S')" ontouchend="mv('S')">&#9650;</button>
    <div></div>
    <button class="dir" onmousedown="mv('L')" ontouchstart="mv('L')" onmouseup="mv('S')" ontouchend="mv('S')">&#9664;</button>
    <button class="stop" style="height:78px" onclick="mv('S')">&#9632;</button>
    <button class="dir" onmousedown="mv('R')" ontouchstart="mv('R')" onmouseup="mv('S')" ontouchend="mv('S')">&#9654;</button>
    <div></div>
    <button class="dir" onmousedown="mv('B')" ontouchstart="mv('B')" onmouseup="mv('S')" ontouchend="mv('S')">&#9660;</button>
    <div></div>
  </div>
  <div class="sp">Manual Speed: <b id="msv">180</b>
    <input type="range" min="60" max="255" value="180" oninput="msv.innerText=this.value" onchange="send('/set?man='+this.value)">
  </div>
  <div class="dist" id="dist2">distance: -- cm</div>
</div>

<div class="panel" id="p3">
  <div class="sp" style="text-align:center;font-size:16px">Robot state: <b id="rstate">--</b></div>
  <div class="sp">Sender:
    <select id="senderSel" style="width:100%;padding:10px;border-radius:8px;margin-top:6px">
      <option value="A">A</option><option value="B">B</option><option value="C">C</option>
      <option value="D">D</option><option value="E">E</option><option value="F">F</option>
      <option value="X" selected>X</option><option value="Y">Y</option><option value="Z">Z</option>
    </select>
  </div>
  <div class="sp">Receiver:
    <select id="receiverSel" style="width:100%;padding:10px;border-radius:8px;margin-top:6px">
      <option value="A">A</option><option value="B">B</option><option value="C">C</option>
      <option value="D">D</option><option value="E">E</option><option value="F">F</option>
      <option value="X">X</option><option value="Y" selected>Y</option><option value="Z">Z</option>
    </select>
  </div>
  <div class="sp"></div>
  <button class="go" onclick="callRobot()">CALL ROBOT</button>
  <div class="sp"></div>
  <button class="go" id="btnLoaded" onclick="send('/itemLoaded')" disabled>ITEM LOADED (at X)</button>
  <div class="sp"></div>
  <button class="go" id="btnReceived" onclick="send('/received')" disabled>RECEIVED (at Y)</button>
  <div class="sp"></div>
  <button class="stop" onclick="send('/cancelAuto')">CANCEL / RESET TO STANDBY</button>
  <div class="dist" id="dist3">distance: -- cm</div>
</div>

<script>
let box=document.getElementById('sens');
for(let i=0;i<8;i++){let d=document.createElement('div');d.className='dot';d.id='d'+i;box.appendChild(d);}
function tab(n){
  t1.className='tab'+(n==1?' on':'');t2.className='tab'+(n==2?' on':'');t3.className='tab'+(n==3?' on':'');
  p1.className='panel'+(n==1?' on':'');p2.className='panel'+(n==2?' on':'');p3.className='panel'+(n==3?' on':'');
  if (n!=3) send('/mode?m='+(n==1?'STOP':'MANUAL'));
}
function send(u){fetch(u);}
function callRobot(){
  const s = document.getElementById('senderSel').value;
  const r = document.getElementById('receiverSel').value;
  if (s === r) { alert('Sender ebong Receiver ekই hote pare na!'); return; }
  fetch('/call?sender='+s+'&receiver='+r).then(resp=>{
    if (!resp.ok) alert('Eyi route ekhono implement kora hoyni! Shudhu X -> Y ekhon kaj kore.');
  });
}
function mv(c){fetch('/manual?c='+c);}
document.addEventListener('keydown',e=>{const k={ArrowUp:'F',ArrowDown:'B',ArrowLeft:'L',ArrowRight:'R'}[e.key];if(k)mv(k);});
document.addEventListener('keyup',e=>{if(e.key.startsWith('Arrow'))mv('S');});
setInterval(()=>{fetch('/status').then(r=>r.json()).then(j=>{
  for(let i=0;i<8;i++)document.getElementById('d'+i).className='dot'+(j.s[i]?' hit':'');
  st.innerText='mode: '+j.mode;
  let txt = 'distance: '+j.dist.toFixed(1)+' cm' + (j.obs ? '  OBSTACLE!' : '');
  dist.innerText = txt; dist.className = 'dist' + (j.obs?' warn':'');
  dist2.innerText = txt; dist2.className = 'dist' + (j.obs?' warn':'');
  dist3.innerText = txt; dist3.className = 'dist' + (j.obs?' warn':'');
  rstate.innerText = j.robotState + ' ('+j.sender+' -> '+j.receiver+') ' + (j.auto ? 'AUTO' : 'idle');
  btnLoaded.disabled = (j.robotState !== 'AT_X');
  btnReceived.disabled = (j.robotState !== 'AT_Y');
});},250);
</script></body></html>
)HTML";

/* ================= HANDLERS ================= */
void handleRoot() { server.send_P(200, "text/html", PAGE); }

void handleMode() {
  mode = server.arg("m");
  integral = 0; lastError = 0;
  if (mode != "LINE" && mode != "MANUAL") { mode = "STOP"; driveStop(); }
  server.send(200, "text/plain", "ok");
}

void handleManual() {
  manualCmd = server.arg("c");
  mode = "MANUAL";
  server.send(200, "text/plain", "ok");
}

void handleSet() {
  if (server.hasArg("base")) baseSpeed   = server.arg("base").toInt();
  if (server.hasArg("man"))  manualSpeed = server.arg("man").toInt();
  if (server.hasArg("kp"))   Kp = server.arg("kp").toFloat();
  if (server.hasArg("kd"))   Kd = server.arg("kd").toFloat();
  server.send(200, "text/plain", "ok");
}

void handleStatus() {
  String stateStr;
  switch (robotState) {
    case STATE_STANDBY: stateStr = "STANDBY"; break;
    case STATE_GOING_TO_X: stateStr = "GOING_TO_X"; break;
    case STATE_AT_X: stateStr = "AT_X"; break;
    case STATE_GOING_TO_Y: stateStr = "GOING_TO_Y"; break;
    case STATE_AT_Y: stateStr = "AT_Y"; break;
    case STATE_RETURNING_TO_BASE: stateStr = "RETURNING_TO_BASE"; break;
  }
  String j = "{\"mode\":\"" + mode + "\",\"s\":[";
  for (int i = 0; i < 8; i++) { j += String(irVal[i]); if (i < 7) j += ","; }
  j += "],\"dist\":" + String(distanceCM, 1) + ",\"obs\":" + String(obstacleDetected ? "true" : "false");
  j += ",\"rfid\":\"" + lastRfidLocation + "\",\"rfidStop\":" + String(rfidStopActive ? "true" : "false");
  j += ",\"robotState\":\"" + stateStr + "\",\"auto\":" + String(autoModeActive ? "true" : "false");
  j += ",\"sender\":\"" + currentSenderName + "\",\"receiver\":\"" + currentReceiverName + "\"}";
  server.send(200, "application/json", j);
}

void handleCall() {
  String sender   = server.hasArg("sender")   ? server.arg("sender")   : "X";
  String receiver = server.hasArg("receiver") ? server.arg("receiver") : "Y";

  // Only the X -> Y route is physically implemented right now
  if (!(sender == "X" && receiver == "Y")) {
    Serial.print("[APP] Route not implemented yet: "); Serial.print(sender);
    Serial.print(" -> "); Serial.println(receiver);
    server.send(400, "text/plain", "route not implemented yet");
    return;
  }

  currentSenderName = sender;
  currentReceiverName = receiver;
  robotState = STATE_GOING_TO_X;
  autoModeActive = true;
  integral = 0; lastError = 0;
  isTurningAtJunction = false;
  isDoingUTurn = false;
  Serial.print("[APP] Call received: "); Serial.print(sender);
  Serial.print(" -> "); Serial.println(receiver);
  server.send(200, "text/plain", "ok");
}

void handleItemLoaded() {
  if (robotState == STATE_AT_X) {
    startUTurn(STATE_GOING_TO_Y);
    Serial.println("[APP] Item Loaded confirmed. Doing 180-turn, then heading to Y.");
    server.send(200, "text/plain", "ok");
  } else {
    server.send(400, "text/plain", "not at X");
  }
}

void handleReceived() {
  if (robotState == STATE_AT_Y) {
    startUTurn(STATE_RETURNING_TO_BASE);
    Serial.println("[APP] Received confirmed. Doing 180-turn, then returning to base.");
    server.send(200, "text/plain", "ok");
  } else {
    server.send(400, "text/plain", "not at Y");
  }
}

void handleCancelAuto() {
  autoModeActive = false;
  robotState = STATE_STANDBY;
  isTurningAtJunction = false;
  isDoingUTurn = false;
  driveStop();
  Serial.println("[APP] Auto/delivery mode cancelled.");
  server.send(200, "text/plain", "ok");
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 8; i++) pinMode(irPin[i], INPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // ledcAttach handles pinMode internally
  ledcAttach(INA, PWM_FREQ, PWM_RES);
  ledcAttach(INB, PWM_FREQ, PWM_RES);
  ledcAttach(INC, PWM_FREQ, PWM_RES);
  ledcAttach(IND, PWM_FREQ, PWM_RES);
  driveStop();

  SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_SDA);
  rfid.PCD_Init();
  rfid.PCD_SetAntennaGain(MFRC522::RxGain_max);
  Serial.println("RFID module initialized.");

  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());   // 192.168.4.1

  server.on("/", handleRoot);
  server.on("/mode", handleMode);
  server.on("/manual", handleManual);
  server.on("/set", handleSet);
  server.on("/status", handleStatus);
  server.on("/call", handleCall);
  server.on("/itemLoaded", handleItemLoaded);
  server.on("/received", handleReceived);
  server.on("/cancelAuto", handleCancelAuto);
  server.begin();
}

/* ================= LOOP ================= */
void loop() {
  server.handleClient();

  updateUltrasonic();
  updateBuzzer();
  checkRFID();

  // Test-mode only: after scanning X or Y (when NOT in autoModeActive), stay stopped briefly
  if (rfidStopActive && (millis() - lastRfidTime >= RFID_STOP_MS)) {
    rfidStopActive = false;
    Serial.println("[RFID] Stop window over, resuming.");
  }

  bool obstacleBlock = obstacleDetected && !(mode == "MANUAL" && manualCmd == "B");

  // ---------- AUTO / DELIVERY MODE (state machine) ----------
  if (autoModeActive) {

    // Handle an in-progress 180-degree U-turn (at X after Item Loaded, or at Y after Received)
    if (isDoingUTurn) {
      if (millis() - uTurnStartTime < UTURN_MS) {
        int s = baseSpeed;
        leftMotor(s); rightMotor(-s);   // fixed spin direction for the U-turn; flip if it spins the wrong way
      } else {
        isDoingUTurn = false;
        robotState = stateAfterUTurn;
        integral = 0; lastError = 0;
        Serial.println("[UTURN] 180-turn complete, resuming.");
      }
    }
    // Handle an in-progress junction turn (open-loop, timed pivot)
    else if (isTurningAtJunction) {
      if (millis() - turnStartTime < JUNCTION_TURN_MS) {
        int s = baseSpeed;
        if      (currentTurnDir == 'L') { leftMotor(s);  rightMotor(-s); }
        else if (currentTurnDir == 'R') { leftMotor(-s); rightMotor(s); }
        else                            { /* 'S' straight, do nothing special */ }
      } else {
        isTurningAtJunction = false;
        integral = 0; lastError = 0;   // reset PID after the turn
        Serial.println("[J1] Turn complete, resuming line follow.");
      }
    }
    else if (obstacleBlock) {
      driveStop();
    }
    else {
      switch (robotState) {
        case STATE_GOING_TO_X:
        case STATE_GOING_TO_Y:
          lineFollow();
          break;

        case STATE_RETURNING_TO_BASE:
          readSensors(lastError);   // keep irVal[] fresh for base marker check
          if (checkBaseMarker()) {
            driveStop();
            robotState = STATE_STANDBY;
            autoModeActive = false;
            Serial.println("[STATE] Base marker detected. Robot is now STANDBY.");
          } else {
            lineFollow();
          }
          break;

        case STATE_AT_X:
        case STATE_AT_Y:
        case STATE_STANDBY:
        default:
          driveStop();   // waiting for App confirmation
          break;
      }
    }
  }
  // ---------- MANUAL / LINE TEST MODE (existing behavior, RFID test-stop included) ----------
  else {
    bool blockDrive = obstacleBlock || rfidStopActive;
    if (blockDrive) {
      driveStop();
    }
    else if (mode == "LINE")   lineFollow();
    else if (mode == "MANUAL") manualDrive();
    else {
      float e; readSensors(e);     // shudhu UI dot update er jonno
      driveStop();
    }
  }
}
