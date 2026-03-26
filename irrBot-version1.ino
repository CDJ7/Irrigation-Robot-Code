// ======================================
// ======== Motor & Pump Controls ========
// ======================================

#include <WiFi.h>
#include <PubSubClient.h>
#include <IBusBM.h>

// WiFi Credentials --------------------------------+
const char* ssid        = "Linksys08736";
const char* password    = "kbygr7ycna";
WiFiClient espClient;

// MQTT Server Details -----------------------------+
const char* mqtt_server = "192.168.1.104";
const int   mqtt_port   = 1883;
PubSubClient client(espClient);

// MQTT TOPICS:
const char* MQTT_MOVE_CMD = "robot/control";
const char* MQTT_PUMP_CMD = "robot/pump";

// ==================================================
// Function Prototypes -----------------------------+
void setupWiFi();
void reconnect();
void setThrottle(int dacPin, float duty01);
void pumpControls(bool on);
void set3Speed(int gear);
void setReverse(bool side, bool rev);
void setBrake(bool on);
void move_control(int forward, int steer, bool stop);
void callback(char* topic, byte* payload, unsigned int length);
void ibusLoop();

IBusBM ibus;
int mode = 0; // Ibus Controls (Global Variables)
bool armed = false;

// ---------------- PIN DEFINITIONS ----------------

#define IBUS_RX_PIN 16

// DAC
#define L_DAC 25
#define R_DAC 26

#define PUMP_PIN    17     

// Motor control pins
#define REV_LEFT   32
#define REV_RIGHT  33
#define SP_HIGH    23
#define SP_LOW     22       
#define BRAKE      21

const float Vmin    = 1.15;
const float Vmax    = 3.30;
const float VdacMax = 3.30;

const bool LEFTM = true;
const bool RIGHTM = false;

float L_throttle;
float R_throttle;

bool L_rev;
bool R_rev;

bool pumpState = false;       // actual pump ON/OFF
bool lastPumpSwitch = false;  // previous switch position

// ---------------- READ IBUS CHANNEL ----------------
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = ibus.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  delay(3000); // Allow power stabilization

  // INITIALIZE: Define Pins
  pinMode(REV_LEFT, OUTPUT);
  pinMode(REV_RIGHT, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  
  // Speed pins start as INPUT (floating/disconnected) for Open-Drain logic
  pinMode(SP_HIGH, INPUT);
  pinMode(SP_LOW, INPUT);

  // Initialize WiFi + MQTT
  setupWiFi();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  delay(1000);

  // IBus (RX-only)
  Serial2.begin(115200, SERIAL_8N1, IBUS_RX_PIN, -1);
  delay(100);  // small pause helps stabilize
  ibus.begin(Serial2);
  Serial.println("iBus receiver initialized.");

  // All commands must be low or off initially
  digitalWrite(REV_LEFT, LOW);
  digitalWrite(REV_RIGHT, HIGH);
  digitalWrite(BRAKE, LOW);
  set3Speed(1);
  digitalWrite(PUMP_PIN, LOW);
  Serial.println("All commands off");

  delay(1000);
}

// ---------------- MAIN LOOP ----------------
void loop() 
{
  int arm = readChannel(4, 0, 100, 0);  // Channel 5 on IBus SWA
  mode = readChannel(5, 0, 100, 0);     // Channel 6 on IBus SWB

  if (arm != 0) {
    if(!armed) {
        Serial.println("Armed!");
        armed = true;
        setBrake(false); // release brake when armed
    }

    if (mode != 0) {
      // Web Mode
      if (!client.connected()) { 
          reconnect(); 
      } else {
          client.loop();
          delay(500);
      }
    } else {
      // Controller Mode
      ibusLoop();
    }

  } else {
    if(armed) {
      // Only stop motors **once** when disarmed
      Serial.println("Disarmed! Stopping motors.");
      setThrottle(L_DAC, 0);
      setThrottle(R_DAC, 0);
      digitalWrite(REV_LEFT, LOW);
      digitalWrite(REV_RIGHT, LOW);
      digitalWrite(PUMP_PIN, LOW);
      setBrake(true); // brake on
      armed = false;
    }
    delay(50); // small delay to reduce CPU hammering
  }
}

// ---------------- WIFI & MQTT FUNCTIONS ----------------
void setupWiFi() {
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  Serial.print("Attempting MQTT connection...");
  if (client.connect("ESP32Client1")) { 
    Serial.println("connected");
    client.subscribe(MQTT_MOVE_CMD);
    client.subscribe(MQTT_PUMP_CMD);
  } else {
    Serial.print("failed, rc=");
    Serial.print(client.state());
    Serial.println(" try again in 5 seconds");
    delay(5000);
  }
}

void callback(char* topic, byte* payload, unsigned int length){
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.println("]: ");

  if (strcmp(topic, MQTT_MOVE_CMD) == 0){
    char move_message[8] = {0};
    memcpy(move_message, payload, min(length, sizeof(move_message)-1));

    // Speed mode
    if (strcmp(move_message, "0") == 0) {
        set3Speed(1);
        Serial.println("Speed: 1");
    }
    else if (strcmp(move_message, "1") == 0) {
        set3Speed(2);
        Serial.println("Speed: 2");
    }
    else if (strcmp(move_message, "2") == 0) {
        set3Speed(3);
        Serial.println("Speed: 3");
    }

    // Movement
    int rasp_forward = 0;
    int rasp_steer = 0;
    
    if(strcmp(move_message, "D") == 0){
      rasp_forward = 50; rasp_steer = 0;
      move_control(rasp_forward, rasp_steer, false);
    }
    else if(strcmp(move_message, "B") == 0){
      rasp_forward = -50; rasp_steer = 0;
      move_control(rasp_forward, rasp_steer, false);
    }
    else if(strcmp(move_message, "L") == 0){
      rasp_forward = 0; rasp_steer = -50;
      move_control(rasp_forward, rasp_steer, false);
    }
    else if(strcmp(move_message, "R") == 0){
      rasp_forward = 0; rasp_steer = 50;
      move_control(rasp_forward, rasp_steer, false);
    }
    else if(strcmp(move_message, "P") == 0){
      rasp_forward = 0; rasp_steer = 0;
      move_control(rasp_forward, rasp_steer, true);
    }
  }

  if (strcmp(topic, MQTT_PUMP_CMD) == 0){
    char pump_message[4] = {0};
    memcpy(pump_message, payload, min(length, sizeof(pump_message)-1));
    int PumpCmd = atoi(pump_message);

    if (PumpCmd == 1){ 
      digitalWrite(PUMP_PIN, HIGH);
      Serial.println("PUMP ON");
    } else { 
      digitalWrite(PUMP_PIN, LOW);
      Serial.println("PUMP OFF");
    }
  }
}
// ---------------- CONTROLLER LOGIC ----------------
void ibusLoop()
{
  int pump_channel = readChannel(7, 0, 100, 0);  // IBus SWD
  
  int forward = readChannel(1, -100, 100, 0); // Vertical
  int steer   = readChannel(0, -100, 100, 0);   // Horizontal
  int speed_channel = readChannel(6, 0, 100, 0);

  // --- RADIAL DEADZONE ---
  float magnitude = sqrt((float)forward * forward + (float)steer * steer);

  if (magnitude < 15) {  // Single threshold for both axes together
      forward = 0;
      steer = 0;
  } else {
      // Scale down steer based on how much forward is being used
      // This creates the "cone" effect
      float steerScale = 1.0 - (abs(forward) / 100.0) * 0.5;
      steer = (int)(steer * steerScale);
  }

  // Pump Control
  bool pumpSwitch = (pump_channel > 50); 
  if (pumpSwitch != lastPumpSwitch) {
      pumpControls(pumpSwitch);
      Serial.println(pumpSwitch ? "Pump ON (IBUS)" : "Pump OFF (IBUS)");
  }
  lastPumpSwitch = pumpSwitch;

  // Speed Logic (with jitter prevention)
  if (speed_channel < 25) {
    set3Speed(1); 
  } else if (speed_channel > 75) {
    set3Speed(3); 
  } else {
    set3Speed(2); 
  }

  // --- 2. PROPORTIONAL ARCADE DRIVE MATH ---
  float L_cmd = steer + forward;
  float R_cmd = steer - forward;  // Right motor physically reversed

  L_throttle = constrain(L_cmd, -100, 100);
  R_throttle = constrain(R_cmd, -100, 100);

  // Set Reverse Pins
  if (L_throttle < 0) { 
    setReverse(LEFTM, true); 
    L_rev = true; }
  else                { 
    setReverse(LEFTM, false);                             
    L_rev = false; }
  
  if (R_throttle < 0) { 
    setReverse(RIGHTM, true);
    R_rev = true; }
  else                { 
    setReverse(RIGHTM, false);                             
    R_rev = false; }
  
  // Output to DAC
  setThrottle(L_DAC, (abs(L_throttle) / 100.0));
  setThrottle(R_DAC, (abs(R_throttle) / 100.0));

  Serial.println(
    String("Speed: ") + speed_channel +
    " | FWD: " + forward +
    " | STR: " + steer +
    " | L_Th: " + L_throttle +
    " | R_Th: " + R_throttle
  );

  delay(60);
}

// ---------------- HELPER FUNCTIONS ----------------
void setThrottle(int dacPin, float duty01) {
  if (duty01 < 0) duty01 = 0;
  if (duty01 > 1) duty01 = 1;
  float v = Vmin + duty01 * (Vmax - Vmin);
  uint8_t code = (uint8_t)((v / VdacMax) * 255.0f);
  Serial.println(v);
  Serial.println(code);
  dacWrite(dacPin, code);
}

// --- OPEN-DRAIN SPEED FIX ---
void set3Speed(int gear) {
  // Reset both pins to act like a disconnected mechanical switch
  pinMode(SP_LOW, INPUT);
  pinMode(SP_HIGH, INPUT);

  if (gear == 1) {        
    // LOW SPEED: Connect SP_LOW to Ground
    pinMode(SP_LOW, OUTPUT);
    digitalWrite(SP_LOW, LOW);
  } else if (gear == 3) { 
    // HIGH SPEED: Connect SP_HIGH to Ground
    pinMode(SP_HIGH, OUTPUT);
    digitalWrite(SP_HIGH, LOW);
  }
  // If gear == 2 (MID SPEED), both remain INPUT (floating)
}

void setReverse(bool side, bool rev){
  digitalWrite(side ? REV_LEFT : REV_RIGHT,  rev ? HIGH : LOW);
}

void setBrake(bool on){
  digitalWrite(BRAKE, on ? HIGH : LOW);
}

// Fixed to scale Web/MQTT speeds properly instead of full blast
void move_control(int forward, int steer, bool stop)
{
  Serial.print("forward: "); Serial.print(forward);
  Serial.print(" | steer:   "); Serial.print(steer);
  Serial.print(" | stop:    "); Serial.println(stop);

  if (stop) {
    setThrottle(L_DAC, 0);
    setThrottle(R_DAC, 0);
    return;
  }

  float L_cmd = forward + steer;
  float R_cmd = forward - steer;

  // Set Reverse
  if (L_cmd < 0) { setReverse(LEFTM, true); L_cmd = -L_cmd; }
  else { setReverse(LEFTM, false); }
  
  if (R_cmd < 0) { setReverse(RIGHTM, true); R_cmd = -R_cmd; }
  else { setReverse(RIGHTM, false); }

  // Scale safely (Dividing by 100 turns a 50 command into 0.5 duty cycle)
  setThrottle(L_DAC, L_cmd / 100.0);
  setThrottle(R_DAC, R_cmd / 100.0);
}

void pumpControls(bool on) {
    digitalWrite(PUMP_PIN, on ? HIGH : LOW);
}