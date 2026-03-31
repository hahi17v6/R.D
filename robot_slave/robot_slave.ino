/**
 * robot_slave.ino
 * 
 * Arduino Uno — Firmware Esclave du Robot Distributeur
 * Optimisé : UART Binaire à 115200 Bauds, PID Centrage
 */

#include <NewPing.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

// ══════════════════════════════════════════════════════════════
// STRUCTURES BINAIRES UART
// ══════════════════════════════════════════════════════════════
#pragma pack(push, 1)
struct CommandPacket {
  uint8_t header; // 0xAA
  uint8_t action; // 1=FWD, 2=BACK, 3=LEFT, 4=RIGHT, 5=STOP, 6=UNLOCK, 7=SERVO_OPEN, 8=SERVO_CLOSE, 9=LCD_MSG
  int32_t arg;    // Argument 
};

struct SensorPacket {
  uint8_t header; // 0xBB
  uint8_t distLeft;
  uint8_t distRight;
  uint8_t distFront;
  uint8_t distBack;
  int16_t imuAngle;
  uint8_t buttonState;
  uint8_t event; // 0=None, 1=Obstacle, 2=DoorOpened
};
#pragma pack(pop)

uint8_t pendingEvent = 0;

// ══════════════════════════════════════════════════════════════
// PINS 
// ══════════════════════════════════════════════════════════════
#define MOT_B_DIR   13
#define MOT_B_FREN  8
#define MOT_B_PWM   11
#define MOT_A_DIR   12
#define MOT_A_FREN  9
#define MOT_A_PWM   3

#define MAX_DISTANCE 200
NewPing leftSensor (4, 4, MAX_DISTANCE);
NewPing rightSensor(5, 5, MAX_DISTANCE);
NewPing frontSensor(6, 6, MAX_DISTANCE);
NewPing backSensor (7, 7, MAX_DISTANCE);

LiquidCrystal_I2C lcd(0x27, 16, 2); 

Servo monServo;
#define SERVO_PIN       10
#define ANGLE_VERROUILLE    0
#define ANGLE_DEVERROUILLE  90

// ══════════════════════════════════════════════════════════════
// IMU 10DOF
// ══════════════════════════════════════════════════════════════
#define MPU_ADDR 0x68
float imuAngleZ = 0.0;
unsigned long lastImuTime = 0;
float gyroZ_offset = 0.0;

void setupIMU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);

  long sum = 0;
  for(int i = 0; i < 500; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47); 
    Wire.endTransmission(false);
    Wire.requestFrom((int)MPU_ADDR, 2, (int)true);
    int16_t gz = Wire.read() << 8 | Wire.read();
    sum += gz;
    delay(3);
  }
  gyroZ_offset = sum / 500.0;
  lastImuTime = millis();
}

void lireIMU() {
  unsigned long now = millis();
  float dt = (now - lastImuTime) / 1000.0;
  lastImuTime = now;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, 2, (int)true);
  int16_t gz = Wire.read() << 8 | Wire.read();

  float gyroRate = (gz - gyroZ_offset) / 131.0;
  if (abs(gyroRate) > 1.0) {
    imuAngleZ += gyroRate * dt;
  }
}

#define BUTTON_PIN A0

#define VITESSE_BASE    180
#define VITESSE_VIRAGE  130
#define PAS_RAMPE       6
#define DELAI_RAMPE     12

#define SEUIL_OBSTACLE_AVANT    20   
#define SEUIL_OBSTACLE_ARRIERE  15
#define SEUIL_CENTRAGE          4
#define SEUIL_INTERSECTION      50

int vitesseA = 0, vitesseB = 0;
int cibleA = 0, cibleB = 0;
bool dirA = HIGH, dirB = HIGH;
bool nouvelleDirA = HIGH, nouvelleDirB = HIGH;

int distLeft = MAX_DISTANCE, distRight = MAX_DISTANCE;
int distFront = MAX_DISTANCE, distBack = MAX_DISTANCE;

unsigned long dernierUpdate = 0;
unsigned long dernierCapteur = 0;
unsigned long dernierEnvoi = 0;

enum RobotMode {
  MODE_IDLE,           
  MODE_FORWARD,        
  MODE_BACKWARD,       
  MODE_TURN_LEFT,      
  MODE_TURN_RIGHT,     
  MODE_DELIVERY,       
  MODE_EMERGENCY_STOP  
};

RobotMode currentMode = MODE_IDLE;
bool obstacleDetected = false;
String unlockCode = "";
bool casierOuvert = false;

// ══════════════════════════════════════════════════════════════
// MOTEURS
// ══════════════════════════════════════════════════════════════

void setMoteurA(bool dir, int pwm) {
  digitalWrite(MOT_A_DIR, dir);
  analogWrite(MOT_A_PWM, pwm);
}

void setMoteurB(bool dir, int pwm) {
  digitalWrite(MOT_B_DIR, dir);
  analogWrite(MOT_B_PWM, pwm);
}

void arretUrgence() {
  vitesseA = 0; vitesseB = 0;
  cibleA = 0; cibleB = 0;
  setMoteurA(dirA, 0);
  setMoteurB(dirB, 0);
}

void mettreAJourRampe() {
  if (millis() - dernierUpdate < DELAI_RAMPE) return;
  dernierUpdate = millis();

  if (vitesseA > 0 && dirA != nouvelleDirA) {
    vitesseA = max(0, vitesseA - PAS_RAMPE);
  } else {
    if (dirA != nouvelleDirA) dirA = nouvelleDirA;
    if (vitesseA < cibleA) vitesseA = min(cibleA, vitesseA + PAS_RAMPE);
    if (vitesseA > cibleA) vitesseA = max(cibleA, vitesseA - PAS_RAMPE);
  }
  setMoteurA(dirA, vitesseA);

  if (vitesseB > 0 && dirB != nouvelleDirB) {
    vitesseB = max(0, vitesseB - PAS_RAMPE);
  } else {
    if (dirB != nouvelleDirB) dirB = nouvelleDirB;
    if (vitesseB < cibleB) vitesseB = min(cibleB, vitesseB + PAS_RAMPE);
    if (vitesseB > cibleB) vitesseB = max(cibleB, vitesseB - PAS_RAMPE);
  }
  setMoteurB(dirB, vitesseB);
}

void cmdAvancer() { nouvelleDirA = HIGH; cibleA = VITESSE_BASE; nouvelleDirB = HIGH; cibleB = VITESSE_BASE; }
void cmdReculer() { nouvelleDirA = LOW; cibleA = VITESSE_BASE; nouvelleDirB = LOW; cibleB = VITESSE_BASE; }
void cmdVirerGauche() { nouvelleDirA = HIGH; cibleA = VITESSE_VIRAGE; nouvelleDirB = HIGH; cibleB = VITESSE_BASE; }
void cmdVirerDroite() { nouvelleDirA = HIGH; cibleA = VITESSE_BASE; nouvelleDirB = HIGH; cibleB = VITESSE_VIRAGE; }
void cmdPivotGauche() { nouvelleDirA = LOW;  cibleA = VITESSE_BASE; nouvelleDirB = HIGH; cibleB = VITESSE_BASE; }
void cmdPivotDroite() { nouvelleDirA = HIGH; cibleA = VITESSE_BASE; nouvelleDirB = LOW;  cibleB = VITESSE_BASE; }
void cmdStop() { cibleA = 0; cibleB = 0; }

// Capteurs
int lireCapteur(NewPing &sensor) {
  unsigned int uS = sensor.ping_median(1); // 1 ping pour aller vite!
  if (uS == 0) return MAX_DISTANCE;
  return uS / US_ROUNDTRIP_CM;
}

void lireTousLesCapteurs() {
  if (millis() - dernierCapteur < 50) return; // 20Hz
  dernierCapteur = millis();
  distFront = lireCapteur(frontSensor);
  distBack  = lireCapteur(backSensor);
  distLeft  = lireCapteur(leftSensor);
  distRight = lireCapteur(rightSensor);
}

// ══════════════════════════════════════════════════════════════
// LOGIQUE DE DÉCISION LOCALE (CENTRAGE / SÉCURITÉ)
// ══════════════════════════════════════════════════════════════

void decisionLocale() {
  if (currentMode == MODE_DELIVERY || currentMode == MODE_IDLE) return;

  int diff = distLeft - distRight;

  // PRIORITÉ 1: OBSTACLE DEVANT
  if (distFront < SEUIL_OBSTACLE_AVANT) {
    if (!obstacleDetected) {
      arretUrgence();
      obstacleDetected = true;
      pendingEvent = 1; // Signal Obstacle
      currentMode = MODE_EMERGENCY_STOP;
    }
    return;
  }

  if (obstacleDetected && distFront >= SEUIL_OBSTACLE_AVANT) {
    obstacleDetected = false;
    if (currentMode == MODE_EMERGENCY_STOP) currentMode = MODE_FORWARD;
  }

  // PRIORITÉ 2: OBSTACLE ARRIERE
  if (distBack < SEUIL_OBSTACLE_ARRIERE && currentMode == MODE_BACKWARD) {
    arretUrgence(); delay(200);
    currentMode = MODE_FORWARD; cmdAvancer();
    return;
  }

  // PRIORITÉ 3: CENTRAGE
  if (currentMode == MODE_FORWARD) {
    if (abs(diff) > SEUIL_INTERSECTION) {
      cmdAvancer(); 
    }
    else if (distLeft < 15 || distRight < 15) {
      if (distLeft < distRight) cmdVirerDroite();
      else cmdVirerGauche();
    }
    else if (abs(diff) <= SEUIL_CENTRAGE) {
      cmdAvancer();
    }
    else if (diff > 0) {
      // distLeft > distRight => plus de place à gauche => se décaler à gauche (donc tourner gauche)
      cmdVirerGauche();
    }
    else {
      // distLeft < distRight => plus de place à droite => se décaler à droite (donc tourner droite)
      cmdVirerDroite();
    }
  }
}

// ══════════════════════════════════════════════════════════════
// COMMUNICATION UART BINAIRE
// ══════════════════════════════════════════════════════════════

void applyCommand(CommandPacket cmd) {
  if (cmd.action == 1) { currentMode = MODE_FORWARD; cmdAvancer(); }
  else if (cmd.action == 2) { currentMode = MODE_BACKWARD; cmdReculer(); }
  else if (cmd.action == 3) { currentMode = MODE_TURN_LEFT; cmdPivotGauche(); }
  else if (cmd.action == 4) { currentMode = MODE_TURN_RIGHT; cmdPivotDroite(); }
  else if (cmd.action == 5) { currentMode = MODE_IDLE; arretUrgence(); }
  else if (cmd.action == 6) { 
    char buf[7]; sprintf(buf, "%06ld", (long)cmd.arg);
    unlockCode = String(buf);
    currentMode = MODE_DELIVERY; arretUrgence();
    lcd.clear(); lcd.setCursor(0, 0); lcd.print("Entrez le code");
  }
  else if (cmd.action == 7) {
    monServo.attach(SERVO_PIN, 500, 2500); monServo.write(ANGLE_DEVERROUILLE); casierOuvert = true;
  }
  else if (cmd.action == 8) {
    monServo.write(ANGLE_VERROUILLE); delay(500); monServo.detach(); casierOuvert = false;
  }
  else if (cmd.action == 9) { // LCD
    lcd.clear(); lcd.setCursor(0, 0);
    if (cmd.arg == 1) lcd.print("Code OK!");
    else if (cmd.arg == 2) lcd.print("Code faux!");
    else if (cmd.arg == 3) lcd.print("Entrez le code");
    else {
      // Nombre d'étoiles
      lcd.print("Code: ");
      for(int i=0; i<cmd.arg; i++) lcd.print("*");
    }
  }
}

void lireUART() {
  if (Serial.available() >= (int)sizeof(CommandPacket)) {
    if (Serial.peek() == 0xAA) {
      CommandPacket cmd;
      Serial.readBytes((uint8_t*)&cmd, sizeof(CommandPacket));
      applyCommand(cmd);
    } else {
      Serial.read(); // Jette le byte invalide
    }
  }
}

void envoyerEtat() {
  if (millis() - dernierEnvoi < 50) return;  // 20Hz
  dernierEnvoi = millis();

  SensorPacket sp;
  sp.header = 0xBB;
  sp.distLeft = min(255, distLeft);
  sp.distRight = min(255, distRight);
  sp.distFront = min(255, distFront);
  sp.distBack = min(255, distBack);
  sp.imuAngle = (int16_t)imuAngleZ;
  sp.buttonState = digitalRead(BUTTON_PIN);
  sp.event = pendingEvent;
  
  Serial.write((uint8_t*)&sp, sizeof(SensorPacket));
  if (pendingEvent != 0) pendingEvent = 0; // Consumé
}

void afficherPret() {
  lcd.clear(); lcd.setCursor(0, 0); lcd.print("Robot pret");
  lcd.setCursor(0, 1); lcd.print("En attente...");
}

void verifierBouton() {
  static bool dernierEtat = false;
  bool etatActuel = digitalRead(BUTTON_PIN) == HIGH;

  if (etatActuel != dernierEtat) {
    dernierEtat = etatActuel;
    if (currentMode == MODE_DELIVERY && casierOuvert && !etatActuel) {
      monServo.write(ANGLE_VERROUILLE); delay(500); monServo.detach(); casierOuvert = false;
      lcd.clear(); lcd.setCursor(0, 0); lcd.print("Bon appetit !");
      delay(2000);
      pendingEvent = 2; // DOOR_OPENED
      currentMode = MODE_IDLE;
      afficherPret();
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(MOT_A_DIR, OUTPUT); pinMode(MOT_A_FREN, OUTPUT); pinMode(MOT_A_PWM, OUTPUT);
  pinMode(MOT_B_DIR, OUTPUT); pinMode(MOT_B_FREN, OUTPUT); pinMode(MOT_B_PWM, OUTPUT);
  digitalWrite(MOT_A_FREN, LOW); digitalWrite(MOT_B_FREN, LOW);

  pinMode(BUTTON_PIN, INPUT);

  lcd.init(); lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("Calibrating IMU.");
  setupIMU();
  afficherPret();

  monServo.attach(SERVO_PIN, 500, 2500);
  monServo.write(ANGLE_VERROUILLE); delay(500); monServo.detach();
}

void loop() {
  lireUART();
  if (currentMode != MODE_DELIVERY) lireTousLesCapteurs();
  decisionLocale();
  lireIMU();
  mettreAJourRampe();
  verifierBouton();
  envoyerEtat();
}
