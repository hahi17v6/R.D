/**
 * robot_slave.ino
 * 
 * Arduino Uno — Firmware Esclave du Robot Distributeur
 * 
 * Fonctions :
 *   1. Réception ordres UART depuis ESP32 (FWD, STOP, LEFT, RIGHT, etc.)
 *   2. Contrôle moteurs (Motor Shield) + rampe d'accélération
 *   3. Capteurs ultrason (4 directions) : centrage couloir + évitement
 *   4. LCD I2C + Servo : affichage état + verrouillage casier
 *   5. Bouton poussoir : détection présence du plat
 *   6. Envoi état vers ESP32 via UART
 * 
 * SÉCURITÉ : Les capteurs ont TOUJOURS priorité sur les ordres ESP32
 * 
 * Câblage :
 *   Moteur A : DIR=12, FREN=9, PWM=3
 *   Moteur B : DIR=13, FREN=8, PWM=11
 *   Ultrason  : Gauche=4, Droite=5, Avant=6, Arrière=7
 *   LCD I2C   : SDA=A4, SCL=A5
 *   Servo     : Pin 10
 *   Bouton    : A0
 *   UART ESP32: RX=0 (← ESP32 TX2=17), TX=1 (→ ESP32 RX2=16)
 *   Clavier   : Sur l'ESP32 (pas sur l'Arduino)
 */

#include <NewPing.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

// ══════════════════════════════════════════════════════════════
// PINS — MOTEURS (Arduino Motor Shield)
// ══════════════════════════════════════════════════════════════
#define MOT_B_DIR   13
#define MOT_B_FREN  8
#define MOT_B_PWM   11
#define MOT_A_DIR   12
#define MOT_A_FREN  9
#define MOT_A_PWM   3

// ══════════════════════════════════════════════════════════════
// PINS — CAPTEURS ULTRASON (NewPing, single-pin mode)
// ══════════════════════════════════════════════════════════════
#define MAX_DISTANCE 200
NewPing leftSensor (4, 4, MAX_DISTANCE);
NewPing rightSensor(5, 5, MAX_DISTANCE);
NewPing frontSensor(6, 6, MAX_DISTANCE);
NewPing backSensor (7, 7, MAX_DISTANCE);

// ══════════════════════════════════════════════════════════════
// LCD I2C + SERVO
// ══════════════════════════════════════════════════════════════
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Adresse 0x27, 16x2
// NOTE: Si ton LCD a l'adresse 0x20, change ci-dessus.
// Tu peux tester avec un I2C scanner.

Servo monServo;
#define SERVO_PIN       10
#define ANGLE_VERROUILLE    0
#define ANGLE_DEVERROUILLE  90

// ══════════════════════════════════════════════════════════════
// BOUTON POUSSOIR — Détection plat
// ══════════════════════════════════════════════════════════════
#define BUTTON_PIN A0

// ══════════════════════════════════════════════════════════════
// PARAMÈTRES MOTEURS
// ══════════════════════════════════════════════════════════════
#define VITESSE_BASE    180
#define VITESSE_VIRAGE  130
#define PAS_RAMPE       6
#define DELAI_RAMPE     12

#define SEUIL_OBSTACLE_AVANT    20   // cm
#define SEUIL_OBSTACLE_ARRIERE  15
#define SEUIL_CENTRAGE          4
#define SEUIL_INTERSECTION      50

// ══════════════════════════════════════════════════════════════
// VARIABLES D'ÉTAT
// ══════════════════════════════════════════════════════════════

// Moteurs
int vitesseA = 0, vitesseB = 0;
int cibleA = 0, cibleB = 0;
bool dirA = HIGH, dirB = HIGH;
bool nouvelleDirA = HIGH, nouvelleDirB = HIGH;

// Capteurs
int distLeft = MAX_DISTANCE, distRight = MAX_DISTANCE;
int distFront = MAX_DISTANCE, distBack = MAX_DISTANCE;

// Timing
unsigned long dernierUpdate = 0;
unsigned long dernierCapteur = 0;
unsigned long dernierEnvoi = 0;

// État
enum RobotMode {
  MODE_IDLE,           // Arrêté, attend les ordres
  MODE_FORWARD,        // Avance (avec centrage ultrason)
  MODE_BACKWARD,       // Recule
  MODE_TURN_LEFT,      // Tourne à gauche
  MODE_TURN_RIGHT,     // Tourne à droite
  MODE_DELIVERY,       // Arrêté, attente livraison (LCD actif)
  MODE_EMERGENCY_STOP  // Arrêt d'urgence (obstacle)
};

RobotMode currentMode = MODE_IDLE;
bool obstacleDetected = false;
String unlockCode = "";
bool casierOuvert = false;

// Buffer UART
String uartBuffer = "";

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

  // Moteur A
  if (vitesseA > 0 && dirA != nouvelleDirA) {
    vitesseA = max(0, vitesseA - PAS_RAMPE);
  } else {
    if (dirA != nouvelleDirA) dirA = nouvelleDirA;
    if (vitesseA < cibleA) vitesseA = min(cibleA, vitesseA + PAS_RAMPE);
    if (vitesseA > cibleA) vitesseA = max(cibleA, vitesseA - PAS_RAMPE);
  }
  setMoteurA(dirA, vitesseA);

  // Moteur B
  if (vitesseB > 0 && dirB != nouvelleDirB) {
    vitesseB = max(0, vitesseB - PAS_RAMPE);
  } else {
    if (dirB != nouvelleDirB) dirB = nouvelleDirB;
    if (vitesseB < cibleB) vitesseB = min(cibleB, vitesseB + PAS_RAMPE);
    if (vitesseB > cibleB) vitesseB = max(cibleB, vitesseB - PAS_RAMPE);
  }
  setMoteurB(dirB, vitesseB);
}

// ══════════════════════════════════════════════════════════════
// COMMANDES MOTEUR
// ══════════════════════════════════════════════════════════════

void cmdAvancer() {
  nouvelleDirA = HIGH; cibleA = VITESSE_BASE;
  nouvelleDirB = HIGH; cibleB = VITESSE_BASE;
}

void cmdReculer() {
  nouvelleDirA = LOW; cibleA = VITESSE_BASE;
  nouvelleDirB = LOW; cibleB = VITESSE_BASE;
}

void cmdVirerGauche() {
  nouvelleDirA = HIGH; cibleA = VITESSE_VIRAGE;
  nouvelleDirB = HIGH; cibleB = VITESSE_BASE;
}

void cmdVirerDroite() {
  nouvelleDirA = HIGH; cibleA = VITESSE_BASE;
  nouvelleDirB = HIGH; cibleB = VITESSE_VIRAGE;
}

void cmdPivotGauche() {
  nouvelleDirA = LOW;  cibleA = VITESSE_BASE;
  nouvelleDirB = HIGH; cibleB = VITESSE_BASE;
}

void cmdPivotDroite() {
  nouvelleDirA = HIGH; cibleA = VITESSE_BASE;
  nouvelleDirB = LOW;  cibleB = VITESSE_BASE;
}

void cmdStop() {
  cibleA = 0;
  cibleB = 0;
}

// ══════════════════════════════════════════════════════════════
// CAPTEURS ULTRASON
// ══════════════════════════════════════════════════════════════

int lireCapteur(NewPing &sensor) {
  unsigned int uS = sensor.ping_median(3);
  if (uS == 0) return MAX_DISTANCE;
  return uS / US_ROUNDTRIP_CM;
}

void lireTousLesCapteurs() {
  if (millis() - dernierCapteur < 100) return;
  dernierCapteur = millis();

  distFront = lireCapteur(frontSensor);
  distBack  = lireCapteur(backSensor);
  distLeft  = lireCapteur(leftSensor);
  distRight = lireCapteur(rightSensor);
}

// ══════════════════════════════════════════════════════════════
// LOGIQUE DE DÉCISION LOCALE (SÉCURITÉ)
// ══════════════════════════════════════════════════════════════

void decisionLocale() {
  // En mode livraison ou idle, pas de navigation
  if (currentMode == MODE_DELIVERY || currentMode == MODE_IDLE) return;

  int diff = distLeft - distRight;

  // *** PRIORITÉ 1 : OBSTACLE DEVANT → ARRÊT IMMÉDIAT ***
  if (distFront < SEUIL_OBSTACLE_AVANT) {
    if (!obstacleDetected) {
      arretUrgence();
      obstacleDetected = true;
      Serial.println("OBSTACLE");  // Informer l'ESP32
      currentMode = MODE_EMERGENCY_STOP;
    }
    return;
  }

  // Si on sort d'un arrêt d'urgence
  if (obstacleDetected && distFront >= SEUIL_OBSTACLE_AVANT) {
    obstacleDetected = false;
    // Reprendre le mode précédent
    if (currentMode == MODE_EMERGENCY_STOP) {
      currentMode = MODE_FORWARD;
    }
  }

  // *** PRIORITÉ 2 : OBSTACLE ARRIÈRE ***
  if (distBack < SEUIL_OBSTACLE_ARRIERE && currentMode == MODE_BACKWARD) {
    arretUrgence();
    delay(200);
    currentMode = MODE_FORWARD;
    cmdAvancer();
    return;
  }

  // *** PRIORITÉ 3 : CENTRAGE COULOIR (quand mode FORWARD) ***
  if (currentMode == MODE_FORWARD) {
    // Intersection détectée (un côté très ouvert)
    if (abs(diff) > SEUIL_INTERSECTION) {
      // Continuer tout droit
      cmdAvancer();
    }
    // Centrage normal
    else if (abs(diff) < SEUIL_CENTRAGE) {
      cmdAvancer();
    }
    else if (diff > 0) {
      // Plus d'espace à gauche → corriger vers droite
      cmdVirerDroite();
    }
    else {
      // Plus d'espace à droite → corriger vers gauche
      cmdVirerGauche();
    }
  }
}

// ══════════════════════════════════════════════════════════════
// COMMUNICATION UART — RÉCEPTION ESP32
// ══════════════════════════════════════════════════════════════

void processESP32Message(String msg) {
  msg.trim();
  if (msg.length() == 0) return;

  if (msg == "FWD") {
    currentMode = MODE_FORWARD;
    cmdAvancer();
  }
  else if (msg == "BACK") {
    currentMode = MODE_BACKWARD;
    cmdReculer();
  }
  else if (msg == "LEFT") {
    currentMode = MODE_TURN_LEFT;
    cmdPivotGauche();
  }
  else if (msg == "RIGHT") {
    currentMode = MODE_TURN_RIGHT;
    cmdPivotDroite();
  }
  else if (msg == "STOP") {
    currentMode = MODE_IDLE;
    arretUrgence();
  }
  else if (msg.startsWith("UNLOCK:")) {
    // Passer en mode livraison — afficher "Entrez le code" sur LCD
    unlockCode = msg.substring(7);
    currentMode = MODE_DELIVERY;
    arretUrgence();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Entrez le code");
  }
  else if (msg.startsWith("LCD:")) {
    // Afficher un texte sur le LCD (envoyé par l'ESP32)
    String text = msg.substring(4);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(text);
  }
  else if (msg == "SERVO:OPEN") {
    // Ouvrir le casier
    monServo.attach(SERVO_PIN, 500, 2500);
    monServo.write(ANGLE_DEVERROUILLE);
    casierOuvert = true;
  }
  else if (msg == "SERVO:CLOSE") {
    // Fermer le casier
    monServo.write(ANGLE_VERROUILLE);
    delay(500);
    monServo.detach();
    casierOuvert = false;
  }
  else if (msg.startsWith("ARUCO:")) {
    // ArUco détecté : info seulement pour le debug
    int id = msg.substring(6).toInt();
    // Rien de spécial à faire côté Arduino, l'ESP32 gère la recalibration
  }
}

void lireUART() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      processESP32Message(uartBuffer);
      uartBuffer = "";
    } else if (c != '\r') {
      uartBuffer += c;
    }
  }
}

// ══════════════════════════════════════════════════════════════
// ENVOI ÉTAT VERS ESP32
// ══════════════════════════════════════════════════════════════

void envoyerEtat() {
  if (millis() - dernierEnvoi < 500) return;  // 2x par seconde
  dernierEnvoi = millis();

  // Envoyer les distances capteurs
  Serial.print("SENSORS:");
  Serial.print(distLeft);  Serial.print(",");
  Serial.print(distRight); Serial.print(",");
  Serial.print(distFront); Serial.print(",");
  Serial.println(distBack);

  // Envoyer l'état du bouton poussoir
  int buttonState = digitalRead(BUTTON_PIN);
  Serial.print("BUTTON:");
  Serial.println(buttonState);
}

// ══════════════════════════════════════════════════════════════
// LCD — AFFICHAGE
// ══════════════════════════════════════════════════════════════

void afficherCodeLivraison() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Code livraison:");
  lcd.setCursor(0, 1);
  lcd.print(unlockCode);

  // Ouvrir le casier (servo)
  monServo.attach(SERVO_PIN, 500, 2500);
  monServo.write(ANGLE_DEVERROUILLE);
  casierOuvert = true;
}

void afficherPret() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Robot pret");
  lcd.setCursor(0, 1);
  lcd.print("En attente...");
}

void afficherEnRoute() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("En route...");
  lcd.setCursor(0, 1);
  lcd.print("Ne pas toucher");
}

// ══════════════════════════════════════════════════════════════
// BOUTON POUSSOIR — Vérification plat
// ══════════════════════════════════════════════════════════════

void verifierBouton() {
  static bool dernierEtat = false;
  bool etatActuel = digitalRead(BUTTON_PIN) == HIGH;

  if (etatActuel != dernierEtat) {
    dernierEtat = etatActuel;
    Serial.print("BUTTON:");
    Serial.println(etatActuel ? "1" : "0");

    // Si le casier est ouvert et que le plat est retiré
    if (currentMode == MODE_DELIVERY && casierOuvert && !etatActuel) {
      // Fermer le casier
      monServo.write(ANGLE_VERROUILLE);
      delay(500);
      monServo.detach();
      casierOuvert = false;

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Bon appetit !");
      delay(2000);

      // Informer l'ESP32
      Serial.println("DOOR_OPENED");

      currentMode = MODE_IDLE;
      afficherPret();
    }
  }
}

// ══════════════════════════════════════════════════════════════
// SETUP
// ══════════════════════════════════════════════════════════════

void setup() {
  // UART vers ESP32 (utilise le port série hardware)
  Serial.begin(9600);

  // Moteurs
  pinMode(MOT_A_DIR,  OUTPUT);
  pinMode(MOT_A_FREN, OUTPUT);
  pinMode(MOT_A_PWM,  OUTPUT);
  pinMode(MOT_B_DIR,  OUTPUT);
  pinMode(MOT_B_FREN, OUTPUT);
  pinMode(MOT_B_PWM,  OUTPUT);
  digitalWrite(MOT_A_FREN, LOW);
  digitalWrite(MOT_B_FREN, LOW);

  // Bouton poussoir
  pinMode(BUTTON_PIN, INPUT);

  // LCD I2C
  lcd.init();
  lcd.backlight();
  afficherPret();

  // Servo — initialiser verrouillé puis détacher
  monServo.attach(SERVO_PIN, 500, 2500);
  monServo.write(ANGLE_VERROUILLE);
  delay(500);
  monServo.detach();

  Serial.println("READY");
}

// ══════════════════════════════════════════════════════════════
// LOOP
// ══════════════════════════════════════════════════════════════

void loop() {
  // 1. Lire les ordres de l'ESP32
  lireUART();

  // 2. Lire les capteurs ultrason (sauf en mode livraison)
  if (currentMode != MODE_DELIVERY) {
    lireTousLesCapteurs();
  }

  // 3. Décision de sécurité locale (capteurs > ordres ESP32)
  decisionLocale();

  // 4. Mise à jour progressive des moteurs (rampe)
  mettreAJourRampe();

  // 5. Vérifier le bouton poussoir
  verifierBouton();

  // 6. Envoyer l'état à l'ESP32
  envoyerEtat();
}
