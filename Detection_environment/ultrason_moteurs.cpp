// ====================================================
// Robot autonome : centrage couloir + évitement obstacles
// Capteurs : Gauche→4  Droite→5  Avant→6  Arrière→7
// Moteurs via Arduino Motor Shield
// ====================================================

#include <NewPing.h>

// ---- Capteurs ----
#define MAX_DISTANCE 200
NewPing leftSensor (4, 4, MAX_DISTANCE);
NewPing rightSensor(5, 5, MAX_DISTANCE);
NewPing frontSensor(6, 6, MAX_DISTANCE);
NewPing backSensor (7, 7, MAX_DISTANCE);

// ---- Broches moteurs ----
#define MOT_B_DIR   13
#define MOT_B_FREN  8
#define MOT_B_PWM   11
#define MOT_A_DIR   12
#define MOT_A_FREN  9
#define MOT_A_PWM   3

// ---- Paramètres ----
#define VITESSE_BASE    180
#define VITESSE_VIRAGE  130
#define PAS_RAMPE       6
#define DELAI_RAMPE     12

#define SEUIL_OBSTACLE_AVANT    20
#define SEUIL_OBSTACLE_ARRIERE  15
#define SEUIL_CENTRAGE          4
#define SEUIL_INTERSECTION      50

// ---- Variables moteurs ----
int vitesseA = 0;
int vitesseB = 0;
int cibleA   = 0;
int cibleB   = 0;
bool dirA         = HIGH;
bool dirB         = HIGH;
bool nouvelleDirA = HIGH;
bool nouvelleDirB = HIGH;

unsigned long dernierUpdate  = 0;
unsigned long dernierCapteur = 0;

int distLeft  = MAX_DISTANCE;
int distRight = MAX_DISTANCE;
int distFront = MAX_DISTANCE;
int distBack  = MAX_DISTANCE;

// ====================================================
// SETUP
// ====================================================
void setup() {
  pinMode(MOT_A_DIR,  OUTPUT);
  pinMode(MOT_A_FREN, OUTPUT);
  pinMode(MOT_A_PWM,  OUTPUT);
  pinMode(MOT_B_DIR,  OUTPUT);
  pinMode(MOT_B_FREN, OUTPUT);
  pinMode(MOT_B_PWM,  OUTPUT);

  digitalWrite(MOT_A_FREN, LOW);
  digitalWrite(MOT_B_FREN, LOW);

  Serial.begin(9600);
  Serial.println("Robot démarré");
}

// ====================================================
// CAPTEURS
// ====================================================
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

  Serial.print("L:"); Serial.print(distLeft);
  Serial.print(" R:"); Serial.print(distRight);
  Serial.print(" F:"); Serial.print(distFront);
  Serial.print(" B:"); Serial.println(distBack);
}

// ====================================================
// MOTEURS
// ====================================================
void setMoteurA(bool dir, int pwm) {
  digitalWrite(MOT_A_DIR, dir);
  analogWrite(MOT_A_PWM, pwm);
}

void setMoteurB(bool dir, int pwm) {
  digitalWrite(MOT_B_DIR, dir);
  analogWrite(MOT_B_PWM, pwm);
}

// Arrêt immédiat — bypass la rampe complètement
void cmdArretUrgence() {
  vitesseA = 0; vitesseB = 0;
  cibleA   = 0; cibleB   = 0;
  setMoteurA(dirA, 0);
  setMoteurB(dirB, 0);
}

// ====================================================
// RAMPE
// ====================================================
void mettreAJourRampe() {
  unsigned long maintenant = millis();
  if (maintenant - dernierUpdate < DELAI_RAMPE) return;
  dernierUpdate = maintenant;

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

// ====================================================
// COMMANDES
// ====================================================
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

// ====================================================
// DÉCISION
// ====================================================
void decider() {
  int diff = distLeft - distRight;

  // Priorité 1 : obstacle devant → arrêt immédiat puis pivot
  if (distFront < SEUIL_OBSTACLE_AVANT) {
    Serial.println("→ OBSTACLE AVANT : arrêt immédiat");
    cmdArretUrgence();
    delay(300);
    if (distLeft > distRight) {
      Serial.println("→ PIVOT GAUCHE");
      cmdPivotGauche();
    } else {
      Serial.println("→ PIVOT DROITE");
      cmdPivotDroite();
    }
  }

  // Priorité 2 : obstacle derrière → arrêt puis avancer
  else if (distBack < SEUIL_OBSTACLE_ARRIERE) {
    Serial.println("→ OBSTACLE ARRIERE : arrêt immédiat");
    cmdArretUrgence();
    delay(300);
    cmdAvancer();
  }

  // Priorité 3 : intersection → tout droit
  else if (abs(diff) > SEUIL_INTERSECTION) {
    Serial.println("→ INTERSECTION : tout droit");
    cmdAvancer();
  }

  // Priorité 4 : centrage couloir
  else {
    if (abs(diff) < SEUIL_CENTRAGE) {
      Serial.println("→ DROIT");
      cmdAvancer();
    }
    else if (diff > 0) {
      Serial.println("→ CORRECTION DROITE");
      cmdVirerDroite();
    }
    else {
      Serial.println("→ CORRECTION GAUCHE");
      cmdVirerGauche();
    }
  }
}

// ====================================================
// LOOP
// ====================================================
void loop() {
  lireTousLesCapteurs();
  decider();
  mettreAJourRampe();
}
