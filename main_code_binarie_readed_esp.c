#include "FS.h"
#include "SD.h"
#include "SPI.h"

// --- 1. CONSTANTES DE PRÉCISION (ACCURACY) ---
const float METERS_PER_PIXEL = 0.007378; // 1 pixel = 7.37 mm
uint32_t MAP_WIDTH, MAP_HEIGHT;
int bytesPerRow;

// --- 2. VARIABLES DE POSITION DU ROBOT ---
float robotX_m = 0.0; // Position X en mètres
float robotY_m = 0.0; // Position Y en mètres
float robotAngle = 0.0; // Angle donné par le gyroscope (en degrés)

// --- 3. FONCTION DE LECTURE HAUTE PRÉCISION SUR SD ---
bool isObstacle(float x_m, float y_m) {
  // Conversion mètres -> pixels
  int px = (int)(x_m / METERS_PER_PIXEL);
  int py = (int)(y_m / METERS_PER_PIXEL);

  // Vérification des limites de la carte
  if (px < 0 || px >= MAP_WIDTH || py < 0 || py >= MAP_HEIGHT) return true;

  File file = SD.open("/map.bin");
  if (!file) return true;

  // Position dans le fichier : 8 octets d'entête + (y * largeur) + (x/8)
  long pos = 8 + ((long)py * bytesPerRow) + (px / 8);
  
  if (file.seek(pos)) {
    uint8_t byteVal = file.read();
    file.close();
    int bitIdx = 7 - (px % 8);
    return (byteVal >> bitIdx) & 0x01;
  }
  file.close();
  return true;
}

void setup() {
  Serial.begin(115200);

  // Initialisation de la SD (Broche CS sur GPIO 5)
  if(!SD.begin(5)){
    Serial.println("ERREUR : Carte SD absente !");
    return;
  }

  // Charger les dimensions du bâtiment au démarrage
  File file = SD.open("/map.bin");
  if(!file) {
    Serial.println("ERREUR : map.bin introuvable !");
    return;
  }
  file.read((uint8_t*)&MAP_WIDTH, 4);
  file.read((uint8_t*)&MAP_HEIGHT, 4);
  file.close();

  bytesPerRow = (MAP_WIDTH + 7) / 8;

  Serial.println("--- ROBOT PRÊT ---");
  Serial.printf("Carte : %u x %u pixels\n", MAP_WIDTH, MAP_HEIGHT);
  Serial.printf("Précision configurée : %.2f mm par pixel\n", METERS_PER_PIXEL * 1000);
}

void loop() {
  // --- EXEMPLE DE LOGIQUE DE NAVIGATION ---
  
  // 1. Lire les capteurs (Gyroscope + Encodeurs)
  // [Ici tu mettras ton code pour mettre à jour robotX_m, robotY_m et robotAngle]

  // 2. Anticiper un obstacle à 20 cm devant
  float angleRad = robotAngle * PI / 180.0;
  float checkX = robotX_m + 0.20 * cos(angleRad);
  float checkY = robotY_m + 0.20 * sin(angleRad);

  if (isObstacle(checkX, checkY)) {
    Serial.println("ALERTE : Mur détecté sur la carte ! Arrêt ou virage.");
    // Commande moteurs : STOP ou TOURNE
  } else {
    // Commande moteurs : AVANCE
  }

  delay(50); // Fréquence de calcul (20 fois par seconde)
}