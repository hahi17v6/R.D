#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "Ultrasonic.h" // La bibliothèque Grove

// --- 1. CONFIGURATION CARTE & POSITION ---
const float METERS_PER_PIXEL = 0.007378; 
uint32_t MAP_WIDTH, MAP_HEIGHT;
int bytesPerRow;
float robotX_m = 0.0, robotY_m = 0.0, robotAngle = 0.0;

// --- 2. INSTANCES DES CAPTEURS GROVE ---
// On définit les pins SIG des 4 capteurs
Ultrasonic rangerAvant(32);
Ultrasonic rangerArriere(33);
Ultrasonic rangerGauche(34);
Ultrasonic rangerDroite(35);

// --- 3. FONCTIONS DE LECTURE ---

// Lit la distance et la convertit en mètres
float lireDistance(Ultrasonic &sensor) {
    long range = sensor.MeasureInCentimeters();
    if (range == 0 || range > 400) return 4.0; // Si erreur ou trop loin, on simule du vide
    return (float)range / 100.0; // Conversion cm -> mètres
}

// Fonction de lecture sur carte SD (Ta fonction isObstacle)
bool isObstacle(float x_m, float y_m) {
    int px = (int)(x_m / METERS_PER_PIXEL);
    int py = (int)(y_m / METERS_PER_PIXEL);
    if (px < 0 || px >= MAP_WIDTH || py < 0 || py >= MAP_HEIGHT) return true;

    File file = SD.open("/map.bin");
    if (!file) return true;

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

// --- 4. SETUP ---
void setup() {
    Serial.begin(115200);
    
    // Initialisation SD
    if(!SD.begin(5)) {
        Serial.println("ERREUR SD");
        while(1); 
    }

    // Lecture des dimensions du fichier binaire
    File file = SD.open("/map.bin");
    file.read((uint8_t*)&MAP_WIDTH, 4);
    file.read((uint8_t*)&MAP_HEIGHT, 4);
    file.close();
    bytesPerRow = (MAP_WIDTH + 7) / 8;

    Serial.println("Robot Initialisé avec Bibliothèque Grove");
}

// --- 5. BOUCLE DE NAVIGATION ---
void loop() {
    // A. SÉCURITÉ
    float distAvant = lireDistance(rangerAvant);
    if (distAvant < 0.25) { // Stop si obstacle à 25cm
        // stopperMoteurs(); 
        Serial.println("STOP !");
        delay(100);
        return;
    }

    // B. LECTURE DES CÔTÉS
    float distG = lireDistance(rangerGauche);
    float distD = lireDistance(rangerDroite);

    // C. VÉRIFICATION CARTE SD
    // On projette un point à 1 mètre à gauche pour voir si c'est un mur ou une ouverture
    float angleRad = (robotAngle + 90.0) * PI / 180.0;
    float checkX = robotX_m + 1.0 * cos(angleRad);
    float checkY = robotY_m + 1.0 * sin(angleRad);

    if (isObstacle(checkX, checkY)) {
        // La carte dit : "C'est un couloir"
        // On s'équilibre entre les deux murs
        float erreur = distG - distD; 
        // exemple: si distG=0.6 et distD=0.4, erreur = 0.2 -> tourner à gauche
        Serial.printf("Mode Couloir - Erreur: %.2f\n", erreur);
    } else {
        // La carte dit : "C'est une ouverture" (intersection/porte)
        // On ignore le capteur gauche et on suit le mur droit à distance fixe
        Serial.println("Mode Intersection - Suivi mur droit");
    }

    // D. ODOMÉTRIE (à compléter avec tes encodeurs)
    // actualiserPosition();

    delay(50); // Boucle à 20Hz
}