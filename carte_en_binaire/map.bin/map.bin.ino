/**
 * carte_sd.ino
 * 
 * ESP32-WROOM — Écriture de map.bin sur carte micro SD
 * (Sans WiFi)
 * 
 * Câblage SPI :
 *   VCC  → 5V
 *   GND  → GND
 *   CS   → GPIO 4
 *   MOSI → GPIO 23
 *   SCK  → GPIO 18
 *   MISO → GPIO 19
 * 
 * Méthode : Placez map.bin dans le dossier data/ du projet,
 * flashez-le via "ESP32 Sketch Data Upload" (SPIFFS),
 * puis ce sketch le copie automatiquement sur la carte SD.
 * 
 * Bibliothèques requises :
 *   - SD     (incluse dans Arduino ESP32)
 *   - SPIFFS (incluse dans Arduino ESP32)
 */

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <SPIFFS.h>

// ── Broches SD ────────────────────────────────────────────────
#define SD_CS   4
#define SD_MOSI 23
#define SD_SCK  18
#define SD_MISO 19

// ── Paramètres ────────────────────────────────────────────────
#define MAP_FILENAME  "/map.bin"
#define BUFFER_SIZE   512

// ── Variables d'état ──────────────────────────────────────────
bool sdReady     = false;
bool spiffsReady = false;

// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== ESP32 — Copie map.bin vers SD ===");

  // 1. Initialisation SPIFFS
  Serial.print("Initialisation SPIFFS... ");
  if (!SPIFFS.begin(true)) {
    Serial.println("ECHEC !");
    Serial.println("→ Flashez le dossier data/ avec l'outil ESP32 Sketch Data Upload.");
    return;
  }
  spiffsReady = true;
  Serial.println("OK");

  // Vérifie que map.bin est présent dans SPIFFS
  if (!SPIFFS.exists(MAP_FILENAME)) {
    Serial.println("ERREUR : map.bin introuvable dans SPIFFS !");
    Serial.println("→ Placez map.bin dans data/ et utilisez ESP32 Sketch Data Upload.");
    return;
  }

  File srcInfo = SPIFFS.open(MAP_FILENAME, FILE_READ);
  Serial.printf("map.bin trouvé : %u octets\n", srcInfo.size());
  srcInfo.close();

  // 2. Initialisation carte SD
  Serial.print("Initialisation carte SD... ");
  SPIClass spiSD(VSPI);
  spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if (!SD.begin(SD_CS, spiSD)) {
    Serial.println("ECHEC !");
    Serial.println("Vérifiez le câblage :");
    Serial.println("  CS   → GPIO 4");
    Serial.println("  MOSI → GPIO 23");
    Serial.println("  SCK  → GPIO 18");
    Serial.println("  MISO → GPIO 19");
    Serial.println("  VCC  → 5V  |  GND → GND");
    return;
  }
  sdReady = true;

  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("Aucune carte SD détectée !");
    return;
  }
  Serial.printf("OK (type: %s, taille: %llu MB)\n",
    cardType == CARD_MMC  ? "MMC"  :
    cardType == CARD_SD   ? "SD"   :
    cardType == CARD_SDHC ? "SDHC" : "INCONNU",
    SD.cardSize() / (1024 * 1024)
  );

  // 3. Copie SPIFFS → SD
  copyMapToSD();
}

// ─────────────────────────────────────────────────────────────
void loop() {
  delay(10000);
}

// ─────────────────────────────────────────────────────────────
void copyMapToSD() {
  Serial.println("\n--- Copie map.bin vers la SD ---");

  File src = SPIFFS.open(MAP_FILENAME, FILE_READ);
  if (!src) {
    Serial.println("ERREUR : impossible d'ouvrir map.bin (SPIFFS).");
    return;
  }

  // Supprime l'ancien fichier si existant
  if (SD.exists(MAP_FILENAME)) {
    Serial.println("Ancien map.bin détecté sur SD → suppression...");
    SD.remove(MAP_FILENAME);
  }

  File dst = SD.open(MAP_FILENAME, FILE_WRITE);
  if (!dst) {
    Serial.println("ERREUR : impossible de créer map.bin sur la SD.");
    src.close();
    return;
  }

  // Copie par blocs
  uint8_t  buf[BUFFER_SIZE];
  uint32_t totalWritten = 0;
  uint32_t totalSize    = src.size();
  uint32_t lastPercent  = 0;

  Serial.printf("Copie de %u octets en cours...\n", totalSize);

  while (src.available()) {
    size_t bytesRead    = src.read(buf, BUFFER_SIZE);
    size_t bytesWritten = dst.write(buf, bytesRead);

    if (bytesWritten != bytesRead) {
      Serial.printf("ERREUR d'écriture à l'octet %u !\n", totalWritten);
      break;
    }

    totalWritten += bytesWritten;

    // Progression tous les 10%
    uint32_t percent = (totalWritten * 100) / totalSize;
    if (percent / 10 != lastPercent / 10) {
      lastPercent = percent;
      Serial.printf("  %3u%% — %u / %u octets\n", percent, totalWritten, totalSize);
    }
  }

  src.close();
  dst.close();

  // Vérification finale
  File check = SD.open(MAP_FILENAME, FILE_READ);
  if (check && check.size() == totalSize) {
    Serial.printf("\n✓ Copie réussie ! map.bin (%u octets) sur la SD.\n", totalSize);
  } else {
    Serial.println("\n✗ Vérification échouée — tailles différentes !");
  }
  if (check) check.close();
}
                               
