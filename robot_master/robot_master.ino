/**
 * robot_master.ino
 * 
 * ESP32-WROOM — Firmware Maître du Robot Distributeur
 * 
 * Fonctions :
 *   1. Serveur Web (site depuis SD card) + WebSocket
 *   2. Navigation via map.bin (plan binaire sur SD)
 *   3. Réception ArUco depuis le téléphone Android
 *   4. Communication UART avec Arduino Uno (esclave)
 *   5. Logique de décision : capteurs > map (sécurité d'abord)
 *   6. Gestion des commandes de livraison
 *   7. Clavier 4x4 membrane (vérification code de livraison)
 * 
 * Câblage SD (SPI) :
 *   CS=GPIO4  MOSI=GPIO23  SCK=GPIO18  MISO=GPIO19
 * 
 * Câblage UART vers Arduino :
 *   ESP32 TX2 (GPIO17) → Arduino RX (pin 0)
 *   ESP32 RX2 (GPIO16) → Arduino TX (pin 1)
 *   (Diviseur de tension 3.3V/5V sur RX si nécessaire)
 * 
 * Câblage Clavier 4x4 :
 *   Rangées  : GPIO 32, 33, 15, 26
 *   Colonnes : GPIO 27, 14, 12, 13
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <SD.h>
#include <Keypad.h>
#include <esp_task_wdt.h>
#include "AStarPathfinder.h"

#pragma pack(push, 1)
struct CommandPacket {
  uint8_t header; 
  uint8_t action; 
  int32_t arg;
};
struct SensorPacket {
  uint8_t header; 
  uint8_t distLeft;
  uint8_t distRight;
  uint8_t distFront;
  uint8_t distBack;
  int16_t imuAngle;
  uint8_t buttonState;
  uint8_t event;
  int8_t deltaDist_cm;
};
#pragma pack(pop)

// ══════════════════════════════════════════════════════════════
// CONFIGURATION
// ══════════════════════════════════════════════════════════════

// --- SD Card ---
#define SD_CS   4
#define SD_MOSI 23
#define SD_SCK  18
#define SD_MISO 19
SPIClass spiSD(VSPI);

// --- WiFi Access Point ---
const char* ssid_ap     = "Robot-WiFi";
const char* password_ap = "12345678";

// --- WiFi Station (optionnel, pour internet) ---
const char* ssid_sta     = "iPhone";
const char* password_sta = "123444567";

// --- Serveurs ---
WebServer server(80);
WebSocketsServer webSocket(81);

// --- UART vers Arduino ---
#define UART_TX 17
#define UART_RX 16
#define UART_BAUD 115200

// --- Clavier 4x4 ---
const byte KP_ROWS = 4;
const byte KP_COLS = 4;

char kpKeys[KP_ROWS][KP_COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

byte kpRowPins[KP_ROWS] = {32, 33, 15, 26};
byte kpColPins[KP_COLS] = {27, 14, 12, 13};

Keypad keypad = Keypad(makeKeymap(kpKeys), kpRowPins, kpColPins, KP_ROWS, KP_COLS);
String keypadBuffer = "";

// --- Map ---
#define MAP_FILENAME "/map.bin"
const float METERS_PER_PIXEL = 0.007378;
uint32_t MAP_WIDTH = 0, MAP_HEIGHT = 0;
int bytesPerRow = 0;
bool mapLoaded = false;

// ══════════════════════════════════════════════════════════════
// ÉTAT DU ROBOT
// ══════════════════════════════════════════════════════════════

// Position robot (en mètres, recalibrée par ArUco)
float robotX_m = 0.0;
float robotY_m = 0.0;
float robotAngle = 0.0;  // en degrés

// État capteurs (reçu de l'Arduino)
int distFront = 200, distBack = 200, distLeft = 200, distRight = 200;
bool platPresent = false;

// État de mission
enum MissionState {
  MISSION_IDLE,              // Pas de mission
  GOING_TO_KITCHEN,  // En route vers la cuisine
  WAITING_LOADING,   // À la cuisine, attend chargement
  GOING_TO_ROOM,     // En route vers la chambre
  WAITING_DELIVERY,  // Arrivé, attend que le résident prenne le plat
  RETURNING_HOME,    // Retour au point de départ (cuisine)
  NEEDS_CHARGE       // Batterie faible, à la cuisine, en charge
};

MissionState missionState = MISSION_IDLE;
int batteryLevel = 100; // Niveau de batterie simulé (%)
String currentOrderId = "";
String currentRoom = "";
String deliveryCode = "";
unsigned long missionStartTime = 0;           // Timestamp début de mission
#define MISSION_TIMEOUT_MS (10UL * 60 * 1000) // 10 minutes max par mission

// ── File d'attente de commandes (max 5) ──
struct QueuedOrder {
  String orderId;
  String room;
};
#define MAX_QUEUE 5
QueuedOrder orderQueue[MAX_QUEUE];
int queueCount = 0;

// ── Balises ArUco connues (coordonnées en mètres) ──
// Coordonnées converties depuis pixels via METERS_PER_PIXEL = 0.007378
struct ArUcoMarker {
  int id;
  float x_m;
  float y_m;
  float angle_mur;  // angle du mur en degrés (à ajuster selon la pose réelle)
};

ArUcoMarker balises[] = {
  // === CHAMBRES (entrées principales) ===
  {41,   7.45,   0.86,  0.0},   // Chambre F-041
  {42,   7.39,   1.31,  0.0},   // Chambre F-042
  {34,   8.16,  14.28,  0.0},   // Chambre F-034
  {134,  8.08,  17.73,  0.0},   // Chambre F-034 (porte 2)
  {33,   7.97,  21.30,  0.0},   // Chambre F-033
  {32,  18.39,  36.88,  0.0},   // Chambre F-032
  {30,  15.34,  51.49,  0.0},   // Chambre F-030
  {29,  13.22,  54.30,  0.0},   // F-029 — Cuisine (entrée principale)
  {129,  3.45,  54.25,  0.0},   // F-029 — Cuisine (entrée 2)
  {28,   1.34,  41.77,  0.0},   // Chambre F-028
  {27,   1.48,  48.54,  0.0},   // Chambre F-027 (1ère porte)
  {127, 15.41,  44.43,  0.0},   // Chambre F-027 (2ème porte)
  {26,   1.48,  51.21,  0.0},   // Chambre F-026
  {25,   1.48,  48.54,  0.0},   // Chambre F-025 (1ère porte)
  {125, 15.07,  48.71,  0.0},   // Chambre F-025 (2ème porte)
  {24,   1.44,  51.32,  0.0},   // Chambre F-024

  // === INTERSECTIONS & REPÈRES ===
  {201,  7.78,   6.26,  0.0},   // Intersection 1
  {202,  8.10,  11.16,  0.0},   // Avant l'Intersection 1
  {203,  7.89,  20.45,  0.0},   // Local technique
  {204,  8.69,  34.85,  0.0},   // Intersection 2
  {205, 10.86,  35.64,  0.0},   // Intersection 3
  {206, 16.76,  35.79,  0.0},   // Intersection 4
  {207, 10.99,  37.02,  0.0},   // Intersection 5
  {208, 15.20,  38.33,  0.0},   // Intersection 6
  {209, 15.26,  53.16,  0.0},   // Avant l'Intersection 2
  {210, 15.10,  35.89,  0.0},   // Début de couloir
  {211,  1.29,  55.49,  0.0},   // Intersection 7
  {212,  1.34,  39.31,  0.0},   // Porte random 1
  {213,  1.46,   5.90,  0.0},   // Porte random 2
};
const int NB_BALISES = sizeof(balises) / sizeof(balises[0]);

// --- Cibles ArUco de Mission ---
const int ARUCO_HOME = 0;     // ID ArUco du point de départ
const int ARUCO_KITCHEN = 29; // ID ArUco de la cuisine (F-029 entrée principale)
int targetArUco = -1;         // ID ArUco de la chambre de destination actuelle

struct RoomArUco {
  String roomName;
  int arucoId;
};

// Registre associant le nom de la chambre à son ID ArUco
RoomArUco roomRegistry[] = {
  {"F-041", 41},
  {"F-042", 42},
  {"F-034", 34},
  {"F-033", 33},
  {"F-032", 32},
  {"F-030", 30},
  {"F-028", 28},
  {"F-027", 27},
  {"F-026", 26},
  {"F-025", 25},
  {"F-024", 24}
};
const int NB_ROOMS = sizeof(roomRegistry) / sizeof(roomRegistry[0]);

// ══════════════════════════════════════════════════════════════
// MAP.BIN — NAVIGATION
// ══════════════════════════════════════════════════════════════

bool initMap() {
  File file = SD.open(MAP_FILENAME);
  if (!file) { Serial.println("[MAP] map.bin introuvable sur SD"); return false; }
  file.read((uint8_t*)&MAP_WIDTH, 4);
  file.read((uint8_t*)&MAP_HEIGHT, 4);
  bytesPerRow = (MAP_WIDTH + 7) / 8;
  
  float realW = MAP_WIDTH * METERS_PER_PIXEL;
  float realH = MAP_HEIGHT * METERS_PER_PIXEL;
  Serial.printf("[MAP] Carte chargée : %ux%u pixels, %.2fx%.2f mètres\n", MAP_WIDTH, MAP_HEIGHT, realW, realH);
  
  if (allocateCoarseMap(realW, realH)) {
     for (int cy = 0; cy < coarseH; cy++) {
       for (int cx = 0; cx < coarseW; cx++) {
          int px = (int)((cx * CELL_SIZE_M) / METERS_PER_PIXEL);
          int py = (int)((cy * CELL_SIZE_M) / METERS_PER_PIXEL);
          long pos = 8 + ((long)py * bytesPerRow) + (px / 8);
          if (file.seek(pos)) {
             uint8_t b = file.read();
             if ((b >> (7 - (px % 8))) & 0x01) setCoarseObstacle(cx, cy, true);
          }
       }
     }
     Serial.println("[MAP] Coarse map générée en RAM");
  }
  file.close();
  return true;
}

// Retourne true si (x_m, y_m) est un obstacle (mur) sur la carte
bool isObstacle(float x_m, float y_m) {
  int px = (int)(x_m / METERS_PER_PIXEL);
  int py = (int)(y_m / METERS_PER_PIXEL);
  if (px < 0 || px >= (int)MAP_WIDTH || py < 0 || py >= (int)MAP_HEIGHT) return true;

  File file = SD.open(MAP_FILENAME);
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

// ══════════════════════════════════════════════════════════════
// ARUCO — RECALIBRATION DE POSITION
// ══════════════════════════════════════════════════════════════

void handleArUcoDetection(int arucoId) {
  // 1. Recalibration de la position sur la carte
  bool knownMarker = false;
  for (int i = 0; i < NB_BALISES; i++) {
    if (balises[i].id == arucoId) {
      float oldX = robotX_m, oldY = robotY_m;
      robotX_m = balises[i].x_m;
      robotY_m = balises[i].y_m;
      robotAngle = balises[i].angle_mur;

      Serial.printf("[ARUCO] Balise #%d détectée → Position recalibrée: (%.2f, %.2f) → (%.2f, %.2f)\n",
        arucoId, oldX, oldY, robotX_m, robotY_m);

      // Envoyer la position recalibrée au dashboard via WebSocket
      JsonDocument doc;
      doc["action"] = "robot_pos";
      doc["x"] = robotX_m;
      doc["y"] = robotY_m;
      doc["angle"] = robotAngle;
      doc["aruco"] = arucoId;
      String json;
      serializeJson(doc, json);
      webSocket.broadcastTXT(json);

      // Informer l'Arduino de la recalibration
      Serial2.println("ARUCO:" + String(arucoId));
      knownMarker = true;
      break;
    }
  }

  if (!knownMarker) {
    // Info si le marqueur n'est pas dans le tableau de recalibration (ex: sert juste de trigger d'arrivée)
    Serial.printf("[ARUCO] Balise #%d détectée (non utilisée pour recalibration XY)\n", arucoId);
  }

  // 2. Vérification des objectifs de la mission (Triggers d'arrivée)
  if (missionState == GOING_TO_KITCHEN && arucoId == ARUCO_KITCHEN) {
    arrivedAtKitchen();
  }
  else if (missionState == GOING_TO_ROOM && arucoId == targetArUco) {
    arrivedAtRoom();
  }
  // On utilise ARUCO_HOME ou ARUCO_KITCHEN comme base de retour (les deux marchent si la base est la cuisine)
  else if (missionState == RETURNING_HOME && (arucoId == ARUCO_HOME || arucoId == ARUCO_KITCHEN)) {
    returnedHome();
  }
}

// ══════════════════════════════════════════════════════════════
// COMMUNICATION UART AVEC ARDUINO
// ══════════════════════════════════════════════════════════════


void sendToArduinoBinary(uint8_t action, int32_t arg = 0) {
  CommandPacket cmd;
  cmd.header = 0xAA;
  cmd.action = action;
  cmd.arg = arg;
  Serial2.write((uint8_t*)&cmd, sizeof(CommandPacket));
}

void sendToArduino(String cmd) {
  if (cmd == "FWD") sendToArduinoBinary(1);
  else if (cmd == "BACK") sendToArduinoBinary(2);
  else if (cmd == "LEFT") sendToArduinoBinary(3);
  else if (cmd == "RIGHT") sendToArduinoBinary(4);
  else if (cmd == "STOP") sendToArduinoBinary(5);
  else if (cmd.startsWith("UNLOCK:")) sendToArduinoBinary(6, cmd.substring(7).toInt());
  else if (cmd == "SERVO:OPEN") sendToArduinoBinary(7);
  else if (cmd == "SERVO:CLOSE") sendToArduinoBinary(8);
  else if (cmd.startsWith("LCD:")) {
    String txt = cmd.substring(4);
    if(txt == "Code OK!") sendToArduinoBinary(9, 1);
    else if(txt == "Code faux!") sendToArduinoBinary(9, 2);
    else if(txt == "Entrez le code") sendToArduinoBinary(9, 3);
    else if(txt.startsWith("Code: ")) sendToArduinoBinary(9, 4);
  }
}

void processArduinoMessageBinary(SensorPacket& sp) {
  distLeft = sp.distLeft;
  distRight = sp.distRight;
  distFront = sp.distFront;
  distBack = sp.distBack;
  
  float currentImuRaw = sp.imuAngle;
  static float lastImuRaw = 0.0;
  float deltaAngle = currentImuRaw - lastImuRaw;
  lastImuRaw = currentImuRaw;
  robotAngle += deltaAngle;
  while (robotAngle >= 360.0) robotAngle -= 360.0;
  while (robotAngle < 0.0) robotAngle += 360.0;
  
  // ── Odométrie Simulée (Temps x Vitesse) ──
  // La distance avancée (en cm) est envoyée par l'Arduino
  float distanceAvance_m = sp.deltaDist_cm / 100.0;
  
  // Trigonométrie avec l'angle actuel (en radians) pour estimer la dérive (X,Y)
  robotX_m += distanceAvance_m * cos(robotAngle * PI / 180.0);
  robotY_m += distanceAvance_m * sin(robotAngle * PI / 180.0);
  
  platPresent = (sp.buttonState == 1);
  
  if (sp.event == 1) Serial.println("[SÉCURITÉ] Obstacle !");
  else if (sp.event == 2) {
     Serial.println("[SÉCURITÉ] Porte ouverte !");
     if (missionState == WAITING_DELIVERY && !platPresent) deliveryConfirmed();
  }
}

void readUART() {
  if (Serial2.available() >= (int)sizeof(SensorPacket)) {
    if (Serial2.peek() == 0xBB) {
      SensorPacket sp;
      Serial2.readBytes((uint8_t*)&sp, sizeof(SensorPacket));
      processArduinoMessageBinary(sp);
    } else {
      Serial2.read();
    }
  }
}

// ══════════════════════════════════════════════════════════════
// LOGIQUE DE NAVIGATION
// ══════════════════════════════════════════════════════════════

void navigationStep() {
  if (missionState == MISSION_IDLE || missionState == WAITING_LOADING || missionState == WAITING_DELIVERY || missionState == NEEDS_CHARGE) {
    return;
  }
  if (distFront < 20) return; // Sécurité locale gérée par Uno

  float targetX = 0, targetY = 0;
  bool foundTarget = false;
  int aid = (missionState == RETURNING_HOME) ? ARUCO_KITCHEN : targetArUco;
  
  for(int i=0; i<NB_BALISES; i++) {
    if(balises[i].id == aid) { targetX = balises[i].x_m; targetY = balises[i].y_m; foundTarget = true; break;}
  }
  
  if(!foundTarget) {
     sendToArduino("FWD"); // Fallback
     return;
  }
  
  static unsigned long lastAstar = 0;
  if(millis() - lastAstar > 2000) { 
    lastAstar = millis();
    computeAStarPath(robotX_m, robotY_m, targetX, targetY);
  }
  
  if(pathLength > 1 && currentPathIdx < pathLength) {
     float wpX = currentPath[currentPathIdx].x * CELL_SIZE_M;
     float wpY = currentPath[currentPathIdx].y * CELL_SIZE_M;
     
     float dx = wpX - robotX_m;
     float dy = wpY - robotY_m;
     float dist = sqrt(dx*dx + dy*dy);
     
     if (dist < 0.3) {
        currentPathIdx++; 
     } else {
        float angleCible = atan2(dy, dx) * 180.0 / PI;
        float erreurAngle = angleCible - robotAngle;
        while(erreurAngle > 180.0) erreurAngle -= 360.0;
        while(erreurAngle < -180.0) erreurAngle += 360.0;
        
        if (abs(erreurAngle) > 20.0) {
           if (erreurAngle > 0) sendToArduino("RIGHT");
           else sendToArduino("LEFT");
        } else {
           sendToArduino("FWD");
        }
     }
  } else {
     sendToArduino("FWD"); // Fallback
  }
}

// ══════════════════════════════════════════════════════════════
// GESTION DES COMMANDES
// ══════════════════════════════════════════════════════════════

// Génère un code de sécurité à 6 chiffres
String generateCode() {
  String code = "";
  for (int i = 0; i < 6; i++) {
    code += String(random(0, 10));
  }
  return code;
}

// Démarre une mission de livraison
void startMission(String orderId, String room) {
  if (missionState != MISSION_IDLE) {
    // Robot occupé → ajouter à la file d'attente
    if (queueCount < MAX_QUEUE) {
      orderQueue[queueCount++] = {orderId, room};
      Serial.printf("[QUEUE] Commande %s ajoutée en file d'attente (position: %d)\n", orderId.c_str(), queueCount);
      
      JsonDocument doc;
      doc["action"] = "order_queued";
      doc["id"] = orderId;
      doc["position"] = queueCount;
      doc["msg"] = "En file d'attente (position " + String(queueCount) + ")";
      String json;
      serializeJson(doc, json);
      webSocket.broadcastTXT(json);
    } else {
      JsonDocument doc;
      doc["action"] = "robot_busy";
      doc["msg"] = "File d'attente pleine (max " + String(MAX_QUEUE) + " commandes)";
      String json;
      serializeJson(doc, json);
      webSocket.broadcastTXT(json);
    }
    return;
  }

  currentOrderId = orderId;
  currentRoom = room;
  deliveryCode = generateCode();
  
  // Déterminer l'ArUco cible en fonction de la chambre
  targetArUco = -1;
  for (int i = 0; i < NB_ROOMS; i++) {
    if (roomRegistry[i].roomName == room) {
      targetArUco = roomRegistry[i].arucoId;
      break;
    }
  }
  // Si la chambre n'est pas dans le registre, on essaie de convertir le nom en ID (ex "101" -> 101)
  if (targetArUco == -1) targetArUco = room.toInt();

  missionState = GOING_TO_KITCHEN;
  missionStartTime = millis(); // Démarrer le timer de timeout

  Serial.printf("[MISSION] Démarrage : %s → Chambre %s (ArUco Cible: %d, code: %s)\n",
    orderId.c_str(), room.c_str(), targetArUco, deliveryCode.c_str());

  // Informer le dashboard
  JsonDocument doc;
  doc["action"] = "robot_status";
  doc["name"] = "Robot Alpha";
  doc["status"] = "En mission";
  doc["battery"] = batteryLevel;
  doc["order_id"] = orderId;
  doc["order_status"] = "En cours";
  String json;
  serializeJson(doc, json);
  webSocket.broadcastTXT(json);

  // Informer du code de livraison
  JsonDocument codeDoc;
  codeDoc["action"] = "order_code";
  codeDoc["order_id"] = orderId;
  codeDoc["code"] = deliveryCode;
  codeDoc["room"] = room;
  String codeJson;
  serializeJson(codeDoc, codeJson);
  webSocket.broadcastTXT(codeJson);

  // Envoyer l'Arduino en route
  sendToArduino("FWD");
}

// Robot arrivé à la cuisine
void arrivedAtKitchen() {
  missionState = WAITING_LOADING;
  sendToArduino("STOP");

  Serial.println("[MISSION] Arrivé à la cuisine — en attente de chargement");

  JsonDocument doc;
  doc["action"] = "status_update";
  doc["id"] = currentOrderId;
  doc["status"] = "En attente chargement";
  String json;
  serializeJson(doc, json);
  webSocket.broadcastTXT(json);
}

// Le personnel a chargé le plat → partir vers la chambre
void kitchenLoaded() {
  missionState = GOING_TO_ROOM;
  Serial.printf("[MISSION] Plat chargé → En route vers Chambre %s\n", currentRoom.c_str());
  sendToArduino("FWD");
}

// Robot arrivé à la chambre
void arrivedAtRoom() {
  missionState = WAITING_DELIVERY;
  sendToArduino("STOP");
  keypadBuffer = "";  // Réinitialiser le buffer clavier

  Serial.printf("[MISSION] Arrivé à Chambre %s — Code: %s\n",
    currentRoom.c_str(), deliveryCode.c_str());

  // Dire à l'Arduino d'afficher "Entrez le code" sur le LCD
  sendToArduino("UNLOCK:" + deliveryCode);

  // Informer le dashboard
  JsonDocument doc;
  doc["action"] = "status_update";
  doc["id"] = currentOrderId;
  doc["status"] = "En cours";
  doc["msg"] = "Arrivé – Code: " + deliveryCode;
  String json;
  serializeJson(doc, json);
  webSocket.broadcastTXT(json);
}

// ══════════════════════════════════════════════════════════════
// CLAVIER 4x4 — VÉRIFICATION DU CODE DE LIVRAISON
// ══════════════════════════════════════════════════════════════

void handleKeypad() {
  // Le clavier n'est actif que pendant la livraison
  if (missionState != WAITING_DELIVERY) return;

  char key = keypad.getKey();
  if (!key) return;

  Serial.printf("[CLAVIER] Touche: %c\n", key);

  if (key == '#') {
    // Effacer la saisie
    keypadBuffer = "";
    sendToArduino("LCD:Entrez le code");
    Serial.println("[CLAVIER] Saisie effacée");
  }
  else if (key == '*') {
    // Valider le code
    if (keypadBuffer == deliveryCode) {
      Serial.println("[CLAVIER] ✓ Code correct !");
      sendToArduino("LCD:Code OK!");
      sendToArduino("SERVO:OPEN");
      // La livraison sera confirmée quand le bouton poussoir détecte
      // que le plat a été retiré (géré par processArduinoMessage)
    } else {
      Serial.println("[CLAVIER] ✗ Code incorrect");
      sendToArduino("LCD:Code faux!");
      keypadBuffer = "";
      delay(1500);
      sendToArduino("LCD:Entrez le code");
    }
  }
  else if (key >= '0' && key <= '9') {
    if (keypadBuffer.length() < 6) {
      keypadBuffer += key;
      // Afficher les étoiles sur le LCD
      String stars = "";
      for (int i = 0; i < (int)keypadBuffer.length(); i++) stars += "*";
      sendToArduino("LCD:Code: " + stars);
    }
  }
  // A, B, C, D sont ignorés
}

// Livraison confirmée (bouton poussoir = plat retiré)
void deliveryConfirmed() {
  Serial.printf("[MISSION] Livraison confirmée pour %s\n", currentOrderId.c_str());

  JsonDocument doc;
  doc["action"] = "delivery_confirmed";
  doc["id"] = currentOrderId;
  doc["status"] = "Livrée";
  String json;
  serializeJson(doc, json);
  webSocket.broadcastTXT(json);

  // Consommation de batterie simulée par livraison complete (-20%)
  batteryLevel -= 20;
  if (batteryLevel < 0) batteryLevel = 0;

  // Retour à la cuisine (base)
  missionState = RETURNING_HOME;
  sendToArduino("FWD");
}

// Retour terminé (arrivé à la cuisine)
void returnedHome() {
  currentOrderId = "";
  currentRoom = "";
  deliveryCode = "";
  sendToArduino("STOP");

  JsonDocument doc;
  doc["action"] = "robot_status";
  doc["name"] = "Robot Alpha";
  doc["battery"] = batteryLevel;

  if (batteryLevel <= 20) {
    missionState = NEEDS_CHARGE;
    doc["status"] = "Batterie faible - En charge";
    
    // Alerte au dashboard (pour le cuisinier)
    JsonDocument alertDoc;
    alertDoc["action"] = "alert";
    alertDoc["msg"] = "Robot déchargé ! Veuillez le brancher à la cuisine.";
    String alertJson;
    serializeJson(alertDoc, alertJson);
    webSocket.broadcastTXT(alertJson);

    Serial.println("[MISSION] De retour à la cuisine — BATTERIE FAIBLE, robot en veille/charge");
  } else {
    missionState = MISSION_IDLE;
    doc["status"] = "Disponible";
    Serial.println("[MISSION] De retour à la cuisine — Robot disponible pour une nouvelle commande");
  }

  String json;
  serializeJson(doc, json);
  webSocket.broadcastTXT(json);

  // ── Lancer automatiquement la commande suivante de la queue ──
  if (missionState == MISSION_IDLE && queueCount > 0) {
    QueuedOrder next = orderQueue[0];
    // Décaler la queue
    for (int i = 0; i < queueCount - 1; i++) {
      orderQueue[i] = orderQueue[i + 1];
    }
    queueCount--;
    Serial.printf("[QUEUE] Lancement auto de %s (restant: %d)\n", next.orderId.c_str(), queueCount);
    startMission(next.orderId, next.room);
  }
}

// ══════════════════════════════════════════════════════════════
// SERVEUR WEB — FICHIERS DEPUIS SD
// ══════════════════════════════════════════════════════════════

String getContentType(String filename) {
  if (filename.endsWith(".html")) return "text/html";
  if (filename.endsWith(".css"))  return "text/css";
  if (filename.endsWith(".js"))   return "application/javascript";
  if (filename.endsWith(".png"))  return "image/png";
  if (filename.endsWith(".jpg") || filename.endsWith(".jpeg")) return "image/jpeg";
  if (filename.endsWith(".gif"))  return "image/gif";
  if (filename.endsWith(".ico"))  return "image/x-icon";
  if (filename.endsWith(".json")) return "application/json";
  return "text/plain";
}

void handleFileRead() {
  String path = server.uri();

  // Endpoint spécial : réception ArUco depuis le téléphone
  if (path == "/aruco") {
    if (server.hasArg("id")) {
      int arucoId = server.arg("id").toInt();
      handleArUcoDetection(arucoId);
      server.send(200, "text/plain", "OK");
    } else {
      server.send(400, "text/plain", "Missing id parameter");
    }
    return;
  }

  // Endpoint : état du robot (pour le téléphone)
  if (path == "/status") {
    JsonDocument doc;
    doc["state"] = (int)missionState;
    doc["x"] = robotX_m;
    doc["y"] = robotY_m;
    doc["angle"] = robotAngle;
    doc["plat"] = platPresent;
    doc["order"] = currentOrderId;
    doc["room"] = currentRoom;
    String json;
    serializeJson(doc, json);
    server.send(200, "application/json", json);
    return;
  }

  // Endpoint : authentification côté serveur (PINs + staff secrets ici, pas dans le HTML)
  if (path == "/auth") {
    if (!server.hasArg("role")) {
      server.send(400, "application/json", "{\"ok\":false,\"msg\":\"Missing role\"}");
      return;
    }
    String role = server.arg("role");
    
    if (role == "staff") {
      String user = server.arg("user");
      String pass = server.arg("pass");
      // Credentials staff (sécurisés côté serveur)
      if ((user == "admin" && pass == "robot2024") || 
          (user == "infirmier" && pass == "soins123")) {
        server.send(200, "application/json", "{\"ok\":true}");
      } else {
        server.send(401, "application/json", "{\"ok\":false,\"msg\":\"Identifiants incorrects\"}");
      }
    }
    else if (role == "patient") {
      String room = server.arg("room");
      String pin = server.arg("pin");
      // PINs résidents (sécurisés côté serveur)
      // Format : les 4 derniers chiffres du numéro de chambre, ou PIN personnalisé
      struct RoomPin { const char* room; const char* pin; };
      RoomPin roomPins[] = {
        {"F-024", "0024"}, {"F-025", "0025"}, {"F-026", "0026"},
        {"F-027", "0027"}, {"F-028", "0028"}, {"F-030", "0030"},
        {"F-032", "0032"}, {"F-033", "0033"}, {"F-034", "0034"},
        {"F-041", "0041"}, {"F-042", "0042"}
      };
      bool valid = false;
      for (auto& rp : roomPins) {
        if (room == rp.room && pin == rp.pin) { valid = true; break; }
      }
      if (valid) {
        server.send(200, "application/json", "{\"ok\":true}");
      } else {
        server.send(401, "application/json", "{\"ok\":false,\"msg\":\"Chambre ou PIN incorrect\"}");
      }
    }
    else {
      server.send(400, "application/json", "{\"ok\":false,\"msg\":\"Invalid role\"}");
    }
    return;
  }

  // Fichier depuis SD
  if (path == "/") path = "/index.html";

  String contentType = getContentType(path);

  if (SD.exists(path)) {
    File file = SD.open(path, FILE_READ);
    if (!file) {
      server.send(500, "text/plain", "Erreur lecture fichier");
      return;
    }
    server.streamFile(file, contentType);
    file.close();
  } else {
    server.send(404, "text/plain", "Fichier non trouvé: " + path);
  }
}

// ══════════════════════════════════════════════════════════════
// WEBSOCKET — COMMUNICATION AVEC LE DASHBOARD
// ══════════════════════════════════════════════════════════════

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED: {
      Serial.printf("[WS] Client #%u connecté\n", num);
      // Envoyer l'état actuel du robot
      JsonDocument doc;
      doc["action"] = "robot_status";
      doc["name"] = "Robot Alpha";
      
      if (missionState == NEEDS_CHARGE) doc["status"] = "En charge (Batterie faible)";
      else if (missionState == MISSION_IDLE) doc["status"] = "Disponible";
      else doc["status"] = "En mission";

      doc["battery"] = batteryLevel;
      if (missionState != MISSION_IDLE && missionState != NEEDS_CHARGE) doc["order_id"] = currentOrderId;
      String json;
      serializeJson(doc, json);
      webSocket.sendTXT(num, json);

      // Envoyer les infos de la carte si chargée
      if (mapLoaded) {
        JsonDocument mapDoc;
        mapDoc["action"] = "map_info";
        mapDoc["width"] = MAP_WIDTH;
        mapDoc["height"] = MAP_HEIGHT;
        mapDoc["cell_cm"] = (int)(METERS_PER_PIXEL * 100);
        String mapJson;
        serializeJson(mapDoc, mapJson);
        webSocket.sendTXT(num, mapJson);
      }
      break;
    }

    case WStype_DISCONNECTED:
      Serial.printf("[WS] Client #%u déconnecté\n", num);
      break;

    case WStype_TEXT: {
      JsonDocument doc;
      DeserializationError err = deserializeJson(doc, payload);
      if (err) {
        Serial.println("[WS] JSON invalide");
        break;
      }

      String action = doc["action"].as<String>();
      Serial.println("[WS] Action reçue : " + action);

      if (action == "hello") {
        Serial.println("[WS] Dashboard connecté");
      }
      else if (action == "new_order") {
        // Nouvelle commande depuis le dashboard
        String orderId = doc["order"]["id"].as<String>();
        String room = doc["order"]["room"].as<String>();
        String status = doc["order"]["status"].as<String>();
        Serial.printf("[WS] Commande %s pour Chambre %s (statut: %s)\n",
          orderId.c_str(), room.c_str(), status.c_str());

        // Si la commande est déjà "En cours", démarrer la mission
        if (status == "En cours") {
          startMission(orderId, room);
        }
      }
      else if (action == "kitchen_loaded") {
        // Personnel confirme que le plat est chargé
        kitchenLoaded();
      }
      else if (action == "order_cancelled") {
        // Commande annulée
        if (currentOrderId == doc["id"].as<String>()) {
          missionState = MISSION_IDLE;
          currentOrderId = "";
          sendToArduino("STOP");
          Serial.println("[MISSION] Annulée");
        }
      }
      break;
    }

    default:
      break;
  }
}

// ══════════════════════════════════════════════════════════════
// SETUP
// ══════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n╔══════════════════════════════════════╗");
  Serial.println("║   ROBOT DISTRIBUTEUR — ESP32 Maître  ║");
  Serial.println("╚══════════════════════════════════════╝\n");

  // 1. UART vers Arduino
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);
  Serial.println("[UART] Communication Arduino initialisée (TX=17, RX=16)");

  // 2. Carte SD
  spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, spiSD)) {
    Serial.println("[SD] ERREUR — Carte SD introuvable !");
    Serial.println("     Vérifiez : CS=4, MOSI=23, SCK=18, MISO=19");
  } else {
    Serial.printf("[SD] Carte SD OK (taille: %llu Mo)\n", SD.cardSize() / (1024 * 1024));
    mapLoaded = initMap();
  }

  // 3. WiFi
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(ssid_ap, password_ap);
  Serial.printf("[WiFi] AP démarré : %s → IP: %s\n", ssid_ap, WiFi.softAPIP().toString().c_str());

  WiFi.begin(ssid_sta, password_sta);
  Serial.printf("[WiFi] Connexion à %s", ssid_sta);
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 20) {
    delay(500);
    Serial.print(".");
    retries++;
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WiFi] Connecté ! IP locale: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("[WiFi] Pas de connexion internet (AP seul actif)");
  }

  // 4. Routes du serveur
  server.onNotFound(handleFileRead);
  server.begin();
  Serial.println("[WEB] Serveur HTTP démarré sur port 80");

  // 5. WebSocket
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("[WS] WebSocket démarré sur port 81");

  // 6. Seed random
  randomSeed(analogRead(0) + millis());

  // 7. Watchdog ESP32 (30 secondes)
  esp_task_wdt_config_t twdt_config = {
      .timeout_ms = 30000,
      .idle_core_mask = (1 << 2) - 1, // Monitor max 2 cores
      .trigger_panic = true,
  };
  esp_task_wdt_init(&twdt_config);
  esp_task_wdt_add(NULL);      // Ajouter la tâche courante

  Serial.println("\n[PRÊT] Robot distributeur opérationnel !\n");
}

// ══════════════════════════════════════════════════════════════
// LOOP
// ══════════════════════════════════════════════════════════════

unsigned long lastNavUpdate = 0;
unsigned long lastPosUpdate = 0;
unsigned long lastHeartbeat = 0;

void loop() {
  // Nourrir le watchdog ESP32
  esp_task_wdt_reset();

  // Gérer les connexions
  server.handleClient();
  webSocket.loop();

  // Lire les messages de l'Arduino
  readUART();

  // Lire le clavier 4x4 (actif uniquement en mode livraison)
  handleKeypad();

  unsigned long now = millis();

  // ── Heartbeat UART vers Arduino (toutes les 500ms) ──
  // Envoie un paquet STOP silencieux pour que le watchdog Arduino reste heureux
  if (now - lastHeartbeat > 500) {
    lastHeartbeat = now;
    if (missionState == MISSION_IDLE || missionState == WAITING_LOADING || 
        missionState == WAITING_DELIVERY || missionState == NEEDS_CHARGE) {
      // En état d'attente, envoyer un heartbeat (commande STOP)
      // Seulement si l'Arduino n'est pas en train d'éviter un obstacle
      if (distFront >= 20) { // Pas d'obstacle devant
        sendToArduinoBinary(5); // STOP = heartbeat silencieux
      }
    }
  }

  // ── Timeout de mission (10 min max) ──
  if (missionState != MISSION_IDLE && missionState != NEEDS_CHARGE) {
    if (now - missionStartTime > MISSION_TIMEOUT_MS) {
      Serial.println("[MISSION] ⚠ TIMEOUT — Mission annulée après 10 minutes");
      sendToArduino("STOP");

      // Nettoyer l'état de la commande
      String timedOutOrderId = currentOrderId;
      currentOrderId = "";
      currentRoom = "";
      deliveryCode = "";

      JsonDocument doc;
      doc["action"] = "mission_timeout";
      doc["id"] = timedOutOrderId;
      doc["msg"] = "Mission expirée (timeout 10 min)";
      String json;
      serializeJson(doc, json);
      webSocket.broadcastTXT(json);

      // Retourner à la cuisine
      missionState = RETURNING_HOME;
      missionStartTime = now; // Reset timer pour le retour
      sendToArduino("FWD");
    }
  }

  // Navigation : toutes les 200ms
  if (now - lastNavUpdate > 200) {
    lastNavUpdate = now;
    navigationStep();
  }

  // Envoyer la position au dashboard : toutes les 2s
  if (now - lastPosUpdate > 2000) {
    lastPosUpdate = now;
    if (missionState != MISSION_IDLE) {
      JsonDocument doc;
      doc["action"] = "robot_pos";
      doc["x"] = robotX_m / METERS_PER_PIXEL;  // en pixels pour la mini-carte
      doc["y"] = robotY_m / METERS_PER_PIXEL;
      doc["angle"] = robotAngle;
      String json;
      serializeJson(doc, json);
      webSocket.broadcastTXT(json);
    }
  }
}
