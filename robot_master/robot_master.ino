/**
 * robot_master.ino
 *
 * ESP32-WROOM — Firmware Maître du Robot Distributeur
 * VERSION MODIFIÉE :
 *   - Serveur Web SUPPRIMÉ
 *   - Bouton physique GPIO34 → commande directe chambre F-027
 *   - Logs de diagnostic WiFi/Caméra/Téléphone
 *
 * Fonctions :
 *   1. WebSocket (sans serveur de fichiers)
 *   2. Navigation via map.bin (plan binaire sur SD)
 *   3. Réception ArUco depuis le téléphone Android
 *   4. Communication UART avec Arduino Uno (esclave)
 *   5. Logique de décision : capteurs > map (sécurité d'abord)
 *   6. Gestion des commandes de livraison
 *   7. Clavier 4x4 membrane (vérification code de livraison)
 *   8. NOUVEAU : Bouton physique GPIO34 → commande F-027
 *
 * Câblage SD (SPI) :
 *   CS=GPIO4  MOSI=GPIO23  SCK=GPIO18  MISO=GPIO19
 *
 * Câblage UART vers Arduino :
 *   ESP32 TX2 (GPIO17) → Arduino RX (pin 0)
 *   ESP32 RX2 (GPIO16) → Arduino TX (pin 1)
 *
 * Câblage Clavier 4x4 :
 *   Rangées  : GPIO 32, 33, 15, 26
 *   Colonnes : GPIO 27, 14, 12, 13
 *
 * NOUVEAU — Câblage Bouton commande :
 *   GPIO34 → Bouton → GND
 *   (GPIO34 est INPUT uniquement sur ESP32, résistance pull-up externe 10kΩ vers 3.3V recommandée)
 */

#include "AStarPathfinder.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Keypad.h>
#include <SD.h>
#include <SPI.h>
// WebServer SUPPRIMÉ
#include <WebSocketsServer.h>
#include <WiFi.h>
#include <esp_task_wdt.h>

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
#define SD_CS 4
#define SD_MOSI 23
#define SD_SCK 18
#define SD_MISO 19
SPIClass spiSD(VSPI);

// --- WiFi Access Point ---
const char *ssid_ap = "Robot-WiFi";
const char *password_ap = "12345678";

// --- WiFi Station (optionnel, pour internet) ---
const char *ssid_sta = "Mt";
const char *password_sta = "amin@2025";

// --- NOUVEAU : Bouton commande F-027 ---
#define BOUTON_COMMANDE 34  // GPIO34 (INPUT uniquement, pull-up externe 10kΩ vers 3.3V)
#define BOUTON_DEBOUNCE_MS 50

// --- WebSocket uniquement (plus de WebServer) ---
WebSocketsServer webSocket(81);

// --- UART vers Arduino ---
#define UART_TX 17
#define UART_RX 16
#define UART_BAUD 115200

// --- Clavier 4x4 ---
const byte KP_ROWS = 4;
const byte KP_COLS = 4;

char kpKeys[KP_ROWS][KP_COLS] = {{'1', '2', '3', 'A'},
                                 {'4', '5', '6', 'B'},
                                 {'7', '8', '9', 'C'},
                                 {'*', '0', '#', 'D'}};

byte kpRowPins[KP_ROWS] = {32, 33, 15, 26};
byte kpColPins[KP_COLS] = {27, 14, 12, 13};

Keypad keypad =
    Keypad(makeKeymap(kpKeys), kpRowPins, kpColPins, KP_ROWS, KP_COLS);
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

float robotX_m = 0.0;
float robotY_m = 0.0;
float robotAngle = 0.0;

int distFront = 200, distBack = 200, distLeft = 200, distRight = 200;
bool platPresent = false;

enum MissionState {
  MISSION_IDLE,
  GOING_TO_KITCHEN,
  WAITING_LOADING,
  GOING_TO_ROOM,
  WAITING_DELIVERY,
  RETURNING_HOME,
  NEEDS_CHARGE
};

MissionState missionState = MISSION_IDLE;
int batteryLevel = 100;
String currentOrderId = "";
String currentRoom = "";
String deliveryCode = "";
unsigned long missionStartTime = 0;
#define MISSION_TIMEOUT_MS (10UL * 60 * 1000)

struct QueuedOrder {
  String orderId;
  String room;
};
#define MAX_QUEUE 5
QueuedOrder orderQueue[MAX_QUEUE];
int queueCount = 0;

struct ArUcoMarker {
  int id;
  float x_m;
  float y_m;
  float angle_mur;
};

ArUcoMarker balises[] = {
    {15, 7.45, 0.86, 0.0},
    {16, 7.39, 1.31, 0.0},
    {13, 8.16, 14.28, 0.0},
    {14, 8.08, 17.73, 0.0},
    {12, 7.97, 21.30, 0.0},
    {11, 18.39, 36.88, 0.0},
    {10, 15.34, 51.49, 0.0},
    {8, 13.22, 54.30, 0.0},
    {9, 3.45, 54.25, 0.0},
    {7, 1.34, 41.77, 0.0},
    {5, 1.48, 48.54, 0.0},
    {6, 15.41, 44.43, 0.0},
    {4, 1.48, 51.21, 0.0},
    {2, 1.48, 48.54, 0.0},
    {3, 15.07, 48.71, 0.0},
    {1, 1.44, 51.32, 0.0},
    {17, 7.78, 6.26, 0.0},
    {18, 8.10, 11.16, 0.0},
    {19, 7.89, 20.45, 0.0},
    {20, 8.69, 34.85, 0.0},
    {21, 10.86, 35.64, 0.0},
    {22, 16.76, 35.79, 0.0},
    {23, 10.99, 37.02, 0.0},
    {24, 15.20, 38.33, 0.0},
    {25, 15.26, 53.16, 0.0},
    {26, 15.10, 35.89, 0.0},
    {27, 1.29, 55.49, 0.0},
    {28, 1.34, 39.31, 0.0},
    {29, 1.46, 5.90, 0.0},
};
const int NB_BALISES = sizeof(balises) / sizeof(balises[0]);

const int ARUCO_HOME = 0;
const int ARUCO_KITCHEN = 8;
int targetArUco = -1;

struct RoomArUco {
  String roomName;
  int arucoId;
};

RoomArUco roomRegistry[] = {{"F-041", 15}, {"F-042", 16}, {"F-034", 13},
                            {"F-033", 12}, {"F-032", 11}, {"F-030", 10},
                            {"F-028", 7},  {"F-027", 5},  {"F-026", 4},
                            {"F-025", 2},  {"F-024", 1}};
const int NB_ROOMS = sizeof(roomRegistry) / sizeof(roomRegistry[0]);

// ══════════════════════════════════════════════════════════════
// DIAGNOSTIC WIFI — Vérifie qui est connecté au hotspot
// ══════════════════════════════════════════════════════════════

void printWiFiDiagnostic() {
  Serial.println("\n╔══════════════════════════════════════════════╗");
  Serial.println("║          DIAGNOSTIC WiFi / RÉSEAU            ║");
  Serial.println("╠══════════════════════════════════════════════╣");

  // --- Point d'accès (AP) ---
  Serial.printf("║ AP SSID      : %-29s║\n", ssid_ap);
  Serial.printf("║ AP IP        : %-29s║\n", WiFi.softAPIP().toString().c_str());
  Serial.printf("║ AP Canal     : %-29d║\n", WiFi.channel());

  int nbClients = WiFi.softAPgetStationNum();
  Serial.printf("║ Clients AP   : %-29d║\n", nbClients);

  // --- Station (STA) ---
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("║ STA SSID     : %-29s║\n", ssid_sta);
    Serial.printf("║ STA IP       : %-29s║\n", WiFi.localIP().toString().c_str());
    Serial.printf("║ STA RSSI     : %-26s dBm║\n", String(WiFi.RSSI()).c_str());
  } else {
    Serial.println("║ STA          : NON CONNECTÉ (pas d'internet) ║");
  }

  Serial.println("╠══════════════════════════════════════════════╣");
  Serial.println("║ CONNEXIONS ATTENDUES :                       ║");
  Serial.println("║  • Téléphone Android → AP 'Robot-WiFi'       ║");
  Serial.println("║    Envoie ArUco via GET /aruco?id=XX          ║");
  Serial.println("║    (WebSocket port 81)                        ║");
  Serial.println("║  • Caméra IP → AP 'Robot-WiFi' (si WiFi cam) ║");
  Serial.println("╠══════════════════════════════════════════════╣");

  if (nbClients == 0) {
    Serial.println("║ ⚠  AUCUN APPAREIL CONNECTÉ au hotspot !      ║");
    Serial.println("║    → Connectez le tél à 'Robot-WiFi'          ║");
  } else {
    Serial.printf("║ ✓  %d appareil(s) connecté(s) au hotspot      ║\n", nbClients);
    Serial.println("║    (Téléphone et/ou caméra)                   ║");
  }

  Serial.println("╚══════════════════════════════════════════════╝\n");
}

// ══════════════════════════════════════════════════════════════
// MAP.BIN — NAVIGATION
// ══════════════════════════════════════════════════════════════

bool initMap() {
  File file = SD.open(MAP_FILENAME);
  if (!file) {
    Serial.println("[MAP] map.bin introuvable sur SD");
    return false;
  }
  file.read((uint8_t *)&MAP_WIDTH, 4);
  file.read((uint8_t *)&MAP_HEIGHT, 4);
  bytesPerRow = (MAP_WIDTH + 7) / 8;

  float realW = MAP_WIDTH * METERS_PER_PIXEL;
  float realH = MAP_HEIGHT * METERS_PER_PIXEL;
  Serial.printf("[MAP] Carte chargée : %ux%u pixels, %.2fx%.2f mètres\n",
                MAP_WIDTH, MAP_HEIGHT, realW, realH);

  if (allocateCoarseMap(realW, realH)) {
    for (int cy = 0; cy < coarseH; cy++) {
      for (int cx = 0; cx < coarseW; cx++) {
        int px = (int)((cx * CELL_SIZE_M) / METERS_PER_PIXEL);
        int py = (int)((cy * CELL_SIZE_M) / METERS_PER_PIXEL);
        long pos = 8 + ((long)py * bytesPerRow) + (px / 8);
        if (file.seek(pos)) {
          uint8_t b = file.read();
          if ((b >> (7 - (px % 8))) & 0x01)
            setCoarseObstacle(cx, cy, true);
        }
      }
    }
    Serial.println("[MAP] Coarse map générée en RAM");
  }
  file.close();
  return true;
}

// ══════════════════════════════════════════════════════════════
// ARUCO — RECALIBRATION DE POSITION
// ══════════════════════════════════════════════════════════════

void handleArUcoDetection(int arucoId) {
  Serial.printf("[ARUCO] ✓ Balise #%d reçue (téléphone → ESP32 OK)\n", arucoId);

  bool knownMarker = false;
  for (int i = 0; i < NB_BALISES; i++) {
    if (balises[i].id == arucoId) {
      float oldX = robotX_m, oldY = robotY_m;
      robotX_m = balises[i].x_m;
      robotY_m = balises[i].y_m;
      robotAngle = balises[i].angle_mur;

      Serial.printf("[ARUCO] Position recalibrée: (%.2f, %.2f) → (%.2f, %.2f)\n",
                    oldX, oldY, robotX_m, robotY_m);

      JsonDocument doc;
      doc["action"] = "robot_pos";
      doc["x"] = robotX_m;
      doc["y"] = robotY_m;
      doc["angle"] = robotAngle;
      doc["aruco"] = arucoId;
      String json;
      serializeJson(doc, json);
      webSocket.broadcastTXT(json);

      Serial2.println("ARUCO:" + String(arucoId));
      knownMarker = true;
      break;
    }
  }

  if (!knownMarker) {
    Serial.printf("[ARUCO] Balise #%d non utilisée pour recalibration XY\n", arucoId);
  }

  if (missionState == GOING_TO_KITCHEN && arucoId == ARUCO_KITCHEN) {
    arrivedAtKitchen();
  } else if (missionState == GOING_TO_ROOM && arucoId == targetArUco) {
    arrivedAtRoom();
  } else if (missionState == RETURNING_HOME &&
             (arucoId == ARUCO_HOME || arucoId == ARUCO_KITCHEN)) {
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
  Serial2.write((uint8_t *)&cmd, sizeof(CommandPacket));
}

void sendToArduino(String cmd) {
  if (cmd == "FWD")         sendToArduinoBinary(1);
  else if (cmd == "BACK")   sendToArduinoBinary(2);
  else if (cmd == "LEFT")   sendToArduinoBinary(3);
  else if (cmd == "RIGHT")  sendToArduinoBinary(4);
  else if (cmd == "STOP")   sendToArduinoBinary(5);
  else if (cmd.startsWith("UNLOCK:"))
    sendToArduinoBinary(6, cmd.substring(7).toInt());
  else if (cmd == "SERVO:OPEN")  sendToArduinoBinary(7);
  else if (cmd == "SERVO:CLOSE") sendToArduinoBinary(8);
  else if (cmd.startsWith("LCD:")) {
    String txt = cmd.substring(4);
    if      (txt == "Code OK!")      sendToArduinoBinary(9, 1);
    else if (txt == "Code faux!")    sendToArduinoBinary(9, 2);
    else if (txt == "Entrez le code") sendToArduinoBinary(9, 3);
    else if (txt.startsWith("Code: ")) sendToArduinoBinary(9, 4);
  }
}

void processArduinoMessageBinary(SensorPacket &sp) {
  distLeft  = sp.distLeft;
  distRight = sp.distRight;
  distFront = sp.distFront;
  distBack  = sp.distBack;

  float currentImuRaw = sp.imuAngle;
  static float lastImuRaw = 0.0;
  float deltaAngle = currentImuRaw - lastImuRaw;
  lastImuRaw = currentImuRaw;
  robotAngle += deltaAngle;
  while (robotAngle >= 360.0) robotAngle -= 360.0;
  while (robotAngle < 0.0)    robotAngle += 360.0;

  float distanceAvance_m = sp.deltaDist_cm / 100.0;
  robotX_m += distanceAvance_m * cos(robotAngle * PI / 180.0);
  robotY_m += distanceAvance_m * sin(robotAngle * PI / 180.0);

  platPresent = (sp.buttonState == 1);

  if (sp.event == 1)
    Serial.println("[SÉCURITÉ] Obstacle !");
  else if (sp.event == 2) {
    Serial.println("[SÉCURITÉ] Porte ouverte !");
    if (missionState == WAITING_DELIVERY && !platPresent)
      deliveryConfirmed();
  }
}

void readUART() {
  if (Serial2.available() >= (int)sizeof(SensorPacket)) {
    if (Serial2.peek() == 0xBB) {
      SensorPacket sp;
      Serial2.readBytes((uint8_t *)&sp, sizeof(SensorPacket));
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
  if (missionState == MISSION_IDLE || missionState == WAITING_LOADING ||
      missionState == WAITING_DELIVERY || missionState == NEEDS_CHARGE) {
    return;
  }
  if (distFront < 20) return;

  float targetX = 0, targetY = 0;
  bool foundTarget = false;
  int aid = (missionState == RETURNING_HOME) ? ARUCO_KITCHEN : targetArUco;

  for (int i = 0; i < NB_BALISES; i++) {
    if (balises[i].id == aid) {
      targetX = balises[i].x_m;
      targetY = balises[i].y_m;
      foundTarget = true;
      break;
    }
  }

  if (!foundTarget) {
    sendToArduino("FWD");
    return;
  }

  static unsigned long lastAstar = 0;
  if (millis() - lastAstar > 2000) {
    lastAstar = millis();
    computeAStarPath(robotX_m, robotY_m, targetX, targetY);
  }

  if (pathLength > 1 && currentPathIdx < pathLength) {
    float wpX = currentPath[currentPathIdx].x * CELL_SIZE_M;
    float wpY = currentPath[currentPathIdx].y * CELL_SIZE_M;

    float dx = wpX - robotX_m;
    float dy = wpY - robotY_m;
    float dist = sqrt(dx * dx + dy * dy);

    if (dist < 0.3) {
      currentPathIdx++;
    } else {
      float angleCible = atan2(dy, dx) * 180.0 / PI;
      float erreurAngle = angleCible - robotAngle;
      while (erreurAngle > 180.0)  erreurAngle -= 360.0;
      while (erreurAngle < -180.0) erreurAngle += 360.0;

      if (abs(erreurAngle) > 20.0) {
        sendToArduino(erreurAngle > 0 ? "RIGHT" : "LEFT");
      } else {
        sendToArduino("FWD");
      }
    }
  } else {
    sendToArduino("FWD");
  }
}

// ══════════════════════════════════════════════════════════════
// GESTION DES COMMANDES
// ══════════════════════════════════════════════════════════════

String generateCode() {
  String code = "";
  for (int i = 0; i < 6; i++) code += String(random(0, 10));
  return code;
}

void startMission(String orderId, String room) {
  if (missionState != MISSION_IDLE) {
    if (queueCount < MAX_QUEUE) {
      orderQueue[queueCount++] = {orderId, room};
      Serial.printf("[QUEUE] Commande %s ajoutée (position: %d)\n",
                    orderId.c_str(), queueCount);

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

  targetArUco = -1;
  for (int i = 0; i < NB_ROOMS; i++) {
    if (roomRegistry[i].roomName == room) {
      targetArUco = roomRegistry[i].arucoId;
      break;
    }
  }
  if (targetArUco == -1) targetArUco = room.toInt();

  missionState = GOING_TO_KITCHEN;
  missionStartTime = millis();

  Serial.printf("[MISSION] ▶ Démarrage : %s → Chambre %s (ArUco: %d, code: %s)\n",
                orderId.c_str(), room.c_str(), targetArUco, deliveryCode.c_str());

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

  JsonDocument codeDoc;
  codeDoc["action"] = "order_code";
  codeDoc["order_id"] = orderId;
  codeDoc["code"] = deliveryCode;
  codeDoc["room"] = room;
  String codeJson;
  serializeJson(codeDoc, codeJson);
  webSocket.broadcastTXT(codeJson);

  sendToArduino("FWD");
}

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

void kitchenLoaded() {
  missionState = GOING_TO_ROOM;
  Serial.printf("[MISSION] Plat chargé → En route vers Chambre %s\n", currentRoom.c_str());
  sendToArduino("FWD");
}

void arrivedAtRoom() {
  missionState = WAITING_DELIVERY;
  sendToArduino("STOP");
  keypadBuffer = "";

  Serial.printf("[MISSION] Arrivé à Chambre %s — Code: %s\n",
                currentRoom.c_str(), deliveryCode.c_str());

  sendToArduino("UNLOCK:" + deliveryCode);

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
  if (missionState != WAITING_DELIVERY) return;

  char key = keypad.getKey();
  if (!key) return;

  Serial.printf("[CLAVIER] Touche: %c\n", key);

  if (key == '#') {
    keypadBuffer = "";
    sendToArduino("LCD:Entrez le code");
    Serial.println("[CLAVIER] Saisie effacée");
  } else if (key == '*') {
    if (keypadBuffer == deliveryCode) {
      Serial.println("[CLAVIER] ✓ Code correct !");
      sendToArduino("LCD:Code OK!");
      sendToArduino("SERVO:OPEN");
    } else {
      Serial.println("[CLAVIER] ✗ Code incorrect");
      sendToArduino("LCD:Code faux!");
      keypadBuffer = "";
      delay(1500);
      sendToArduino("LCD:Entrez le code");
    }
  } else if (key >= '0' && key <= '9') {
    if (keypadBuffer.length() < 6) {
      keypadBuffer += key;
      String stars = "";
      for (int i = 0; i < (int)keypadBuffer.length(); i++) stars += "*";
      sendToArduino("LCD:Code: " + stars);
    }
  }
}

void deliveryConfirmed() {
  Serial.printf("[MISSION] ✓ Livraison confirmée pour %s\n", currentOrderId.c_str());

  JsonDocument doc;
  doc["action"] = "delivery_confirmed";
  doc["id"] = currentOrderId;
  doc["status"] = "Livrée";
  String json;
  serializeJson(doc, json);
  webSocket.broadcastTXT(json);

  batteryLevel -= 20;
  if (batteryLevel < 0) batteryLevel = 0;

  missionState = RETURNING_HOME;
  sendToArduino("FWD");
}

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

    JsonDocument alertDoc;
    alertDoc["action"] = "alert";
    alertDoc["msg"] = "Robot déchargé ! Veuillez le brancher à la cuisine.";
    String alertJson;
    serializeJson(alertDoc, alertJson);
    webSocket.broadcastTXT(alertJson);

    Serial.println("[MISSION] De retour — BATTERIE FAIBLE, robot en charge");
  } else {
    missionState = MISSION_IDLE;
    doc["status"] = "Disponible";
    Serial.println("[MISSION] De retour — Robot disponible");
  }

  String json;
  serializeJson(doc, json);
  webSocket.broadcastTXT(json);

  if (missionState == MISSION_IDLE && queueCount > 0) {
    QueuedOrder next = orderQueue[0];
    for (int i = 0; i < queueCount - 1; i++) orderQueue[i] = orderQueue[i + 1];
    queueCount--;
    Serial.printf("[QUEUE] Lancement auto de %s (restant: %d)\n",
                  next.orderId.c_str(), queueCount);
    startMission(next.orderId, next.room);
  }
}

// ══════════════════════════════════════════════════════════════
// NOUVEAU — BOUTON PHYSIQUE : commande directe → F-027
// ══════════════════════════════════════════════════════════════

void handleBoutonCommande() {
  static bool dernierEtat = HIGH;
  static unsigned long dernierChangement = 0;

  bool etatActuel = digitalRead(BOUTON_COMMANDE);
  unsigned long now = millis();

  // Anti-rebond
  if (etatActuel == dernierEtat) return;
  if (now - dernierChangement < BOUTON_DEBOUNCE_MS) return;

  dernierChangement = now;
  dernierEtat = etatActuel;

  // Front descendant = bouton appuyé (pull-up → GND)
  if (etatActuel == LOW) {
    Serial.println("\n[BOUTON] ═══════════════════════════════════");
    Serial.println("[BOUTON] ▶ Bouton pressé → Commande F-027 !");
    Serial.println("[BOUTON] ═══════════════════════════════════");

    String orderId = "BTN-" + String(millis());
    startMission(orderId, "F-027");
  }
}

// ══════════════════════════════════════════════════════════════
// WEBSOCKET — COMMUNICATION AVEC LE DASHBOARD
// ══════════════════════════════════════════════════════════════

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
  case WStype_CONNECTED: {
    Serial.printf("[WS] ✓ Client #%u connecté (IP: %s)\n", num,
                  webSocket.remoteIP(num).toString().c_str());

    // Log diagnostic : qui se connecte ?
    Serial.printf("[WS] → Vérifiez que c'est bien votre téléphone/dashboard\n");

    JsonDocument doc;
    doc["action"] = "robot_status";
    doc["name"] = "Robot Alpha";

    if (missionState == NEEDS_CHARGE)    doc["status"] = "En charge (Batterie faible)";
    else if (missionState == MISSION_IDLE) doc["status"] = "Disponible";
    else                                  doc["status"] = "En mission";

    doc["battery"] = batteryLevel;
    if (missionState != MISSION_IDLE && missionState != NEEDS_CHARGE)
      doc["order_id"] = currentOrderId;
    String json;
    serializeJson(doc, json);
    webSocket.sendTXT(num, json);

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
    Serial.printf("[WS] ✗ Client #%u déconnecté\n", num);
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
    } else if (action == "new_order") {
      String orderId = doc["order"]["id"].as<String>();
      String room    = doc["order"]["room"].as<String>();
      String status  = doc["order"]["status"].as<String>();
      Serial.printf("[WS] Commande %s pour Chambre %s (statut: %s)\n",
                    orderId.c_str(), room.c_str(), status.c_str());
      if (status == "En cours") startMission(orderId, room);
    } else if (action == "kitchen_loaded") {
      kitchenLoaded();
    } else if (action == "order_cancelled") {
      if (currentOrderId == doc["id"].as<String>()) {
        missionState = MISSION_IDLE;
        currentOrderId = "";
        sendToArduino("STOP");
        Serial.println("[MISSION] Annulée");
      }
    }
    // NOUVEAU : réception ArUco via WebSocket (alternative au GET HTTP)
    else if (action == "aruco") {
      int arucoId = doc["id"].as<int>();
      Serial.printf("[WS] ArUco reçu via WebSocket : #%d\n", arucoId);
      handleArUcoDetection(arucoId);
    }
    break;
  }

  default:
    break;
  }
}

// ══════════════════════════════════════════════════════════════
// ENDPOINT ARUCO — géré directement via WebSocket maintenant
// (Le serveur HTTP est supprimé, le téléphone envoie via WS)
// Pour compatibilité, on garde aussi un mini-serveur TCP sur port 82
// si le téléphone utilise encore des requêtes HTTP GET /aruco?id=X
// ══════════════════════════════════════════════════════════════

// Mini-serveur TCP léger pour recevoir les GET /aruco?id=X du téléphone
// sans avoir besoin de la librairie WebServer complète
WiFiServer arucoServer(80);

void handleArucoHTTP() {
  WiFiClient client = arucoServer.available();
  if (!client) return;

  unsigned long timeout = millis() + 200;
  String request = "";

  while (client.connected() && millis() < timeout) {
    if (client.available()) {
      char c = client.read();
      request += c;
      if (request.endsWith("\r\n\r\n")) break;
    }
  }

  // Parser "GET /aruco?id=XX"
  if (request.startsWith("GET /aruco")) {
    int idxId = request.indexOf("id=");
    if (idxId >= 0) {
      int idxEnd = request.indexOf(' ', idxId);
      String idStr = request.substring(idxId + 3, idxEnd);
      int arucoId = idStr.toInt();

      Serial.printf("[HTTP] ✓ ArUco #%d reçu via HTTP GET (téléphone connecté !)\n", arucoId);
      handleArUcoDetection(arucoId);

      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: text/plain");
      client.println("Connection: close");
      client.println();
      client.println("OK");
    }
  } else if (request.startsWith("GET /status")) {
    JsonDocument doc;
    doc["state"]  = (int)missionState;
    doc["x"]      = robotX_m;
    doc["y"]      = robotY_m;
    doc["angle"]  = robotAngle;
    doc["plat"]   = platPresent;
    doc["order"]  = currentOrderId;
    doc["room"]   = currentRoom;
    doc["battery"] = batteryLevel;
    String json;
    serializeJson(doc, json);

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: application/json");
    client.println("Connection: close");
    client.println();
    client.println(json);
  } else {
    client.println("HTTP/1.1 404 Not Found");
    client.println("Connection: close");
    client.println();
    client.println("Not Found");
  }

  client.stop();
}

// ══════════════════════════════════════════════════════════════
// SETUP
// ══════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n╔══════════════════════════════════════════╗");
  Serial.println("║  ROBOT DISTRIBUTEUR — ESP32 Maître v2.0  ║");
  Serial.println("║  (Sans site web — Bouton F-027 activé)   ║");
  Serial.println("╚══════════════════════════════════════════╝\n");

  // 1. UART vers Arduino
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);
  Serial.println("[UART] Arduino initialisé (TX=17, RX=16)");

  // 2. NOUVEAU : Bouton commande GPIO34
  pinMode(BOUTON_COMMANDE, INPUT);  // GPIO34 = INPUT uniquement (pas de pull-up interne)
  Serial.println("[BOUTON] GPIO34 configuré (pull-up externe 10kΩ requis)");

  // 3. Carte SD
  spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, spiSD)) {
    Serial.println("[SD] ERREUR — Carte SD introuvable !");
  } else {
    Serial.printf("[SD] Carte SD OK (%llu Mo)\n", SD.cardSize() / (1024 * 1024));
  }

  // 4. WiFi
  WiFi.mode(WIFI_AP_STA);

  WiFi.begin(ssid_sta, password_sta);
  Serial.printf("[WiFi] Connexion à '%s'", ssid_sta);
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 10) {
    delay(500);
    Serial.print(".");
    retries++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WiFi STA] ✓ Connecté ! IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("[WiFi STA] ✗ Pas de connexion internet (AP seul)");
  }

  WiFi.softAP(ssid_ap, password_ap);
  Serial.printf("[WiFi AP] ✓ Hotspot '%s' démarré → IP: %s\n",
                ssid_ap, WiFi.softAPIP().toString().c_str());
  Serial.printf("[WiFi AP] Canal: %d\n", WiFi.channel());

  // 5. Mini-serveur HTTP pour ArUco (port 80)
  arucoServer.begin();
  Serial.println("[HTTP] Mini-serveur ArUco sur port 80 (GET /aruco?id=X)");

  // 6. WebSocket
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("[WS] WebSocket sur port 81");

  // 7. Seed random
  randomSeed(analogRead(0) + millis());

  // 8. Désactiver watchdog avant initMap
  esp_task_wdt_delete(NULL);

  // 9. Chargement carte
  mapLoaded = initMap();

  // 10. Diagnostic WiFi initial
  printWiFiDiagnostic();

  Serial.println("\n╔══════════════════════════════════════════╗");
  Serial.println("║  PRÊT ! Appuyez sur GPIO34 → F-027       ║");
  Serial.println("╚══════════════════════════════════════════╝\n");
}

// ══════════════════════════════════════════════════════════════
// LOOP
// ══════════════════════════════════════════════════════════════

unsigned long lastNavUpdate   = 0;
unsigned long lastPosUpdate   = 0;
unsigned long lastHeartbeat   = 0;
unsigned long lastDiagnostic  = 0;

void loop() {
  // Gérer les connexions WebSocket
  webSocket.loop();

  // Mini-serveur HTTP ArUco
  handleArucoHTTP();

  // Lire les messages de l'Arduino
  readUART();

  // Lire le clavier 4x4
  handleKeypad();

  // NOUVEAU : Lire le bouton commande F-027
  handleBoutonCommande();

  unsigned long now = millis();

  // Heartbeat UART (toutes les 500ms)
  if (now - lastHeartbeat > 500) {
    lastHeartbeat = now;
    if (missionState == MISSION_IDLE || missionState == WAITING_LOADING ||
        missionState == WAITING_DELIVERY || missionState == NEEDS_CHARGE) {
      if (distFront >= 20) sendToArduinoBinary(5);
    }
  }

  // Timeout de mission (10 min)
  if (missionState != MISSION_IDLE && missionState != NEEDS_CHARGE) {
    if (now - missionStartTime > MISSION_TIMEOUT_MS) {
      Serial.println("[MISSION] ⚠ TIMEOUT — Mission annulée après 10 minutes");
      sendToArduino("STOP");

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

      missionState = RETURNING_HOME;
      missionStartTime = now;
      sendToArduino("FWD");
    }
  }

  // Navigation (toutes les 200ms)
  if (now - lastNavUpdate > 200) {
    lastNavUpdate = now;
    navigationStep();
  }

  // Position vers dashboard (toutes les 2s)
  if (now - lastPosUpdate > 2000) {
    lastPosUpdate = now;
    if (missionState != MISSION_IDLE) {
      JsonDocument doc;
      doc["action"] = "robot_pos";
      doc["x"]      = robotX_m / METERS_PER_PIXEL;
      doc["y"]      = robotY_m / METERS_PER_PIXEL;
      doc["angle"]  = robotAngle;
      String json;
      serializeJson(doc, json);
      webSocket.broadcastTXT(json);
    }
  }

  // ── Diagnostic WiFi périodique (toutes les 30s) ──
  // Affiche combien d'appareils sont connectés au hotspot
  if (now - lastDiagnostic > 30000) {
    lastDiagnostic = now;
    int nbClients = WiFi.softAPgetStationNum();
    Serial.printf("[DIAG] Hotspot '%s' : %d appareil(s) connecté(s)\n",
                  ssid_ap, nbClients);
    if (nbClients == 0) {
      Serial.println("[DIAG] ⚠  Téléphone et/ou caméra non connectés !");
      Serial.println("[DIAG]    → Connectez-les au réseau 'Robot-WiFi'");
    } else {
      Serial.printf("[DIAG] ✓  %d appareil(s) sur le hotspot\n", nbClients);
    }
  }
}
