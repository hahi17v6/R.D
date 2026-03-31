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
#define UART_BAUD 9600

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
  IDLE,              // Pas de mission
  GOING_TO_KITCHEN,  // En route vers la cuisine
  WAITING_LOADING,   // À la cuisine, attend chargement
  GOING_TO_ROOM,     // En route vers la chambre
  WAITING_DELIVERY,  // Arrivé, attend que le résident prenne le plat
  RETURNING_HOME,    // Retour au point de départ (cuisine)
  NEEDS_CHARGE       // Batterie faible, à la cuisine, en charge
};

MissionState missionState = IDLE;
int batteryLevel = 100; // Niveau de batterie simulé (%)
String currentOrderId = "";
String currentRoom = "";
String deliveryCode = "";

// ── Balises ArUco connues (coordonnées en mètres) ──
struct ArUcoMarker {
  int id;
  float x_m;
  float y_m;
  float angle_mur;  // angle du mur en degrés
};

ArUcoMarker balises[] = {
  {1, 3.98, 23.61, 90.0},
  {2, 15.20, 50.4, 45.0},
  // Ajoute tes balises ici quand tu les places physiquement
};
const int NB_BALISES = sizeof(balises) / sizeof(balises[0]);

// --- Cibles ArUco de Mission ---
const int ARUCO_HOME = 0;     // ID ArUco du point de départ
const int ARUCO_KITCHEN = 50; // ID ArUco de la cuisine
int targetArUco = -1;         // ID ArUco de la chambre de destination actuelle

struct RoomArUco {
  String roomName;
  int arucoId;
};

// Registre associant le nom de la chambre à son ID ArUco spécifique
RoomArUco roomRegistry[] = {
  {"101", 101},
  {"103", 103},
  {"205", 205},
  {"207", 207},
  {"302", 302}
  // Ajoute d'autres chambres ici
};
const int NB_ROOMS = sizeof(roomRegistry) / sizeof(roomRegistry[0]);

// ══════════════════════════════════════════════════════════════
// MAP.BIN — NAVIGATION
// ══════════════════════════════════════════════════════════════

bool initMap() {
  File file = SD.open(MAP_FILENAME);
  if (!file) {
    Serial.println("[MAP] map.bin introuvable sur SD");
    return false;
  }
  file.read((uint8_t*)&MAP_WIDTH, 4);
  file.read((uint8_t*)&MAP_HEIGHT, 4);
  file.close();
  bytesPerRow = (MAP_WIDTH + 7) / 8;
  Serial.printf("[MAP] Carte chargée : %ux%u pixels, %.2fx%.2f mètres\n",
    MAP_WIDTH, MAP_HEIGHT,
    MAP_WIDTH * METERS_PER_PIXEL,
    MAP_HEIGHT * METERS_PER_PIXEL);
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
      StaticJsonDocument<256> doc;
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

String uartBuffer = "";

void sendToArduino(String cmd) {
  Serial2.println(cmd);
  Serial.println("[UART→] " + cmd);
}

void processArduinoMessage(String msg) {
  msg.trim();
  if (msg.length() == 0) return;

  Serial.println("[UART←] " + msg);

  // Format: SENSORS:L,R,F,B,ANGLE (l'angle est optionnel/nouveau)
  if (msg.startsWith("SENSORS:")) {
    String data = msg.substring(8);
    float vals[5] = {0, 0, 0, 0, 0};
    int idx = 0;
    int start = 0;
    for (int i = 0; i <= (int)data.length() && idx < 5; i++) {
      if (i == (int)data.length() || data[i] == ',') {
        vals[idx++] = data.substring(start, i).toFloat();
        start = i + 1;
      }
    }
    if (idx >= 4) {
      distLeft  = (int)vals[0];
      distRight = (int)vals[1];
      distFront = (int)vals[2];
      distBack  = (int)vals[3];
    }
    if (idx == 5) {
      float currentImuRaw = vals[4];
      static float lastImuRaw = 0.0;
      float deltaAngle = currentImuRaw - lastImuRaw;
      lastImuRaw = currentImuRaw;
      
      // Intégrer la variation du gyroscope dans l'angle global
      robotAngle += deltaAngle;
      
      // Garder l'angle entre 0 et 360
      while (robotAngle >= 360.0) robotAngle -= 360.0;
      while (robotAngle < 0.0) robotAngle += 360.0;
    }
  }
  // Format: BUTTON:0 ou BUTTON:1
  else if (msg.startsWith("BUTTON:")) {
    platPresent = (msg.substring(7).toInt() == 1);
    Serial.printf("[BUTTON] Plat %s\n", platPresent ? "PRÉSENT" : "ABSENT");

    // Si on attendait la livraison et le plat a été retiré
    if (missionState == WAITING_DELIVERY && !platPresent) {
      deliveryConfirmed();
    }
  }
  // Format: OBSTACLE
  else if (msg == "OBSTACLE") {
    Serial.println("[SÉCURITÉ] Obstacle détecté par Arduino → Arrêt");
    // L'Arduino gère déjà l'arrêt, on met à jour l'état
  }
  // Format: DOOR_OPENED
  else if (msg == "DOOR_OPENED") {
    Serial.println("[SÉCURITÉ] Porte du casier ouverte par le résident");
  }
  // Format: ARRIVED
  else if (msg == "ARRIVED") {
    Serial.println("[NAV] Arduino confirme : position atteinte");
  }
}

void readUART() {
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      processArduinoMessage(uartBuffer);
      uartBuffer = "";
    } else if (c != '\r') {
      uartBuffer += c;
    }
  }
}

// ══════════════════════════════════════════════════════════════
// LOGIQUE DE NAVIGATION
// ══════════════════════════════════════════════════════════════

// Décide de la prochaine action basée sur la map + état
void navigationStep() {
  if (missionState == IDLE || missionState == WAITING_LOADING || missionState == WAITING_DELIVERY || missionState == NEEDS_CHARGE) {
    return;  // Pas de mouvement nécessaire
  }

  // SÉCURITÉ D'ABORD : Si obstacle devant, on s'arrête (l'Arduino gère ça aussi)
  if (distFront < 20) {
    // L'Arduino a déjà stoppé les moteurs, pas besoin d'envoyer STOP
    return;
  }

  // Vérification via la carte : est-ce qu'il y a une ouverture à gauche ?
  float angleRad = (robotAngle + 90.0) * PI / 180.0;
  float checkX = robotX_m + 1.0 * cos(angleRad);
  float checkY = robotY_m + 1.0 * sin(angleRad);

  if (isObstacle(checkX, checkY)) {
    // Couloir : l'Arduino gère le centrage avec ses ultrason
    sendToArduino("FWD");
  } else {
    // Intersection/ouverture : décision selon la destination
    // Pour l'instant : continuer tout droit (l'Arduino suit le mur droit)
    sendToArduino("FWD");
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
  if (missionState != IDLE) {
    // Robot occupé
    StaticJsonDocument<128> doc;
    doc["action"] = "robot_busy";
    doc["msg"] = "Robot déjà en mission (" + currentOrderId + ")";
    String json;
    serializeJson(doc, json);
    webSocket.broadcastTXT(json);
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

  Serial.printf("[MISSION] Démarrage : %s → Chambre %s (ArUco Cible: %d, code: %s)\n",
    orderId.c_str(), room.c_str(), targetArUco, deliveryCode.c_str());

  // Informer le dashboard
  StaticJsonDocument<256> doc;
  doc["action"] = "robot_status";
  doc["name"] = "Robot Alpha";
  doc["status"] = "En mission";
  doc["battery"] = batteryLevel;
  doc["order_id"] = orderId;
  doc["order_status"] = "En cours";
  String json;
  serializeJson(doc, json);
  webSocket.broadcastTXT(json);

  // Envoyer l'Arduino en route
  sendToArduino("FWD");
}

// Robot arrivé à la cuisine
void arrivedAtKitchen() {
  missionState = WAITING_LOADING;
  sendToArduino("STOP");

  Serial.println("[MISSION] Arrivé à la cuisine — en attente de chargement");

  StaticJsonDocument<256> doc;
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
  StaticJsonDocument<256> doc;
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

  StaticJsonDocument<256> doc;
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

  StaticJsonDocument<256> doc;
  doc["action"] = "robot_status";
  doc["name"] = "Robot Alpha";
  doc["battery"] = batteryLevel;

  if (batteryLevel <= 20) {
    missionState = NEEDS_CHARGE;
    doc["status"] = "Batterie faible - En charge";
    
    // Alerte au dashboard (pour le cuisinier)
    StaticJsonDocument<256> alertDoc;
    alertDoc["action"] = "alert";
    alertDoc["msg"] = "Robot déchargé ! Veuillez le brancher à la cuisine.";
    String alertJson;
    serializeJson(alertDoc, alertJson);
    webSocket.broadcastTXT(alertJson);

    Serial.println("[MISSION] De retour à la cuisine — BATTERIE FAIBLE, robot en veille/charge");
  } else {
    missionState = IDLE;
    doc["status"] = "Disponible";
    Serial.println("[MISSION] De retour à la cuisine — Robot disponible pour une nouvelle commande");
  }

  String json;
  serializeJson(doc, json);
  webSocket.broadcastTXT(json);
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
    StaticJsonDocument<256> doc;
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
      StaticJsonDocument<256> doc;
      doc["action"] = "robot_status";
      doc["name"] = "Robot Alpha";
      
      if (missionState == NEEDS_CHARGE) doc["status"] = "En charge (Batterie faible)";
      else if (missionState == IDLE) doc["status"] = "Disponible";
      else doc["status"] = "En mission";

      doc["battery"] = batteryLevel;
      if (missionState != IDLE && missionState != NEEDS_CHARGE) doc["order_id"] = currentOrderId;
      String json;
      serializeJson(doc, json);
      webSocket.sendTXT(num, json);

      // Envoyer les infos de la carte si chargée
      if (mapLoaded) {
        StaticJsonDocument<128> mapDoc;
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
      StaticJsonDocument<512> doc;
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
          missionState = IDLE;
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
// UPLOAD DE FICHIERS (pour le script deploy_to_esp32.py)
// ══════════════════════════════════════════════════════════════

File uploadFile;

void handleFileUpload() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    String filename = upload.filename;
    if (!filename.startsWith("/")) filename = "/" + filename;
    Serial.println("[UPLOAD] Réception : " + filename);
    uploadFile = SD.open(filename, FILE_WRITE);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (uploadFile) uploadFile.write(upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (uploadFile) {
      uploadFile.close();
      Serial.printf("[UPLOAD] Terminé : %u octets\n", upload.totalSize);
    }
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
  server.on("/mkdir", HTTP_POST, []() {
    String dir = server.arg("path");
    if (SD.mkdir(dir)) server.send(200, "text/plain", "OK");
    else server.send(500, "text/plain", "Erreur");
  });

  server.on("/upload", HTTP_POST, []() {
    server.send(200, "text/plain", "OK");
  }, handleFileUpload);

  server.onNotFound(handleFileRead);
  server.begin();
  Serial.println("[WEB] Serveur HTTP démarré sur port 80");

  // 5. WebSocket
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("[WS] WebSocket démarré sur port 81");

  // 6. Seed random
  randomSeed(analogRead(0) + millis());

  Serial.println("\n[PRÊT] Robot distributeur opérationnel !\n");
}

// ══════════════════════════════════════════════════════════════
// LOOP
// ══════════════════════════════════════════════════════════════

unsigned long lastNavUpdate = 0;
unsigned long lastPosUpdate = 0;

void loop() {
  // Gérer les connexions
  server.handleClient();
  webSocket.loop();

  // Lire les messages de l'Arduino
  readUART();

  // Lire le clavier 4x4 (actif uniquement en mode livraison)
  handleKeypad();

  // Navigation : toutes les 200ms
  unsigned long now = millis();
  if (now - lastNavUpdate > 200) {
    lastNavUpdate = now;
    navigationStep();
  }

  // Envoyer la position au dashboard : toutes les 2s
  if (now - lastPosUpdate > 2000) {
    lastPosUpdate = now;
    if (missionState != IDLE) {
      StaticJsonDocument<128> doc;
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
