#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <SD.h>

// ── Broches SD (même configuration que map.bin) ─────────────────────────
#define SD_CS   4
#define SD_MOSI 23
#define SD_SCK  18
#define SD_MISO 19

SPIClass spiSD(VSPI);

// ── Configuration WiFi ──────────────────────────────────────────────────
// Paramètres pour le mode Station (connexion à votre box avec internet)
const char* ssid_sta = "iPhone"; 
const char* password_sta = "123444567";

// Paramètres pour le mode Access Point (réseau du robot, sans internet)
const char* ssid_ap = "Robot-WiFi";
const char* password_ap = "12345678"; // Doit faire au moins 8 caractères, ou laissez vide "" pour aucun mot de passe

// ── Serveur Web ─────────────────────────────────────────────────────────
WebServer server(80);

// ── Fonctions Utilitaires ───────────────────────────────────────────────

// Obtenir le type Mime en fonction de l'extension du fichier
String getContentType(String filename) {
  if (filename.endsWith(".html")) return "text/html";
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".js")) return "application/javascript";
  else if (filename.endsWith(".png")) return "image/png";
  else if (filename.endsWith(".jpg") || filename.endsWith(".jpeg")) return "image/jpeg";
  else if (filename.endsWith(".gif")) return "image/gif";
  else if (filename.endsWith(".ico")) return "image/x-icon";
  else if (filename.endsWith(".xml")) return "text/xml";
  else if (filename.endsWith(".pdf")) return "application/pdf";
  else if (filename.endsWith(".zip")) return "application/zip";
  return "text/plain";
}

// Fonction appelée lors d'une requête HTTP
void handleFileRead() {
  String path = server.uri();
  Serial.println("Requete reçue: " + path);
  
  // Par défaut, si l'utilisateur demande "/", on renvoie "/index.html"
  if (path == "/") {
    path = "/index.html";
  }
  
  String contentType = getContentType(path);
  
  if (SD.exists(path)) {
    // Le fichier existe sur la carte SD, on l'ouvre
    File file = SD.open(path, FILE_READ);
    if (!file) {
      server.send(500, "text/plain", "Erreur lors de la lecture du fichier");
      return;
    }
    
    // Envoyer le fichier au client
    server.streamFile(file, contentType);
    file.close();
    Serial.println("Fichier envoyé: " + path);
  } else {
    // Si le fichier n'existe pas
    Serial.println("Fichier NON TROUVE: " + path);
    server.send(404, "text/plain", "Fichier non trouve (404)");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- Demarrage Serveur Web ESP32 (SD Card) ---");

  // 1. Initialisation de la carte SD
  spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, spiSD)) {
    Serial.println("Erreur: Carte SD introuvable ou erreur de cablage.");
  } else {
    Serial.println("Carte SD initialisee avec succes !");
  }

  // 2. Configuration WiFi (AP + STA)
  WiFi.mode(WIFI_AP_STA);
  
  // Démarrer l'Access Point (Robot-WiFi)
  WiFi.softAP(ssid_ap, password_ap);
  Serial.print("Point d'accès demarre. Nom: ");
  Serial.print(ssid_ap);
  Serial.print(" - IP du robot: ");
  Serial.println(WiFi.softAPIP());

  // Se connecter au routeur (Box Internet)
  Serial.print("Tentative de connexion au WiFi: ");
  Serial.println(ssid_sta);
  WiFi.begin(ssid_sta, password_sta);
  
  // Attendre 10 secondes maximum pour la connexion STA
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 20) {
    delay(500);
    Serial.print(".");
    retries++;
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Connecté à Internet ! IP locale: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Echec de la connexion a la box (pas d'internet). Le mode Access Point reste actif.");
  }

  // 3. Configuration des routes du serveur
  // Toutes les requêtes sont redirigées vers handleFileRead
  server.onNotFound(handleFileRead);

  // Lancement du serveur
  server.begin();
  Serial.println("Serveur Web demarre !");
}

void loop() {
  // Gérer en permanence les connexions des clients Web
  server.handleClient();
}
