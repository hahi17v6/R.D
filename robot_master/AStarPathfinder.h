#ifndef ASTAR_PATHFINDER_H
#define ASTAR_PATHFINDER_H

#include <Arduino.h>

#define CELL_SIZE_M 0.20 // 20cm par cellule
#define MAX_PATH    300  // Chemin max (suffisant pour partie du bâtiment)
#define MAX_OPEN    400  // Taille max de la open list

struct Point {
  int x, y;
};

// Map grossière allouée en mémoire au démarrage (downsamplée depuis la carte SD)
uint8_t* coarseMap = nullptr;
int coarseW = 0;
int coarseH = 0;

void freeCoarseMap() {
  if (coarseMap) {
    free(coarseMap);
    coarseMap = nullptr;
  }
}

// Alloue et crée une carte de collision en RAM
bool allocateCoarseMap(float realWidthM, float realHeightM) {
  freeCoarseMap();
  coarseW = (int)(realWidthM / CELL_SIZE_M) + 1;
  coarseH = (int)(realHeightM / CELL_SIZE_M) + 1;
  
  // Limite la taille pour éviter OutOfMemory (max ~50000 cellules = 50KB)
  if (coarseW * coarseH > 50000) {
    Serial.println("[ASTAR] Map trop grande pour la RAM, on limite à 50000 cellules.");
    if (coarseW > 250) coarseW = 250;
    if (coarseH > 250) coarseH = 250;
  }
  
  coarseMap = (uint8_t*)malloc(coarseW * coarseH);
  if (!coarseMap) {
    Serial.println("[ASTAR] Erreur malloc coarseMap !");
    return false;
  }
  memset(coarseMap, 0, coarseW * coarseH);
  Serial.printf("[ASTAR] Map RAM allouée: %d x %d\n", coarseW, coarseH);
  return true;
}

void setCoarseObstacle(int cx, int cy, bool isObstacle) {
  if (cx >= 0 && cx < coarseW && cy >= 0 && cy < coarseH && coarseMap != nullptr) {
    if (isObstacle) coarseMap[cy * coarseW + cx] = 1;
  }
}

bool getCoarseObstacle(int cx, int cy) {
  if (cx >= 0 && cx < coarseW && cy >= 0 && cy < coarseH && coarseMap != nullptr) {
    return coarseMap[cy * coarseW + cx] == 1;
  }
  return true; // En dehors de la carte = obstacle
}

// ══════════════════════════════════════════════════════════════
// VRAI A* — avec open list, parent tracking, reconstruction
// ══════════════════════════════════════════════════════════════

struct AStarNode {
  int16_t x, y;
  float f;       // f = g + h
};

Point currentPath[MAX_PATH]; // Chemin calculé
int pathLength = 0;
int currentPathIdx = 0;

// Heuristique (Distance octile — meilleure pour mouvements 8 directions)
float heuristic(int x1, int y1, int x2, int y2) {
  int dx = abs(x1 - x2);
  int dy = abs(y1 - y2);
  return (dx + dy) + (1.414f - 2.0f) * min(dx, dy);
}

// Encode la position parent dans un uint16_t (optimisé pour maps < 250x250)
#define ENCODE_POS(x, y) ((uint16_t)((y) * coarseW + (x)))

bool computeAStarPath(float startX_m, float startY_m, float endX_m, float endY_m) {
  if (!coarseMap) return false;
  
  int startX = (int)(startX_m / CELL_SIZE_M);
  int startY = (int)(startY_m / CELL_SIZE_M);
  int endX = (int)(endX_m / CELL_SIZE_M);
  int endY = (int)(endY_m / CELL_SIZE_M);

  // Clamp aux limites de la carte
  startX = constrain(startX, 0, coarseW - 1);
  startY = constrain(startY, 0, coarseH - 1);
  endX = constrain(endX, 0, coarseW - 1);
  endY = constrain(endY, 0, coarseH - 1);

  if (getCoarseObstacle(endX, endY)) {
    // Chercher la cellule libre la plus proche de la destination
    Serial.println("[ASTAR] Destination dans un mur, recherche adjacente...");
    bool found = false;
    for (int r = 1; r <= 5 && !found; r++) {
      for (int dx = -r; dx <= r && !found; dx++) {
        for (int dy = -r; dy <= r && !found; dy++) {
          if (abs(dx) == r || abs(dy) == r) {
            int nx = endX + dx, ny = endY + dy;
            if (!getCoarseObstacle(nx, ny)) {
              endX = nx; endY = ny; found = true;
            }
          }
        }
      }
    }
    if (!found) {
      Serial.println("[ASTAR] Aucune cellule libre autour de la destination !");
      return false;
    }
  }

  pathLength = 0;
  currentPathIdx = 0;

  int mapSize = coarseW * coarseH;

  // gScore : coût réel depuis le départ (stocké en uint16_t, x10 pour précision)
  // parentMap : index du parent (pour reconstruction)
  uint16_t* parentMap = (uint16_t*)malloc(mapSize * sizeof(uint16_t));
  uint8_t* closedSet = (uint8_t*)malloc(mapSize);
  float* gScore = (float*)malloc(mapSize * sizeof(float));
  
  if (!parentMap || !closedSet || !gScore) {
    Serial.println("[ASTAR] Erreur malloc A* !");
    if (parentMap) free(parentMap);
    if (closedSet) free(closedSet);
    if (gScore) free(gScore);
    return false;
  }

  memset(closedSet, 0, mapSize);
  for (int i = 0; i < mapSize; i++) gScore[i] = 999999.0f;

  // Open list (min-heap simplifié)
  AStarNode openList[MAX_OPEN];
  int openCount = 0;

  // Insérer le noeud de départ
  int startIdx = startY * coarseW + startX;
  gScore[startIdx] = 0;
  parentMap[startIdx] = ENCODE_POS(startX, startY); // parent de soi-même
  openList[openCount++] = {(int16_t)startX, (int16_t)startY, heuristic(startX, startY, endX, endY)};

  // Directions 8-connectées
  const int8_t dx8[] = {0, 0, 1, -1, 1, 1, -1, -1};
  const int8_t dy8[] = {1, -1, 0, 0, 1, -1, 1, -1};
  const float cost8[] = {1.0f, 1.0f, 1.0f, 1.0f, 1.414f, 1.414f, 1.414f, 1.414f};

  bool found = false;
  int iterations = 0;
  const int MAX_ITER = 5000;

  while (openCount > 0 && iterations < MAX_ITER) {
    iterations++;

    // Trouver le noeud avec le plus petit f dans l'open list
    int bestIdx = 0;
    for (int i = 1; i < openCount; i++) {
      if (openList[i].f < openList[bestIdx].f) bestIdx = i;
    }

    AStarNode current = openList[bestIdx];
    // Retirer de l'open list (swap avec le dernier)
    openList[bestIdx] = openList[--openCount];

    int cx = current.x;
    int cy = current.y;
    int cIdx = cy * coarseW + cx;

    if (closedSet[cIdx]) continue;
    closedSet[cIdx] = 1;

    // Arrivé ?
    if (cx == endX && cy == endY) {
      found = true;
      break;
    }

    // Explorer les 8 voisins
    for (int i = 0; i < 8; i++) {
      int nx = cx + dx8[i];
      int ny = cy + dy8[i];
      
      if (nx < 0 || nx >= coarseW || ny < 0 || ny >= coarseH) continue;
      
      int nIdx = ny * coarseW + nx;
      if (closedSet[nIdx] || coarseMap[nIdx] == 1) continue;

      // Vérifier les diagonales (éviter de couper les coins)
      if (i >= 4) { // directions diagonales
        if (coarseMap[cy * coarseW + nx] == 1 || coarseMap[ny * coarseW + cx] == 1) continue;
      }

      float tentG = gScore[cIdx] + cost8[i];
      
      if (tentG < gScore[nIdx]) {
        gScore[nIdx] = tentG;
        parentMap[nIdx] = ENCODE_POS(cx, cy);
        float f = tentG + heuristic(nx, ny, endX, endY);
        
        if (openCount < MAX_OPEN) {
          openList[openCount++] = {(int16_t)nx, (int16_t)ny, f};
        }
      }
    }
  }

  // ── Reconstruction du chemin ──
  if (found) {
    // Reconstruire en sens inverse
    Point reversePath[MAX_PATH];
    int reverseLen = 0;
    int cx = endX, cy = endY;
    
    while (reverseLen < MAX_PATH) {
      reversePath[reverseLen++] = {cx, cy};
      int idx = cy * coarseW + cx;
      int px = parentMap[idx] % coarseW;
      int py = parentMap[idx] / coarseW;
      if (px == cx && py == cy) break; // arrivé au départ
      cx = px; cy = py;
    }

    // Inverser dans currentPath
    for (int i = reverseLen - 1; i >= 0 && pathLength < MAX_PATH; i--) {
      currentPath[pathLength++] = reversePath[i];
    }

    Serial.printf("[ASTAR] ✓ Chemin trouvé : %d étapes, %d itérations\n", pathLength, iterations);
  } else {
    Serial.printf("[ASTAR] ✗ Pas de chemin trouvé après %d itérations\n", iterations);
    
    // Fallback : se diriger vers la cellule la plus proche de la destination qu'on a visitée
    int bestDist = 999999;
    int bx = startX, by = startY;
    for (int y = 0; y < coarseH; y++) {
      for (int x = 0; x < coarseW; x++) {
        if (closedSet[y * coarseW + x]) {
          int d = abs(x - endX) + abs(y - endY);
          if (d < bestDist) { bestDist = d; bx = x; by = y; }
        }
      }
    }
    
    // Reconstruire le chemin partiel vers cette cellule
    Point reversePath[MAX_PATH];
    int reverseLen = 0;
    int cx = bx, cy = by;
    while (reverseLen < MAX_PATH) {
      reversePath[reverseLen++] = {cx, cy};
      int idx = cy * coarseW + cx;
      int px = parentMap[idx] % coarseW;
      int py = parentMap[idx] / coarseW;
      if (px == cx && py == cy) break;
      cx = px; cy = py;
    }
    for (int i = reverseLen - 1; i >= 0 && pathLength < MAX_PATH; i--) {
      currentPath[pathLength++] = reversePath[i];
    }
    Serial.printf("[ASTAR] Chemin partiel : %d étapes (dist restante: %d)\n", pathLength, bestDist);
  }

  free(parentMap);
  free(closedSet);
  free(gScore);

  return pathLength > 0;
}

#endif
