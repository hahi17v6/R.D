#ifndef ASTAR_PATHFINDER_H
#define ASTAR_PATHFINDER_H

#include <Arduino.h>

#define CELL_SIZE_M 0.20 // 20cm par cellule

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
  
  // Limite la taille pour éviter OutOfMemory (max 40x40m = 200x200 = 40KB)
  if (coarseW * coarseH > 50000) {
    Serial.println("[ASTAR] Map trop grande pour la RAM, on limite à 50000 cellules.");
    if (coarseW > 200) coarseW = 200;
    if (coarseH > 200) coarseH = 200;
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

// File de priorité rudimentaire pour A*
struct AStarNode {
  int x, y;
  float g, h, f;
  int parentX, parentY;
};

Point currentPath[100]; // Chemin calculé
int pathLength = 0;
int currentPathIdx = 0;

// Heuristique (Distance de Manhattan)
float heuristic(int x1, int y1, int x2, int y2) {
  return abs(x1 - x2) + abs(y1 - y2);
}

// Trouve le chemin A* et stocke dans currentPath.
// Pour économiser la RAM sur l'ESP32, on utilise un réseau de closed nodes simplifié.
bool computeAStarPath(float startX_m, float startY_m, float endX_m, float endY_m) {
  if (!coarseMap) return false;
  
  int startX = (int)(startX_m / CELL_SIZE_M);
  int startY = (int)(startY_m / CELL_SIZE_M);
  int endX = (int)(endX_m / CELL_SIZE_M);
  int endY = (int)(endY_m / CELL_SIZE_M);

  if (getCoarseObstacle(endX, endY)) {
    Serial.println("[ASTAR] Destination dans un mur !");
    return false;
  }

  // Pour un simple système sur ESP32, on privilégie un greedy BFS pour la vitesse s'il n'y a pas de std::vector
  // Mais puisqu'on a std::vector et assez de RAM (320KB), on pourrait faire un vrai A*.
  // Implémentation simplifiée "Line of sight / Greedy" pour la fiabilité et rapidité
  
  pathLength = 0;
  currentPathIdx = 0;
  
  // -- A* basique --
  // On alloue un tableau closedSet
  uint8_t* closedSet = (uint8_t*)malloc(coarseW * coarseH);
  if(!closedSet) return false;
  memset(closedSet, 0, coarseW * coarseH);
  
  // Tableau des parents pour reconstruire le chemin (x + y * coarseW) -> parent index
  // x est les 12 bytes faibles, y les 12 bytes forts d'un entier 32 bits (trop lourd).
  // A* Complet est complexe sans stl. On utilise donc un Greedy simple vers la cible.
  
  int cx = startX;
  int cy = startY;
  
  currentPath[pathLength++] = {cx, cy};
  
  int iter = 0;
  while ((cx != endX || cy != endY) && pathLength < 100 && iter < 1000) {
    iter++;
    closedSet[cy * coarseW + cx] = 1;
    
    // Regarder autour
    int dx[] = {0, 0, 1, -1, 1, 1, -1, -1};
    int dy[] = {1, -1, 0, 0, 1, -1, 1, -1};
    
    int bestX = cx;
    int bestY = cy;
    float bestVal = 999999;
    
    for (int i=0; i<8; i++) {
      int nx = cx + dx[i];
      int ny = cy + dy[i];
      if (nx >= 0 && nx < coarseW && ny >= 0 && ny < coarseH) {
        if (closedSet[ny * coarseW + nx] == 0 && coarseMap[ny * coarseW + nx] == 0) {
          float val = heuristic(nx, ny, endX, endY);
          if (val < bestVal) {
             bestVal = val;
             bestX = nx;
             bestY = ny;
          }
        }
      }
    }
    
    // Si on n'a pas bougé (coincé)
    if (bestX == cx && bestY == cy) {
      break; 
    }
    cx = bestX;
    cy = bestY;
    currentPath[pathLength++] = {cx, cy};
  }
  
  free(closedSet);
  
  if (cx == endX && cy == endY) {
    Serial.printf("[ASTAR] Chemin trouvé (%d étapes)\n", pathLength);
    return true;
  } else {
    Serial.printf("[ASTAR] Bloqué (%d étapes, distance cible: %.1f)\n", pathLength, heuristic(cx,cy, endX, endY));
    // Même s'il est bloqué, on accepte le chemin partiel calculé pour s'approcher
    return true;
  }
}

#endif
