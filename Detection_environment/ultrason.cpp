#include <NewPing.h>

#define MAX_DISTANCE 200

// Sensors
NewPing leftSensor(8, 8, MAX_DISTANCE);
NewPing rightSensor(5, 5, MAX_DISTANCE);
NewPing frontSensor(6, 6, MAX_DISTANCE);
NewPing backSensor(7, 7, MAX_DISTANCE);

int readAvg(NewPing &sensor) {
  int total = 0;

  for(int i = 0; i < 5; i++) {   // average 5 readings
    total += sensor.ping_cm();
    delay(10);
  }

  return total / 5;
}

void setup() {
  Serial.begin(9600);
}

void loop() {

  int left = readAvg(leftSensor);
  int right = readAvg(rightSensor);
  int front = readAvg(frontSensor);
  int back = readAvg(backSensor);

  int diff = abs(left - right);

  Serial.print("L:");
  Serial.print(left);
  Serial.print(" R:");
  Serial.print(right);
  Serial.print(" F:");
  Serial.print(front);
  Serial.print(" B:");
  Serial.println(back);

  // -------- OBSTACLE --------
  if(front > 0 && front < 15) {
    Serial.println("OBSTACLE FRONT → avoid");
  }

  else if(back > 0 && back < 15) {
    Serial.println("OBSTACLE BACK → move forward");
  }

  // -------- INTERSECTION --------
  else if(diff > 50) {
    Serial.println("INTERSECTION DETECTED");
  }

  // -------- CENTERING --------
  else {

    if(diff < 4) {
      Serial.println("GO STRAIGHT");
    }

    else if(left > right) {
      Serial.println("MOVE LEFT");
    }

    else {
      Serial.println("MOVE RIGHT");
    }

  }

  delay(80);
}
