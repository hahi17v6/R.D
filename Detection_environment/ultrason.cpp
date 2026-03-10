#include <NewPing.h>

#define MAX_DISTANCE 200

// Grove sensors (1 pin each)
NewPing leftSensor(8, 8, MAX_DISTANCE);
NewPing rightSensor(5, 5, MAX_DISTANCE);
NewPing frontSensor(6, 6, MAX_DISTANCE);
NewPing backSensor(7, 7, MAX_DISTANCE);

// Function to get stable reading
int readSensor(NewPing &sensor) {

  delay(40); // avoid ultrasonic interference

  unsigned int uS = sensor.ping_median(5);

  if(uS == 0) return MAX_DISTANCE;

  return uS / US_ROUNDTRIP_CM;
}

void setup() {
  Serial.begin(9600);
}

void loop() {

  int left  = readSensor(leftSensor);
  int right = readSensor(rightSensor);
  int front = readSensor(frontSensor);
  int back  = readSensor(backSensor);

  int diff = abs(left - right);

  Serial.print("L:");
  Serial.print(left);
  Serial.print(" R:");
  Serial.print(right);
  Serial.print(" F:");
  Serial.print(front);
  Serial.print(" B:");
  Serial.println(back);

  // OBSTACLE FRONT
  if(front < 15) {
    Serial.println("OBSTACLE FRONT → avoid");
  }

  // OBSTACLE BACK
  else if(back < 15) {
    Serial.println("OBSTACLE BACK → move forward");
  }

  // INTERSECTION
  else if(diff > 50) {
    Serial.println("INTERSECTION DETECTED");
  }

  // CENTERING
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
