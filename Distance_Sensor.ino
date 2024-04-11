int pins[] = {A0, A1};
char headers[] = {'D', 'W'};
int sensorPin = 0; //Set the first sensor to pin 0
int sensorPint = 1; //Set the second sensor to pin 1
int distance; // declare distance variable to get the cm values

void setup() {
  Serial.begin(9600); // serial at 9600
}

void loop() {
  int val = analogRead(pins[0]); // declare val as the output value for the first sensor
  int valt = analogRead(pins[1]);  // declare valt as the output value for the second sensor
  int i = 0;

  //  Serial.println("Distance : ");
  //  Serial.print(distance);
  //  Serial.println("cm");
  for (int z = 0; z < 2; z++) {
    //if the values of the first sensor are in the range of 435 -465 or the values of the second sensor are in the range of 380 - 410, distance equals to 160
      if (val >= 435 && val < 465 || valt >= 380 && valt < 410) {
        distance = 160;       
        if (i < 2) {
          Serial.print(headers[0]);
          Serial.println(distance);
          Serial.print(headers[1]);
          Serial.println(distance);
          i++;
        }
        break;
    }
    //if the values of the first sensor are in the range of 465-515 or the values of the second sensor are in the range of 410-450, distance equals to 120
    if (val >= 465 && val < 515 || valt >= 410 && valt <= 450) {
      distance = 120;
      if (i < 2) {
        Serial.print(headers[0]);
        Serial.println(distance);
        Serial.print(headers[1]);
        Serial.println(distance);
        i++;
      }
      break;
    }
    //if the values of the first sensor are in the range of 515 - 580 or the values of the second sensor are in the range of 510 - 550, distance equals to 80
    if (val >= 515 && val <= 580 || valt >= 510 && valt <= 550) {
      distance = 80;
      if (i < 2) {
        Serial.print(headers[0]);
        Serial.println(distance);
        Serial.print(headers[1]);
        Serial.println(distance);
        i++;
      }
      break;
    }
  }

  delay(2000);
}
