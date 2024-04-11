int sensorInterrupt; 
int sensorPin;
//int solenoidValve;


float calibrationFactor; //Hall-effecf flow sensor outputs 
volatile byte pulseCount;  
float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;
unsigned int goal;

unsigned long oldTime;

void setup()
{

  // Initialize a serial connection for reporting values to the host
  Serial.begin(9600);

  sensorInterrupt = 0;
  sensorPin = 2; //Digital Pin 2
  //solenoidValve = 5; //Digital Pin 5
  calibrationFactor= 5; //5 pulses per second per L/minute
  
  //pinMode(solenoidValve, OUTPUT);
  //pinMode(solenoidValve, HIGH);
  pinMode(sensorPin, INPUT);
  digitalWrite(sensorPin, HIGH);

  pulseCount        = 0;
  flowRate          = 0.0;
  flowMilliLitres   = 0;
  totalMilliLitres  = 0;
  oldTime           = 0;
  goal              = 500;// to give total output 500mL

  
  //FALLING state change (transition from HIGH state to LOW state)
  attachInterrupt(sensorInterrupt, pulseCounter, FALLING);
}

void loop()
{
   
   if((millis() - oldTime) > 1000)    // Only process counters once per second
  { 
    detachInterrupt(sensorInterrupt);
    flowRate = ((1000.0 / (millis() - oldTime)) * pulseCount) / calibrationFactor;
    oldTime = millis();
    flowMilliLitres = (flowRate / 60) * 1000;
    totalMilliLitres += flowMilliLitres;
    
    }
    unsigned int frac;
    
    // Print the flow rate for this second in litres / minute
    Serial.print("Flow rate: ");
    Serial.print(flowMilliLitres);
    //Serial.print(int(flowRate));
    Serial.print("L/min");
    Serial.print("Hey! We detect a leakage!")
    Serial.print("\t");    

    // Print the cumulative total of litres flowed since starting
    Serial.print("Quantity ");        
    Serial.print(totalMilliLitres);
    Serial.println("mL"); 
    Serial.print("\t");  

//    if(totalMilliLitres>100){
//      digitalWrite(solenoidValve,LOW);
//    }
    // Reset the pulse counter so we can start incrementing again
    pulseCount = 0;
    
    // Enable the interrupt again now that we've finished sending output
    attachInterrupt(sensorInterrupt, pulseCounter, FALLING);
  }
}

/*
Insterrupt Service Routine
 */
void pulseCounter()
{
  pulseCount++;
}
