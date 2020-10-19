/* Continuous Mode with the VL53L0X.

  ------------------------------------
  --------12C BUS PINS (ESP32)--------
  ------ SDA : 21 ---- SCL : 22 ------
  ------------------------------------
  -----------Sensor 1 PINS------------
  ------ XSHUT : 25 - GPIO : 26 ------
  ------------------------------------
  -----------Sensor 2 PINS------------
  ------ XSHUT : 32 - GPIO : 33 ------
  ------------------------------------
  -----------Sensor 3 PINS------------
  ------ XSHUT : 27 - GPIO : 14 ------
  ------------------------------------
  ------------STATUS LED--------------
  -------------PIN : 04---------------
  ------------------------------------

- Tested Using VL53L0X Library by Polulu (v1.3.0)
  https://github.com/pololu/vl53l0x-arduino

- To Hide Sensor Values But Print Samples Per Second and LED State,
  Press "h" in Serial Monitor.

- XSHUT Represent Shutdown Pins and GPIO Represent Interrupt Pins,
  Interrupts is not used in this example.

- The Range Readings are in Units of mm.
*/

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

#define SENSOR1_WIRE Wire
// #define SENSOR2_WIRE Wire
// #define SENSOR3_WIRE Wire1
// #define SENSOR4_WIRE Wire1

typedef struct
{
  VL53L0X *psensor;  // Pointer To Object
  int id;            // Address For The Sensor
  int shutdown_pin;  // Pin Used For Shutdown
  int interrupt_pin; // Pin Used For interrupt
  uint16_t range;    // Range Measurement In Continuous Mode
} sensorList_t;

VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;

sensorList_t sensors[] = {
    {&sensor1, 0x30, 25, 26, 0},
    {&sensor2, 0x31, 32, 33, 0},
    {&sensor3, 0x32, 27, 14, 0},
};

const int COUNT_SENSORS = sizeof(sensors) / sizeof(sensors[0]); // Sensor Count

// #define SENSORVALUESPRINT 1
bool HideSerialPrint = false; // To Control Serial Print

unsigned long now; // For Millis
unsigned long prevMillis = 0;
int SamplesPerSecond = 0; // Samples Per Second

// LED and LEDSTATE
#define LEDPIN 4                   // LED Connected to GPIO 4
bool LEDSTATE = false;             // State of LED
const int ThresholdDistance = 200; // LED Trigger Minimum Distance
const int LEDONTIME = 1000;        // LED Stays on for 1000ms
long prevLEDTrigger = 0;

// Functions
void Initialize_sensors(); // Intialise Sensors

void setup()
{
  Serial.begin(115200);

  delay(1000);

  pinMode(LEDPIN, OUTPUT); // LED
  Wire.begin();

  // Initialize SHUTDOWN PINS & INTERRUPT PINS
  Serial.println(F("Mulitple VL53LOX, Initialize IO Pins"));
  for (int i = 0; i < COUNT_SENSORS; i++)
  {
    pinMode(sensors[i].shutdown_pin, OUTPUT);
    digitalWrite(sensors[i].shutdown_pin, LOW);

    if (sensors[i].interrupt_pin >= 0)
      pinMode(sensors[i].interrupt_pin, INPUT_PULLUP);
  }
  Serial.println(F("Starting..."));

  Initialize_sensors();
}

void loop()
{
  if (Serial.available())
  {
    uint8_t ch = Serial.read();

    switch (ch)
    {
    case 'h':
    case 'H':
      HideSerialPrint = !(HideSerialPrint);
      break;

    default:
      HideSerialPrint = false;
    }
  }

  now = millis();
  if (now - prevMillis > 1000)
  {
    Serial.print("-----Samples/Second : ");
    Serial.print(SamplesPerSecond);
    Serial.println("-----");
    prevMillis = now;
    SamplesPerSecond = 0;
  }

  for (uint8_t i = 0; i < COUNT_SENSORS; i++)
  {
    sensors[i].range = sensors[i].psensor->readRangeContinuousMillimeters();
    if (!HideSerialPrint)
    {
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.print(" : ");
      Serial.print(sensors[i].range);
      Serial.print(" - ");
    }
    if (sensors[i].psensor->timeoutOccurred())
    {
      Serial.print(" TIMEOUT");
    }
  }
  if (!HideSerialPrint)
  {
    Serial.println();
  }

  for (uint8_t i = 0; i < COUNT_SENSORS; i++)
  {
    int distance = sensors[i].range;
    if (distance != 0 && distance <= ThresholdDistance && LEDSTATE == false)
    {
      LEDSTATE = true;
      Serial.print("-Less Than Threshold Distance : ");
      Serial.print(distance);
      Serial.println("-");
      Serial.print("---------Sensor : ");
      Serial.print(i);
      Serial.println(" -------");
      digitalWrite(LEDPIN, HIGH);
      prevLEDTrigger = now;
    }
  }

  if ((now - prevLEDTrigger > LEDONTIME) && LEDSTATE == true)
  {
    LEDSTATE = false;
    Serial.println("---------LED Turned Off----------");
    digitalWrite(LEDPIN, LOW);
  }
  SamplesPerSecond++;
}

void Initialize_sensors()
{
  // Setting All Shutdown Pins to LOW
  for (int i = 0; i < COUNT_SENSORS; i++)
    digitalWrite(sensors[i].shutdown_pin, LOW);
  delay(10);

  for (int i = 0; i < COUNT_SENSORS; i++)
  {
    // Turning on Sensors One By One
    digitalWrite(sensors[i].shutdown_pin, HIGH);
    delay(10);

    sensors[i].psensor->setAddress(sensors[i].id); // Setting ID for Turned on Sensor
    sensors[i].psensor->setTimeout(500);

    if (sensors[i].psensor->init())
    {
      // Start continuous back-to-back mode (take readings as
      // fast as possible).  To use continuous timed mode
      // instead, provide a desired inter-measurement period in
      // ms (e.g. sensor.startContinuous(100)).
      sensors[i].psensor->startContinuous();
    }
    else
    {
      Serial.print(i, DEC);
      Serial.print(F(": failed to start\n"));
    }
  }
}