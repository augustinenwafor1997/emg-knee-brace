/*
  MyoWare BLE Central (Artemis Nano) - Robust Version
  Outputs ONLY the difference: THIGH value - HAMSTRING value
  Normalized to -1 (full hamstring) to 1 (full thigh)

  - Scans for MyoWare Wireless Shields advertising the MyoWare service UUID
  - Connects to HAMSTRING and THIGH sensors by name
  - Discovers attributes with retries + delays
  - Computes and prints: (THIGH - HAMSTRING)

  Serial: 115200
*/

#include <ArduinoBLE.h>
#include <MyoWare.h>

#include <vector>
#include <algorithm>  // <-- REQUIRED for std::find

// -------------------- User knobs --------------------
static const bool debugLogging = false;     // set true to see connection logs
static const unsigned long scanWindowMs = 5000; // how long to scan on boot
static const unsigned long readPeriodMs = 20;    // rate to read each connected shield
static const int maxPeripherals = 4;       // ArduinoBLE limit is typically 4
static const int smoothingWindowSize = 10; // rolling average window (higher = more smooth)
// ----------------------------------------------------

BLEDevice thighShield;
BLEDevice hamstringShield;
MyoWare myoware;

// Rolling average buffer
std::vector<double> smoothingBuffer;
size_t bufferIndex = 0;

// Forward decl
static void PrintPeripheralInfo(BLEDevice peripheral);
static double ReadBLEData(BLECharacteristic &dataCharacteristic);

void setup()
{
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(myoware.getStatusLEDPin(), OUTPUT);

  if (!BLE.begin())
  {
    Serial.println("Starting BLE failed!");
    while (1) {}
  }

  if (debugLogging)
  {
    Serial.println();
    Serial.println("MyoWare BLE Central - THIGH vs HAMSTRING");
    Serial.println("----------------------------------------");
    Serial.print("Scanning for service UUID: ");
    Serial.println(MyoWareBLE::uuidMyoWareService.c_str());
  }

  // Start scan (active scan true)
  BLE.scanForUuid(MyoWareBLE::uuidMyoWareService.c_str(), true);

  const unsigned long startMillis = millis();
  while (millis() - startMillis < scanWindowMs)
  {
    myoware.blinkStatusLED();

    // Stop early if we have both sensors
    if (thighShield && hamstringShield)
      break;

    BLEDevice peripheral = BLE.available();
    if (!peripheral)
      continue;

    String localName = peripheral.localName();

    // Check if this is THIGH or HAMSTRING
    bool isThigh = (localName == "THIGH");
    bool isHamstring = (localName == "HAMSTRING");

    if (!isThigh && !isHamstring)
      continue;  // Skip if not one of our target sensors

    // Skip if we already have this sensor
    if (isThigh && thighShield)
      continue;
    if (isHamstring && hamstringShield)
      continue;

    if (debugLogging)
    {
      Serial.print("Found: ");
      PrintPeripheralInfo(peripheral);
      Serial.println("Attempting connect...");
    }

    BLE.stopScan();
    delay(200); // small breather for BLE stack

    if (!peripheral.connect())
    {
      if (debugLogging)
      {
        Serial.print("Connect FAILED: ");
        PrintPeripheralInfo(peripheral);
      }
      // Resume scanning
      BLE.scanForUuid(MyoWareBLE::uuidMyoWareService.c_str(), true);
      delay(100);
      continue;
    }

    // Give the peripheral time to settle before discovery (important!)
    delay(500);

    bool discovered = peripheral.discoverAttributes();
    if (!discovered)
    {
      if (debugLogging) Serial.println("discoverAttributes() failed, retrying...");
      delay(500);
      discovered = peripheral.discoverAttributes();
    }

    if (!discovered)
    {
      if (debugLogging)
      {
        Serial.print("Discovering Attributes FAILED. Disconnecting: ");
        PrintPeripheralInfo(peripheral);
      }
      peripheral.disconnect();
      delay(200);

      // Resume scanning
      BLE.scanForUuid(MyoWareBLE::uuidMyoWareService.c_str(), true);
      delay(100);
      continue;
    }

    if (isThigh)
    {
      thighShield = peripheral;
      if (debugLogging) Serial.println("THIGH connected + discovered");
    }
    else if (isHamstring)
    {
      hamstringShield = peripheral;
      if (debugLogging) Serial.println("HAMSTRING connected + discovered");
    }

    // Resume scanning for the other sensor
    BLE.scanForUuid(MyoWareBLE::uuidMyoWareService.c_str(), true);
    delay(100);
  }

  BLE.stopScan();

  if (!thighShield || !hamstringShield)
  {
    Serial.println("Could not find both THIGH and HAMSTRING sensors!");
    while (1)
    {
      myoware.blinkStatusLED();
      delay(100);
    }
  }

  digitalWrite(myoware.getStatusLEDPin(), HIGH);

  // Initialize smoothing buffer
  smoothingBuffer.resize(smoothingWindowSize, 0.0);
  bufferIndex = 0;
}

void loop()
{

  double thighValue = 0.0;
  double hamstringValue = 0.0;

  // Read THIGH
  if (thighShield && thighShield.connected())
  {
    BLEService myoWareService = thighShield.service(MyoWareBLE::uuidMyoWareService.c_str());
    if (myoWareService)
    {
      BLECharacteristic sensorCharacteristic =
          myoWareService.characteristic(MyoWareBLE::uuidMyoWareCharacteristic.c_str());

      thighValue = ReadBLEData(sensorCharacteristic);
      thighValue = thighValue / 2000;
      if (thighValue < 0.0) thighValue = 0.0;
      if (thighValue > 1.0) thighValue = 1.0;
    }
  }
  else
  {
    if (debugLogging) Serial.println("THIGH disconnected!");
    thighShield.disconnect();
  }

  // Read HAMSTRING
  if (hamstringShield && hamstringShield.connected())
  {
    BLEService myoWareService = hamstringShield.service(MyoWareBLE::uuidMyoWareService.c_str());
    if (myoWareService)
    {
      BLECharacteristic sensorCharacteristic =
          myoWareService.characteristic(MyoWareBLE::uuidMyoWareCharacteristic.c_str());

      hamstringValue = ReadBLEData(sensorCharacteristic);
      hamstringValue = hamstringValue / 1200;
      if (hamstringValue < 0.0) hamstringValue = 0.0;
      if (hamstringValue > 1.0) hamstringValue = 1.0;
    }
  }
  else
  {
    if (debugLogging) Serial.println("HAMSTRING disconnected!");
    hamstringShield.disconnect();
  }

  // Compute difference: THIGH - HAMSTRING
  // Full thigh (1.0), no hamstring (0.0) = 1.0
  // No thigh (0.0), full hamstring (1.0) = -1.0
  double difference = thighValue - hamstringValue;

  // Add to rolling average buffer
  smoothingBuffer[bufferIndex] = difference;
  bufferIndex = (bufferIndex + 1) % smoothingWindowSize;

  // Compute and output rolling average
  double sum = 0.0;
  for (int i = 0; i < smoothingWindowSize; i++)
  {
    sum += smoothingBuffer[i];
  }
  double smoothedValue = sum / smoothingWindowSize;

  Serial.println(smoothedValue, 6);

  // If both disconnected, stop
  if (!thighShield.connected() || !hamstringShield.connected())
  {
    Serial.println("One or both sensors disconnected.");
    while (1)
    {
      myoware.blinkStatusLED();
      delay(100);
    }
  }
}

// Reads the sensor value from the characteristic (string -> double)
static double ReadBLEData(BLECharacteristic &dataCharacteristic)
{
  if (!dataCharacteristic)
  {
    if (debugLogging) Serial.println("Characteristic not found!");
    return 0.0;
  }

  if (!dataCharacteristic.canRead())
  {
    if (debugLogging)
    {
      Serial.print("Characteristic not readable: ");
      Serial.println(dataCharacteristic.uuid());
    }
    return 0.0;
  }

  // Read as char buffer (ArduinoBLE does not guarantee null termination)
  char buf[32];
  memset(buf, 0, sizeof(buf));
  int n = dataCharacteristic.readValue((void *)buf, (int)sizeof(buf) - 1);
  if (n <= 0) return 0.0;

  // Ensure null termination
  buf[sizeof(buf) - 1] = '\0';

  String s(buf);
  s.trim();
  return s.toDouble();
}

static void PrintPeripheralInfo(BLEDevice peripheral)
{
  Serial.print(peripheral.address());
  Serial.print(" '");
  Serial.print(peripheral.localName());
  Serial.print("' ");
  Serial.print(peripheral.advertisedServiceUuid());
  Serial.println();
}
