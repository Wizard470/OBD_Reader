#include <CAN.h>
const bool useStandardAddressing = true;
unsigned long _lastPidResponseMillis = 0;
unsigned long _responseTimeout = 2000;
bool _useExtendedAddressing = false;

void setup() {
  Serial.begin(9600);
  Serial.println("OBDII Teszt");

  CAN.setPins(10, 2);
  CAN.setClockFrequency(8E6);
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }

  if (useStandardAddressing) {
    CAN.filter(0x7e8);
  } else {
    CAN.filterExtended(0x18daf110);
  }
}

void loop() {
    int RequestedPIDs[] = {0x04, 0x05, 0x0c, 0x0e, 0x0d, 0x0f, 0x11, 0x1f, 0x2f, 0x33, 0x42, 0x43, 0x46, 0x47, 0x49, 0x4a, 0x4c, 0x5c};

    for (int i = 0; i < sizeof(RequestedPIDs); i++){
        printPid(RequestedPIDs[i]);
    }
    Serial.println("-------------------------------------------");
}

int pidRead(uint8_t mode, uint8_t pid, void* data, int length)
{
  // make sure at least 60 ms have passed since the last response
  unsigned long lastResponseDelta = millis() - _lastPidResponseMillis;
  if (lastResponseDelta < 60) {
    delay(60 - lastResponseDelta);
  }

  for (int retries = 10; retries > 0; retries--) {
    if (_useExtendedAddressing) {
      CAN.beginExtendedPacket(0x18db33f1, 8);
    } else {
      CAN.beginPacket(0x7df, 8);
    }
    CAN.write(0x02); // number of additional bytes
    CAN.write(mode);
    CAN.write(pid);
    if (CAN.endPacket()) {
      // send success
      break;
    } else if (retries <= 1) {
      return 0;
    }
  }

  bool splitResponse = (length > 5);

  for (unsigned long start = millis(); (millis() - start) < _responseTimeout;) {
    if (CAN.parsePacket() != 0 &&
          (splitResponse ? (CAN.read() == 0x10 && CAN.read()) : CAN.read()) &&
          (CAN.read() == (mode | 0x40) && CAN.read() == pid)) {

      _lastPidResponseMillis = millis();

      // got a response
      if (!splitResponse) {
        return CAN.readBytes((uint8_t*)data, length);
      }

      int read = CAN.readBytes((uint8_t*)data, 3);

      for (int i = 0; read < length; i++) {
        delay(60);

        // send the request for the next chunk
        if (_useExtendedAddressing) {
          CAN.beginExtendedPacket(0x18db33f1, 8);
        } else {
          CAN.beginPacket(0x7df, 8);
        }
        CAN.write(0x30);
        CAN.endPacket();

        // wait for response
        while (CAN.parsePacket() == 0 ||
               CAN.read() != (0x21 + i)); // correct sequence number

        while (CAN.available()) {
          ((uint8_t*)data)[read++] = CAN.read();
        }
      }

      _lastPidResponseMillis = millis();

      return read;
    }
  }

  return 0;
}
void printPid(int pid) {
    switch (pid) {
        case 0x04:
            Serial.print("Calculated engine load: ");
            Serial.println(pidReadAndProcessing(pid));
        break;
        case 0x05:
            Serial.print("Engine coolant temperature: ");
            Serial.println(pidReadAndProcessing(pid));
        break;
        case 0x0c:
            Serial.print("Engine speed: ");
            Serial.println(pidReadAndProcessing(pid));
        break;
        case 0x0e:
            Serial.print("Timing advance: ");
            Serial.println(pidReadAndProcessing(pid));
        break;
        case 0x0d:
            Serial.print("Vehicle speed: ");
            Serial.println(pidReadAndProcessing(pid));
        break;
        case 0x0f:
            Serial.print("Intake air temperature: ");
            Serial.println(pidReadAndProcessing(pid));
        break;
        case 0x11:
            Serial.print("Throttle position: ");
            Serial.println(pidReadAndProcessing(pid));
        break;
        case 0x1f:
            Serial.print("Run time since engine start: ");
            Serial.println(pidReadAndProcessing(pid));
        break;
        case 0x2f:
            Serial.print("Fuel Tank Level Input: ");
            Serial.println(pidReadAndProcessing(pid));
        break;
        case 0x33:
            Serial.print("Absolute Barometric Pressure: ");
            Serial.println(pidReadAndProcessing(pid));
        break;
        case 0x42:
            Serial.print("Control module voltage: ");
            Serial.println(pidReadAndProcessing(pid));
        break;
        case 0x43:
            Serial.print("Absolute load value: ");
            Serial.println(pidReadAndProcessing(pid));
        break;
        case 0x46:
            Serial.print("Ambient air temperature: ");
            Serial.println(pidReadAndProcessing(pid));
        break;
        case 0x47:
            Serial.print("Absolute throttle position B: ");
            Serial.println(pidReadAndProcessing(pid));
        break;
        case 0x49:
            Serial.print("Accelerator pedal position D: ");
            Serial.println(pidReadAndProcessing(pid));
        break;
        case 0x4a:
            Serial.print("Accelerator pedal position E: ");
            Serial.println(pidReadAndProcessing(pid));
        break;
        case 0x4c:
            Serial.print("Commanded throttle actuator: ");
            Serial.println(pidReadAndProcessing(pid));
        break;
        case 0x5c:
            Serial.print("Engine oil temperature: ");
            Serial.println(pidReadAndProcessing(pid));
        break;
        default:
            Serial.println("Nem támogatott PID!");
  }
}
float pidReadAndProcessing(int pid) {
  #define A value[0]
  #define B value[1]
  #define C value[2]
  #define D value[3]
  uint8_t value[4];

  if (!pidRead(0x01, pid, value, sizeof(value))){
    return NAN;
  }

    switch (pid) {
        case 0x04:
            return ((100*A)/255);
        break;
        case 0x05:
            return (A-40);
        break;
        case 0x0c:
            return (((256*A)+B)/4);
        break;
        case 0x0e:
            return ((A/2)-64);
        break;
        case 0x0d:
            return A;
        break;
        case 0x0f:
            return (A-40);
        break;
        case 0x11:
            return ((100*A)/255);
        break;
        case 0x1f:
            return ((256*A)+B);
        break;
        case 0x2f:
            return ((100*A)/255);
        break;
        case 0x33:
            return A;
        break;
        case 0x42:
            return (((256*A)+B)/1000);
        break;
        case 0x43:
            return ((100*((256*A)+B))/255);
        break;
        case 0x46:
            return (A-40);
        break;
        case 0x47:
            return ((100*A)/255);
        break;
        case 0x49:
            return ((100*A)/255);
        break;
        case 0x4a:
            return ((100*A)/255);
        break;
        case 0x4c:
            return ((100*A)/255);
        break;
        case 0x5c:
            return (A-40);
        break;
        default:
            return A;
  }
}
