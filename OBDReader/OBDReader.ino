#include <CAN.h>
const bool useStandardAddressing = true;
unsigned long _lastPidResponseMillis = 0;
unsigned long _responseTimeout = 2000;
bool _useExtendedAddressing = false;

void setup() {
  Serial.begin(9600);
  Serial.println("CAN OBD-II supported pids");

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
  #define A value[0]
  #define B value[1]
  #define C value[2]
  #define D value[3]
  uint8_t value[4];
  int pid = 0x00;
  
  if (pidRead(0x01, pid, value, sizeof(value))){
    Serial.print("A: ");
    Serial.println(A);
    Serial.print("B: ");
    Serial.println(B);
    Serial.print("C: ");
    Serial.println(C);
    Serial.print("D: ");
    Serial.println(D);
  } else {
     Serial.println("Hibás olvasás");
  }
  
  while(1);
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
