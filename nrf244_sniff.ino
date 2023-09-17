#include <RF24.h>
#include <RF24_config.h>
#include <nRF24L01.h>
#include <LibPrintf.h>
#include <SPI.h>

const int CE = 7;
const int CSN = 8;

// TODO: Iterate over different channels
int channel = 34;

RF24 rf(CE, CSN);

uint8_t msg[32];

// To sniff out the packet without knowing the MAC addresss.
// These bytes will be found out in the preamble of the packet,
// and can be used to confuse the chip to read this as MAC addr.
// Another alternate preamble is 0x0055.
// uint8_t addr[2] = {0x00, 0xAA};
// uint8_t addr[3] = {0xCD, 0x34, 0xDA};
uint8_t addr[5] = {0xCD, 0x02, 0x03, 0x04, 0x05};

void wait_forever() {
  while (true)
    asm volatile ("wfe");
}

void printMsg() {
  printf("[");
  for (int i = 0; i < sizeof(msg); i++) {
    if (i != 0) printf(", ");
    printf("%02x", msg[i]);
  }
  printf("]");
}

void write_register(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  SPI.transfer(W_REGISTER | reg);
  SPI.transfer(val);
  SPI.endTransaction();
}

void setup() {
  Serial.begin(9600);

  if (!rf.begin()) {
    printf("NRF24 setup failed\n");
    wait_forever();
  }

  rf.setAutoAck(false);
  rf.setPALevel(RF24_PA_MIN);
  rf.setDataRate(RF24_2MBPS);
  rf.setPayloadSize(sizeof(msg));
  rf.setChannel(channel);

  rf.setAddressWidth(sizeof(addr));
  write_register(0x02, 0x00);
  rf.openReadingPipe(0, addr);
  rf.disableCRC();
  rf.startListening();
  rf.printDetails();

  while (true) {
    if (rf.available()) {
      rf.read(msg, sizeof(msg));
      printf("Read msg: ");
      printMsg();
      printf("\n");
    }
    delay(500);
  }
}

void loop() {
}
