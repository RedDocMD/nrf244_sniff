#include <RF24.h>
#include <RF24_config.h>
#include <nRF24L01.h>
#include <LibPrintf.h>
#include <SPI.h>
#include <Gaussian.h>
#include <algorithm>

const int CE = 7;
const int CSN = 8;

RF24 rf(CE, CSN);

uint8_t msg[32];

// To sniff out the packet without knowing the MAC addresss.
// These bytes will be found out in the preamble of the packet,
// and can be used to confuse the chip to read this as MAC addr.
// Another alternate preamble is 0x0055.
uint8_t addr[2] = {0xAA, 0x00};
// uint8_t addr[3] = {0xCD, 0x34, 0xDA};
// uint8_t addr[5] = {0xCD, 0x02, 0x03, 0x04, 0x05};

struct Payload {
  uint8_t dev_type;
  uint8_t pkt_type;
  uint8_t model;
  uint8_t _unk;
  uint16_t seq;
  uint8_t flags;
  uint8_t meta;
  uint8_t _pad1;
  uint8_t hid_code;
  uint8_t _pad2[5];
  uint8_t cksum;
};

struct __attribute__((packed)) Message {
  uint8_t addr[5]; // In big-endian
  Payload pay;
};

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

void print_addr(Message *mess) {
  uint8_t *a = mess->addr;
  printf("MAC addr: %02x:%02x:%02x:%02x:%02x\n",
    a[0], a[1], a[2], a[3], a[4]);
}

void write_register(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  SPI.transfer(W_REGISTER | reg);
  SPI.transfer(val);
  SPI.endTransaction();
}

void print_result(Message *mess, int chan, long time) {
  printf("{");
  printf("\"channel\": %d", chan);
  printf(",");
  printf("\"time\": %ld", time);
  printf(",");
  uint8_t *a = mess->addr;
  printf("\"mac\": \"%02x:%02x:%02x:%02x:%02x\"",
    a[0], a[1], a[2], a[3], a[4]);
  printf("}\n\n");
}

#define CHAN_MIN 3
#define CHAN_MAX 80
#define CHAN_CNT (CHAN_MAX - CHAN_MIN + 1)

int channels[CHAN_CNT];

int uniform(int m) {
  return random() % m;
}

void permute_channels() {
  int chan = CHAN_MIN;
  for (int i = 0; i < CHAN_CNT; i++)
    channels[i] = chan++;
  for (int i = 0; i < CHAN_CNT - 2; i++) {
    int j = i + uniform(CHAN_CNT - i);
    std::swap(channels[i], channels[j]);
  }
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

  rf.setAddressWidth(sizeof(addr));
  write_register(0x02, 0x00);
  rf.openReadingPipe(0, addr);
  rf.disableCRC();
  rf.startListening();
  // rf.printDetails();

  long time = 0;
  constexpr long CHANNEL_WAIT = 500;
  constexpr long CHANNEL_WAIT_DEV = 15;
  Gaussian wait_distr(CHANNEL_WAIT, CHANNEL_WAIT_DEV);

  int chan, chan_idx = 0;
  while (true) {
    if (chan_idx >= CHAN_CNT)
      chan_idx = 0;
    if (chan_idx == 0)
      permute_channels();
    chan = channels[chan_idx];
    
    // printf("Tuning to channel: %d\n", chan);
    rf.setChannel(chan);

    time = millis();
    long wait = wait_distr.random();
    while (millis() - time < wait) {
      if (rf.available()) {
        rf.read(msg, sizeof(msg));
        Message *mess = (Message *)msg;
        Payload *pay = &mess->pay;
        if (pay->model == 0x06 && pay->dev_type == 0x0A) {
          // printf("Found keyboard on channel: %d\n", channel);
          // print_addr(mess);
          // printf("Took %ld ms to find keyboard\n", millis());
          // return;
          long time = millis();
          print_result(mess, chan, time);
          NVIC_SystemReset();
        }
      }
    }

    ++chan_idx;
  }
}

void loop() {
}
