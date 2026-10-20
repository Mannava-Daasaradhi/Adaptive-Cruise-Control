// cacc hardware-in-the-loop device for Arduino (Uno, Nano, Mega, ESP32).
//
// Speaks the serial protocol of cacc/adapters/hil_device.py:
//   request  0xA5 + REQ (40 bytes) + checksum
//   reply    0x5A + REP (12 bytes) + checksum      checksum = byte sum mod 256
// REQ = magic 0xCACC (u16), car (u16), seq (u32), t e e_dot dv u_ff gap v a (float32)
// REP = magic, car, seq, u (float32)          all little-endian (AVR and ESP32 are)
//
// Runs the Ploeg-2014 CACC law  h*xi' = -xi + kp*e + kd*e' + u_ff  per car.
// Replace control() with your own law.
//
// PC side:  cacc hil --serial COM5 --maneuver hard-brake --realtime

const float KP = 0.2, KD = 0.7, H = 0.7;
const int MAX_CARS = 16;
const uint16_t MAGIC = 0xCACC;

struct __attribute__((packed)) Req {
  uint16_t magic, car;
  uint32_t seq;
  float t, e, e_dot, dv, u_ff, gap, v, a;
};
struct __attribute__((packed)) Rep {
  uint16_t magic, car;
  uint32_t seq;
  float u;
};

float xi[MAX_CARS];
float t_last[MAX_CARS];
bool seen[MAX_CARS];

uint8_t checksum(const uint8_t *b, size_t n) {
  uint8_t s = 0;
  for (size_t i = 0; i < n; i++) s += b[i];
  return s;
}

float control(const Req &r) {
  int c = r.car % MAX_CARS;
  float dt = seen[c] ? r.t - t_last[c] : 0.0f;
  if (dt < 0) dt = 0;
  xi[c] += dt / H * (-xi[c] + KP * r.e + KD * r.e_dot + r.u_ff);
  t_last[c] = r.t;
  seen[c] = true;
  return xi[c];
}

void setup() {
  Serial.begin(115200);
}

void loop() {
  if (Serial.available() < 1 || Serial.read() != 0xA5) return;
  Req req;
  uint8_t sum;
  if (Serial.readBytes((char *)&req, sizeof(req)) != sizeof(req)) return;
  if (Serial.readBytes((char *)&sum, 1) != 1) return;
  if (sum != checksum((const uint8_t *)&req, sizeof(req)) || req.magic != MAGIC) return;
  Rep rep = {MAGIC, req.car, req.seq, control(req)};
  Serial.write(0x5A);
  Serial.write((const uint8_t *)&rep, sizeof(rep));
  Serial.write(checksum((const uint8_t *)&rep, sizeof(rep)));
}
