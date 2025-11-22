#include <AccelStepper.h>    
#include <BluetoothSerial.h>
#include <ESP32Servo.h>
#include <vector>

BluetoothSerial SerialBT;

// ESC Pins
const int esc1Pin = 21;
const int esc2Pin = 19;
const int esc3Pin = 18;

// Stepper motor pins
#define DIR_PIN 14
#define STEP_PIN 13
#define STEP_DELAY_MS 5
#define STEPS_TO_MOVE 30

//stepper loading ball
#define STEP_PIN_LOADING 26
#define DIR_PIN_LOADING 27
#define M0_PIN 12
#define M1_PIN 32
#define M2_PIN 33

AccelStepper stepperLoad(AccelStepper::DRIVER, STEP_PIN_LOADING, DIR_PIN_LOADING);
int lastStepperSpeed = 0;
unsigned long lastStepTime = 0;
const int loadingStepDelay = 20;  // Kecepatan stepper loading
bool stepState = false;

// Proximity sensor pin
const int proximitySensor = 4;

// Motor & ESC
Servo esc1, esc2, esc3;

// Stepper posisi
int posisiStepper = 0;
String targetSebelumnya = "";

// Struktur sesi
struct Sesi {
  int jumlah;
  String spin;
  String target;
};

std::vector<Sesi> sesiList;
int sesiIndex = 0;
bool sesiAktif = false;
bool semuaSesiSelesai = false;
int detectedCount = 0;
const int delayAntarSesi = 1000;

// Sensor IR debounce
int lastVal = 0;
unsigned long lastTriggerTime = 0;
const unsigned long debounceDelay = 200;

// PWM values
int pwmSpin1 = 0;
int pwmSpin2 = 0;
int pwmSpin3 = 0;

int getPosisiDariTarget(String target) {
  if (target == "Target 1" || target == "Target 3") return -1;
  if (target == "Target 2" || target == "Target 4") return 1;
  return 0;
}

// Stepper movement
void stepMotor(int steps) {
  Serial.print("Stepper jalan: ");
  Serial.println(steps);
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delay(STEP_DELAY_MS);
    digitalWrite(STEP_PIN, LOW);
    delay(STEP_DELAY_MS);
  }
}


// Fungsi reset posisi ke tengah
void resetStepper() {
  if (posisiStepper == 0) return;

  if (posisiStepper > 0) {
    digitalWrite(DIR_PIN, LOW);  // balik ke kiri
    delay(5);
    stepMotor(posisiStepper * STEPS_TO_MOVE);
  }
  else if (posisiStepper < 0) {
    digitalWrite(DIR_PIN, HIGH);  // balik ke kanan
    delay(5);
    stepMotor(abs(posisiStepper) * STEPS_TO_MOVE);
  }

  posisiStepper = 0;
}


// Cek sisi target: kanan = Target 1 & 3, kiri = Target 2 & 4
bool sisiKanan(String target) {
  return (target == "Target 1" || target == "Target 3");
}

// Arahkan stepper ke target
void aturStepperUntukTarget(String target) {
  Serial.println("Memanggil aturStepperUntukTarget");

  int posisiBaru = getPosisiDariTarget(target);
  int delta = posisiBaru - posisiStepper;

  if (delta == 0) return;  // Sudah di posisi

  digitalWrite(DIR_PIN, (delta > 0) ? LOW : HIGH);  // LOW = kiri, HIGH = kanan
  stepMotor(abs(delta) * STEPS_TO_MOVE);
  posisiStepper = posisiBaru;
}

// Fungsi atur spin
void aturSpin(String spin, String target) {
  pwmSpin1 = pwmSpin2 = pwmSpin3 = 1000;

  if (spin == "NOSPIN") {
    if (target == "Target 1")      { pwmSpin1 = 1065; pwmSpin2 = 1065; pwmSpin3 = 1065; }
    else if (target == "Target 2") { pwmSpin1 = 1065; pwmSpin2 = 1065; pwmSpin3 = 1065; }
    else if (target == "Target 3") { pwmSpin1 = 1065; pwmSpin2 = 1065; pwmSpin3 = 1065; }
    else if (target == "Target 4") { pwmSpin1 = 1065; pwmSpin2 = 1065; pwmSpin3 = 1065; }
  } 
  else if (spin == "TOPSPIN") {
    if (target == "Target 1")      { pwmSpin1 = 1000; pwmSpin2 = 1095; pwmSpin3 = 1095; }
    else if (target == "Target 2") { pwmSpin1 = 1000; pwmSpin2 = 1095; pwmSpin3 = 1095; }
    else if (target == "Target 3") { pwmSpin1 = 1000; pwmSpin2 = 1055; pwmSpin3 = 1055; }
    else if (target == "Target 4") { pwmSpin1 = 1000; pwmSpin2 = 1055; pwmSpin3 = 1055; }
  } 
  else if (spin == "BACKSPIN") {
    if (target == "Target 1")      { pwmSpin1 = 1155; pwmSpin2 = 1000; pwmSpin3 = 1000; }
    else if (target == "Target 2") { pwmSpin1 = 1155; pwmSpin2 = 1000; pwmSpin3 = 1000; }
    else if (target == "Target 3") { pwmSpin1 = 1135; pwmSpin2 = 1000; pwmSpin3 = 1000; }
    else if (target == "Target 4") { pwmSpin1 = 1135; pwmSpin2 = 1000; pwmSpin3 = 1000; }
  } 
  else if (spin == "SIDESPIN_T") {
    if (target == "Target 1")      { pwmSpin1 = 1065; pwmSpin2 = 1075; pwmSpin3 = 1000; }
    else if (target == "Target 2") { pwmSpin1 = 1065; pwmSpin2 = 1000; pwmSpin3 = 1075; }
    else if (target == "Target 3") { pwmSpin1 = 1065; pwmSpin2 = 1075; pwmSpin3 = 1000; }
    else if (target == "Target 4") { pwmSpin1 = 1065; pwmSpin2 = 1000; pwmSpin3 = 1075; }
  }
  else if (spin == "SIDESPIN_B") {
    if (target == "Target 1")      { pwmSpin1 = 1075; pwmSpin2 = 1065; pwmSpin3 = 1000; }
    else if (target == "Target 2") { pwmSpin1 = 1075; pwmSpin2 = 1000; pwmSpin3 = 1065; }
    else if (target == "Target 3") { pwmSpin1 = 1075; pwmSpin2 = 1065; pwmSpin3 = 1000; }
    else if (target == "Target 4") { pwmSpin1 = 1075; pwmSpin2 = 1000; pwmSpin3 = 1065; }
  }

  esc1.writeMicroseconds(pwmSpin1);
  esc2.writeMicroseconds(pwmSpin2);
  esc3.writeMicroseconds(pwmSpin3);

  Serial.printf("Spin: %s | Target: %s | PWM1: %d | PWM2: %d | PWM3: %d\n",
                spin.c_str(), target.c_str(), pwmSpin1, pwmSpin2, pwmSpin3);
}

// Mulai sesi berikutnya
void mulaiSesiBerikutnya() {
  if (sesiIndex >= sesiList.size()) {
    Serial.println("Semua sesi selesai.");
    sesiAktif = false;
    semuaSesiSelesai = true;

    esc1.writeMicroseconds(1000);
    esc2.writeMicroseconds(1000);
    esc3.writeMicroseconds(1000);
    resetStepper();

    if (SerialBT.hasClient()) {
      SerialBT.println("Motor Mati");
    }
    return;
  }

  Sesi sesi = sesiList[sesiIndex];
  Serial.printf("[SESI %d] Jumlah: %d | Spin: %s | Target: %s\n",
                sesiIndex + 1, sesi.jumlah, sesi.spin.c_str(), sesi.target.c_str());

  // Tambahkan ini untuk reset jika pindah sisi (kecuali untuk SIDESPIN)
  if (sesi.spin != "SIDESPIN_T" && sesi.spin != "SIDESPIN_B" &&
      targetSebelumnya != "" && sisiKanan(targetSebelumnya) != sisiKanan(sesi.target)) {
    resetStepper();
    delay(300);
  }

  // SIDESPIN_B → posisi selalu 0
  if (sesi.spin == "SIDESPIN_B") {
    if (posisiStepper != 0) {
      resetStepper();
      delay(300);
    }
  }

  // SIDESPIN_T → posisi 0, kecuali Target 3 → +1 (kanan)
  else if (sesi.spin == "SIDESPIN_T") {
    if (sesi.target == "Target 3") {
      if (posisiStepper != -1) {
        digitalWrite(DIR_PIN, LOW);  // ke kanan
        stepMotor(abs(1 - posisiStepper) * STEPS_TO_MOVE);
        posisiStepper = -1;
        delay(300);
      }
    } else {
      if (posisiStepper != 0) {
        if (posisiStepper == 1) {
          digitalWrite(DIR_PIN, LOW);  // dari kiri ke netral
        } else if (posisiStepper == -1) {
          digitalWrite(DIR_PIN, HIGH);  // dari kanan ke netral
        }
        stepMotor(abs(posisiStepper) * STEPS_TO_MOVE);
        posisiStepper = 0;
        delay(300);
      }
    }
  }


  // TOPSPIN, BACKSPIN, NOSPIN → atur berdasarkan target
  else if (sesi.spin == "TOPSPIN" || sesi.spin == "BACKSPIN" || sesi.spin == "NOSPIN") {
    int posisiBaru = getPosisiDariTarget(sesi.target);  // -1 atau 1
    if (posisiBaru != posisiStepper) {
      digitalWrite(DIR_PIN, (posisiBaru > posisiStepper) ? HIGH : LOW);  // kanan / kiri
      stepMotor(abs(posisiBaru - posisiStepper) * STEPS_TO_MOVE);
      posisiStepper = posisiBaru;
      delay(300);
    }
  }

  aturSpin(sesi.spin, sesi.target);
  detectedCount = 0;
  sesiAktif = true;
  targetSebelumnya = sesi.target;
}

// Parsing data
void prosesData(String data) {
  data.trim();
  sesiList.clear();
  sesiIndex = 0;
  semuaSesiSelesai = false;

  int start = 0;
  int semiColonIndex = data.indexOf(';');

  while (semiColonIndex != -1) {
    String part = data.substring(start, semiColonIndex);
    part.trim();

    if (part.startsWith("Random,")) {
      String level = part.substring(part.indexOf(',') + 1);
      if (level == "Easy") {
        sesiList.push_back({10, "TOPSPIN", "Target 1"});
      } else if (level == "Medium") {
        sesiList.push_back({10, "TOPSPIN", "Target 2"});
        sesiList.push_back({10, "BACKSPIN", "Target 3"});
      } else if (level == "Hard") {
        sesiList.push_back({10, "TOPSPIN", "Target 2"});
        sesiList.push_back({10, "BACKSPIN", "Target 3"});
        sesiList.push_back({10, "SIDESPIN_T", "Target 1"});
      }
    } else {
      int p1 = part.indexOf(',');
      int p2 = part.indexOf(',', p1 + 1);
      if (p1 != -1 && p2 != -1) {
        int jumlah = part.substring(0, p1).toInt();
        String spin = part.substring(p1 + 1, p2);
        String target = part.substring(p2 + 1);
        sesiList.push_back({jumlah, spin, target});
      }
    }

    start = semiColonIndex + 1;
    semiColonIndex = data.indexOf(';', start);
  }

  String last = data.substring(start);
  last.trim();
  if (last.length() > 0) {
    if (last.startsWith("Random,")) {
      String level = last.substring(last.indexOf(',') + 1);
      if (level == "Easy") {
        sesiList.push_back({10, "TOPSPIN", "Target 1"});
      } else if (level == "Medium") {
        sesiList.push_back({10, "TOPSPIN", "Target 2"});
        sesiList.push_back({10, "BACKSPIN", "Target 3"});
      } else if (level == "Hard") {
        sesiList.push_back({10, "TOPSPIN", "Target 2"});
        sesiList.push_back({10, "BACKSPIN", "Target 3"});
        sesiList.push_back({10, "SIDESPIN_T", "Target 1"});
    }
    } else {
      int p1 = last.indexOf(',');
      int p2 = last.indexOf(',', p1 + 1);
      if (p1 != -1 && p2 != -1) {
        int jumlah = last.substring(0, p1).toInt();
        String spin = last.substring(p1 + 1, p2);
        String target = last.substring(p2 + 1);
        sesiList.push_back({jumlah, spin, target});
      }
    }
  }

  Serial.printf("Total sesi: %d\n", sesiList.size());
  if (!sesiList.empty()) {
    delay(1000);
    mulaiSesiBerikutnya();
  }
}

void jalanLoadingBall() {
  unsigned long now = millis();
  if (now - lastStepTime >= loadingStepDelay) {
    stepState = !stepState;
    digitalWrite(STEP_PIN_LOADING, stepState);
    lastStepTime = now;
  }
}

// SETUP
void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_BT");

  pinMode(proximitySensor, INPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);

  pinMode(STEP_PIN_LOADING, OUTPUT);
  pinMode(DIR_PIN_LOADING, OUTPUT);
  digitalWrite(DIR_PIN_LOADING, HIGH);  // Arah default loading

  // Microstepping DRV8825 (1/16 step)
  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);

  digitalWrite(M0_PIN, LOW);
  digitalWrite(M1_PIN, LOW);
  digitalWrite(M2_PIN, HIGH);

  stepperLoad.setMaxSpeed(1000);
  stepperLoad.setSpeed(0);  // awalnya diam
  lastStepperSpeed = 0;

  esc1.attach(esc1Pin, 1000, 2000);
  esc2.attach(esc2Pin, 1000, 2000);
  esc3.attach(esc3Pin, 1000, 2000);

  Serial.println("Kalibrasi ESC...");
  esc1.writeMicroseconds(2000);
  esc2.writeMicroseconds(2000);
  esc3.writeMicroseconds(2000);
  delay(2000);
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  delay(2000);
  Serial.println("Motor siap.");

  SerialBT.register_callback([](esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    if (event == ESP_SPP_SRV_OPEN_EVT) {
      Serial.println("Bluetooth terhubung.");
    } else if (event == ESP_SPP_CLOSE_EVT) {
      Serial.println("Bluetooth terputus.");
    }
  });

  Serial.println("Menunggu data...");
}

// LOOP
void loop() {
static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 100) {
    int sensorNow = digitalRead(proximitySensor);
    // Serial.print("Sensor raw: ");
    // Serial.println(sensorNow);
    lastPrint = millis();
  }

  if (SerialBT.available()) {
    String received = SerialBT.readStringUntil('\n');
    received.trim();
    Serial.println("Data diterima: " + received);
    prosesData(received);
  }

  if (sesiAktif && sesiIndex < sesiList.size()) {
    jalanLoadingBall();  // ← diganti dengan ini:
    if (lastStepperSpeed == 0) {
      stepperLoad.setSpeed(200);  // mulai loading saat sesi aktif
      lastStepperSpeed = 200;
      Serial.println("[DEBUG] Stepper Loading START dengan kecepatan 200");
    }
    stepperLoad.runSpeed();  // jalankan stepper loading

    int val = digitalRead(proximitySensor);
    unsigned long now = millis();

    // Hanya naikkan count saat transisi dari 0 -> 1
    if (lastVal == 0 && val == 1 && (now - lastTriggerTime > debounceDelay)) {
      detectedCount++;
      lastTriggerTime = now;
      Serial.printf("Bola ke-%d terdeteksi\n", detectedCount);
    }

    lastVal = val;  // Simpan nilai terakhir sensor

    if (detectedCount >= sesiList[sesiIndex].jumlah) {
      Serial.println("Jumlah bola selesai.");

      // 1. Matikan semua motor BLDC
      esc1.writeMicroseconds(1000);
      esc2.writeMicroseconds(1000);
      esc3.writeMicroseconds(1000);

      // 2. Matikan loading bola
      stepperLoad.setSpeed(0);
      lastStepperSpeed = 0;
      Serial.println("[DEBUG] Stepper Loading STOP");

      // 3. Lanjut sesi
      sesiAktif = false;
      sesiIndex++;
      delay(delayAntarSesi);
      mulaiSesiBerikutnya();
    }
  }
}
