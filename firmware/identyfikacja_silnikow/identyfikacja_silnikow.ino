#include <Arduino.h>

const int ENC1_A = 17;
const int ENC1_B = 13;
const int DIR1   = 5;
const int PWM1   = 4;

const int ENC2_A = 15;
const int ENC2_B = 16;
const int DIR2   = 7;
const int PWM2   = 6;

volatile long ticks_L = 0;
volatile long ticks_P = 0;

void IRAM_ATTR isrA1() { ticks_L += (digitalRead(ENC1_A) == digitalRead(ENC1_B)) ? 1 : -1; }
void IRAM_ATTR isrB1() { ticks_L += (digitalRead(ENC1_A) == digitalRead(ENC1_B)) ? -1 : 1; }
void IRAM_ATTR isrA2() { ticks_P += (digitalRead(ENC2_A) == digitalRead(ENC2_B)) ? 1 : -1; }
void IRAM_ATTR isrB2() { ticks_P += (digitalRead(ENC2_A) == digitalRead(ENC2_B)) ? -1 : 1; }

const int MOT_DIR[2] = {DIR1, DIR2};
const int MOT_PWM[2] = {PWM1, PWM2};
volatile long *const MOT_TICKS[2] = {&ticks_L, &ticks_P};
const char MOT_NAME[2] = {'L', 'P'};

const int      N       = 240;
const int      STEP_AT = 40;
const uint32_t TS_US   = 5000;
const float    TS      = TS_US * 1e-6f;
const int      H       = 2;
const int      N_SS    = 100;
const float    MIN_DW  = 150.0f;
const float    MIN_W   = 150.0f;

const int LEVELS[] = {20, 30, 40, 50, 60, 70, 80, 90};
const int N_LEVELS = sizeof(LEVELS) / sizeof(LEVELS[0]);
const int SIGN_TEST_PCT = 40;

float cprSum[2] = {0.0f, 0.0f};
int   cprRuns[2] = {0, 0};

long  pos[N];
float w[N];

bool cprKnown(int m) { return cprRuns[m] > 0; }
float getCpr(int m) { return cprKnown(m) ? cprSum[m] / cprRuns[m] : 9600.0f; }

void stopMotors() {
  analogWrite(PWM1, 0);
  analogWrite(PWM2, 0);
}

float findCross(float level) {
  for (int i = STEP_AT; i < N - H; i++) {
    if (w[i] >= level) {
      float t1 = (i - STEP_AT + 1) * TS;
      float t0 = t1 - TS;
      float y0 = w[i - 1], y1 = w[i];
      if (y1 == y0) return t1;
      float t = t0 + (level - y0) / (y1 - y0) * (t1 - t0);
      return t < 0 ? 0 : t;
    }
  }
  return -1.0f;
}

void runStep(int m, bool fwd, int uPrev, int uNew, float sgn, float &wss, float &tau, float &L) {
  digitalWrite(MOT_DIR[m], fwd ? HIGH : LOW);
  analogWrite(MOT_PWM[m], (int)(uPrev * 2.55f));

  uint32_t next = micros() + TS_US;
  for (int i = 0; i < N; i++) {
    while ((int32_t)(micros() - next) < 0) {}
    next += TS_US;
    pos[i] = *MOT_TICKS[m];
    if (i == STEP_AT - 1) analogWrite(MOT_PWM[m], (int)(uNew * 2.55f));
  }

  for (int i = H; i < N - H; i++)
    w[i] = sgn * (pos[i + H] - pos[i - H]) / (2 * H * TS);
  for (int i = 0; i < H; i++) w[i] = w[H];
  for (int i = N - H; i < N; i++) w[i] = w[N - H - 1];

  float w0 = 0.0f;
  int c0 = 0;
  for (int i = H; i < STEP_AT - H - 1; i++) { w0 += w[i]; c0++; }
  w0 /= c0;

  wss = 0.0f;
  for (int i = N - N_SS - H; i < N - H; i++) wss += w[i];
  wss /= N_SS;

  tau = -1.0f;
  L = -1.0f;
  float dW = wss - w0;
  if (uPrev > 0 && dW > MIN_DW) {
    float t283 = findCross(w0 + 0.283f * dW);
    float t632 = findCross(w0 + 0.632f * dW);
    if (t283 >= 0 && t632 > t283) {
      tau = 1.5f * (t632 - t283);
      L = t632 - tau;
      if (L < 0) L = 0;
    }
  }
}

bool regress(const float *x, const float *y, int n, float &slope, float &icpt, float &r2) {
  if (n < 3) return false;
  float sx = 0, sy = 0, sxx = 0, sxy = 0, syy = 0;
  for (int i = 0; i < n; i++) {
    sx += x[i]; sy += y[i];
    sxx += x[i] * x[i]; sxy += x[i] * y[i]; syy += y[i] * y[i];
  }
  float den = n * sxx - sx * sx;
  if (fabsf(den) < 1e-9f) return false;
  slope = (n * sxy - sx * sy) / den;
  icpt = (sy - slope * sx) / n;
  float ssTot = syy - sy * sy / n;
  float ssRes = 0;
  for (int i = 0; i < n; i++) {
    float e = y[i] - (slope * x[i] + icpt);
    ssRes += e * e;
  }
  r2 = ssTot > 0 ? 1.0f - ssRes / ssTot : 1.0f;
  return true;
}

float median(float *a, int n) {
  for (int i = 1; i < n; i++) {
    float v = a[i];
    int j = i - 1;
    while (j >= 0 && a[j] > v) { a[j + 1] = a[j]; j--; }
    a[j + 1] = v;
  }
  return (n % 2) ? a[n / 2] : 0.5f * (a[n / 2 - 1] + a[n / 2]);
}

void identMotor(int m) {
  Serial.printf("\n##### Silnik %c #####\n", MOT_NAME[m]);
  stopMotors();
  delay(500);

  long t0 = *MOT_TICKS[m];
  digitalWrite(MOT_DIR[m], HIGH);
  analogWrite(MOT_PWM[m], (int)(SIGN_TEST_PCT * 2.55f));
  delay(500);
  long dTicks = *MOT_TICKS[m] - t0;
  stopMotors();
  delay(1500);

  if (labs(dTicks) < 20) {
    Serial.printf("Test kierunku: ticki=%ld. Silnik nie rusza albo enkoder nie liczy. Przerywam.\n", dTicks);
    return;
  }
  float encSign = dTicks > 0 ? 1.0f : -1.0f;
  Serial.printf("Test kierunku (DIR=HIGH, %d%%): ticki=%ld  ->  ENC_SIGN_%c = %.1f\n",
                SIGN_TEST_PCT, dTicks, MOT_NAME[m], encSign);

  float cpr = getCpr(m);
  float toRad = 2.0f * PI / cpr;
  if (!cprKnown(m)) Serial.printf("UWAGA: CPR_%c nieskalibrowany, uzyto 64. Wyniki w rad/s sa orientacyjne.\n", MOT_NAME[m]);
  Serial.printf("CPR_%c = %.2f\n", MOT_NAME[m], cpr);

  float fwdK = 0, fwdU0 = 0, fwdTau = 0, fwdL = 0, fwdMax = 0, fwdR2 = 0;
  bool fwdOk = false;

  for (int d = 0; d < 2; d++) {
    bool fwd = (d == 0);
    float sgn = encSign * (fwd ? 1.0f : -1.0f);

    float xs[N_LEVELS], ys[N_LEVELS], taus[N_LEVELS], Ls[N_LEVELS];
    int nm = 0, nt = 0;
    float wMax = 0.0f;

    Serial.printf("\n--- kierunek: %s ---\n", fwd ? "do przodu (DIR=HIGH)" : "do tylu (DIR=LOW)");
    int uPrev = 0;
    for (int l = 0; l < N_LEVELS; l++) {
      float wss, tau, L;
      runStep(m, fwd, uPrev, LEVELS[l], sgn, wss, tau, L);
      float om = wss * toRad;
      if (om > wMax) wMax = om;
      Serial.printf("u=%2d%%  omega_ust=%7.2f rad/s", LEVELS[l], om);
      if (tau >= 0) Serial.printf("  tau=%.3f s  L=%.3f s", tau, L);
      Serial.println();
      if (wss > MIN_W) { xs[nm] = LEVELS[l]; ys[nm] = om; nm++; }
      if (tau >= 0) { taus[nt] = tau; Ls[nt] = L; nt++; }
      uPrev = LEVELS[l];
    }
    stopMotors();
    delay(1500);

    float K, b, r2;
    if (!regress(xs, ys, nm, K, b, r2) || K <= 0) {
      Serial.println("Za malo punktow z ruchem albo regresja bez sensu.");
      continue;
    }
    float u0 = -b / K;
    float tauMed = nt > 0 ? median(taus, nt) : -1.0f;
    float LMed = nt > 0 ? median(Ls, nt) : -1.0f;
    Serial.printf("K=%.4f rad/s na 1%% PWM  u0=%.2f %%  R2=%.4f  omega_max(%d%%)=%.2f rad/s\n",
                  K, u0, r2, LEVELS[N_LEVELS - 1], wMax);
    if (nt > 0) Serial.printf("tau(mediana z %d krokow)=%.4f s  L(mediana)=%.4f s\n", nt, tauMed, LMed);

    if (fwd) {
      fwdK = K; fwdU0 = u0; fwdTau = tauMed; fwdL = LMed; fwdMax = wMax; fwdR2 = r2;
      fwdOk = true;
    } else if (fwdOk) {
      Serial.printf("Porownanie przod/tyl: K %.4f / %.4f,  u0 %.2f / %.2f\n", fwdK, K, fwdU0, u0);
    }
  }

  if (fwdOk) {
    Serial.printf("\n=== DO WKLEJENIA (silnik %c, jazda do przodu) ===\n", MOT_NAME[m]);
    Serial.printf("const int   CPR_%c = %.0f;\n", MOT_NAME[m], cpr);
    Serial.printf("const float ENC_SIGN_%c = %.1ff;\n", MOT_NAME[m], encSign);
    Serial.printf("const float K_%c = %.4ff;\n", MOT_NAME[m], fwdK);
    Serial.printf("const float u0_%c = %.2ff;\n", MOT_NAME[m], fwdU0);
    if (fwdTau > 0) Serial.printf("const float tau_%c = %.4ff;\n", MOT_NAME[m], fwdTau);
    Serial.printf("// R2=%.4f, omega_max=%.2f rad/s\n", fwdR2, fwdMax);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC1_A), isrA1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_B), isrB1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), isrA2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_B), isrB2, CHANGE);

  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM2, OUTPUT);
  stopMotors();

  Serial.println("Komendy (zakoncz Enter):");
  Serial.println("  z        zeruj liczniki obu enkoderow");
  Serial.println("  pL N     CPR lewego po N obrotach walu (np. pL 10), uzywa sredniej z prob");
  Serial.println("  pP N     to samo dla prawego");
  Serial.println("  clr      wyczysc skalibrowane CPR");
  Serial.println("  t        pelna identyfikacja obu silnikow (kola w powietrzu!)");
  Serial.println("  tL / tP  identyfikacja jednego silnika");
}

void loop() {
  if (!Serial.available()) return;

  String s = Serial.readStringUntil('\n');
  s.trim();

  if (s == "z") {
    ticks_L = 0;
    ticks_P = 0;
    Serial.println("Liczniki wyzerowane.");
  } else if (s.startsWith("pL") || s.startsWith("pP")) {
    int m = (s[1] == 'L') ? 0 : 1;
    float revs = s.substring(2).toFloat();
    if (revs <= 0) revs = 10;
    long t = *MOT_TICKS[m];
    float cpr = fabsf((float)t) / revs;
    cprSum[m] += cpr;
    cprRuns[m]++;
    Serial.printf("Silnik %c: ticki=%ld  obroty=%.1f  CPR=%.2f  |  srednia z %d prob: %.2f\n",
                  MOT_NAME[m], t, revs, cpr, cprRuns[m], cprSum[m] / cprRuns[m]);
  } else if (s == "clr") {
    for (int m = 0; m < 2; m++) { cprSum[m] = 0.0f; cprRuns[m] = 0; }
    Serial.println("CPR wyczyszczone.");
  } else if (s == "t") {
    identMotor(0);
    identMotor(1);
    Serial.println("\nKoniec.");
  } else if (s == "tL") {
    identMotor(0);
    Serial.println("\nKoniec.");
  } else if (s == "tP") {
    identMotor(1);
    Serial.println("\nKoniec.");
  }
}
