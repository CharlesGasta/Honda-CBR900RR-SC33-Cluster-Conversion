/*
  Honda CBR900RR SC33 1998 - Module tableau de bord Wi-Fi
  Version V20.2 WIFI CALIBRATION

  Base : V19.1 FINAL CONTINU validee sur la moto.

  Fonctions principales :
  - Lecture de la sonde de temperature d'origine sur A0
  - Pilotage de la jauge de temperature sur D5 via IRLZ44N
  - Animation de demarrage de la jauge
  - Lecture du regime moteur sur D6 via PC817
  - Shift-light sur D1 via IRLZ44N
  - Point d'acces Wi-Fi autonome :
      SSID     : CBR900RR
      Password : 00365412
      Adresse  : 192.168.4.1
  - Interface web locale :
      Dashboard RPM / temperature / jauge / shift
      Reglages shift-light et calibration temperature
      Tests jauge / shift / reset
      Diagnostic complet + journal d'evenements
  - Sauvegarde des reglages dans l'EEPROM

  IMPORTANT :
  - La moto fonctionne normalement meme si aucun telephone n'est connecte.
  - Le Wi-Fi ne remplace aucune fonction de securite.
  - Aucune sortie ESP n'est branchee sur le signal compte-tours d'origine.
*/

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <EEPROM.h>
#include <math.h>

// =============================================================
// VERSION / WIFI
// =============================================================

constexpr const char *FW_VERSION = "V20.2.1 WIFI CALIBRATION";
constexpr const char *WIFI_SSID = "CBR900RR";
constexpr const char *WIFI_PASSWORD = "00365412";

IPAddress AP_IP(192, 168, 4, 1);
IPAddress AP_GATEWAY(192, 168, 4, 1);
IPAddress AP_MASK(255, 255, 255, 0);

ESP8266WebServer server(80);
DNSServer dnsServer;

// =============================================================
// BROCHES
// =============================================================

constexpr uint8_t PIN_TEMP_SENSOR = A0;
constexpr uint8_t PIN_TEMP_GAUGE  = D5;
constexpr uint8_t PIN_TACH_INPUT  = D6;
constexpr uint8_t PIN_SHIFT_LIGHT = D1;
constexpr uint8_t PIN_WARNING_LED = D4;

// =============================================================
// PWM JAUGE
// =============================================================

constexpr uint16_t PWM_RANGE = 1023;
constexpr uint32_t PWM_FREQUENCY_HZ = 1000;
constexpr uint16_t TEMP_PWM_LIMIT = 1023;
constexpr uint16_t TEMP_SLEW_STEP = 4;
constexpr uint32_t TEMP_CONTROL_PERIOD_MS = 50;

// =============================================================
// SONDE / ADC
// =============================================================

constexpr uint32_t SENSOR_PULLUP_OHM = 10000UL;
constexpr uint8_t ADC_SAMPLES = 32;
constexpr uint16_t ADC_SHORT_THRESHOLD = 2;
constexpr uint16_t ADC_OPEN_THRESHOLD = 1018;

// =============================================================
// ANIMATION DEMARRAGE
// =============================================================

constexpr uint16_t STARTUP_MAX_PWM = 880;
constexpr uint16_t STARTUP_STEP_PWM = 5;
constexpr uint32_t STARTUP_STEP_MS = 14;
constexpr uint32_t STARTUP_TOP_PAUSE_MS = 500;

constexpr uint16_t STARTUP_SHIFT_WINDOWS[3][2] = {
  {450, 520},
  {600, 670},
  {750, 820}
};

// =============================================================
// COMPTE-TOURS
// =============================================================

constexpr uint8_t TACH_PULSES_PER_REVOLUTION = 4;
constexpr uint32_t TACH_MIN_PERIOD_US = 700;
constexpr uint32_t TACH_ZERO_TIMEOUT_MS = 800;
constexpr uint32_t RPM_UPDATE_MS = 100;

constexpr uint16_t SHIFT_HYSTERESIS_RPM = 200;
constexpr uint32_t SHIFT_CONFIRM_MS = 100;
constexpr uint32_t SHIFT_FLASH_HALF_PERIOD_MS = 60;

// =============================================================
// REGLAGES SAUVEGARDES
// =============================================================

constexpr uint32_t SETTINGS_MAGIC = 0x43425232UL; // "CBR2"
constexpr uint16_t SETTINGS_VERSION = 3;
constexpr size_t EEPROM_SIZE_BYTES = 512;

struct Settings {
  uint32_t magic;
  uint16_t version;

  uint16_t shiftOnRpm;
  uint16_t shiftFlashRpm;

  // Facteur de tarage du compte-tours. 1.0 = aucune correction.
  float tachCalibrationFactor;

  // Courbe resistance -> position de jauge.
  float gaugeColdOhm;   // 0 %
  float gaugeHotOhm;    // 110 %

  // Calibration thermometre NTC par deux points.
  float tempColdOhm;
  float tempColdC;
  float tempHotOhm;
  float tempHotC;
  float tempErrorPercent;

  // Calibration position -> PWM de la jauge.
  uint16_t pwm0;
  uint16_t pwm25;
  uint16_t pwm50;
  uint16_t pwm75;
  uint16_t pwm100;
  uint16_t pwm110;
};

Settings settings;

void loadDefaultSettings() {
  settings.magic = SETTINGS_MAGIC;
  settings.version = SETTINGS_VERSION;

  settings.shiftOnRpm = 9000;
  settings.shiftFlashRpm = 10000;
  settings.tachCalibrationFactor = 1.0f;

  // Calibration jauge validee / retenue.
  settings.gaugeColdOhm = 35000.0f;
  settings.gaugeHotOhm = 800.0f;

  // Thermometre :
  // point froid = moteur a temperature ambiante
  // point chaud = declenchement ventilateur retenu a 102,5 C
  settings.tempColdOhm = 35000.0f;
  settings.tempColdC = 25.0f;
  settings.tempHotOhm = 1250.0f;
  settings.tempHotC = 102.5f;
  settings.tempErrorPercent = 5.0f;

  settings.pwm0 = 400;
  settings.pwm25 = 650;
  settings.pwm50 = 725;
  settings.pwm75 = 800;
  settings.pwm100 = 880;
  settings.pwm110 = 900;
}

bool settingsAreValid() {
  if (settings.magic != SETTINGS_MAGIC) return false;
  if (settings.version != SETTINGS_VERSION) return false;

  if (settings.shiftOnRpm < 1000 || settings.shiftOnRpm > 15000) return false;
  if (settings.shiftFlashRpm < 1000 || settings.shiftFlashRpm > 16000) return false;
  if (settings.shiftFlashRpm <= settings.shiftOnRpm) return false;

  if (!isfinite(settings.tachCalibrationFactor) ||
      settings.tachCalibrationFactor < 0.50f ||
      settings.tachCalibrationFactor > 1.50f) return false;

  if (settings.gaugeColdOhm < 1000.0f || settings.gaugeColdOhm > 100000.0f) return false;
  if (settings.gaugeHotOhm < 50.0f || settings.gaugeHotOhm >= settings.gaugeColdOhm) return false;

  if (settings.tempColdOhm < 1000.0f || settings.tempColdOhm > 100000.0f) return false;
  if (settings.tempHotOhm < 50.0f || settings.tempHotOhm >= settings.tempColdOhm) return false;
  if (settings.tempColdC < -20.0f || settings.tempColdC > 60.0f) return false;
  if (settings.tempHotC < 60.0f || settings.tempHotC > 140.0f) return false;
  if (settings.tempHotC <= settings.tempColdC) return false;
  if (settings.tempErrorPercent < 0.0f || settings.tempErrorPercent > 30.0f) return false;

  if (!(settings.pwm0 < settings.pwm25 &&
        settings.pwm25 < settings.pwm50 &&
        settings.pwm50 < settings.pwm75 &&
        settings.pwm75 < settings.pwm100 &&
        settings.pwm100 < settings.pwm110 &&
        settings.pwm110 <= TEMP_PWM_LIMIT)) {
    return false;
  }

  return true;
}

// =============================================================
// JOURNAL EVENEMENTS
// =============================================================

constexpr uint8_t LOG_COUNT = 24;
constexpr uint8_t LOG_LEN = 86;

char eventLog[LOG_COUNT][LOG_LEN];
uint8_t eventLogHead = 0;
uint8_t eventLogUsed = 0;

void addLog(const char *message) {
  const uint32_t seconds = millis() / 1000UL;
  const uint32_t mm = seconds / 60UL;
  const uint32_t ss = seconds % 60UL;

  snprintf(
    eventLog[eventLogHead],
    LOG_LEN,
    "[%02lu:%02lu] %s",
    static_cast<unsigned long>(mm),
    static_cast<unsigned long>(ss),
    message
  );

  eventLogHead = (eventLogHead + 1) % LOG_COUNT;
  if (eventLogUsed < LOG_COUNT) eventLogUsed++;
}

void saveSettings() {
  settings.magic = SETTINGS_MAGIC;
  settings.version = SETTINGS_VERSION;

  EEPROM.put(0, settings);
  EEPROM.commit();
  addLog("Reglages sauvegardes en EEPROM");
}

void loadSettings() {
  EEPROM.begin(EEPROM_SIZE_BYTES);
  EEPROM.get(0, settings);

  if (!settingsAreValid()) {
    loadDefaultSettings();
    EEPROM.put(0, settings);
    EEPROM.commit();
    addLog("EEPROM invalide : reglages par defaut charges");
  } else {
    addLog("Reglages EEPROM charges");
  }
}

// =============================================================
// ETATS
// =============================================================

enum TempMode {
  TEMP_AUTOMATIC,
  TEMP_MANUAL
};

enum SensorState {
  SENSOR_OK,
  SENSOR_SHORT,
  SENSOR_OPEN
};

enum TachMode {
  TACH_REAL,
  TACH_SIMULATION
};

TempMode tempMode = TEMP_AUTOMATIC;
SensorState sensorState = SENSOR_OPEN;
TachMode tachMode = TACH_REAL;

uint16_t lastRawAdc = 0;
uint16_t lastAdc = 0;
uint32_t filteredAdcX16 = 0;
bool filteredAdcInitialized = false;

uint16_t measuredResistanceOhm = 0;
float calculatedTempC = NAN;
float calculatedTempErrorC = NAN;
float needlePercent = 0.0f;

uint16_t targetTempPwm = 400;
uint16_t appliedTempPwm = 400;
uint16_t manualTempPwm = 400;

uint16_t simulatedRpm = 0;
uint16_t rawRpm = 0;
uint16_t activeRpm = 0;

bool shiftEnabled = true;
bool warningLedState = false;
uint32_t lastWarningBlinkMs = 0;

// Redémarrage différé demandé depuis l'interface Web.
// Ces variables doivent être déclarées avant handleAction().
bool restartPending = false;
uint32_t restartAtMs = 0;

// =============================================================
// TACH ISR
// =============================================================

volatile uint32_t tachLastPulseUs = 0;
volatile uint32_t tachPeriodUs = 0;
volatile uint32_t tachLastValidPulseUs = 0;

void IRAM_ATTR tachPulseInterrupt() {
  const uint32_t nowUs = micros();

  if (tachLastPulseUs == 0) {
    tachLastPulseUs = nowUs;
    tachLastValidPulseUs = nowUs;
    return;
  }

  const uint32_t period = nowUs - tachLastPulseUs;
  tachLastPulseUs = nowUs;

  if (period < TACH_MIN_PERIOD_US) return;

  tachPeriodUs = period;
  tachLastValidPulseUs = nowUs;
}

// =============================================================
// OUTILS SORTIES
// =============================================================

void setWarningLed(bool on) {
  warningLedState = on;
  digitalWrite(PIN_WARNING_LED, on ? LOW : HIGH);
}

void setShiftLight(bool on) {
  digitalWrite(PIN_SHIFT_LIGHT, on ? HIGH : LOW);
}

void applyTempPwm(uint16_t pwm) {
  if (pwm > TEMP_PWM_LIMIT) pwm = TEMP_PWM_LIMIT;
  appliedTempPwm = pwm;
  analogWrite(PIN_TEMP_GAUGE, pwm);
}

uint16_t slewPwm(uint16_t current, uint16_t target) {
  if (target > current + TEMP_SLEW_STEP) return current + TEMP_SLEW_STEP;
  if (current > target + TEMP_SLEW_STEP) return current - TEMP_SLEW_STEP;
  return target;
}

// =============================================================
// LECTURE SONDE
// =============================================================

uint16_t readAdcAverage() {
  uint32_t total = 0;

  for (uint8_t i = 0; i < ADC_SAMPLES; ++i) {
    total += analogRead(PIN_TEMP_SENSOR);
    delayMicroseconds(200);
  }

  return static_cast<uint16_t>(total / ADC_SAMPLES);
}

void updateTemperatureSensor() {
  const uint16_t rawAdc = readAdcAverage();
  lastRawAdc = rawAdc;

  if (!filteredAdcInitialized) {
    filteredAdcX16 = static_cast<uint32_t>(rawAdc) * 16UL;
    filteredAdcInitialized = true;
  } else {
    const uint32_t target = static_cast<uint32_t>(rawAdc) * 16UL;
    filteredAdcX16 = (filteredAdcX16 * 7UL + target) / 8UL;
  }

  lastAdc = static_cast<uint16_t>(filteredAdcX16 / 16UL);

  if (lastAdc <= ADC_SHORT_THRESHOLD) {
    sensorState = SENSOR_SHORT;
    measuredResistanceOhm = 0;
    calculatedTempC = NAN;
    calculatedTempErrorC = NAN;
    return;
  }

  if (lastAdc >= ADC_OPEN_THRESHOLD) {
    sensorState = SENSOR_OPEN;
    measuredResistanceOhm = 65535;
    calculatedTempC = NAN;
    calculatedTempErrorC = NAN;
    return;
  }

  sensorState = SENSOR_OK;

  const uint32_t numerator =
    SENSOR_PULLUP_OHM * static_cast<uint32_t>(lastAdc);

  const uint32_t denominator =
    static_cast<uint32_t>(1023U - lastAdc);

  uint32_t resistance = numerator / denominator;
  if (resistance > 65535UL) resistance = 65535UL;

  measuredResistanceOhm = static_cast<uint16_t>(resistance);
}

// =============================================================
// CONVERSION NTC -> TEMPERATURE EN DEGRES C
// =============================================================

float calculateBeta() {
  const float t1 = settings.tempColdC + 273.15f;
  const float t2 = settings.tempHotC + 273.15f;

  const float numerator =
    logf(settings.tempColdOhm / settings.tempHotOhm);

  const float denominator =
    (1.0f / t1) - (1.0f / t2);

  if (fabsf(denominator) < 0.0000001f) return NAN;

  return numerator / denominator;
}

float resistanceToTemperatureC(float resistanceOhm) {
  if (resistanceOhm <= 0.0f) return NAN;

  const float beta = calculateBeta();
  if (!isfinite(beta) || beta <= 0.0f) return NAN;

  const float tRef = settings.tempColdC + 273.15f;

  const float inverseT =
    (1.0f / tRef) +
    (logf(resistanceOhm / settings.tempColdOhm) / beta);

  if (inverseT <= 0.0f) return NAN;

  return (1.0f / inverseT) - 273.15f;
}

// =============================================================
// CONVERSION RESISTANCE -> POSITION JAUGE
// =============================================================

float resistanceToNeedlePercent(float resistanceOhm) {
  if (resistanceOhm >= settings.gaugeColdOhm) return 0.0f;
  if (resistanceOhm <= settings.gaugeHotOhm) return 110.0f;

  const float coldLog = logf(settings.gaugeColdOhm);
  const float hotLog = logf(settings.gaugeHotOhm);
  const float rLog = logf(resistanceOhm);

  float t = (coldLog - rLog) / (coldLog - hotLog);
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;

  return t * 110.0f;
}

uint16_t interpolatePwm(float percent, float p1, uint16_t pwm1, float p2, uint16_t pwm2) {
  const float ratio = (percent - p1) / (p2 - p1);
  const float pwm = static_cast<float>(pwm1) +
    ratio * static_cast<float>(static_cast<int32_t>(pwm2) - static_cast<int32_t>(pwm1));

  if (pwm < 0.0f) return 0;
  if (pwm > TEMP_PWM_LIMIT) return TEMP_PWM_LIMIT;
  return static_cast<uint16_t>(pwm + 0.5f);
}

uint16_t needlePercentToGaugePwm(float percent) {
  if (percent <= 0.0f) return settings.pwm0;
  if (percent <= 25.0f) return interpolatePwm(percent, 0, settings.pwm0, 25, settings.pwm25);
  if (percent <= 50.0f) return interpolatePwm(percent, 25, settings.pwm25, 50, settings.pwm50);
  if (percent <= 75.0f) return interpolatePwm(percent, 50, settings.pwm50, 75, settings.pwm75);
  if (percent <= 100.0f) return interpolatePwm(percent, 75, settings.pwm75, 100, settings.pwm100);
  if (percent <= 110.0f) return interpolatePwm(percent, 100, settings.pwm100, 110, settings.pwm110);
  return settings.pwm110;
}

void updateTemperatureCalculations() {
  if (sensorState != SENSOR_OK) {
    needlePercent = 0.0f;
    calculatedTempC = NAN;
    calculatedTempErrorC = NAN;
    return;
  }

  needlePercent =
    resistanceToNeedlePercent(static_cast<float>(measuredResistanceOhm));

  calculatedTempC =
    resistanceToTemperatureC(static_cast<float>(measuredResistanceOhm));

  if (isfinite(calculatedTempC)) {
    calculatedTempErrorC =
      fabsf(calculatedTempC) * settings.tempErrorPercent / 100.0f;
  } else {
    calculatedTempErrorC = NAN;
  }
}

void updateTempTarget() {
  if (tempMode == TEMP_MANUAL) {
    targetTempPwm = manualTempPwm;
    return;
  }

  if (sensorState == SENSOR_OPEN) {
    targetTempPwm = settings.pwm0;
    return;
  }

  if (sensorState == SENSOR_SHORT) {
    targetTempPwm = settings.pwm110;
    return;
  }

  targetTempPwm = needlePercentToGaugePwm(needlePercent);
}

// =============================================================
// LED DIAGNOSTIC
// =============================================================

void updateTemperatureWarning(uint32_t nowMs) {
  if (sensorState == SENSOR_OPEN) {
    if (nowMs - lastWarningBlinkMs >= 500) {
      lastWarningBlinkMs = nowMs;
      setWarningLed(!warningLedState);
    }
    return;
  }

  if (sensorState == SENSOR_SHORT) {
    if (nowMs - lastWarningBlinkMs >= 250) {
      lastWarningBlinkMs = nowMs;
      setWarningLed(!warningLedState);
    }
    return;
  }

  setWarningLed(false);
}

// =============================================================
// ANIMATION DEMARRAGE
// =============================================================

void runStartupAnimation() {
  addLog("Animation demarrage lancee");

  applyTempPwm(settings.pwm0);
  setShiftLight(false);
  delay(200);

  // 400 -> 880 par defaut ; respecte pwm100 si modifie.
  const uint16_t animationMax =
    min<uint16_t>(STARTUP_MAX_PWM, settings.pwm100);

  for (uint16_t pwm = settings.pwm0;
       pwm <= animationMax;
       pwm += STARTUP_STEP_PWM) {

    applyTempPwm(pwm);

    bool sOn = false;
    for (uint8_t i = 0; i < 3; ++i) {
      if (pwm >= STARTUP_SHIFT_WINDOWS[i][0] &&
          pwm < STARTUP_SHIFT_WINDOWS[i][1]) {
        sOn = true;
        break;
      }
    }

    setShiftLight(sOn);
    delay(STARTUP_STEP_MS);
    yield();
  }

  setShiftLight(false);
  delay(STARTUP_TOP_PAUSE_MS);

  for (int pwm = animationMax;
       pwm >= static_cast<int>(settings.pwm0);
       pwm -= STARTUP_STEP_PWM) {

    applyTempPwm(static_cast<uint16_t>(pwm));
    delay(STARTUP_STEP_MS);
    yield();
  }

  setShiftLight(false);
  applyTempPwm(settings.pwm0);

  delay(250);

  // Initialisation propre de la lecture capteur.
  filteredAdcInitialized = false;

  for (uint8_t i = 0; i < 8; ++i) {
    updateTemperatureSensor();
    delay(60);
    yield();
  }

  updateTemperatureCalculations();
  tempMode = TEMP_AUTOMATIC;
  updateTempTarget();

  while (appliedTempPwm != targetTempPwm) {
    applyTempPwm(slewPwm(appliedTempPwm, targetTempPwm));
    delay(20);
    yield();
  }

  addLog("Animation terminee, mode AUTO");
}

// =============================================================
// REGIME MOTEUR
// =============================================================

uint16_t calculateRawRpm() {
  uint32_t localPeriod;
  uint32_t localLastValid;

  noInterrupts();
  localPeriod = tachPeriodUs;
  localLastValid = tachLastValidPulseUs;
  interrupts();

  if (localLastValid == 0) return 0;

  const uint32_t ageUs = micros() - localLastValid;
  if (ageUs > TACH_ZERO_TIMEOUT_MS * 1000UL) return 0;
  if (localPeriod < TACH_MIN_PERIOD_US) return 0;

  const uint32_t denominator =
    localPeriod * static_cast<uint32_t>(TACH_PULSES_PER_REVOLUTION);

  if (denominator == 0) return 0;

  uint32_t rpm = 60000000UL / denominator;
  if (rpm > 16000UL) rpm = 16000UL;

  return static_cast<uint16_t>(rpm);
}

uint16_t applyTachCalibration(uint16_t rpm) {
  float corrected =
    static_cast<float>(rpm) * settings.tachCalibrationFactor;

  if (corrected < 0.0f) corrected = 0.0f;
  if (corrected > 16000.0f) corrected = 16000.0f;

  return static_cast<uint16_t>(corrected + 0.5f);
}

// =============================================================
// SHIFT LIGHT
// =============================================================

bool shiftOnLatched = false;
bool shiftFlashLatched = false;
uint32_t shiftAboveOnSinceMs = 0;
uint32_t shiftAboveFlashSinceMs = 0;
uint8_t lastLoggedShiftMode = 255;

uint8_t currentShiftMode() {
  if (!shiftEnabled) return 0;
  if (shiftFlashLatched) return 2;
  if (shiftOnLatched) return 1;
  return 0;
}

void updateShiftLight(uint32_t nowMs) {
  if (!shiftEnabled) {
    shiftOnLatched = false;
    shiftFlashLatched = false;
    setShiftLight(false);
  } else {
    const uint16_t rpm = activeRpm;

    if (rpm >= settings.shiftFlashRpm) {
      if (shiftAboveFlashSinceMs == 0) shiftAboveFlashSinceMs = nowMs;

      if (nowMs - shiftAboveFlashSinceMs >= SHIFT_CONFIRM_MS) {
        shiftFlashLatched = true;
        shiftOnLatched = true;
      }
    } else {
      shiftAboveFlashSinceMs = 0;

      if (rpm + SHIFT_HYSTERESIS_RPM < settings.shiftFlashRpm) {
        shiftFlashLatched = false;
      }
    }

    if (rpm >= settings.shiftOnRpm) {
      if (shiftAboveOnSinceMs == 0) shiftAboveOnSinceMs = nowMs;

      if (nowMs - shiftAboveOnSinceMs >= SHIFT_CONFIRM_MS) {
        shiftOnLatched = true;
      }
    } else {
      shiftAboveOnSinceMs = 0;

      if (rpm + SHIFT_HYSTERESIS_RPM < settings.shiftOnRpm) {
        shiftOnLatched = false;
        shiftFlashLatched = false;
      }
    }

    if (shiftFlashLatched) {
      const bool on =
        ((nowMs / SHIFT_FLASH_HALF_PERIOD_MS) & 1U) != 0;
      setShiftLight(on);
    } else {
      setShiftLight(shiftOnLatched);
    }
  }

  const uint8_t mode = currentShiftMode();
  if (mode != lastLoggedShiftMode) {
    lastLoggedShiftMode = mode;

    if (mode == 0) addLog("Shift-light OFF");
    else if (mode == 1) addLog("Shift-light FIXE");
    else addLog("Shift-light FLASH");
  }
}

// =============================================================
// JSON
// =============================================================

String jsonEscape(const String &input) {
  String out;
  out.reserve(input.length() + 8);

  for (size_t i = 0; i < input.length(); ++i) {
    const char c = input[i];

    if (c == '\\' || c == '"') {
      out += '\\';
      out += c;
    } else if (c == '\n') {
      out += "\\n";
    } else if (c == '\r') {
      // ignore
    } else {
      out += c;
    }
  }

  return out;
}

const char *sensorStateText() {
  if (sensorState == SENSOR_OK) return "OK";
  if (sensorState == SENSOR_SHORT) return "COURT-CIRCUIT";
  return "OUVERTE";
}

const char *tempModeText() {
  return tempMode == TEMP_AUTOMATIC ? "AUTOMATIQUE" : "MANUEL";
}

const char *tachModeText() {
  return tachMode == TACH_REAL ? "REEL" : "SIMULATION";
}

const char *shiftModeText() {
  const uint8_t mode = currentShiftMode();
  if (mode == 2) return "FLASH";
  if (mode == 1) return "FIXE";
  return "OFF";
}

String buildStatusJson() {
  String s;
  s.reserve(2600);

  uint32_t localPeriod;
  noInterrupts();
  localPeriod = tachPeriodUs;
  interrupts();

  s += "{";

  s += "\"fw\":\"";
  s += FW_VERSION;
  s += "\",";

  s += "\"uptime\":";
  s += String(millis());
  s += ",";

  s += "\"wifi\":{";
  s += "\"ssid\":\"";
  s += WIFI_SSID;
  s += "\",";
  s += "\"ip\":\"";
  s += WiFi.softAPIP().toString();
  s += "\",";
  s += "\"clients\":";
  s += String(WiFi.softAPgetStationNum());
  s += "},";

  s += "\"temp\":{";
  s += "\"rawAdc\":";
  s += String(lastRawAdc);
  s += ",";
  s += "\"adc\":";
  s += String(lastAdc);
  s += ",";
  s += "\"sensor\":\"";
  s += sensorStateText();
  s += "\",";
  s += "\"resistance\":";
  s += String(measuredResistanceOhm);
  s += ",";
  s += "\"valid\":";
  s += (sensorState == SENSOR_OK && isfinite(calculatedTempC)) ? "true" : "false";
  s += ",";
  s += "\"c\":";
  s += isfinite(calculatedTempC) ? String(calculatedTempC, 1) : "0";
  s += ",";
  s += "\"errorPct\":";
  s += String(settings.tempErrorPercent, 1);
  s += ",";
  s += "\"errorC\":";
  s += isfinite(calculatedTempErrorC) ? String(calculatedTempErrorC, 1) : "0";
  s += ",";
  s += "\"needle\":";
  s += String(needlePercent, 1);
  s += ",";
  s += "\"targetPwm\":";
  s += String(targetTempPwm);
  s += ",";
  s += "\"appliedPwm\":";
  s += String(appliedTempPwm);
  s += ",";
  s += "\"mode\":\"";
  s += tempModeText();
  s += "\"},";

  s += "\"rpm\":{";
  s += "\"value\":";
  s += String(activeRpm);
  s += ",";
  s += "\"raw\":";
  s += String(rawRpm);
  s += ",";
  s += "\"calFactor\":";
  s += String(settings.tachCalibrationFactor, 5);
  s += ",";
  s += "\"periodUs\":";
  s += String(localPeriod);
  s += ",";
  s += "\"ppr\":";
  s += String(TACH_PULSES_PER_REVOLUTION);
  s += ",";
  s += "\"mode\":\"";
  s += tachModeText();
  s += "\"},";

  s += "\"shift\":{";
  s += "\"enabled\":";
  s += shiftEnabled ? "true" : "false";
  s += ",";
  s += "\"mode\":\"";
  s += shiftModeText();
  s += "\",";
  s += "\"on\":";
  s += String(settings.shiftOnRpm);
  s += ",";
  s += "\"flash\":";
  s += String(settings.shiftFlashRpm);
  s += "},";

  s += "\"system\":{";
  s += "\"freeHeap\":";
  s += String(ESP.getFreeHeap());
  s += ",";
  s += "\"maxBlock\":";
  s += String(ESP.getMaxFreeBlockSize());
  s += ",";
  s += "\"fragmentation\":";
  s += String(ESP.getHeapFragmentation());
  s += ",";
  s += "\"chipId\":\"";
  s += String(ESP.getChipId(), HEX);
  s += "\",";
  s += "\"resetReason\":\"";
  s += jsonEscape(ESP.getResetReason());
  s += "\"},";

  s += "\"constants\":{";
  s += "\"pwmFreq\":";
  s += String(PWM_FREQUENCY_HZ);
  s += ",";
  s += "\"pwmRange\":";
  s += String(PWM_RANGE);
  s += ",";
  s += "\"pullup\":";
  s += String(SENSOR_PULLUP_OHM);
  s += ",";
  s += "\"gaugeCold\":";
  s += String(settings.gaugeColdOhm, 0);
  s += ",";
  s += "\"gaugeHot\":";
  s += String(settings.gaugeHotOhm, 0);
  s += ",";
  s += "\"tempColdOhm\":";
  s += String(settings.tempColdOhm, 0);
  s += ",";
  s += "\"tempColdC\":";
  s += String(settings.tempColdC, 1);
  s += ",";
  s += "\"tempHotOhm\":";
  s += String(settings.tempHotOhm, 0);
  s += ",";
  s += "\"tempHotC\":";
  s += String(settings.tempHotC, 1);
  s += ",";
  s += "\"beta\":";
  s += String(calculateBeta(), 1);
  s += ",";
  s += "\"pwm0\":";
  s += String(settings.pwm0);
  s += ",";
  s += "\"pwm25\":";
  s += String(settings.pwm25);
  s += ",";
  s += "\"pwm50\":";
  s += String(settings.pwm50);
  s += ",";
  s += "\"pwm75\":";
  s += String(settings.pwm75);
  s += ",";
  s += "\"pwm100\":";
  s += String(settings.pwm100);
  s += ",";
  s += "\"pwm110\":";
  s += String(settings.pwm110);
  s += "}";

  s += "}";

  return s;
}

String buildLogsJson() {
  String s = "{\"logs\":[";
  s.reserve(2500);

  for (uint8_t i = 0; i < eventLogUsed; ++i) {
    const int index =
      (eventLogHead - eventLogUsed + i + LOG_COUNT) % LOG_COUNT;

    if (i > 0) s += ",";
    s += "\"";
    s += jsonEscape(String(eventLog[index]));
    s += "\"";
  }

  s += "]}";
  return s;
}

// =============================================================
// INTERFACE WEB
// =============================================================

const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="fr">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>CBR900RR</title>
<style>
:root{
  --bg:#07090c;--panel:#11151a;--line:#2a3139;--text:#f5f7fa;
  --muted:#8f9aa6;--accent:#df2f35;--ok:#4bd078;--warn:#ffb52e;
}
*{box-sizing:border-box}
html,body{margin:0;background:var(--bg);color:var(--text);font-family:Arial,Helvetica,sans-serif}
body{min-height:100vh}
.app{
  width:min(100vw,56.25vh);height:min(100vh,177.78vw);max-height:100vh;
  margin:0 auto;background:linear-gradient(180deg,#090c10 0%,#07090c 100%);
  border-left:1px solid #111;border-right:1px solid #111;overflow:hidden;position:relative
}
.page{position:absolute;inset:0;display:none;background:var(--bg)}
.page.active{display:flex;flex-direction:column}
.topbar{
  height:7.2%;min-height:48px;display:flex;align-items:center;justify-content:space-between;
  padding:0 4.2%;border-bottom:1px solid var(--line);background:#0b0e12
}
.brand{font-weight:800;letter-spacing:.4px;font-size:clamp(16px,2.4vh,21px)}
.statusdot{width:8px;height:8px;border-radius:50%;background:var(--ok);box-shadow:0 0 8px var(--ok);display:inline-block;margin-right:6px}
.small{font-size:clamp(10px,1.45vh,12px);color:var(--muted)}
.main{height:85.6%;display:flex;flex-direction:column;padding:2.5% 3.4% 2.2%;gap:1.8%}
.tachWrap{flex:0 0 54%;display:flex;align-items:center;justify-content:center;position:relative}
.tach{
  width:min(92%,43vh);aspect-ratio:1/1;position:relative;border-radius:50%;
  background:radial-gradient(circle at 50% 55%,#161b21 0 22%,#0b0e12 23% 57%,#151a20 58% 61%,#07090c 62% 100%);
  border:2px solid #232a32;box-shadow:inset 0 0 24px #000,0 10px 30px #0008
}
.tick{position:absolute;left:50%;top:50%;width:1px;height:47%;transform-origin:50% 100%}
.tick:before{content:"";position:absolute;top:0;left:50%;transform:translateX(-50%);width:2px;height:7px;background:#7f8994;border-radius:1px}
.tick.major:before{width:3px;height:13px;background:#e8edf2}.tick.red:before{background:#ff4b4f}
.num{position:absolute;left:50%;top:50%;color:#dce2e8;font-weight:700;font-size:clamp(9px,1.7vh,14px)}
.redzone{
  position:absolute;inset:4%;border-radius:50%;
  background:conic-gradient(from 210deg,transparent 0deg 244deg,#b9222d 244deg 302deg,transparent 302deg 360deg);
  -webkit-mask:radial-gradient(circle,transparent 0 79%,#000 80% 84%,transparent 85%);
  mask:radial-gradient(circle,transparent 0 79%,#000 80% 84%,transparent 85%);opacity:.9
}
.needle{
  position:absolute;left:50%;top:50%;width:4px;height:36%;
  background:linear-gradient(180deg,#ff595f,#c71f2c);border-radius:3px;
  transform-origin:50% 92%;transform:translate(-50%,-92%) rotate(-135deg);
  box-shadow:0 0 8px #e6323277;transition:transform .12s linear;z-index:5
}
.hub{
  position:absolute;left:50%;top:50%;width:11%;aspect-ratio:1/1;transform:translate(-50%,-50%);
  border-radius:50%;background:#222a32;border:3px solid #090b0e;z-index:6;box-shadow:0 0 0 2px #4c555f
}
.rpmDigital{position:absolute;left:50%;top:64%;transform:translateX(-50%);font-weight:900;font-size:clamp(22px,4.3vh,38px);letter-spacing:1px;z-index:7}
.rpmLabel{position:absolute;left:50%;top:76%;transform:translateX(-50%);color:var(--muted);font-size:clamp(9px,1.4vh,12px)}
.shiftLamp{
  position:absolute;left:50%;top:14%;transform:translateX(-50%);width:8%;aspect-ratio:1/1;border-radius:50%;
  background:#381416;border:2px solid #591b20;box-shadow:inset 0 0 8px #000;z-index:8
}
.shiftLamp.on{background:#ff3038;box-shadow:0 0 18px #ff3038,inset 0 0 6px #fff8}
.infoGrid{flex:1;display:grid;grid-template-columns:1fr 1fr;grid-template-rows:1fr 1fr;gap:2.4%;min-height:0}
.card{
  background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:8% 7%;
  display:flex;flex-direction:column;justify-content:center;min-height:0
}
.card .label{font-size:clamp(9px,1.35vh,11px);color:var(--muted);margin-bottom:4px}
.card .bigv{font-weight:900;font-size:clamp(22px,3.7vh,32px);line-height:1}
.card .subv{font-size:clamp(9px,1.35vh,11px);color:var(--muted);margin-top:5px}
.tempGood{color:#67c6ff}.tempHot{color:#ffb52e}.tempDanger{color:#ff565d}
.bottomnav{
  height:7.2%;min-height:48px;display:grid;grid-template-columns:1fr 1fr 1fr;
  border-top:1px solid var(--line);background:#0b0e12
}
.bottomnav button{border:0;background:transparent;color:var(--muted);font-weight:700;font-size:clamp(10px,1.45vh,12px)}
.bottomnav button.active{color:#fff;background:#13181e}
.subpageBody{flex:1;padding:3.5%;overflow:auto}
.panel{background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:4%;margin-bottom:3%}
.panel h2{font-size:clamp(14px,2vh,18px);margin:0 0 10px}
.row{display:flex;justify-content:space-between;gap:12px;padding:8px 0;border-bottom:1px solid #252b31}
.row:last-child{border-bottom:0}.row span:first-child{color:var(--muted)}
input{width:100%;background:#0d1115;color:#fff;border:1px solid #343c45;border-radius:9px;padding:10px;font-size:16px}
label{display:block;color:var(--muted);font-size:11px;margin:10px 0 5px}
.two{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.actions{display:flex;flex-wrap:wrap;gap:8px}
.btn{border:1px solid #343c45;background:#171c22;color:#fff;border-radius:10px;padding:10px 12px;font-weight:700}
.btn.red{background:#8e1f25;border-color:#b92b31}.btn.green{background:#176c35;border-color:#23894a}
.btn.orange{background:#7e4f12;border-color:#a86b1b}
pre{white-space:pre-wrap;word-break:break-word;background:#0d1115;border:1px solid var(--line);border-radius:10px;padding:12px;color:#cbd5df;font-size:11px;line-height:1.4}
.note{font-size:11px;color:var(--muted);line-height:1.4;margin-top:8px}
@media(max-width:420px){.main{padding-left:3%;padding-right:3%}.card{padding:7% 6%}}
</style>
</head>
<body>
<div class="app">

<section id="dash" class="page active">
  <div class="topbar">
    <div>
      <div class="brand">CBR900RR</div>
      <div class="small"><span class="statusdot"></span>NodeMCU connecté</div>
    </div>
    <div class="small">192.168.4.1</div>
  </div>

  <div class="main">
    <div class="tachWrap">
      <div class="tach" id="tach">
        <div class="redzone"></div>
        <div class="shiftLamp" id="shiftLamp"></div>
        <div class="needle" id="needle"></div>
        <div class="hub"></div>
        <div class="rpmDigital" id="rpmDigital">0</div>
        <div class="rpmLabel">RPM</div>
      </div>
    </div>

    <div class="infoGrid">
      <div class="card">
        <div class="label">TEMPÉRATURE</div>
        <div class="bigv" id="tempBig">-- °C</div>
        <div class="subv" id="tempErr">±5 %</div>
      </div>

      <div class="card">
        <div class="label">JAUGE</div>
        <div class="bigv" id="gaugeBig">0 %</div>
        <div class="subv" id="pwmText">PWM 400</div>
      </div>

      <div class="card">
        <div class="label">SHIFT-LIGHT</div>
        <div class="bigv" id="shiftText">OFF</div>
        <div class="subv" id="shiftThresholds">9000 / 10000</div>
      </div>

      <div class="card">
        <div class="label">SONDE</div>
        <div class="bigv" id="resBig">--</div>
        <div class="subv" id="sensorText">--</div>
      </div>
    </div>
  </div>

  <div class="bottomnav">
    <button class="active" onclick="showPage('dash')">Dashboard</button>
    <button onclick="showPage('settings')">Réglages</button>
    <button onclick="showPage('diag')">Diagnostic</button>
  </div>
</section>

<section id="settings" class="page">
  <div class="topbar"><div class="brand">Réglages</div><div class="small">CBR900RR</div></div>
  <div class="subpageBody">

    <div class="panel">
      <h2>Shift-light</h2>
      <div class="two">
        <div><label>Fixe (tr/min)</label><input id="setShiftOn" type="number" min="1000" max="15000"></div>
        <div><label>Clignotant (tr/min)</label><input id="setShiftFlash" type="number" min="1000" max="16000"></div>
      </div>
    </div>

    <div class="panel">
      <h2>Thermomètre NTC</h2>
      <div class="two">
        <div><label>Point froid Ω</label><input id="setTempColdOhm" type="number"></div>
        <div><label>Point froid °C</label><input id="setTempColdC" type="number" step="0.1"></div>
        <div><label>Point chaud Ω</label><input id="setTempHotOhm" type="number"></div>
        <div><label>Point chaud °C</label><input id="setTempHotC" type="number" step="0.1"></div>
        <div><label>Erreur affichée %</label><input id="setTempError" type="number" step="0.1"></div>
      </div>
    </div>

    <div class="panel">
      <h2>Calibration jauge</h2>
      <div class="two">
        <div><label>R 0 % (Ω)</label><input id="setGaugeCold" type="number"></div>
        <div><label>R 110 % (Ω)</label><input id="setGaugeHot" type="number"></div>
        <div><label>PWM 0 %</label><input id="setPwm0" type="number"></div>
        <div><label>PWM 25 %</label><input id="setPwm25" type="number"></div>
        <div><label>PWM 50 %</label><input id="setPwm50" type="number"></div>
        <div><label>PWM 75 %</label><input id="setPwm75" type="number"></div>
        <div><label>PWM 100 %</label><input id="setPwm100" type="number"></div>
        <div><label>PWM 110 %</label><input id="setPwm110" type="number"></div>
      </div>
    </div>

    <div class="panel">
      <h2>Tarage rapide</h2>

      <div class="note">
        <b>Compte-tours :</b> moteur chaud et ralenti parfaitement stable.
        Appuyer ici lorsque le moteur est à 1100 tr/min.
      </div>
      <div class="actions">
        <button class="btn" onclick="calibrateTachIdle()">Tarer ralenti = 1100 tr/min</button>
      </div>

      <div class="note" style="margin-top:14px">
        <b>Température à froid :</b> moteur complètement froid.
        Indiquer la température ambiante réelle puis enregistrer la résistance actuelle.
      </div>
      <label>Température ambiante réelle (°C)</label>
      <input id="ambientCalibrationC" type="number" min="-10" max="50" step="0.1" value="25">
      <div class="actions">
        <button class="btn" onclick="calibrateTempCold()">Tarer température à froid</button>
      </div>

      <div class="note" style="margin-top:14px">
        <b>Déclenchement ventilateur :</b> appuyer exactement au moment où le ventilateur démarre.
        La température de référence est le « point chaud » ci-dessus (102,5 °C par défaut).
      </div>
      <div class="actions">
        <button class="btn" onclick="calibrateTempFan()">Tarer au déclenchement ventilateur</button>
      </div>
    </div>

    <div class="panel">
      <h2>Maintenance</h2>
      <div class="actions">
        <button class="btn green" onclick="saveSettings()">Sauvegarder</button>
        <button class="btn" onclick="action('shiftTest')">Test shift</button>
        <button class="btn" onclick="action('sweep')">Test jauge</button>
        <button class="btn" onclick="action('auto')">Mode AUTO</button>
        <button class="btn red" onclick="action('reset')">Redémarrer ESP</button>
      </div>
    </div>

    <div class="panel">
      <h2>Réinitialisation</h2>
      <div class="note">
        Restaure les valeurs par défaut validées : facteur compte-tours 1,00000,
        shift 9000/10000 tr/min, NTC 35000 Ω @ 25 °C et 1250 Ω @ 102,5 °C,
        jauge 400 / 650 / 725 / 800 / 880 / 900.
      </div>
      <div class="actions">
        <button class="btn orange" onclick="factoryReset()">Restaurer les paramètres par défaut</button>
      </div>
    </div>

    <div id="message" class="small" style="margin:8px 0 18px"></div>
  </div>

  <div class="bottomnav">
    <button onclick="showPage('dash')">Dashboard</button>
    <button class="active" onclick="showPage('settings')">Réglages</button>
    <button onclick="showPage('diag')">Diagnostic</button>
  </div>
</section>

<section id="diag" class="page">
  <div class="topbar"><div class="brand">Diagnostic</div><div class="small" id="fwText">V20.1</div></div>
  <div class="subpageBody">
    <div class="panel"><h2>Status complet</h2><pre id="diagText">Chargement...</pre></div>
    <div class="panel"><h2>Journal</h2><pre id="logText">Chargement...</pre></div>
  </div>
  <div class="bottomnav">
    <button onclick="showPage('dash')">Dashboard</button>
    <button onclick="showPage('settings')">Réglages</button>
    <button class="active" onclick="showPage('diag')">Diagnostic</button>
  </div>
</section>

</div>

<script>
const tach=document.getElementById('tach');
let settingsLoaded=false;

function addTicks(){
  const start=-135,end=135;
  for(let i=0;i<=28;i++){
    const a=start+(end-start)*(i/28);
    const d=document.createElement('div');
    d.className='tick '+(i%2===0?'major ':'')+(i>=20?'red':'');
    d.style.transform=`translate(-50%,-100%) rotate(${a}deg)`;
    tach.appendChild(d);
  }
  for(let r=0;r<=14;r+=2){
    const a=start+(end-start)*(r/14);
    const rad=(a-90)*Math.PI/180;
    const radius=39;
    const x=50+radius*Math.cos(rad), y=50+radius*Math.sin(rad);
    const n=document.createElement('div');
    n.className='num'; n.textContent=r;
    n.style.left=x+'%'; n.style.top=y+'%';
    n.style.transform='translate(-50%,-50%)';
    tach.appendChild(n);
  }
}
addTicks();

function showPage(id){
  document.querySelectorAll('.page').forEach(p=>p.classList.remove('active'));
  document.getElementById(id).classList.add('active');
  document.querySelectorAll('.bottomnav button').forEach(b=>b.classList.remove('active'));
  document.querySelectorAll(`button[onclick="showPage('${id}')"]`).forEach(b=>b.classList.add('active'));
  if(id==='settings' && !settingsLoaded) loadSettings();
  if(id==='diag'){updateStatus();updateLogs();}
}

function fmtR(r){
  if(r>=1000)return (r/1000).toFixed(r>=10000?1:2)+' kΩ';
  return Math.round(r)+' Ω';
}

async function updateStatus(){
  try{
    const r=await fetch('/api/status',{cache:'no-store'});
    const d=await r.json();

    fwText.textContent=d.fw;

    const rpm=d.rpm.value;
    const angle=-135+270*Math.min(1,rpm/14000);
    needle.style.transform=`translate(-50%,-92%) rotate(${angle}deg)`;
    rpmDigital.textContent=rpm;

    shiftLamp.classList.toggle('on',d.shift.mode==='FIXE'||d.shift.mode==='FLASH');
    shiftText.textContent=d.shift.mode;
    shiftThresholds.textContent=d.shift.on+' / '+d.shift.flash;

    if(d.temp.valid){
      tempBig.textContent=Math.round(d.temp.c)+' °C';
      tempBig.className='bigv '+(d.temp.c>=105?'tempDanger':d.temp.c>=95?'tempHot':'tempGood');
      tempErr.textContent='±'+d.temp.errorPct.toFixed(1)+' %';
    }else{
      tempBig.textContent='-- °C';
      tempBig.className='bigv';
      tempErr.textContent='Température indisponible';
    }

    gaugeBig.textContent=d.temp.needle.toFixed(0)+' %';
    pwmText.textContent='PWM '+d.temp.appliedPwm;
    resBig.textContent=fmtR(d.temp.resistance);
    sensorText.textContent=d.temp.sensor+' · '+d.temp.mode;

    diagText.textContent =
`Firmware            ${d.fw}
Uptime              ${(d.uptime/1000).toFixed(1)} s
Wi-Fi               ${d.wifi.ssid}
IP                  ${d.wifi.ip}
Clients             ${d.wifi.clients}

--- TEMPERATURE ---
ADC brut             ${d.temp.rawAdc}
ADC filtré           ${d.temp.adc}
Sonde                ${d.temp.sensor}
Résistance           ${d.temp.resistance} Ω
Température          ${d.temp.valid?d.temp.c.toFixed(1)+' °C':'--'}
Erreur affichée      ±${d.temp.errorPct.toFixed(1)} %
Position jauge       ${d.temp.needle.toFixed(1)} %
PWM cible            ${d.temp.targetPwm}
PWM appliqué         ${d.temp.appliedPwm}
Mode                 ${d.temp.mode}

--- COMPTE-TOURS ---
RPM corrigé         ${d.rpm.value}
RPM brut            ${d.rpm.raw}
Facteur tarage      ${d.rpm.calFactor.toFixed(5)}
Période impulsion   ${d.rpm.periodUs} µs
Impulsions/tour      ${d.rpm.ppr}
Mode                 ${d.rpm.mode}

--- SHIFT-LIGHT ---
Actif                ${d.shift.enabled?'OUI':'NON'}
Etat                 ${d.shift.mode}
Fixe                 ${d.shift.on} tr/min
Flash                ${d.shift.flash} tr/min

--- SYSTEME ---
Mémoire libre        ${d.system.freeHeap}
Bloc libre max       ${d.system.maxBlock}
Fragmentation        ${d.system.fragmentation} %
Chip ID              ${d.system.chipId}
Dernier reset        ${d.system.resetReason}

--- CONSTANTES ---
PWM fréquence        ${d.constants.pwmFreq} Hz
PWM plage            0..${d.constants.pwmRange}
Pull-up sonde        ${d.constants.pullup} Ω
R jauge 0 %          ${d.constants.gaugeCold} Ω
R jauge 110 %        ${d.constants.gaugeHot} Ω
Point froid NTC      ${d.constants.tempColdOhm} Ω @ ${d.constants.tempColdC} °C
Point chaud NTC      ${d.constants.tempHotOhm} Ω @ ${d.constants.tempHotC} °C
Beta                 ${d.constants.beta}
PWM 0/25/50/75       ${d.constants.pwm0} / ${d.constants.pwm25} / ${d.constants.pwm50} / ${d.constants.pwm75}
PWM 100/110          ${d.constants.pwm100} / ${d.constants.pwm110}`;
  }catch(e){}
}

async function updateLogs(){
  try{
    const r=await fetch('/api/logs',{cache:'no-store'});
    const d=await r.json();
    logText.textContent=d.logs.join('\n');
  }catch(e){}
}

async function loadSettings(){
  const r=await fetch('/api/status',{cache:'no-store'});
  const d=await r.json();

  setShiftOn.value=d.shift.on;
  setShiftFlash.value=d.shift.flash;

  setTempColdOhm.value=d.constants.tempColdOhm;
  setTempColdC.value=d.constants.tempColdC;
  setTempHotOhm.value=d.constants.tempHotOhm;
  setTempHotC.value=d.constants.tempHotC;
  setTempError.value=d.temp.errorPct;

  setGaugeCold.value=d.constants.gaugeCold;
  setGaugeHot.value=d.constants.gaugeHot;

  setPwm0.value=d.constants.pwm0;
  setPwm25.value=d.constants.pwm25;
  setPwm50.value=d.constants.pwm50;
  setPwm75.value=d.constants.pwm75;
  setPwm100.value=d.constants.pwm100;
  setPwm110.value=d.constants.pwm110;

  settingsLoaded=true;
}

async function saveSettings(){
  const p=new URLSearchParams({
    shiftOn:setShiftOn.value,shiftFlash:setShiftFlash.value,
    tempColdOhm:setTempColdOhm.value,tempColdC:setTempColdC.value,
    tempHotOhm:setTempHotOhm.value,tempHotC:setTempHotC.value,
    tempError:setTempError.value,gaugeCold:setGaugeCold.value,
    gaugeHot:setGaugeHot.value,pwm0:setPwm0.value,pwm25:setPwm25.value,
    pwm50:setPwm50.value,pwm75:setPwm75.value,pwm100:setPwm100.value,pwm110:setPwm110.value
  });

  const r=await fetch('/api/settings',{
    method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:p.toString()
  });

  message.textContent=await r.text();
  if(r.ok){settingsLoaded=false;await loadSettings();}
}

async function action(type){
  if(type==='reset'&&!confirm('Redémarrer le NodeMCU ?'))return;
  const r=await fetch('/api/action?type='+encodeURIComponent(type));
  message.textContent=await r.text();
}

async function factoryReset(){
  if(!confirm('Restaurer tous les paramètres par défaut validés ?'))return;
  if(!confirm('Cette action écrasera les réglages sauvegardés. Continuer ?'))return;

  const r=await fetch('/api/factory-reset',{method:'POST'});
  message.textContent=await r.text();

  if(r.ok){
    settingsLoaded=false;
    await loadSettings();
    await updateStatus();
  }
}


async function calibrateTachIdle(){
  if(!confirm('Le moteur est-il bien au ralenti stable à 1100 tr/min ?'))return;

  const r=await fetch('/api/calibrate/tach-idle',{method:'POST'});
  message.textContent=await r.text();

  if(r.ok){
    settingsLoaded=false;
    await updateStatus();
  }
}

async function calibrateTempCold(){
  const ambient=parseFloat(ambientCalibrationC.value);

  if(!Number.isFinite(ambient)){
    message.textContent='Indique une température ambiante valide.';
    return;
  }

  if(!confirm('Le moteur est-il complètement froid et stabilisé à la température ambiante ?'))return;

  const p=new URLSearchParams({ambientC:ambient.toString()});

  const r=await fetch('/api/calibrate/temp-cold',{
    method:'POST',
    headers:{'Content-Type':'application/x-www-form-urlencoded'},
    body:p.toString()
  });

  message.textContent=await r.text();

  if(r.ok){
    settingsLoaded=false;
    await loadSettings();
    await updateStatus();
  }
}

async function calibrateTempFan(){
  if(!confirm('Le ventilateur vient-il EXACTEMENT de se déclencher ?'))return;

  const p=new URLSearchParams({fanC:setTempHotC.value});

  const r=await fetch('/api/calibrate/temp-fan',{
    method:'POST',
    headers:{'Content-Type':'application/x-www-form-urlencoded'},
    body:p.toString()
  });

  message.textContent=await r.text();

  if(r.ok){
    settingsLoaded=false;
    await loadSettings();
    await updateStatus();
  }
}

setInterval(updateStatus,250);
setInterval(updateLogs,1000);
updateStatus();
updateLogs();
</script>
</body>
</html>
)HTML";

// =============================================================
// HANDLERS WEB
// =============================================================

float argFloat(const char *name, float fallback) {
  if (!server.hasArg(name)) return fallback;
  return server.arg(name).toFloat();
}

long argLong(const char *name, long fallback) {
  if (!server.hasArg(name)) return fallback;
  return server.arg(name).toInt();
}

void handleRoot() {
  server.send_P(200, PSTR("text/html; charset=utf-8"), INDEX_HTML);
}

void handleStatus() {
  server.send(200, "application/json; charset=utf-8", buildStatusJson());
}

void handleLogs() {
  server.send(200, "application/json; charset=utf-8", buildLogsJson());
}

void handleSettingsPost() {
  Settings old = settings;

  settings.shiftOnRpm =
    static_cast<uint16_t>(argLong("shiftOn", settings.shiftOnRpm));

  settings.shiftFlashRpm =
    static_cast<uint16_t>(argLong("shiftFlash", settings.shiftFlashRpm));

  settings.tempColdOhm = argFloat("tempColdOhm", settings.tempColdOhm);
  settings.tempColdC = argFloat("tempColdC", settings.tempColdC);
  settings.tempHotOhm = argFloat("tempHotOhm", settings.tempHotOhm);
  settings.tempHotC = argFloat("tempHotC", settings.tempHotC);
  settings.tempErrorPercent = argFloat("tempError", settings.tempErrorPercent);

  settings.gaugeColdOhm = argFloat("gaugeCold", settings.gaugeColdOhm);
  settings.gaugeHotOhm = argFloat("gaugeHot", settings.gaugeHotOhm);

  settings.pwm0 = static_cast<uint16_t>(argLong("pwm0", settings.pwm0));
  settings.pwm25 = static_cast<uint16_t>(argLong("pwm25", settings.pwm25));
  settings.pwm50 = static_cast<uint16_t>(argLong("pwm50", settings.pwm50));
  settings.pwm75 = static_cast<uint16_t>(argLong("pwm75", settings.pwm75));
  settings.pwm100 = static_cast<uint16_t>(argLong("pwm100", settings.pwm100));
  settings.pwm110 = static_cast<uint16_t>(argLong("pwm110", settings.pwm110));

  settings.magic = SETTINGS_MAGIC;
  settings.version = SETTINGS_VERSION;

  if (!settingsAreValid()) {
    settings = old;
    server.send(400, "text/plain; charset=utf-8",
      "Valeurs refusees : verifier les seuils et l'ordre des PWM.");
    return;
  }

  updateTemperatureCalculations();
  updateTempTarget();
  saveSettings();

  server.send(200, "text/plain; charset=utf-8",
    "Reglages sauvegardes.");
}

void handleAction() {
  if (!server.hasArg("type")) {
    server.send(400, "text/plain", "Action manquante");
    return;
  }

  const String type = server.arg("type");

  if (type == "reset") {
    addLog("Reset demande depuis interface");
    server.send(
      200,
      "text/plain; charset=utf-8",
      "Redemarrage programme dans 1 seconde..."
    );
    restartPending = true;
    restartAtMs = millis() + 1000UL;
    return;
  }

  if (type == "sweep") {
    server.send(200, "text/plain; charset=utf-8", "Animation lancee");
    runStartupAnimation();
    return;
  }

  if (type == "auto") {
    tempMode = TEMP_AUTOMATIC;
    addLog("Temperature mode AUTO");
    server.send(200, "text/plain; charset=utf-8", "Mode AUTO active");
    return;
  }

  if (type == "shiftTest") {
    addLog("Test shift-light interface");
    setShiftLight(true);
    delay(600);
    setShiftLight(false);
    server.send(200, "text/plain; charset=utf-8", "Test shift-light termine");
    return;
  }

  server.send(400, "text/plain; charset=utf-8", "Action inconnue");
}



void handleCalibrateTachIdle() {
  if (tachMode != TACH_REAL) {
    server.send(400, "text/plain; charset=utf-8",
      "Tarage refuse : remettre le compte-tours en mode REEL.");
    return;
  }

  if (rawRpm < 500 || rawRpm > 2500) {
    server.send(400, "text/plain; charset=utf-8",
      "Tarage refuse : RPM brut hors plage de ralenti (500..2500).");
    return;
  }

  const float factor = 1100.0f / static_cast<float>(rawRpm);

  if (factor < 0.50f || factor > 1.50f) {
    server.send(400, "text/plain; charset=utf-8",
      "Tarage refuse : correction trop importante.");
    return;
  }

  settings.tachCalibrationFactor = factor;
  saveSettings();

  char msg[110];
  snprintf(msg, sizeof(msg),
    "Tarage RPM : brut %u -> 1100 tr/min | facteur %.5f",
    rawRpm, settings.tachCalibrationFactor);
  addLog(msg);

  server.send(200, "text/plain; charset=utf-8",
    String("Tarage compte-tours sauvegarde. Facteur = ") +
    String(settings.tachCalibrationFactor, 5));
}

void handleCalibrateTempCold() {
  if (sensorState != SENSOR_OK ||
      measuredResistanceOhm < 1000 ||
      measuredResistanceOhm > 65000) {
    server.send(400, "text/plain; charset=utf-8",
      "Tarage froid refuse : lecture de sonde invalide.");
    return;
  }

  if (!server.hasArg("ambientC")) {
    server.send(400, "text/plain; charset=utf-8",
      "Temperature ambiante manquante.");
    return;
  }

  const float ambientC = server.arg("ambientC").toFloat();

  if (ambientC < -10.0f || ambientC > 50.0f) {
    server.send(400, "text/plain; charset=utf-8",
      "Temperature ambiante invalide (-10..50 C).");
    return;
  }

  Settings old = settings;

  settings.tempColdOhm = static_cast<float>(measuredResistanceOhm);
  settings.tempColdC = ambientC;
  settings.gaugeColdOhm = static_cast<float>(measuredResistanceOhm);

  if (!settingsAreValid()) {
    settings = old;
    server.send(400, "text/plain; charset=utf-8",
      "Tarage froid refuse : incoherence avec le point chaud actuel.");
    return;
  }

  saveSettings();
  updateTemperatureCalculations();
  updateTempTarget();

  char msg[110];
  snprintf(msg, sizeof(msg),
    "Tarage froid : %u ohm = %.1f C",
    measuredResistanceOhm, ambientC);
  addLog(msg);

  server.send(200, "text/plain; charset=utf-8",
    String("Point froid sauvegarde : ") +
    String(measuredResistanceOhm) + " ohm @ " +
    String(ambientC, 1) + " C");
}

void handleCalibrateTempFan() {
  if (sensorState != SENSOR_OK ||
      measuredResistanceOhm < 100 ||
      measuredResistanceOhm > 10000) {
    server.send(400, "text/plain; charset=utf-8",
      "Tarage ventilateur refuse : lecture de sonde invalide.");
    return;
  }

  float fanC = settings.tempHotC;

  if (server.hasArg("fanC")) {
    const float requested = server.arg("fanC").toFloat();
    if (requested >= 80.0f && requested <= 120.0f) {
      fanC = requested;
    }
  }

  Settings old = settings;

  settings.tempHotOhm = static_cast<float>(measuredResistanceOhm);
  settings.tempHotC = fanC;

  if (!settingsAreValid()) {
    settings = old;
    server.send(400, "text/plain; charset=utf-8",
      "Tarage ventilateur refuse : incoherence avec le point froid actuel.");
    return;
  }

  saveSettings();
  updateTemperatureCalculations();
  updateTempTarget();

  char msg[120];
  snprintf(msg, sizeof(msg),
    "Tarage ventilateur : %u ohm = %.1f C",
    measuredResistanceOhm, fanC);
  addLog(msg);

  server.send(200, "text/plain; charset=utf-8",
    String("Point ventilateur sauvegarde : ") +
    String(measuredResistanceOhm) + " ohm @ " +
    String(fanC, 1) + " C");
}

void handleFactoryReset() {
  loadDefaultSettings();

  if (!settingsAreValid()) {
    server.send(500, "text/plain; charset=utf-8",
      "ERREUR : les valeurs par defaut internes sont invalides.");
    return;
  }

  // Sauvegarde reelle en EEPROM.
  saveSettings();

  // Retour immediat au fonctionnement normal.
  tempMode = TEMP_AUTOMATIC;
  tachMode = TACH_REAL;
  simulatedRpm = 0;
  activeRpm = 0;
  shiftEnabled = true;
  shiftOnLatched = false;
  shiftFlashLatched = false;
  setShiftLight(false);

  updateTemperatureCalculations();
  updateTempTarget();

  addLog("PARAMETRES PAR DEFAUT RESTAURES");

  server.send(
    200,
    "text/plain; charset=utf-8",
    "Parametres par defaut restaures et sauvegardes."
  );
}

void handleNotFound() {
  server.sendHeader("Location", String("http://") + AP_IP.toString(), true);
  server.send(302, "text/plain", "");
}

void setupWebServer() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_MASK);

  const bool apOk = WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);

  if (apOk) addLog("Wi-Fi CBR900RR demarre");
  else addLog("ERREUR demarrage Wi-Fi");

  dnsServer.start(53, "*", AP_IP);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/logs", HTTP_GET, handleLogs);
  server.on("/api/settings", HTTP_POST, handleSettingsPost);
  server.on("/api/factory-reset", HTTP_POST, handleFactoryReset);
  server.on("/api/calibrate/tach-idle", HTTP_POST, handleCalibrateTachIdle);
  server.on("/api/calibrate/temp-cold", HTTP_POST, handleCalibrateTempCold);
  server.on("/api/calibrate/temp-fan", HTTP_POST, handleCalibrateTempFan);
  server.on("/api/action", HTTP_GET, handleAction);
  server.onNotFound(handleNotFound);

  server.begin();
  addLog("Serveur web 192.168.4.1 actif");
}

// =============================================================
// COMMANDES SERIE
// =============================================================

char commandBuffer[64];
uint8_t commandLength = 0;

void printStatus() {
  Serial.println();
  Serial.println("----- ETAT V20.2.1 -----");

  Serial.print("Wi-Fi : ");
  Serial.print(WIFI_SSID);
  Serial.print(" | ");
  Serial.println(WiFi.softAPIP());

  Serial.print("Temperature : ");
  if (isfinite(calculatedTempC)) {
    Serial.print(calculatedTempC, 1);
    Serial.print(" C +/- ");
    Serial.print(settings.tempErrorPercent, 1);
    Serial.println(" %");
  } else {
    Serial.println("INDISPONIBLE");
  }

  Serial.print("ADC brut / filtre : ");
  Serial.print(lastRawAdc);
  Serial.print(" / ");
  Serial.println(lastAdc);

  Serial.print("Resistance : ");
  Serial.print(measuredResistanceOhm);
  Serial.println(" ohm");

  Serial.print("Jauge : ");
  Serial.print(needlePercent, 1);
  Serial.print(" % | PWM ");
  Serial.print(appliedTempPwm);
  Serial.print(" / cible ");
  Serial.println(targetTempPwm);

  Serial.print("RPM brut / corrige : ");
  Serial.print(rawRpm);
  Serial.print(" / ");
  Serial.println(activeRpm);

  Serial.print("Facteur tarage RPM : ");
  Serial.println(settings.tachCalibrationFactor, 5);

  Serial.print("Shift fixe / flash : ");
  Serial.print(settings.shiftOnRpm);
  Serial.print(" / ");
  Serial.println(settings.shiftFlashRpm);

  Serial.print("Heap libre : ");
  Serial.println(ESP.getFreeHeap());

  Serial.println("---------------------");
}

void printHelp() {
  Serial.println();
  Serial.println("Commandes :");
  Serial.println("  status");
  Serial.println("  reset");
  Serial.println("  sweep");
  Serial.println("  auto");
  Serial.println("  manual");
  Serial.println("  pwm 0..1023");
  Serial.println("  tach real");
  Serial.println("  tach sim");
  Serial.println("  rpm 0..16000");
  Serial.println("  shift on");
  Serial.println("  shift off");
  Serial.println("  shifton 1000..15000");
  Serial.println("  shiftflash 1000..16000");
  Serial.println("  wifi");
  Serial.println("  factoryreset");
  Serial.println();
}

void processCommand(char *cmd) {
  while (*cmd == ' ') ++cmd;

  if (strcmp(cmd, "help") == 0) {
    printHelp();
    return;
  }

  if (strcmp(cmd, "status") == 0) {
    printStatus();
    return;
  }

  if (strcmp(cmd, "factoryreset") == 0) {
    loadDefaultSettings();
    saveSettings();

    tempMode = TEMP_AUTOMATIC;
    tachMode = TACH_REAL;
    simulatedRpm = 0;
    activeRpm = 0;
    shiftEnabled = true;
    setShiftLight(false);

    updateTemperatureCalculations();
    updateTempTarget();

    addLog("PARAMETRES PAR DEFAUT RESTAURES (SERIE)");
    Serial.println("Parametres par defaut restaures.");
    return;
  }

  if (strcmp(cmd, "wifi") == 0) {
    Serial.print("SSID : ");
    Serial.println(WIFI_SSID);
    Serial.print("IP   : ");
    Serial.println(WiFi.softAPIP());
    Serial.print("Clients : ");
    Serial.println(WiFi.softAPgetStationNum());
    return;
  }

  if (strcmp(cmd, "reset") == 0 || strcmp(cmd, "restart") == 0) {
    Serial.println("Redemarrage ESP...");
    Serial.flush();
    delay(100);
    ESP.restart();
    return;
  }

  if (strcmp(cmd, "sweep") == 0) {
    runStartupAnimation();
    return;
  }

  if (strcmp(cmd, "auto") == 0) {
    tempMode = TEMP_AUTOMATIC;
    addLog("Temperature mode AUTO");
    Serial.println("Temperature : mode AUTOMATIQUE.");
    return;
  }

  if (strcmp(cmd, "manual") == 0) {
    tempMode = TEMP_MANUAL;
    manualTempPwm = appliedTempPwm;
    addLog("Temperature mode MANUEL");
    Serial.println("Temperature : mode MANUEL.");
    return;
  }

  if (strncmp(cmd, "pwm ", 4) == 0) {
    const long value = strtol(cmd + 4, nullptr, 10);

    if (value < 0 || value > TEMP_PWM_LIMIT) {
      Serial.println("Erreur : pwm 0..1023");
      return;
    }

    tempMode = TEMP_MANUAL;
    manualTempPwm = static_cast<uint16_t>(value);
    Serial.print("PWM manuel = ");
    Serial.println(manualTempPwm);
    return;
  }

  if (strcmp(cmd, "tach real") == 0) {
    tachMode = TACH_REAL;
    activeRpm = 0;

    noInterrupts();
    tachLastPulseUs = 0;
    tachPeriodUs = 0;
    tachLastValidPulseUs = 0;
    interrupts();

    Serial.println("Compte-tours : REEL.");
    return;
  }

  if (strcmp(cmd, "tach sim") == 0) {
    tachMode = TACH_SIMULATION;
    activeRpm = simulatedRpm;
    Serial.println("Compte-tours : SIMULATION.");
    return;
  }

  if (strncmp(cmd, "rpm ", 4) == 0) {
    const long value = strtol(cmd + 4, nullptr, 10);

    if (value < 0 || value > 16000) {
      Serial.println("Erreur : rpm 0..16000");
      return;
    }

    tachMode = TACH_SIMULATION;
    simulatedRpm = static_cast<uint16_t>(value);
    activeRpm = simulatedRpm;

    Serial.print("RPM simule = ");
    Serial.println(simulatedRpm);
    return;
  }

  if (strcmp(cmd, "shift on") == 0) {
    shiftEnabled = true;
    Serial.println("Shift-light ACTIVE.");
    return;
  }

  if (strcmp(cmd, "shift off") == 0) {
    shiftEnabled = false;
    setShiftLight(false);
    Serial.println("Shift-light DESACTIVE.");
    return;
  }

  if (strncmp(cmd, "shifton ", 8) == 0) {
    const long value = strtol(cmd + 8, nullptr, 10);

    if (value < 1000 || value > 15000 ||
        value >= settings.shiftFlashRpm) {
      Serial.println("Erreur seuil shift fixe.");
      return;
    }

    settings.shiftOnRpm = static_cast<uint16_t>(value);
    saveSettings();

    Serial.print("Shift fixe = ");
    Serial.println(settings.shiftOnRpm);
    return;
  }

  if (strncmp(cmd, "shiftflash ", 11) == 0) {
    const long value = strtol(cmd + 11, nullptr, 10);

    if (value < 1000 || value > 16000 ||
        value <= settings.shiftOnRpm) {
      Serial.println("Erreur seuil shift flash.");
      return;
    }

    settings.shiftFlashRpm = static_cast<uint16_t>(value);
    saveSettings();

    Serial.print("Shift clignotant = ");
    Serial.println(settings.shiftFlashRpm);
    return;
  }

  Serial.println("Commande inconnue. Taper help.");
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());

    if (c == '\r') continue;

    if (c == '\n') {
      commandBuffer[commandLength] = '\0';

      if (commandLength > 0) {
        processCommand(commandBuffer);
      }

      commandLength = 0;
      continue;
    }

    if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = c;
    } else {
      commandLength = 0;
    }
  }
}

// =============================================================
// SETUP / LOOP
// =============================================================

uint32_t lastTempControlMs = 0;
uint32_t lastRpmUpdateMs = 0;
uint32_t lastHeartbeatMs = 0;

SensorState previousSensorState = SENSOR_OPEN;

void setup() {
  pinMode(PIN_TEMP_GAUGE, OUTPUT);
  pinMode(PIN_SHIFT_LIGHT, OUTPUT);
  pinMode(PIN_WARNING_LED, OUTPUT);
  pinMode(PIN_TACH_INPUT, INPUT_PULLUP);

  analogWriteRange(PWM_RANGE);
  analogWriteFreq(PWM_FREQUENCY_HZ);

  Serial.begin(115200);
  delay(250);

  addLog("Boot ESP8266");
  loadSettings();

  applyTempPwm(settings.pwm0);
  setShiftLight(false);
  setWarningLed(false);

  Serial.println();
  Serial.println("=== CBR900RR V20.2.1 WIFI CALIBRATION ===");
  Serial.print("Reset : ");
  Serial.println(ESP.getResetReason());

  runStartupAnimation();

  attachInterrupt(
    digitalPinToInterrupt(PIN_TACH_INPUT),
    tachPulseInterrupt,
    FALLING
  );

  tempMode = TEMP_AUTOMATIC;
  tachMode = TACH_REAL;

  setupWebServer();

  lastTempControlMs = millis();
  lastRpmUpdateMs = millis();

  printHelp();
}

void loop() {
  readSerialCommands();

  // Le web reste secondaire : le moteur et la jauge ne dependent pas du Wi-Fi.
  dnsServer.processNextRequest();
  server.handleClient();

  const uint32_t nowMs = millis();

  if (restartPending &&
      static_cast<int32_t>(nowMs - restartAtMs) >= 0) {
    Serial.println("Redemarrage ESP demande par interface...");
    Serial.flush();
    delay(50);
    ESP.restart();
  }

  if (nowMs - lastTempControlMs >= TEMP_CONTROL_PERIOD_MS) {
    lastTempControlMs = nowMs;

    updateTemperatureSensor();

    if (sensorState != previousSensorState) {
      previousSensorState = sensorState;

      if (sensorState == SENSOR_OK) addLog("Sonde temperature OK");
      else if (sensorState == SENSOR_OPEN) addLog("Sonde temperature OUVERTE");
      else addLog("Sonde temperature COURT-CIRCUIT");
    }

    updateTemperatureCalculations();
    updateTempTarget();

    const uint16_t nextPwm =
      slewPwm(appliedTempPwm, targetTempPwm);

    applyTempPwm(nextPwm);
    updateTemperatureWarning(nowMs);
  }

  if (nowMs - lastRpmUpdateMs >= RPM_UPDATE_MS) {
    lastRpmUpdateMs = nowMs;

    if (tachMode == TACH_REAL) {
      rawRpm = calculateRawRpm();
      activeRpm = applyTachCalibration(rawRpm);
    } else {
      rawRpm = simulatedRpm;
      activeRpm = simulatedRpm;
    }
  }

  updateShiftLight(nowMs);

  if (nowMs - lastHeartbeatMs >= 5000) {
    lastHeartbeatMs = nowMs;

    Serial.print("LIVE | ADC=");
    Serial.print(lastAdc);
    Serial.print(" | R=");
    Serial.print(measuredResistanceOhm);
    Serial.print(" | T=");
    if (isfinite(calculatedTempC)) Serial.print(calculatedTempC, 1);
    else Serial.print("--");
    Serial.print("C | pos=");
    Serial.print(needlePercent, 1);
    Serial.print("% | cible=");
    Serial.print(targetTempPwm);
    Serial.print(" | PWM=");
    Serial.print(appliedTempPwm);
    Serial.print(" | RPM=");
    Serial.print(activeRpm);
    Serial.print(" | shift=");
    Serial.print(shiftModeText());
    Serial.print(" | clients=");
    Serial.println(WiFi.softAPgetStationNum());
  }

  yield();
}
