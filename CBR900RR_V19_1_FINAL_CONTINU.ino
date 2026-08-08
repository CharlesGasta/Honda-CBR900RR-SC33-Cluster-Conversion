/*
  Honda CBR900RR SC33 1998 - Module final tableau de bord
  Version V19.1 FINAL CONTINU

  Fonctions :
  - Lecture de la sonde de temperature d'origine sur A0
  - Pilotage de l'aiguille de temperature sur D5 via IRLZ44N
  - Animation de demarrage de l'aiguille de temperature
  - 3 clignotements du voyant S pendant l'animation
  - Lecture du regime moteur sur D6 via PC817
  - Shift-light sur le voyant S via D1 / IRLZ44N
  - Commande serie "reset" pour redemarrer l'ESP

  IMPORTANT :
  - Aucune sortie ESP n'est branchee sur le signal du compte-tours d'origine.
  - Le signal compte-tours est uniquement LU via le PC817.
  - Le fil sonde temperature cote moteur est separe du fil jauge cote compteur.

  Broches :
    A0 = lecture sonde temperature
    D5 = sortie PWM jauge temperature
    D6 = lecture regime via PC817
    D1 = voyant S / shift-light
    D4 = LED integree diagnostic temperature
*/

#include <Arduino.h>

// =============================================================
// BROCHES
// =============================================================

constexpr uint8_t PIN_TEMP_SENSOR = A0;
constexpr uint8_t PIN_TEMP_GAUGE  = D5;
constexpr uint8_t PIN_TACH_INPUT  = D6;
constexpr uint8_t PIN_SHIFT_LIGHT = D1;
constexpr uint8_t PIN_WARNING_LED = D4; // LED NodeMCU active a LOW

// =============================================================
// PWM JAUGE TEMPERATURE
// =============================================================

// Frequence finale retenue : 1 kHz, comportement le plus stable de la jauge.
constexpr uint16_t PWM_RANGE = 1023;
constexpr uint32_t PWM_FREQUENCY_HZ = 1000;

constexpr uint16_t TEMP_PWM_LIMIT = 1023;

// 900 correspond a la zone surchauffe (~110 %) de la jauge.
constexpr uint16_t TEMP_PWM_NORMAL_MAX = 900;

// Variation normale volontairement douce.
constexpr uint16_t TEMP_SLEW_STEP = 4;
constexpr uint32_t TEMP_CONTROL_PERIOD_MS = 50;

// =============================================================
// SONDE TEMPERATURE
// =============================================================

/*
  Cablage lecture sonde :

      3V3
       |
      10 kOhm
       |
       +------ A0
       |
     sonde Honda
       |
      GND moteur

  La sonde de tableau de bord Honda ancienne generation est une
  thermistance a faible resistance. Les anciennes valeurs 800..16000
  etaient trop elevees d'un facteur 10 pour le montage reel.

  Table finale utilisee :
    resistance sonde -> PWM jauge
*/
constexpr uint32_t SENSOR_PULLUP_OHM = 10000UL;

// =============================================================
// CALIBRATION CONTINUE TEMPERATURE
// =============================================================
//
// La sonde est une NTC : sa resistance baisse quand la temperature monte.
// On ne travaille plus avec des paliers fixes.
//
// Calibration observee sur la moto :
//   moteur froid : environ 35 kOhm -> aiguille 0 %
//   zone tres chaude : environ 800 Ohm -> aiguille 110 %
//
// Calibration physique de la jauge a 1 kHz :
//   0 %   = PWM 400
//   25 %  = PWM 650
//   50 %  = PWM 725
//   75 %  = PWM 800
//   110 % = PWM 900
//
// Entre ces points, interpolation lineaire.
// Entre les resistances, conversion logarithmique continue,
// mieux adaptee au comportement d'une NTC.

constexpr float TEMP_R_COLD_OHM = 35000.0f;
constexpr float TEMP_R_HOT_OHM  =   800.0f;

constexpr float NEEDLE_MAX_PERCENT = 110.0f;

struct GaugePoint {
  float percent;
  uint16_t pwm;
};

constexpr GaugePoint GAUGE_TABLE[] = {
  {  0.0f, 400},
  { 25.0f, 650},
  { 50.0f, 725},
  { 75.0f, 800},
  {110.0f, 900}
};

constexpr uint8_t GAUGE_POINT_COUNT =
  sizeof(GAUGE_TABLE) / sizeof(GAUGE_TABLE[0]);

constexpr uint8_t ADC_SAMPLES = 32;
constexpr uint16_t ADC_SHORT_THRESHOLD = 2;
constexpr uint16_t ADC_OPEN_THRESHOLD = 1018;

// Valeurs de diagnostic / alerte. Les seuils historiques restent conserves.
constexpr uint16_t TEMP_WARNING_FIXED_OHM = 135;
constexpr uint16_t TEMP_WARNING_BLINK_OHM = 125;
constexpr uint32_t WARNING_BLINK_MS = 250;

// =============================================================
// ANIMATION TEMPERATURE
// =============================================================

// Plus lente que la version precedente pour laisser l'aiguille suivre.
constexpr uint16_t STARTUP_MAX_PWM = 880;
constexpr uint16_t STARTUP_STEP_PWM = 5;
constexpr uint32_t STARTUP_STEP_MS = 14;
constexpr uint32_t STARTUP_TOP_PAUSE_MS = 500;

// Trois flashs du S pendant la montee.
constexpr uint16_t SHIFT_BLINK_WINDOWS[3][2] = {
  {450, 520},
  {600, 670},
  {750, 820}
};

// =============================================================
// COMPTE-TOURS / SHIFT-LIGHT
// =============================================================

// Calibration validee sur la moto.
constexpr uint8_t TACH_PULSES_PER_REVOLUTION = 4;

// L'ancien filtre a 2400 us aurait supprime les impulsions a haut regime.
// 700 us autorise largement 10 000 tr/min avec PPR=4.
constexpr uint32_t TACH_MIN_PERIOD_US = 700;
constexpr uint32_t TACH_ZERO_TIMEOUT_MS = 800;
constexpr uint32_t RPM_UPDATE_MS = 100;

constexpr uint16_t SHIFT_ON_DEFAULT_RPM = 9000;
constexpr uint16_t SHIFT_FLASH_DEFAULT_RPM = 10000;
constexpr uint16_t SHIFT_HYSTERESIS_RPM = 200;
constexpr uint32_t SHIFT_CONFIRM_MS = 100;
constexpr uint32_t SHIFT_FLASH_HALF_PERIOD_MS = 60;

// Pour essais au moniteur serie.
uint16_t shiftOnRpm = SHIFT_ON_DEFAULT_RPM;
uint16_t shiftFlashRpm = SHIFT_FLASH_DEFAULT_RPM;
bool shiftEnabled = true;

enum TachMode {
  TACH_REAL,
  TACH_SIMULATION
};

TachMode tachMode = TACH_REAL;
uint16_t simulatedRpm = 0;
uint16_t activeRpm = 0;

// =============================================================
// ETATS TEMPERATURE
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

TempMode tempMode = TEMP_AUTOMATIC;
SensorState sensorState = SENSOR_OPEN;

uint16_t lastAdc = 0;
uint32_t filteredAdcX16 = 0;
bool filteredAdcInitialized = false;
uint16_t measuredResistanceOhm = 0;
uint16_t targetTempPwm = 0;
uint16_t appliedTempPwm = 0;
uint16_t manualTempPwm = 0;

bool warningLedState = false;
uint32_t lastWarningBlinkMs = 0;

// =============================================================
// TACH ISR
// =============================================================

volatile uint32_t tachLastPulseUs = 0;
volatile uint32_t tachPeriodUs = 0;
volatile uint32_t tachLastValidPulseUs = 0;
volatile bool tachNewPeriod = false;

void ICACHE_RAM_ATTR tachPulseInterrupt() {
  const uint32_t nowUs = micros();

  if (tachLastPulseUs == 0) {
    tachLastPulseUs = nowUs;
    tachLastValidPulseUs = nowUs;
    return;
  }

  const uint32_t period = nowUs - tachLastPulseUs;
  tachLastPulseUs = nowUs;

  if (period < TACH_MIN_PERIOD_US) {
    return;
  }

  tachPeriodUs = period;
  tachLastValidPulseUs = nowUs;
  tachNewPeriod = true;
}

// =============================================================
// OUTILS
// =============================================================

void setWarningLed(bool on) {
  warningLedState = on;
  digitalWrite(PIN_WARNING_LED, on ? LOW : HIGH);
}

void setShiftLight(bool on) {
  digitalWrite(PIN_SHIFT_LIGHT, on ? HIGH : LOW);
}

void applyTempPwm(uint16_t pwm) {
  if (pwm > TEMP_PWM_LIMIT) {
    pwm = TEMP_PWM_LIMIT;
  }

  appliedTempPwm = pwm;
  analogWrite(PIN_TEMP_GAUGE, pwm);
}

uint16_t slewPwm(uint16_t current, uint16_t target) {
  if (target > current + TEMP_SLEW_STEP) {
    return current + TEMP_SLEW_STEP;
  }

  if (current > target + TEMP_SLEW_STEP) {
    return current - TEMP_SLEW_STEP;
  }

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

  if (!filteredAdcInitialized) {
    filteredAdcX16 =
      static_cast<uint32_t>(rawAdc) * 16UL;
    filteredAdcInitialized = true;
  } else {
    // EMA : environ 1/8 de nouvelle mesure.
    const uint32_t target =
      static_cast<uint32_t>(rawAdc) * 16UL;

    filteredAdcX16 =
      (filteredAdcX16 * 7UL + target) / 8UL;
  }

  lastAdc =
    static_cast<uint16_t>(filteredAdcX16 / 16UL);

  if (lastAdc <= ADC_SHORT_THRESHOLD) {
    sensorState = SENSOR_SHORT;
    measuredResistanceOhm = 0;
    return;
  }

  if (lastAdc >= ADC_OPEN_THRESHOLD) {
    sensorState = SENSOR_OPEN;
    measuredResistanceOhm = 65535;
    return;
  }

  sensorState = SENSOR_OK;

  const uint32_t numerator =
    SENSOR_PULLUP_OHM * static_cast<uint32_t>(lastAdc);

  const uint32_t denominator =
    static_cast<uint32_t>(1023U - lastAdc);

  uint32_t resistance = numerator / denominator;

  if (resistance > 65535UL) {
    resistance = 65535UL;
  }

  measuredResistanceOhm =
    static_cast<uint16_t>(resistance);
}

float resistanceToNeedlePercent(uint16_t resistanceOhm) {
  float r = static_cast<float>(resistanceOhm);

  if (r >= TEMP_R_COLD_OHM) {
    return 0.0f;
  }

  if (r <= TEMP_R_HOT_OHM) {
    return NEEDLE_MAX_PERCENT;
  }

  // Progression logarithmique continue 0..1.
  // Une NTC evolue approximativement de facon exponentielle
  // avec la temperature : le log donne donc un mouvement beaucoup
  // plus regulier de l'aiguille sur toute la plage.
  const float coldLog = logf(TEMP_R_COLD_OHM);
  const float hotLog  = logf(TEMP_R_HOT_OHM);
  const float rLog    = logf(r);

  float t = (coldLog - rLog) / (coldLog - hotLog);

  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;

  return t * NEEDLE_MAX_PERCENT;
}

uint16_t needlePercentToGaugePwm(float percent) {
  if (percent <= GAUGE_TABLE[0].percent) {
    return GAUGE_TABLE[0].pwm;
  }

  if (percent >= GAUGE_TABLE[GAUGE_POINT_COUNT - 1].percent) {
    return GAUGE_TABLE[GAUGE_POINT_COUNT - 1].pwm;
  }

  for (uint8_t i = 0; i < GAUGE_POINT_COUNT - 1; ++i) {
    const GaugePoint &a = GAUGE_TABLE[i];
    const GaugePoint &b = GAUGE_TABLE[i + 1];

    if (percent >= a.percent && percent <= b.percent) {
      const float span = b.percent - a.percent;
      const float pos  = (percent - a.percent) / span;

      const float pwm =
        static_cast<float>(a.pwm) +
        pos * static_cast<float>(
          static_cast<int32_t>(b.pwm) -
          static_cast<int32_t>(a.pwm)
        );

      if (pwm <= 0.0f) return 0;
      if (pwm >= TEMP_PWM_NORMAL_MAX) return TEMP_PWM_NORMAL_MAX;

      return static_cast<uint16_t>(pwm + 0.5f);
    }
  }

  return 400;
}

uint16_t resistanceToGaugePwm(uint16_t resistanceOhm) {
  const float percent =
    resistanceToNeedlePercent(resistanceOhm);

  return needlePercentToGaugePwm(percent);
}

void updateTempTarget() {
  if (tempMode == TEMP_MANUAL) {
    targetTempPwm = manualTempPwm;
    return;
  }

  // En circuit ouvert, on met la jauge au froid.
  if (sensorState == SENSOR_OPEN) {
    targetTempPwm = 400;
    return;
  }

  // Court-circuit franc = resistance quasi nulle :
  // comportement equivalent a une temperature extreme.
  if (sensorState == SENSOR_SHORT) {
    targetTempPwm = 900;
    return;
  }

  targetTempPwm =
    resistanceToGaugePwm(measuredResistanceOhm);
}

// =============================================================
// ALERTE TEMPERATURE LED INTEGREE
// =============================================================

void updateTemperatureWarning(uint32_t nowMs) {
  if (sensorState == SENSOR_OPEN) {
    // Capteur debranche : clignotement lent de diagnostic.
    if (nowMs - lastWarningBlinkMs >= 500) {
      lastWarningBlinkMs = nowMs;
      setWarningLed(!warningLedState);
    }
    return;
  }

  if (sensorState == SENSOR_SHORT ||
      measuredResistanceOhm <= TEMP_WARNING_BLINK_OHM) {

    if (nowMs - lastWarningBlinkMs >= WARNING_BLINK_MS) {
      lastWarningBlinkMs = nowMs;
      setWarningLed(!warningLedState);
    }
    return;
  }

  if (measuredResistanceOhm <= TEMP_WARNING_FIXED_OHM) {
    setWarningLed(true);
    return;
  }

  setWarningLed(false);
}

// =============================================================
// ANIMATION DE DEMARRAGE
// =============================================================

void runStartupAnimation() {
  Serial.println();
  Serial.println("Animation demarrage : temperature 0% -> 880 -> 0% + S x3");

  // La jauge commence sa course utile a PWM 400.
  applyTempPwm(400);
  setShiftLight(false);
  delay(200);

  for (uint16_t pwm = 400;
       pwm <= STARTUP_MAX_PWM;
       pwm += STARTUP_STEP_PWM) {

    applyTempPwm(pwm);

    bool sOn = false;

    for (uint8_t i = 0; i < 3; ++i) {
      if (pwm >= SHIFT_BLINK_WINDOWS[i][0] &&
          pwm < SHIFT_BLINK_WINDOWS[i][1]) {
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

  for (int pwm = STARTUP_MAX_PWM;
       pwm >= 400;
       pwm -= STARTUP_STEP_PWM) {

    applyTempPwm(static_cast<uint16_t>(pwm));
    delay(STARTUP_STEP_MS);
    yield();
  }

  setShiftLight(false);
  applyTempPwm(400);

  // Stabilisation de la lecture sonde avant passage automatique.
  delay(300);

  uint32_t adcAccumulator = 0;
  constexpr uint8_t STARTUP_SENSOR_READS = 20;

  for (uint8_t i = 0; i < STARTUP_SENSOR_READS; ++i) {
    adcAccumulator += readAdcAverage();
    delay(100);
    yield();
  }

  lastAdc = static_cast<uint16_t>(
    adcAccumulator / STARTUP_SENSOR_READS
  );

  if (lastAdc <= ADC_SHORT_THRESHOLD) {
    sensorState = SENSOR_SHORT;
    measuredResistanceOhm = 0;
  } else if (lastAdc >= ADC_OPEN_THRESHOLD) {
    sensorState = SENSOR_OPEN;
    measuredResistanceOhm = 65535;
  } else {
    sensorState = SENSOR_OK;

    const uint32_t numerator =
      SENSOR_PULLUP_OHM * static_cast<uint32_t>(lastAdc);

    const uint32_t denominator =
      static_cast<uint32_t>(1023U - lastAdc);

    uint32_t resistance = numerator / denominator;

    if (resistance > 65535UL) {
      resistance = 65535UL;
    }

    measuredResistanceOhm =
      static_cast<uint16_t>(resistance);
  }

  // V19 FINAL : passage AUTOMATIQUE immediat apres la sequence.
  tempMode = TEMP_AUTOMATIC;
  updateTempTarget();

  // Rejoint doucement la vraie position de temperature.
  while (appliedTempPwm != targetTempPwm) {
    applyTempPwm(
      slewPwm(appliedTempPwm, targetTempPwm)
    );
    delay(20);
    yield();
  }

  Serial.println("Animation terminee -> mode AUTOMATIQUE.");
  Serial.print("ADC sonde = ");
  Serial.print(lastAdc);
  Serial.print(" | R = ");
  Serial.print(measuredResistanceOhm);
  Serial.print(" ohm | PWM auto = ");
  Serial.println(targetTempPwm);
  Serial.println();
}

// =============================================================
// REGIME MOTEUR
// =============================================================

uint16_t calculateRealRpm() {
  uint32_t localPeriod;
  uint32_t localLastValid;

  noInterrupts();
  localPeriod = tachPeriodUs;
  localLastValid = tachLastValidPulseUs;
  interrupts();

  if (localLastValid == 0) {
    return 0;
  }

  const uint32_t ageUs = micros() - localLastValid;

  if (ageUs > TACH_ZERO_TIMEOUT_MS * 1000UL) {
    return 0;
  }

  if (localPeriod < TACH_MIN_PERIOD_US) {
    return 0;
  }

  const uint32_t denominator =
    localPeriod *
    static_cast<uint32_t>(TACH_PULSES_PER_REVOLUTION);

  if (denominator == 0) {
    return 0;
  }

  uint32_t rpm = 60000000UL / denominator;

  if (rpm > 16000UL) {
    rpm = 16000UL;
  }

  return static_cast<uint16_t>(rpm);
}

// =============================================================
// SHIFT LIGHT
// =============================================================

bool shiftOnLatched = false;
bool shiftFlashLatched = false;

uint32_t shiftAboveOnSinceMs = 0;
uint32_t shiftAboveFlashSinceMs = 0;

void updateShiftLight(uint32_t nowMs) {
  if (!shiftEnabled) {
    shiftOnLatched = false;
    shiftFlashLatched = false;
    setShiftLight(false);
    return;
  }

  const uint16_t rpm = activeRpm;

  // Zone flash.
  if (rpm >= shiftFlashRpm) {
    if (shiftAboveFlashSinceMs == 0) {
      shiftAboveFlashSinceMs = nowMs;
    }

    if (nowMs - shiftAboveFlashSinceMs >= SHIFT_CONFIRM_MS) {
      shiftFlashLatched = true;
      shiftOnLatched = true;
    }
  } else {
    shiftAboveFlashSinceMs = 0;

    if (rpm + SHIFT_HYSTERESIS_RPM < shiftFlashRpm) {
      shiftFlashLatched = false;
    }
  }

  // Zone fixe.
  if (rpm >= shiftOnRpm) {
    if (shiftAboveOnSinceMs == 0) {
      shiftAboveOnSinceMs = nowMs;
    }

    if (nowMs - shiftAboveOnSinceMs >= SHIFT_CONFIRM_MS) {
      shiftOnLatched = true;
    }
  } else {
    shiftAboveOnSinceMs = 0;

    if (rpm + SHIFT_HYSTERESIS_RPM < shiftOnRpm) {
      shiftOnLatched = false;
      shiftFlashLatched = false;
    }
  }

  if (shiftFlashLatched) {
    const bool on =
      ((nowMs / SHIFT_FLASH_HALF_PERIOD_MS) & 1U) != 0;
    setShiftLight(on);
    return;
  }

  setShiftLight(shiftOnLatched);
}

// =============================================================
// COMMANDES SERIE
// =============================================================

char commandBuffer[64];
uint8_t commandLength = 0;

void printHelp() {
  Serial.println();
  Serial.println("Commandes :");
  Serial.println("  status");
  Serial.println("  reset                 redemarre l'ESP");
  Serial.println("  sweep                 relance l'animation");
  Serial.println("  auto                  temperature reelle");
  Serial.println("  manual                mode temperature manuel");
  Serial.println("  pwm 0..1023          mode MANUEL / TEST");
  Serial.println("                         >900 = uniquement pour test court");
  Serial.println("  tach real");
  Serial.println("  tach sim");
  Serial.println("  rpm 0..16000");
  Serial.println("  shift on");
  Serial.println("  shift off");
  Serial.println("  shifton 1000..15000");
  Serial.println("  shiftflash 1000..16000");
  Serial.println();
}

void printStatus() {
  Serial.println();
  Serial.println("----- ETAT V19.1 -----");

  Serial.print("Temperature mode : ");
  Serial.println(
    tempMode == TEMP_AUTOMATIC ? "AUTOMATIQUE" : "MANUEL"
  );

  Serial.print("ADC : ");
  Serial.println(lastAdc);

  Serial.print("Sonde : ");
  if (sensorState == SENSOR_OK) {
    Serial.println("OK");
  } else if (sensorState == SENSOR_SHORT) {
    Serial.println("COURT-CIRCUIT");
  } else {
    Serial.println("OUVERTE");
  }

  Serial.print("Resistance sonde : ");
  Serial.print(measuredResistanceOhm);
  Serial.println(" ohm");

  Serial.print("PWM cible : ");
  Serial.println(targetTempPwm);

  Serial.print("PWM applique : ");
  Serial.println(appliedTempPwm);

  Serial.print("PWM frequence : ");
  Serial.print(PWM_FREQUENCY_HZ);
  Serial.println(" Hz");

  Serial.print("Tach : ");
  Serial.println(tachMode == TACH_REAL ? "REEL" : "SIMULATION");

  Serial.print("RPM : ");
  Serial.println(activeRpm);

  Serial.print("Shift ON : ");
  Serial.println(shiftOnRpm);

  Serial.print("Shift FLASH : ");
  Serial.println(shiftFlashRpm);

  Serial.println("----------------------");
  Serial.println();
}

void processCommand(char *cmd) {
  while (*cmd == ' ') {
    ++cmd;
  }

  if (strcmp(cmd, "help") == 0) {
    printHelp();
    return;
  }

  if (strcmp(cmd, "status") == 0) {
    updateTemperatureSensor();
    updateTempTarget();
    printStatus();
    return;
  }

  if (strcmp(cmd, "reset") == 0 ||
      strcmp(cmd, "restart") == 0) {
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
    Serial.println("Temperature : mode AUTOMATIQUE.");
    return;
  }

  if (strcmp(cmd, "manual") == 0) {
    tempMode = TEMP_MANUAL;
    manualTempPwm = appliedTempPwm;
    Serial.println("Temperature : mode MANUEL.");
    return;
  }

  if (strncmp(cmd, "pwm ", 4) == 0) {
    char *endPtr = nullptr;
    long value = strtol(cmd + 4, &endPtr, 10);

    if (*endPtr != '\0' || value < 0 || value > TEMP_PWM_LIMIT) {
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
    char *endPtr = nullptr;
    long value = strtol(cmd + 4, &endPtr, 10);

    if (*endPtr != '\0' || value < 0 || value > 16000) {
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
    char *endPtr = nullptr;
    long value = strtol(cmd + 8, &endPtr, 10);

    if (*endPtr != '\0' || value < 1000 || value > 15000) {
      Serial.println("Erreur : shifton 1000..15000");
      return;
    }

    shiftOnRpm = static_cast<uint16_t>(value);

    Serial.print("Shift fixe = ");
    Serial.println(shiftOnRpm);
    return;
  }

  if (strncmp(cmd, "shiftflash ", 11) == 0) {
    char *endPtr = nullptr;
    long value = strtol(cmd + 11, &endPtr, 10);

    if (*endPtr != '\0' || value < 1000 || value > 16000) {
      Serial.println("Erreur : shiftflash 1000..16000");
      return;
    }

    shiftFlashRpm = static_cast<uint16_t>(value);

    Serial.print("Shift clignotant = ");
    Serial.println(shiftFlashRpm);
    return;
  }

  Serial.println("Commande inconnue. Taper help.");
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());

    if (c == '\r') {
      continue;
    }

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

void setup() {
  pinMode(PIN_TEMP_GAUGE, OUTPUT);
  pinMode(PIN_SHIFT_LIGHT, OUTPUT);
  pinMode(PIN_WARNING_LED, OUTPUT);
  pinMode(PIN_TACH_INPUT, INPUT_PULLUP);

  analogWriteRange(PWM_RANGE);
  analogWriteFreq(PWM_FREQUENCY_HZ);

  applyTempPwm(400);
  setShiftLight(false);
  setWarningLed(false);

  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("=== CBR900RR V19.1 FINAL CONTINU ===");
  Serial.print("Reset : ");
  Serial.println(ESP.getResetReason());

  // Animation AVANT activation du fonctionnement normal.
  runStartupAnimation();

  attachInterrupt(
    digitalPinToInterrupt(PIN_TACH_INPUT),
    tachPulseInterrupt,
    FALLING
  );

  // runStartupAnimation() a deja active le mode automatique.
  tempMode = TEMP_AUTOMATIC;
  tachMode = TACH_REAL;

  lastTempControlMs = millis();
  lastRpmUpdateMs = millis();

  printHelp();
}

void loop() {
  readSerialCommands();

  const uint32_t nowMs = millis();

  // Temperature.
  if (nowMs - lastTempControlMs >= TEMP_CONTROL_PERIOD_MS) {
    lastTempControlMs = nowMs;

    updateTemperatureSensor();
    updateTempTarget();

    const uint16_t nextPwm =
      slewPwm(appliedTempPwm, targetTempPwm);

    applyTempPwm(nextPwm);

    updateTemperatureWarning(nowMs);
  }

  // Regime.
  if (nowMs - lastRpmUpdateMs >= RPM_UPDATE_MS) {
    lastRpmUpdateMs = nowMs;

    if (tachMode == TACH_REAL) {
      activeRpm = calculateRealRpm();
    } else {
      activeRpm = simulatedRpm;
    }
  }

  updateShiftLight(nowMs);

  // Log leger, toutes les 5 secondes.
  if (nowMs - lastHeartbeatMs >= 5000) {
    lastHeartbeatMs = nowMs;

    Serial.print("LIVE | ADC=");
    Serial.print(lastAdc);
    Serial.print(" | R=");
    Serial.print(measuredResistanceOhm);
    Serial.print(" | cible=");
    Serial.print(targetTempPwm);
    Serial.print(" | PWM=");
    Serial.print(appliedTempPwm);
    Serial.print(" | pos=");
    if (sensorState == SENSOR_OK) {
      Serial.print(resistanceToNeedlePercent(measuredResistanceOhm), 1);
      Serial.print("%");
    } else {
      Serial.print("ERR");
    }
    Serial.print(" | RPM=");
    Serial.print(activeRpm);
    Serial.print(" | shift=");
    Serial.println(
      digitalRead(PIN_SHIFT_LIGHT) ? "ON" : "OFF"
    );
  }

  yield();
}
