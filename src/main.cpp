#include <Arduino.h>
#include <AccelStepper.h>
#include <Preferences.h>
#include <Update.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ESPmDNS.h>
#include <time.h>
#include <mbedtls/sha256.h>

#define BUTTON_PIN 3
#define LONGPRESS_MS 1000
#define STARTUP_LOG_WINDOW_MS 10000
#define STARTUP_LOG_INTERVAL_MS 1000
#define TOUCH_DEBOUNCE_MS 50
#define CALIBRATION_REVERSE_WINDOW_MS 2500
#define CALIBRATION_SPEED_STEPS_PER_SEC 120
#define WIFI_CONNECT_TIMEOUT_MS 10000
#define NTP_SYNC_TIMEOUT_MS 5000
#define WATCH_PROFILES 3
#define LOCAL_TIME_ZONE "MST7MDT,M3.2.0,M11.1.0"
#define MIN_FIRMWARE_SIZE_BYTES 100000
#define DEFERRED_RESTART_DELAY_MS 2500
#define CENTER_RETURN_RPM 20
#define MDNS_HOSTNAME "chrono-winder"
#define FIRMWARE_VERSION "v1.9"
#define RELEASE_MANIFEST_URL "https://github.com/Lerxtwood/ChronoWinder/releases/latest/download/manifest.json"

void processTouch();
void processOperating();
void processDeferredRestart();
bool isTouchPressed(int pin);
void logButtonEdge(bool isPressed);
void logStartupHeartbeat();
void updateDebouncedTouch();
void processAutoDailyWinding();
void loadConfig();
void saveConfig();
void parseConfigFromRequest();
void applyAutomaticTpdSettings();
void calculateAutomaticTpdSettings(uint16_t turnsPerDay, uint16_t &activeRpm, uint16_t &turnsPerBurst, uint16_t &restMinutes);
void setupWiFi();
void setupTime();
void setupWebServer();
void setupMdns();
void handleRoot();
void handleStatus();
void handleSave();
void handleAction();
void handleProfile();
void handleRestart();
void handleUpdatePage();
void handlePrepareUpdate();
void handleRemoteUpdateCheck();
void handleRemoteUpdateInstall();
void handleRemoteUpdateAuto();
void handleUpdateFinished();
void handleFirmwareUpload();
bool fetchRemoteUpdateManifest(String &manifest, String &error);
bool parseRemoteUpdateManifest(const String &manifest, String &version, String &firmwareUrl, String &sha256, uint32_t &size, String &error);
bool isRemoteVersionNewer(const String &remoteVersion);
bool installRemoteFirmware(const String &firmwareUrl, const String &expectedSha256, uint32_t expectedSize, String &error);
String jsonEscape(const String &value);
String pageStart(const String &title);
String pageEnd();
String htmlEscape(const String &value);
String jsStringEscape(const String &value);
void updateDailyTurnCounter();
int currentDateKey();
uint16_t remainingTurnsToday();
void recordCompletedTurns(uint16_t turns);
void persistDailyTurnCounter();
uint16_t turnsForNextBurst();
bool shouldUseDailyTurnLimit();
bool dailyTurnLimitReached();
void startConfiguredBurst(uint16_t turns);
long directionalCenterPosition();
long nearestCenterPosition();
void moveToNearestCenter(const char *reason);
float rpmToStepsPerSecond(uint16_t rpm);
void applyMotorMotion(float maxStepsPerSecond);
const char *directionName();
const char *phaseName();
String actionButton(const String &command, const String &label);
String statusText();
String nextActionText();
String statusTile(const String &label, const String &value);
String burstProgressText();
void stopWinderNow(const char *reason);
void requestRestart(const char *reason, unsigned long delayMs = DEFERRED_RESTART_DELAY_MS);
bool centerAndStopForMaintenance(const char *reason);
void centerAndStopForFirmwareUpdate(const char *reason);
void resetDailyCounter();
void loadProfile(uint8_t index);
void saveProfile(uint8_t index);
String profileName(uint8_t index);

const int TOTAL_RUN_TIMES_STEPPER = 3;
const byte Fullstep = 4;
const byte Halfstep = 8;
const short fullResolution = 2038;
const float StepDegreeHalf = 11.32;
const float StepDegreeFull = 5.82;
bool firstMove = true;
int runTimes = 0;
const float StepsPerOutputRotation = 360.0 * StepDegreeHalf;
const uint16_t DefaultStepsPerOutputRotation = lround(StepsPerOutputRotation);

long long touchStart = 0;
unsigned long startupLogUntil = 0;
unsigned long lastStartupLog = 0;
bool rawTouchPressed = false;
bool lastRawTouchPressed = false;
unsigned long rawTouchChangedAt = 0;
unsigned long lastCalibrationEndedAt = 0;
bool firmwareUploadRejected = false;
String firmwareUpdateError = "";
bool firmwareUpdateInProgress = false;
bool restartPending = false;
unsigned long restartAt = 0;
int calibrationDirection = 1;
int lastCalibrationDirection = 1;
uint8_t selectedProfileSlot = 0;
bool nextBothBurstCcw = false;

enum OperationState {
  STANDBY = 0,
  CALIBRATION_ENTRY = 1,
  CALIBRATION = 2,
  CALIBRATION_STOP = 3,
  OPERATION_START = 4,
  OPERATION = 5,
  OPERATION_STOP = 6
};

enum MotorPhase {
  MOTOR_IDLE = 0,
  MOTOR_CW = 1,
  MOTOR_CCW = 2,
  MOTOR_REST = 3
};

enum RotationDirection {
  DIRECTION_CW = 0,
  DIRECTION_CCW = 1,
  DIRECTION_BOTH = 2
};

enum TouchState {
  TOUCH_STANDBY = 0,
  TOUCH_START = 1,
  TOUCH = 2,
  TOUCH_STOP = 3
};

enum TouchType {
  NONE = 0,
  SIMPLE = 1,
  LONG = 2
};

const char *operationStateName(OperationState state);
const char *touchStateName(TouchState state);
const char *touchTypeName(TouchType type);
void setOperationState(OperationState nextState, const char *reason);
void setTouchState(TouchState nextState);
void setTouchType(TouchType nextType, const char *reason);

struct AppConfig {
  String wifiSsid;
  String wifiPassword;
  RotationDirection direction = DIRECTION_CW;
  uint16_t turnsPerDay = 650;
  uint16_t activeRpm = 6;
  uint16_t turnsPerBurst = 10;
  uint16_t restMinutes = 5;
  uint16_t stepsPerRotation = DefaultStepsPerOutputRotation;
  uint16_t calibrationSpeed = CALIBRATION_SPEED_STEPS_PER_SEC;
  bool disableMotorDuringRest = true;
  bool manualRunUsesDailyLimit = true;
  bool autoDailyWinding = false;
  bool automaticBasedOnTpd = false;
  bool autoInstallFirmwareUpdates = false;
};

struct WatchProfile {
  String name;
  RotationDirection direction = DIRECTION_CW;
  uint16_t turnsPerDay = 650;
  uint16_t activeRpm = 6;
  uint16_t turnsPerBurst = 10;
  uint16_t restMinutes = 5;
  uint16_t stepsPerRotation = DefaultStepsPerOutputRotation;
};

Preferences preferences;
WebServer server(80);
AppConfig config;
OperationState opState = STANDBY;
MotorPhase motorPhase = MOTOR_IDLE;
TouchState touchState = TOUCH_STANDBY;
TouchType touchType = NONE;
bool currentOperationManual = false;
bool autoDailySuppressedByManualStop = false;
int dailyTurnDate = 0;
uint16_t completedTurnsToday = 0;
uint16_t activeBurstTurns = 0;
long activeBurstStartPosition = 0;
unsigned long restUntil = 0;
bool singleTurnTestMode = false;

struct TouchButton {
  byte wasPressed = LOW;
  byte isPressed = LOW;
};

TouchButton touch;

// IN1-IN3-IN2-IN4
AccelStepper motor1(Halfstep, 4, 6, 5, 7);

float degreeNormal = 180;
float degreeCalibration = 360;

void setup(void) {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("watch_winder firmware started");
  Serial.printf("Build: %s %s\n", __DATE__, __TIME__);
  Serial.printf("Chip model: %s, revision: %d, CPU: %d MHz\n", ESP.getChipModel(), ESP.getChipRevision(), ESP.getCpuFreqMHz());
  Serial.printf("Flash size: %u bytes\n", ESP.getFlashChipSize());
  Serial.println("Motor outputs disabled until an operation starts.");
  loadConfig();
  Serial.printf("Configured winding: %u TPD, direction=%s, %u RPM, %u turns/burst, %u rest minute(s), steps/rotation=%u, calibration=%u steps/s\n",
                config.turnsPerDay, directionName(), config.activeRpm, config.turnsPerBurst, config.restMinutes, config.stepsPerRotation, config.calibrationSpeed);

  motor1.setMaxSpeed(1000.0);
  motor1.setAcceleration(400);
  motor1.setSpeed(0);
  motor1.setCurrentPosition(0);
  motor1.disableOutputs();
  pinMode(BUTTON_PIN, INPUT_PULLDOWN);
  rawTouchPressed = isTouchPressed(BUTTON_PIN);
  lastRawTouchPressed = rawTouchPressed;
  touch.isPressed = rawTouchPressed;
  touch.wasPressed = rawTouchPressed;
  rawTouchChangedAt = millis();
  Serial.printf("Touch/button input: GPIO%d, INPUT_PULLDOWN, debounce=%d ms\n", BUTTON_PIN, TOUCH_DEBOUNCE_MS);
  Serial.println("Ready. Short touch starts/stops operation; long touch enters calibration.");
  setupWiFi();
  setupWebServer();
  setupMdns();
  startupLogUntil = millis() + STARTUP_LOG_WINDOW_MS;
}

void loop(void) {
  server.handleClient();
  processDeferredRestart();
  if (restartPending) {
    return;
  }

  logStartupHeartbeat();
  updateDebouncedTouch();
  updateDailyTurnCounter();

  processTouch();
  processAutoDailyWinding();
  processOperating();
}

void processTouch() {
  if (touchState == TOUCH_STANDBY && touch.isPressed) {
    touchStart = millis();
    setTouchState(TOUCH_START);
  } else if (touchState == TOUCH_START) {
    setTouchState(TOUCH);
  } else if (touchState == TOUCH) {
    if (!touch.isPressed) {
      setTouchState(TOUCH_STOP);
      if (millis() - touchStart < LONGPRESS_MS) {
        setTouchType(SIMPLE, "released before long-press threshold");
      }
    } else {
      if (millis() - touchStart >= LONGPRESS_MS && touchType != LONG) {
        setTouchType(LONG, "held past long-press threshold");
      }
    }
  } else if (touchState == TOUCH_STOP) {
    setTouchState(TOUCH_STANDBY);
  }
}

void processOperating() {
  if (opState == STANDBY && touchType == LONG) {
    setOperationState(CALIBRATION_ENTRY, "long touch");
  } else if (opState == STANDBY && touchType == SIMPLE) {
    currentOperationManual = true;
    setOperationState(OPERATION_START, "short touch");
  } else if (opState == CALIBRATION_ENTRY) {
    bool quickFollowUp = lastCalibrationEndedAt > 0 && millis() - lastCalibrationEndedAt <= CALIBRATION_REVERSE_WINDOW_MS;
    calibrationDirection = quickFollowUp ? -lastCalibrationDirection : 1;
    motor1.setMaxSpeed(config.calibrationSpeed + 40.0);
    motor1.setAcceleration(100);
    motor1.setCurrentPosition(0);
    motor1.setSpeed(config.calibrationSpeed * calibrationDirection);
    motor1.enableOutputs();
    Serial.printf("Calibration mode: motor enabled while touch is held, rotating %s%s.\n",
                  calibrationDirection > 0 ? "clockwise" : "counter clockwise",
                  quickFollowUp ? " after quick follow-up long touch" : "");

    setOperationState(CALIBRATION, "calibration configured");
  } else if (opState == CALIBRATION) {
    if (touchState == TOUCH) {
      motor1.runSpeed();
    } else {
      motor1.stop();
      setOperationState(CALIBRATION_STOP, "touch released");
    }
  } else if (opState == CALIBRATION_STOP) {
    motor1.setCurrentPosition(0);
    motor1.disableOutputs();
    lastCalibrationEndedAt = millis();
    lastCalibrationDirection = calibrationDirection;
    Serial.println("Calibration stopped. Position reset to 0 and motor outputs disabled.");
    setOperationState(STANDBY, "calibration complete");
    setTouchType(NONE, "calibration consumed touch");
  } else if (opState == OPERATION_START) {
    applyMotorMotion(rpmToStepsPerSecond(config.activeRpm));
    runTimes = 0;
    motorPhase = MOTOR_IDLE;
    activeBurstTurns = 0;
    activeBurstStartPosition = motor1.currentPosition();
    restUntil = 0;
    if (!dailyTurnLimitReached()) {
      Serial.printf("Starting %s run. Remaining turns today: %u/%u.\n", currentOperationManual ? "manual" : "automatic", remainingTurnsToday(), config.turnsPerDay);
      startConfiguredBurst(turnsForNextBurst());
    } else {
      Serial.printf("Operation start skipped because today's TPD target is complete: %u/%u.\n", completedTurnsToday, config.turnsPerDay);
      setOperationState(OPERATION_STOP, "daily TPD complete");
    }
    setTouchType(NONE, "operation consumed touch");
    if (opState != OPERATION_STOP) {
      setOperationState(OPERATION, "operation configured");
    }
  } else if (opState == OPERATION) {
    if (touchType == SIMPLE) {
      autoDailySuppressedByManualStop = true;
      setOperationState(OPERATION_STOP, "short touch stop");
      setTouchType(NONE, "stop consumed touch");
      moveToNearestCenter("manual stop requested");
    } else {
      if (motorPhase == MOTOR_REST) {
        if (millis() >= restUntil) {
          motorPhase = MOTOR_IDLE;
          if (dailyTurnLimitReached()) {
            setOperationState(OPERATION_STOP, "daily TPD complete");
          } else if (!currentOperationManual && !config.autoDailyWinding) {
            setOperationState(OPERATION_STOP, "automatic daily winding disabled");
          } else {
            startConfiguredBurst(turnsForNextBurst());
          }
        }
      } else if (motor1.distanceToGo() == 0) {
        recordCompletedTurns(activeBurstTurns);
        if (singleTurnTestMode) {
          motorPhase = MOTOR_IDLE;
          setOperationState(OPERATION_STOP, "one-turn test complete");
          moveToNearestCenter("one-turn test complete");
        } else if (dailyTurnLimitReached()) {
          motorPhase = MOTOR_IDLE;
          setOperationState(OPERATION_STOP, "daily TPD complete");
        } else if (!currentOperationManual && !config.autoDailyWinding) {
          motorPhase = MOTOR_IDLE;
          setOperationState(OPERATION_STOP, "automatic daily winding disabled");
        } else {
          motorPhase = MOTOR_REST;
          restUntil = millis() + ((unsigned long)config.restMinutes * 60000UL);
          if (config.disableMotorDuringRest) {
            motor1.disableOutputs();
          } else {
            motor1.enableOutputs();
          }
          Serial.printf("Burst complete. Resting for %u minute(s) with motor outputs %s.\n", config.restMinutes, config.disableMotorDuringRest ? "disabled" : "enabled");
        }
      }
    }
  } else if (opState == OPERATION_STOP && motor1.distanceToGo() == 0) {
    motor1.setCurrentPosition(0);
    setOperationState(STANDBY, "returned to position 0");
    runTimes = 0;
    currentOperationManual = false;
    singleTurnTestMode = false;
    persistDailyTurnCounter();
    motor1.disableOutputs();
    Serial.println("Operation stopped. Motor outputs disabled.");
  }

  if (opState != STANDBY && opState != CALIBRATION) {
    motor1.run();
  }
}

bool isTouchPressed(int pin) {
  return digitalRead(pin) == HIGH;
}

void processDeferredRestart() {
  if (!restartPending || millis() < restartAt) {
    return;
  }

  Serial.println("Performing deferred restart now.");
  motor1.moveTo(motor1.currentPosition());
  motor1.disableOutputs();
  server.close();
  WiFi.disconnect(false);
  WiFi.mode(WIFI_OFF);
  delay(250);
  ESP.restart();
}

void processAutoDailyWinding() {
  if (firmwareUpdateInProgress) {
    return;
  }

  if (config.autoDailyWinding && opState == STANDBY && touchType == NONE && !autoDailySuppressedByManualStop && remainingTurnsToday() > 0) {
    currentOperationManual = false;
    setOperationState(OPERATION_START, "automatic daily winding");
  }
}

void updateDebouncedTouch() {
  rawTouchPressed = isTouchPressed(BUTTON_PIN);
  unsigned long now = millis();

  if (rawTouchPressed != lastRawTouchPressed) {
    lastRawTouchPressed = rawTouchPressed;
    rawTouchChangedAt = now;
    Serial.printf("GPIO%d raw %s at %lu ms\n", BUTTON_PIN, rawTouchPressed ? "HIGH" : "LOW", now);
  }

  if (touch.isPressed != rawTouchPressed && now - rawTouchChangedAt >= TOUCH_DEBOUNCE_MS) {
    touch.wasPressed = touch.isPressed;
    touch.isPressed = rawTouchPressed;
    logButtonEdge(touch.isPressed);
  }
}

void loadConfig() {
  preferences.begin("watchwinder", true);
  config.wifiSsid = preferences.getString("wifiSsid", "");
  config.wifiPassword = preferences.getString("wifiPass", "");
  config.direction = (RotationDirection)preferences.getUChar("dir", preferences.getBool("dirCcw", false) ? DIRECTION_CCW : DIRECTION_CW);
  if (config.direction > DIRECTION_BOTH) {
    config.direction = DIRECTION_CW;
  }
  config.turnsPerDay = preferences.getUShort("tpd", 650);
  config.activeRpm = preferences.getUShort("rpm", 6);
  config.turnsPerBurst = preferences.getUShort("burst", 10);
  config.restMinutes = preferences.getUShort("restMin", 5);
  config.stepsPerRotation = preferences.getUShort("stepsRot", DefaultStepsPerOutputRotation);
  config.calibrationSpeed = preferences.getUShort("calSpd", CALIBRATION_SPEED_STEPS_PER_SEC);
  config.disableMotorDuringRest = preferences.getBool("idleOff", true);
  config.manualRunUsesDailyLimit = preferences.getBool("manualTpd", true);
  config.autoDailyWinding = preferences.getBool("autoDaily", preferences.getBool("sch0En", false));
  config.automaticBasedOnTpd = preferences.getBool("autoTpd", false);
  config.autoInstallFirmwareUpdates = preferences.getBool("fwAuto", false);
  if (config.automaticBasedOnTpd) {
    applyAutomaticTpdSettings();
  }
  dailyTurnDate = preferences.getInt("turnDate", 0);
  completedTurnsToday = preferences.getUShort("turnsDone", 0);
  autoDailySuppressedByManualStop = preferences.getBool("autoSupp", false);
  selectedProfileSlot = constrain(preferences.getUChar("selProfile", 0), 0, WATCH_PROFILES - 1);
  preferences.end();
}

void saveConfig() {
  preferences.begin("watchwinder", false);
  preferences.putString("wifiSsid", config.wifiSsid);
  preferences.putString("wifiPass", config.wifiPassword);
  preferences.putUChar("dir", config.direction);
  preferences.putBool("dirCcw", config.direction == DIRECTION_CCW);
  preferences.putUShort("tpd", config.turnsPerDay);
  preferences.putUShort("rpm", config.activeRpm);
  preferences.putUShort("burst", config.turnsPerBurst);
  preferences.putUShort("restMin", config.restMinutes);
  preferences.putUShort("stepsRot", config.stepsPerRotation);
  preferences.putUShort("calSpd", config.calibrationSpeed);
  preferences.putBool("idleOff", config.disableMotorDuringRest);
  preferences.putBool("manualTpd", config.manualRunUsesDailyLimit);
  preferences.putBool("autoDaily", config.autoDailyWinding);
  preferences.putBool("autoTpd", config.automaticBasedOnTpd);
  preferences.putBool("fwAuto", config.autoInstallFirmwareUpdates);
  preferences.putUChar("selProfile", selectedProfileSlot);
  preferences.end();
}

void parseConfigFromRequest() {
  config.wifiSsid = server.arg("ssid");
  config.wifiPassword = server.arg("password");
  String direction = server.arg("direction");
  if (direction == "both") {
    config.direction = DIRECTION_BOTH;
  } else if (direction == "ccw") {
    config.direction = DIRECTION_CCW;
  } else {
    config.direction = DIRECTION_CW;
  }
  config.turnsPerDay = constrain(server.arg("tpd").toInt(), 1, 2000);
  config.automaticBasedOnTpd = server.hasArg("autoTpd");
  if (config.automaticBasedOnTpd) {
    applyAutomaticTpdSettings();
  } else {
    config.activeRpm = constrain(server.arg("rpm").toInt(), 4, 10);
    config.turnsPerBurst = constrain(server.arg("burst").toInt(), 1, 100);
    config.restMinutes = constrain(server.arg("restMin").toInt(), 0, 240);
  }
  config.stepsPerRotation = constrain(server.arg("stepsRot").toInt(), 3900, 4300);
  config.calibrationSpeed = constrain(server.arg("calSpd").toInt(), 60, 180);
  config.disableMotorDuringRest = server.hasArg("idleOff");
  config.manualRunUsesDailyLimit = server.hasArg("manualTpd");
  config.autoDailyWinding = server.hasArg("autoDaily");
}

void applyAutomaticTpdSettings() {
  calculateAutomaticTpdSettings(config.turnsPerDay, config.activeRpm, config.turnsPerBurst, config.restMinutes);
}

void calculateAutomaticTpdSettings(uint16_t turnsPerDay, uint16_t &activeRpm, uint16_t &turnsPerBurst, uint16_t &restMinutes) {
  uint32_t bestError = UINT32_MAX;
  uint8_t bestRpm = 4;
  uint8_t bestBurst = 3;
  uint16_t bestRest = 1;

  for (uint8_t rpm = 4; rpm <= 10; rpm++) {
    for (uint8_t burst = 3; burst <= 6; burst++) {
      float idealRest = ((1440.0f * burst) / turnsPerDay) - ((float)burst / rpm);
      uint16_t rest = constrain((int)lroundf(idealRest), 1, 240);
      float cycleMinutes = ((float)burst / rpm) + rest;
      uint16_t calculatedTurns = lroundf((1440.0f * burst) / cycleMinutes);
      uint32_t error = abs((int32_t)calculatedTurns - (int32_t)turnsPerDay);

      if (error < bestError || (error == bestError && rpm < bestRpm) || (error == bestError && rpm == bestRpm && burst < bestBurst)) {
        bestError = error;
        bestRpm = rpm;
        bestBurst = burst;
        bestRest = rest;
      }
    }
  }

  activeRpm = bestRpm;
  turnsPerBurst = bestBurst;
  restMinutes = bestRest;
}

void setupWiFi() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.setHostname(MDNS_HOSTNAME);

  if (config.wifiSsid.length() > 0) {
    Serial.printf("Connecting to Wi-Fi SSID '%s'", config.wifiSsid.c_str());
    WiFi.begin(config.wifiSsid.c_str(), config.wifiPassword.c_str());
    unsigned long startedAt = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startedAt < WIFI_CONNECT_TIMEOUT_MS) {
      delay(250);
      Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("Wi-Fi connected. Config page: http://%s/\n", WiFi.localIP().toString().c_str());
      Serial.printf("Friendly config URL: http://%s.local/\n", MDNS_HOSTNAME);
      setupTime();
      return;
    }

    Serial.println("Wi-Fi connection failed. Starting setup access point.");
  } else {
    Serial.println("No saved Wi-Fi SSID. Starting setup access point.");
  }

  WiFi.softAP("WatchWinder-Setup");
  Serial.printf("Setup access point started. Connect to 'WatchWinder-Setup', then open http://%s/\n", WiFi.softAPIP().toString().c_str());
}

void setupTime() {
  configTzTime(LOCAL_TIME_ZONE, "pool.ntp.org", "time.nist.gov");
  Serial.print("Syncing local time");
  unsigned long startedAt = millis();
  struct tm timeInfo;
  while (!getLocalTime(&timeInfo, 250) && millis() - startedAt < NTP_SYNC_TIMEOUT_MS) {
    Serial.print(".");
  }
  Serial.println();

  if (getLocalTime(&timeInfo, 100)) {
    char buffer[32];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeInfo);
    Serial.printf("Local time synced: %s\n", buffer);
  } else {
    Serial.println("Local time sync failed. Daily counter reset will wait until time is available.");
  }
}

void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/action", HTTP_POST, handleAction);
  server.on("/profile", HTTP_POST, handleProfile);
  server.on("/restart", HTTP_POST, handleRestart);
  server.on("/update", HTTP_GET, handleUpdatePage);
  server.on("/prepare-update", HTTP_POST, handlePrepareUpdate);
  server.on("/remote-update-check", HTTP_GET, handleRemoteUpdateCheck);
  server.on("/remote-update-install", HTTP_POST, handleRemoteUpdateInstall);
  server.on("/remote-update-auto", HTTP_POST, handleRemoteUpdateAuto);
  server.on("/update", HTTP_POST, handleUpdateFinished, handleFirmwareUpload);
  server.begin();
  Serial.println("Configuration web server started.");
}

void setupMdns() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("mDNS not started because the device is not connected to Wi-Fi.");
    return;
  }

  if (!MDNS.begin(MDNS_HOSTNAME)) {
    Serial.println("mDNS responder failed to start.");
    return;
  }

  MDNS.addService("http", "tcp", 80);
  Serial.printf("mDNS responder started: http://%s.local/\n", MDNS_HOSTNAME);
}

void handleRoot() {
  String page;
  page.reserve(7600);
  page += pageStart("ChronoWinder");
  page += F("<header class=\"top\"><div><a class=\"eyebrow\" href=\"/\">ChronoWinder</a><h1>Settings</h1></div><nav><a class=\"active\" href=\"/\">Settings</a><a href=\"/status\">Status</a><a href=\"/update\">Firmware</a></nav></header><p class=\"status\">");
  if (WiFi.status() == WL_CONNECTED) {
    page += "Connected to ";
    page += htmlEscape(WiFi.SSID());
    page += " at ";
    page += WiFi.localIP().toString();
    page += " or http://";
    page += MDNS_HOSTNAME;
    page += ".local/";
  } else {
    page += "Setup access point active at ";
    page += WiFi.softAPIP().toString();
  }
  page += F("</p>");
  page += F("<form id=\"settingsForm\" method=\"post\" action=\"/save\">");
  page += F("<section class=\"panel\"><h2>Wi-Fi</h2>");
  page += F("<label>Wi-Fi SSID<input name=\"ssid\" value=\"");
  page += htmlEscape(config.wifiSsid);
  page += F("\" autocomplete=\"off\"></label>");
  page += F("<label>Wi-Fi password<input name=\"password\" type=\"password\" value=\"");
  page += htmlEscape(config.wifiPassword);
  page += F("\"></label>");
  page += F("</section><section class=\"panel\"><h2>Winder settings</h2>");
  page += F("<label>Steps per full rotation (increase if each turn stops short)<input name=\"stepsRot\" type=\"number\" min=\"3900\" max=\"4300\" value=\"");
  page += String(config.stepsPerRotation);
  page += F("\"></label>");
  page += F("<label>Centering speed<select name=\"calSpd\">");
  page += F("<option value=\"60\"");
  if (config.calibrationSpeed <= 60) {
    page += F(" selected");
  }
  page += F(">Slow</option><option value=\"120\"");
  if (config.calibrationSpeed > 60 && config.calibrationSpeed <= 120) {
    page += F(" selected");
  }
  page += F(">Medium</option><option value=\"180\"");
  if (config.calibrationSpeed > 120) {
    page += F(" selected");
  }
  page += F(">Fast</option></select></label>");
  page += F("<label class=\"check\"><input type=\"checkbox\" name=\"idleOff\"");
  if (config.disableMotorDuringRest) {
    page += F(" checked");
  }
  page += F(">Disable motor during rest</label>");
  page += F("<label class=\"check\"><input type=\"checkbox\" name=\"manualTpd\"");
  if (config.manualRunUsesDailyLimit) {
    page += F(" checked");
  }
  page += F(">Manual starts count toward daily TPD</label>");
  page += F("<label class=\"check\"><input type=\"checkbox\" name=\"autoDaily\"");
  if (config.autoDailyWinding) {
    page += F(" checked");
  }
  page += F(">Automatic daily winding</label>");
  page += F("</section><section class=\"panel\"><h2>Winding</h2>");
  page += F("<a class=\"resourceLink\" href=\"https://watch-winder.store/watch-winding-table/\" target=\"_blank\" rel=\"noopener\">Watch winding table</a>");
  page += F("<label>Rotation direction<select name=\"direction\">");
  page += F("<option value=\"cw\"");
  if (config.direction == DIRECTION_CW) {
    page += F(" selected");
  }
  page += F(">Clockwise</option><option value=\"ccw\"");
  if (config.direction == DIRECTION_CCW) {
    page += F(" selected");
  }
  page += F(">Counter clockwise</option><option value=\"both\"");
  if (config.direction == DIRECTION_BOTH) {
    page += F(" selected");
  }
  page += F(">Both</option></select></label>");
  page += F("<label>Turns per day (TPD)<input id=\"tpd\" name=\"tpd\" type=\"number\" min=\"1\" max=\"2000\" value=\"");
  page += String(config.turnsPerDay);
  page += F("\"></label>");
  page += F("<label class=\"check\"><input id=\"autoTpd\" type=\"checkbox\" name=\"autoTpd\"");
  if (config.automaticBasedOnTpd) {
    page += F(" checked");
  }
  page += F(">Automatic based on TPD</label>");
  page += F("<label>Active RPM (Typically 4 RPM - 10 RPM)<input id=\"rpm\" name=\"rpm\" type=\"number\" min=\"4\" max=\"10\" value=\"");
  page += String(config.activeRpm);
  page += F("\"");
  if (config.automaticBasedOnTpd) {
    page += F(" disabled");
  }
  page += F("></label>");
  page += F("<label>Turns per burst (Typically 3 - 6)<input id=\"burst\" name=\"burst\" type=\"number\" min=\"1\" max=\"100\" value=\"");
  page += String(config.turnsPerBurst);
  page += F("\"");
  if (config.automaticBasedOnTpd) {
    page += F(" disabled");
  }
  page += F("></label>");
  page += F("<label>Rest minutes between bursts (Typically 4 - 8)<input id=\"restMin\" name=\"restMin\" type=\"number\" min=\"0\" max=\"240\" value=\"");
  page += String(config.restMinutes);
  page += F("\"");
  if (config.automaticBasedOnTpd) {
    page += F(" disabled");
  }
  page += F("></label>");
  page += F("<p class=\"status\">Completed today: ");
  page += String(completedTurnsToday);
  page += F(" / ");
  page += String(config.turnsPerDay);
  page += F(" turns.</p>");
  page += F("<script>const autoTpd=document.getElementById('autoTpd'),tpd=document.getElementById('tpd'),rpm=document.getElementById('rpm'),burst=document.getElementById('burst'),restMin=document.getElementById('restMin');let autoTpdTimer=0;function calcAuto(t){let best={err:1e9,rpm:4,burst:3,rest:1};for(let r=4;r<=10;r++){for(let b=3;b<=6;b++){let ideal=(1440*b/t)-(b/r),rest=Math.min(240,Math.max(1,Math.round(ideal))),turns=Math.round(1440*b/(b/r+rest)),err=Math.abs(turns-t);if(err<best.err||(err===best.err&&r<best.rpm)||(err===best.err&&r===best.rpm&&b<best.burst))best={err:err,rpm:r,burst:b,rest:rest};}}return best;}function syncAutoTpd(){const on=autoTpd.checked,t=Math.min(2000,Math.max(1,parseInt(tpd.value||'1',10)));if(on){const v=calcAuto(t);rpm.value=v.rpm;burst.value=v.burst;restMin.value=v.rest;}rpm.disabled=on;burst.disabled=on;restMin.disabled=on;}function queueAutoTpd(){clearTimeout(autoTpdTimer);autoTpdTimer=setTimeout(syncAutoTpd,500);}autoTpd.addEventListener('change',syncAutoTpd);tpd.addEventListener('input',queueAutoTpd);tpd.addEventListener('change',syncAutoTpd);syncAutoTpd();</script>");
  page += F("</section><section class=\"panel\"><h2>Watch Profiles</h2>");
  page += F("<label>Profile<select id=\"profileSlot\" name=\"profileSlot\">");
  for (int i = 0; i < WATCH_PROFILES; i++) {
    page += F("<option value=\"");
    page += String(i);
    if (i == selectedProfileSlot) {
      page += F("\" selected>");
    } else {
      page += F("\">");
    }
    page += String(i + 1);
    page += F(" - ");
    page += htmlEscape(profileName(i));
    page += F("</option>");
  }
  page += F("</select></label><label>Profile name<input id=\"profileName\" name=\"profileName\" value=\"");
  page += htmlEscape(profileName(selectedProfileSlot));
  page += F("\" maxlength=\"24\"></label><div class=\"actions\"><button name=\"profileAction\" value=\"save\" type=\"submit\">Save current</button></div>");
  page += F("<script>const profileNames=[");
  for (int i = 0; i < WATCH_PROFILES; i++) {
    if (i > 0) {
      page += F(",");
    }
    page += F("'");
    page += jsStringEscape(profileName(i));
    page += F("'");
  }
  page += F("],settingsForm=document.getElementById('settingsForm'),profileSlot=document.getElementById('profileSlot'),profileNameInput=document.getElementById('profileName');profileSlot.addEventListener('change',()=>{profileNameInput.value=profileNames[profileSlot.value]||'';const action=document.createElement('input');action.type='hidden';action.name='profileAction';action.value='load';settingsForm.appendChild(action);settingsForm.submit();});</script>");
  page += F("</section><button class=\"primary\" name=\"saveAction\" value=\"settings\" type=\"submit\">Save settings</button>");
  page += F("</form>");
  page += pageEnd();
  server.send(200, "text/html", page);
}

void handleStatus() {
  String page;
  page.reserve(2800);
  page += pageStart("Winder Status");
  page += F("<header class=\"top\"><div><a class=\"eyebrow\" href=\"/\">ChronoWinder</a><h1>Status</h1></div><nav><a href=\"/\">Settings</a><a class=\"active\" href=\"/status\">Status</a><a href=\"/update\">Firmware</a></nav></header><section class=\"panel\"><h2>Current State</h2><div class=\"metrics\">");
  page += statusTile("State", operationStateName(opState));
  page += statusTile("Phase", phaseName());
  page += statusTile("Burst turn", burstProgressText());
  page += statusTile("Completed today", String(completedTurnsToday) + " / " + String(config.turnsPerDay));
  page += statusTile("Auto daily", config.autoDailyWinding ? "On" : "Off");
  page += statusTile("Direction", directionName());
  page += F("</div><p class=\"status next\">");
  page += nextActionText();
  page += F("</p></section><section class=\"panel\"><h2>Actions</h2><div class=\"actions\">");
  page += actionButton("testTurn", "Run one turn");
  page += actionButton("center", "Return to center");
  page += actionButton("resetCounter", "Reset today's turns");
  page += F("</div></section>");
  page += pageEnd();
  server.sendHeader("Refresh", "5");
  server.send(200, "text/html", page);
}

void handleSave() {
  String profileAction = server.arg("profileAction");
  if (profileAction == "load" || profileAction.startsWith("load:")) {
    uint8_t slot = profileAction.startsWith("load:")
                       ? constrain(profileAction.substring(5).toInt(), 0, WATCH_PROFILES - 1)
                       : constrain(server.arg("profileSlot").toInt(), 0, WATCH_PROFILES - 1);
    selectedProfileSlot = slot;
    loadProfile(slot);
    saveConfig();
    Serial.printf("Loaded profile %u: %s\n", slot + 1, profileName(slot).c_str());
    server.sendHeader("Location", "/");
    server.send(303, "text/plain", "See Other");
    return;
  }

  parseConfigFromRequest();
  selectedProfileSlot = constrain(server.arg("profileSlot").toInt(), 0, WATCH_PROFILES - 1);
  saveConfig();

  if (profileAction == "save" || profileAction.startsWith("save:")) {
    uint8_t slot = profileAction.startsWith("save:")
                       ? constrain(profileAction.substring(5).toInt(), 0, WATCH_PROFILES - 1)
                       : constrain(server.arg("profileSlot").toInt(), 0, WATCH_PROFILES - 1);
    selectedProfileSlot = slot;
    String name = profileAction.startsWith("save:") ? server.arg("p" + String(slot) + "Name") : server.arg("profileName");
    name.trim();
    if (name.length() == 0) {
      name = "Profile " + String(slot + 1);
    }
    preferences.begin("watchwinder", false);
    char key[12];
    snprintf(key, sizeof(key), "p%dName", slot);
    preferences.putString(key, name.substring(0, 24));
    preferences.end();
    saveProfile(slot);
    saveConfig();
    Serial.printf("Saved current settings to profile %u: %s\n", slot + 1, name.c_str());
    server.sendHeader("Location", "/");
    server.send(303, "text/plain", "See Other");
    return;
  }

  Serial.printf("Saved config: SSID='%s', direction=%s, TPD=%u, RPM=%u, burst=%u, rest=%u, steps/rotation=%u, calibration=%u\n",
                config.wifiSsid.c_str(), directionName(), config.turnsPerDay, config.activeRpm, config.turnsPerBurst, config.restMinutes, config.stepsPerRotation, config.calibrationSpeed);
  autoDailySuppressedByManualStop = false;
  persistDailyTurnCounter();
  bool centered = centerAndStopForMaintenance("settings saved");
  String page = pageStart("Settings saved");
  page += F("<header class=\"top\"><div><a class=\"eyebrow\" href=\"/\">ChronoWinder</a><h1>Restarting</h1></div><nav><a class=\"active\" href=\"/\">Settings</a><a href=\"/status\">Status</a><a href=\"/update\">Firmware</a></nav></header>");
  page += F("<section class=\"panel\"><h2>Settings saved</h2><div class=\"checklist\"><div><span class=\"badge ok\">OK</span>Settings saved</div><div><span class=\"badge ");
  page += centered ? F("ok\">OK") : F("notok\">NOT OK");
  page += F("</span>Center watch before restart</div><div><span class=\"badge ok\">OK</span>Restart queued</div></div><p class=\"status\">The device is restarting. The Settings page will reload automatically in a few seconds.</p></section>");
  page += F("<script>setTimeout(()=>{location.href='/'},8000);</script>");
  page += pageEnd();
  server.sendHeader("Connection", "close");
  server.sendHeader("Refresh", "8; url=/");
  server.send(200, "text/html", page);
  requestRestart("settings saved");
}

void handleAction() {
  String command = server.arg("command");

  if (command == "resetCounter") {
    resetDailyCounter();
  } else if (command == "testTurn") {
    if (opState == STANDBY) {
      singleTurnTestMode = true;
      currentOperationManual = true;
      setOperationState(OPERATION_START, "web one-turn test");
    }
  } else if (command == "center") {
    if (opState == STANDBY) {
      moveToNearestCenter("web center requested");
      setOperationState(OPERATION_STOP, "web center requested");
    } else {
      stopWinderNow("web center requested");
    }
  }

  server.sendHeader("Location", "/status");
  server.send(303, "text/plain", "See Other");
}

void handleProfile() {
  uint8_t slot = constrain(server.arg("slot").toInt(), 0, WATCH_PROFILES - 1);
  String action = server.arg("action");
  selectedProfileSlot = slot;

  if (action == "save") {
    String name = server.arg("name");
    name.trim();
    if (name.length() == 0) {
      name = "Profile " + String(slot + 1);
    }
    preferences.begin("watchwinder", false);
    char key[12];
    snprintf(key, sizeof(key), "p%dName", slot);
    preferences.putString(key, name.substring(0, 24));
    preferences.end();
    saveProfile(slot);
    saveConfig();
    Serial.printf("Saved profile %u: %s\n", slot + 1, name.c_str());
  } else if (action == "load") {
    loadProfile(slot);
    saveConfig();
    Serial.printf("Loaded profile %u: %s\n", slot + 1, profileName(slot).c_str());
  }

  server.sendHeader("Location", "/");
  server.send(303, "text/plain", "See Other");
}

void handleRestart() {
  Serial.println("Restart requested from web page.");
  String page = pageStart("Restarting");
  page += F("<header class=\"top\"><div><a class=\"eyebrow\" href=\"/\">ChronoWinder</a><h1>Restarting</h1></div><nav><a class=\"active\" href=\"/\">Settings</a><a href=\"/status\">Status</a><a href=\"/update\">Firmware</a></nav></header>");
  page += F("<section class=\"panel\"><h2>Restart queued</h2><p class=\"status\">The device is restarting. The Settings page will reload automatically in a few seconds.</p></section>");
  page += F("<script>setTimeout(()=>{location.href='/'},8000);</script>");
  page += pageEnd();
  server.sendHeader("Connection", "close");
  server.sendHeader("Refresh", "8; url=/");
  server.send(200, "text/html", page);
  requestRestart("web restart requested");
}

void handleUpdatePage() {
  String page;
  page.reserve(5600);
  page += pageStart("Firmware Update");
  page += F("<header class=\"top\"><div><a class=\"eyebrow\" href=\"/\">ChronoWinder</a><h1>Firmware</h1></div><nav><a href=\"/\">Settings</a><a href=\"/status\">Status</a><a class=\"active\" href=\"/update\">Firmware</a></nav></header>");
  page += F("<section class=\"panel\"><h2>GitHub update</h2><div class=\"hint\"><span>Current version</span><code>");
  page += FIRMWARE_VERSION;
  page += F("</code><span>Manifest</span><code>");
  page += RELEASE_MANIFEST_URL;
  page += F("</code></div><label class=\"check\"><input id=\"autoInstallFw\" type=\"checkbox\"");
  if (config.autoInstallFirmwareUpdates) {
    page += F(" checked");
  }
  page += F(">Auto-install firmware updates</label><div class=\"actions\"><button id=\"remoteCheck\" type=\"button\">Check GitHub</button><button id=\"remoteInstall\" type=\"button\" disabled>Install GitHub update</button></div><div class=\"panelRule\"></div><div class=\"progressBox\"><progress id=\"remoteProgress\" max=\"100\" value=\"0\"></progress><div id=\"remotePercent\" class=\"percent\">0%</div><p id=\"remoteStage\" class=\"stage\">Waiting</p><p id=\"remoteMessage\" class=\"message\">Check GitHub Releases for a newer firmware package.</p></div></section>");
  page += F("<section class=\"panel\"><h2>Manual Update</h2><div class=\"hint\"><span>Expected file</span><code>.pio/build/esp32-c3-devkitm-1/firmware.bin</code></div>");
  page += F("<form id=\"updateForm\" method=\"post\" action=\"/update\" enctype=\"multipart/form-data\">");
  page += F("<label class=\"uploadBox\"><span>Choose firmware .bin</span><small id=\"fileName\">No file selected</small><input type=\"file\" name=\"firmware\" accept=\".bin\"></label>");
  page += F("<button id=\"uploadButton\" class=\"primary\" type=\"submit\" disabled>Upload firmware</button></form><div class=\"panelRule\"></div>");
  page += F("<div class=\"progressBox\"><progress id=\"progress\" max=\"100\" value=\"0\"></progress><div id=\"percent\" class=\"percent\">0%</div><p id=\"stage\" class=\"stage\">Waiting</p><p id=\"message\" class=\"message\">Waiting for firmware file.</p></div></section>");
  page += F("<script>");
  page += F("const minSize=");
  page += String(MIN_FIRMWARE_SIZE_BYTES);
  page += F(",form=document.getElementById('updateForm'),bar=document.getElementById('progress'),pct=document.getElementById('percent'),stage=document.getElementById('stage'),msg=document.getElementById('message'),fileName=document.getElementById('fileName'),btn=document.getElementById('uploadButton'),remoteCheck=document.getElementById('remoteCheck'),remoteInstall=document.getElementById('remoteInstall'),autoInstallFw=document.getElementById('autoInstallFw'),remoteBar=document.getElementById('remoteProgress'),remotePct=document.getElementById('remotePercent'),remoteStage=document.getElementById('remoteStage'),remoteMessage=document.getElementById('remoteMessage');");
  page += F("function setStage(s,m){stage.textContent=s;msg.textContent=m;}");
  page += F("let remoteTimer=0;function setRemoteProgress(p){remoteBar.value=p;remotePct.textContent=p+'%';}function setRemote(s,m,p){remoteStage.textContent=s;remoteMessage.textContent=m;if(p!==undefined)setRemoteProgress(p);}function setRemoteButtons(){remoteCheck.disabled=autoInstallFw.checked;remoteInstall.disabled=autoInstallFw.checked||remoteInstall.dataset.ready!=='1';}function startRemoteProgress(){clearInterval(remoteTimer);let p=remoteBar.value||0;remoteTimer=setInterval(()=>{p=Math.min(90,p+3);setRemoteProgress(p);},900);}function stopRemoteProgress(p){clearInterval(remoteTimer);setRemoteProgress(p);}function saveAutoInstall(){fetch('/remote-update-auto',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'enabled='+(autoInstallFw.checked?'1':'0')}).catch(()=>{});setRemoteButtons();}");
  page += F("function checkRemote(autoRun){remoteCheck.disabled=true;remoteInstall.disabled=true;setRemote('Checking','Reading the latest GitHub Release manifest...',15);return fetch('/remote-update-check').then(r=>r.json().then(j=>({ok:r.ok,j:j}))).then(({ok,j})=>{if(!ok)throw new Error(j.error||'Check failed');remoteInstall.dataset.ready=j.updateAvailable?'1':'0';setRemoteButtons();setRemote(j.updateAvailable?'Update available':'Up to date',j.message,j.updateAvailable?35:100);if(autoRun&&j.updateAvailable)return installRemote();}).catch(e=>setRemote('Check failed',e.message,0)).finally(()=>{if(!autoInstallFw.checked)remoteCheck.disabled=false;});}");
  page += F("function installRemote(){remoteCheck.disabled=true;remoteInstall.disabled=true;setRemote('Installing','Centering winder and downloading firmware from GitHub...',40);startRemoteProgress();return fetch('/remote-update-install',{method:'POST'}).then(r=>r.text().then(t=>({ok:r.ok,t:t}))).then(({ok,t})=>{if(ok){stopRemoteProgress(100);document.open();document.write(t);document.close();}else{throw new Error(t||'Install failed');}}).catch(e=>{stopRemoteProgress(0);setRemoteButtons();setRemote('Install failed',e.message,0);});}");
  page += F("autoInstallFw.addEventListener('change',saveAutoInstall);remoteCheck.addEventListener('click',()=>checkRemote(false));remoteInstall.addEventListener('click',installRemote);setRemoteButtons();if(autoInstallFw.checked)checkRemote(true);");
  page += F("form.firmware.addEventListener('change',()=>{const f=form.firmware.files[0];fileName.textContent=f?f.name:'No file selected';bar.value=0;pct.textContent='0%';if(!f){btn.disabled=true;setStage('Waiting','Waiting for firmware file.');return;}const extOk=f.name.toLowerCase().endsWith('.bin'),sizeOk=f.size>=minSize;btn.disabled=!(extOk&&sizeOk);setStage(extOk&&sizeOk?'Ready':'File check failed',extOk&&sizeOk?'Ready to upload.':'Selected file must be a .bin and at least '+minSize+' bytes.');});");
  page += F("form.addEventListener('submit',e=>{e.preventDefault();const file=form.firmware.files[0];if(!file||btn.disabled){setStage('File check failed','Choose a valid firmware .bin file first.');return;}btn.disabled=true;");
  page += F("setStage('Centering watch','Returning to position 0 before upload...');fetch('/prepare-update',{method:'POST'}).then(r=>{if(!r.ok)throw new Error('prepare failed');return r.text();}).then(()=>{setStage('Uploading firmware','Sending firmware to the ESP32...');");
  page += F("const xhr=new XMLHttpRequest();xhr.open('POST','/update');xhr.upload.onprogress=ev=>{if(ev.lengthComputable){const p=Math.round(ev.loaded*100/ev.total);bar.value=p;pct.textContent=p+'%';if(p>=100)setStage('Validating firmware','Upload complete. ESP32 is validating the image...');}};");
  page += F("xhr.onload=()=>{setStage('Restarting','Firmware accepted. Restarting device...');document.open();document.write(xhr.responseText);document.close();};xhr.onerror=()=>{btn.disabled=false;setStage('Upload failed','Upload failed before validation.');};const data=new FormData();data.append('firmware',file,file.name);xhr.send(data);}).catch(()=>{btn.disabled=false;setStage('Centering failed','Could not center watch for update.');});});");
  page += F("</script>");
  page += pageEnd();
  server.send(200, "text/html", page);
}

void handlePrepareUpdate() {
  centerAndStopForFirmwareUpdate("firmware update requested");
  server.send(200, "text/plain", "ready");
}

void handleRemoteUpdateCheck() {
  String manifest;
  String error;
  if (!fetchRemoteUpdateManifest(manifest, error)) {
    server.send(500, "application/json", "{\"error\":\"" + jsonEscape(error) + "\"}");
    return;
  }

  String version;
  String firmwareUrl;
  String sha256;
  uint32_t size = 0;
  if (!parseRemoteUpdateManifest(manifest, version, firmwareUrl, sha256, size, error)) {
    server.send(500, "application/json", "{\"error\":\"" + jsonEscape(error) + "\"}");
    return;
  }

  bool updateAvailable = isRemoteVersionNewer(version);
  String message = updateAvailable
                       ? "Version " + version + " is available. Firmware size " + String(size) + " bytes."
                       : "Current firmware " + String(FIRMWARE_VERSION) + " is up to date.";
  String response = F("{\"currentVersion\":\"");
  response += jsonEscape(FIRMWARE_VERSION);
  response += F("\",\"remoteVersion\":\"");
  response += jsonEscape(version);
  response += F("\",\"updateAvailable\":");
  response += updateAvailable ? F("true") : F("false");
  response += F(",\"size\":");
  response += String(size);
  response += F(",\"message\":\"");
  response += jsonEscape(message);
  response += F("\"}");
  server.send(200, "application/json", response);
}

void handleRemoteUpdateInstall() {
  String manifest;
  String error;
  if (!fetchRemoteUpdateManifest(manifest, error)) {
    server.send(500, "text/plain", error);
    return;
  }

  String version;
  String firmwareUrl;
  String sha256;
  uint32_t size = 0;
  if (!parseRemoteUpdateManifest(manifest, version, firmwareUrl, sha256, size, error)) {
    server.send(500, "text/plain", error);
    return;
  }

  if (!isRemoteVersionNewer(version)) {
    server.send(409, "text/plain", "No newer firmware is available.");
    return;
  }

  centerAndStopForFirmwareUpdate("remote firmware update requested");
  if (!installRemoteFirmware(firmwareUrl, sha256, size, error)) {
    firmwareUpdateInProgress = false;
    server.send(500, "text/plain", error);
    return;
  }

  String page = pageStart("Firmware updated");
  page += F("<header class=\"top\"><div><a class=\"eyebrow\" href=\"/\">ChronoWinder</a><h1>Firmware</h1></div><nav><a href=\"/\">Settings</a><a href=\"/status\">Status</a><a class=\"active\" href=\"/update\">Firmware</a></nav></header>");
  page += F("<section class=\"panel\"><h2>Firmware updated</h2><div class=\"checklist\"><div><span class=\"badge ok\">OK</span>Downloaded ");
  page += htmlEscape(version);
  page += F(" from GitHub Releases</div><div><span class=\"badge ok\">OK</span>SHA-256 verification passed</div><div><span class=\"badge ok\">OK</span>Restart queued</div></div><p class=\"status\">The device is restarting. The Settings page will reload automatically in a few seconds.</p></section>");
  page += F("<script>setTimeout(()=>{location.href='/'},8000);</script>");
  page += pageEnd();
  server.sendHeader("Connection", "close");
  server.sendHeader("Refresh", "8; url=/");
  server.send(200, "text/html", page);
  autoDailySuppressedByManualStop = false;
  persistDailyTurnCounter();
  requestRestart("remote firmware update complete");
}

void handleRemoteUpdateAuto() {
  config.autoInstallFirmwareUpdates = server.arg("enabled") == "1" || server.arg("enabled") == "true";
  preferences.begin("watchwinder", false);
  preferences.putBool("fwAuto", config.autoInstallFirmwareUpdates);
  preferences.end();
  server.send(200, "application/json", config.autoInstallFirmwareUpdates ? "{\"enabled\":true}" : "{\"enabled\":false}");
}

void handleUpdateFinished() {
  bool updateOk = !firmwareUploadRejected && !Update.hasError();
  String page = pageStart(updateOk ? "Firmware updated" : "Update failed");
  page += F("<header class=\"top\"><div><a class=\"eyebrow\" href=\"/\">ChronoWinder</a><h1>Firmware</h1></div><nav><a href=\"/\">Settings</a><a href=\"/status\">Status</a><a class=\"active\" href=\"/update\">Firmware</a></nav></header>");
  if (updateOk) {
    page += F("<section class=\"panel\"><h2>Firmware updated</h2><div class=\"checklist\"><div><span class=\"badge ok\">OK</span>ESP32 image validation passed</div><div><span class=\"badge ok\">OK</span>Restart queued</div></div><p class=\"status\">The device is restarting. The Settings page will reload automatically in a few seconds.</p></section>");
    page += F("<script>setTimeout(()=>{location.href='/'},8000);</script>");
  } else {
    page += F("<section class=\"panel\"><h2>Update failed</h2><div class=\"checklist\"><div><span class=\"badge notok\">NOT OK</span>Firmware was not installed</div><div><span class=\"badge notok\">NOT OK</span>");
    page += htmlEscape(firmwareUpdateError.length() > 0 ? firmwareUpdateError : "Firmware validation failed.");
    page += F("</div></div></section>");
  }
  page += pageEnd();
  server.sendHeader("Connection", "close");
  if (updateOk) {
    server.sendHeader("Refresh", "8; url=/");
  }
  server.send(updateOk ? 200 : 500, "text/html", page);
  if (updateOk) {
    autoDailySuppressedByManualStop = false;
    persistDailyTurnCounter();
    requestRestart("firmware update complete");
  } else {
    firmwareUpdateInProgress = false;
  }
}

void handleFirmwareUpload() {
  HTTPUpload &upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    firmwareUpdateInProgress = true;
    firmwareUploadRejected = false;
    firmwareUpdateError = "";
    Serial.printf("Firmware upload started: %s\n", upload.filename.c_str());

    if (!upload.filename.endsWith(".bin")) {
      firmwareUploadRejected = true;
      firmwareUpdateError = "Firmware filename must end with .bin.";
      Serial.println(firmwareUpdateError);
      return;
    }

    if (!firmwareUpdateInProgress) {
      centerAndStopForFirmwareUpdate("firmware update direct upload");
    } else {
      motor1.disableOutputs();
    }

    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (firmwareUploadRejected) {
      return;
    }
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (firmwareUploadRejected) {
      return;
    }
    if (upload.totalSize < MIN_FIRMWARE_SIZE_BYTES) {
      firmwareUploadRejected = true;
      firmwareUpdateError = "Firmware file is too small to be a valid application image.";
      Update.abort();
      Serial.println(firmwareUpdateError);
      return;
    }
    if (Update.end(true)) {
      Serial.printf("Firmware update complete: %u bytes\n", upload.totalSize);
    } else {
      firmwareUpdateError = "ESP32 firmware validation failed.";
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    Update.abort();
    firmwareUploadRejected = true;
    firmwareUpdateError = "Firmware upload was aborted.";
    Serial.println("Firmware update aborted.");
  }
}

String jsonValueForKey(const String &json, const String &key) {
  String pattern = "\"" + key + "\"";
  int keyIndex = json.indexOf(pattern);
  if (keyIndex < 0) {
    return "";
  }
  int colonIndex = json.indexOf(':', keyIndex + pattern.length());
  if (colonIndex < 0) {
    return "";
  }
  int valueStart = colonIndex + 1;
  while (valueStart < (int)json.length() && isspace((unsigned char)json[valueStart])) {
    valueStart++;
  }
  if (valueStart >= (int)json.length()) {
    return "";
  }

  if (json[valueStart] == '"') {
    valueStart++;
    String value;
    bool escaped = false;
    for (int i = valueStart; i < (int)json.length(); i++) {
      char c = json[i];
      if (escaped) {
        value += c;
        escaped = false;
      } else if (c == '\\') {
        escaped = true;
      } else if (c == '"') {
        return value;
      } else {
        value += c;
      }
    }
    return "";
  }

  int valueEnd = valueStart;
  while (valueEnd < (int)json.length() && json[valueEnd] != ',' && json[valueEnd] != '}') {
    valueEnd++;
  }
  String value = json.substring(valueStart, valueEnd);
  value.trim();
  return value;
}

bool fetchRemoteUpdateManifest(String &manifest, String &error) {
  if (WiFi.status() != WL_CONNECTED) {
    error = "Wi-Fi is not connected.";
    return false;
  }

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setTimeout(15000);
  if (!http.begin(client, RELEASE_MANIFEST_URL)) {
    error = "Could not open release manifest URL.";
    return false;
  }

  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    error = "Manifest request failed with HTTP " + String(code) + ".";
    http.end();
    return false;
  }

  manifest = http.getString();
  http.end();
  if (manifest.length() == 0) {
    error = "Manifest was empty.";
    return false;
  }
  return true;
}

bool parseRemoteUpdateManifest(const String &manifest, String &version, String &firmwareUrl, String &sha256, uint32_t &size, String &error) {
  version = jsonValueForKey(manifest, "version");
  firmwareUrl = jsonValueForKey(manifest, "firmware");
  sha256 = jsonValueForKey(manifest, "sha256");
  size = jsonValueForKey(manifest, "size").toInt();
  sha256.toLowerCase();

  if (version.length() == 0 || firmwareUrl.length() == 0 || sha256.length() != 64 || size < MIN_FIRMWARE_SIZE_BYTES) {
    error = "Manifest is missing version, firmware URL, SHA-256, or a valid size.";
    return false;
  }
  if (!firmwareUrl.startsWith("https://")) {
    error = "Firmware URL must use HTTPS.";
    return false;
  }
  return true;
}

uint16_t versionPart(const String &version, uint8_t partIndex) {
  uint8_t currentPart = 0;
  uint16_t value = 0;
  bool collecting = false;

  for (uint16_t i = 0; i <= version.length(); i++) {
    char c = i < version.length() ? version[i] : '.';
    if (isdigit((unsigned char)c)) {
      if (currentPart == partIndex) {
        value = (value * 10) + (c - '0');
        collecting = true;
      }
    } else if (c == '.') {
      if (currentPart == partIndex) {
        return collecting ? value : 0;
      }
      currentPart++;
      value = 0;
      collecting = false;
    }
  }
  return value;
}

bool isRemoteVersionNewer(const String &remoteVersion) {
  String current = FIRMWARE_VERSION;
  for (uint8_t part = 0; part < 3; part++) {
    uint16_t remotePart = versionPart(remoteVersion, part);
    uint16_t currentPart = versionPart(current, part);
    if (remotePart > currentPart) {
      return true;
    }
    if (remotePart < currentPart) {
      return false;
    }
  }
  return false;
}

bool installRemoteFirmware(const String &firmwareUrl, const String &expectedSha256, uint32_t expectedSize, String &error) {
  if (WiFi.status() != WL_CONNECTED) {
    error = "Wi-Fi is not connected.";
    return false;
  }

  firmwareUpdateInProgress = true;
  firmwareUploadRejected = false;
  firmwareUpdateError = "";
  motor1.disableOutputs();

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setTimeout(20000);
  if (!http.begin(client, firmwareUrl)) {
    error = "Could not open firmware URL.";
    return false;
  }

  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    error = "Firmware download failed with HTTP " + String(code) + ".";
    http.end();
    return false;
  }

  if (!Update.begin(expectedSize)) {
    error = "Could not start firmware update.";
    Update.printError(Serial);
    http.end();
    return false;
  }

  mbedtls_sha256_context shaContext;
  mbedtls_sha256_init(&shaContext);
  mbedtls_sha256_starts_ret(&shaContext, 0);

  WiFiClient *stream = http.getStreamPtr();
  stream->setTimeout(10000);
  uint8_t buffer[1024];
  uint32_t written = 0;
  unsigned long lastReadAt = millis();

  while (http.connected() && written < expectedSize) {
    size_t available = stream->available();
    if (available == 0) {
      if (millis() - lastReadAt > 20000UL) {
        error = "Firmware download timed out.";
        Update.abort();
        mbedtls_sha256_free(&shaContext);
        http.end();
        return false;
      }
      delay(1);
      yield();
      continue;
    }

    size_t toRead = available < sizeof(buffer) ? available : sizeof(buffer);
    size_t remainingBytes = expectedSize - written;
    toRead = toRead < remainingBytes ? toRead : remainingBytes;
    int bytesRead = stream->readBytes(buffer, toRead);
    if (bytesRead <= 0) {
      continue;
    }
    lastReadAt = millis();
    mbedtls_sha256_update_ret(&shaContext, buffer, bytesRead);
    if (Update.write(buffer, bytesRead) != (size_t)bytesRead) {
      error = "Firmware flash write failed.";
      Update.abort();
      mbedtls_sha256_free(&shaContext);
      http.end();
      return false;
    }
    written += bytesRead;
  }

  http.end();
  if (written != expectedSize) {
    error = "Firmware download size mismatch.";
    Update.abort();
    mbedtls_sha256_free(&shaContext);
    return false;
  }

  uint8_t digest[32];
  mbedtls_sha256_finish_ret(&shaContext, digest);
  mbedtls_sha256_free(&shaContext);

  char actualSha256[65];
  for (uint8_t i = 0; i < 32; i++) {
    snprintf(actualSha256 + (i * 2), 3, "%02x", digest[i]);
  }
  actualSha256[64] = '\0';

  if (expectedSha256 != String(actualSha256)) {
    error = "Firmware SHA-256 verification failed.";
    Update.abort();
    return false;
  }

  if (!Update.end(true)) {
    error = "ESP32 firmware validation failed.";
    Update.printError(Serial);
    return false;
  }

  Serial.printf("Remote firmware update complete: %u bytes, sha256=%s\n", written, actualSha256);
  return true;
}

String pageStart(const String &title) {
  String page;
  page.reserve(2600);
  page += F("<!doctype html><html><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">");
  page += F("<title>");
  page += htmlEscape(title);
  page += F("</title><style>");
  page += F("html{min-height:100%;background:#000}");
  page += F("body{font-family:-apple-system,BlinkMacSystemFont,Segoe UI,Roboto,Helvetica,Arial,sans-serif;margin:0;min-height:100%;color:#f5f5f7;background:linear-gradient(180deg,#161617 0%,#050505 55%,#000 100%)}");
  page += F("*{box-sizing:border-box}.page{position:relative;max-width:760px;margin:0 auto;padding:30px 18px 48px}h1,h2,.eyebrow{font-family:-apple-system,BlinkMacSystemFont,Segoe UI,Roboto,Helvetica,Arial,sans-serif;letter-spacing:0;color:#f5f5f7;text-transform:uppercase}h1{position:relative;flex:0 0 100%;order:3;margin:0;padding-top:22px;font-size:28px;line-height:1.08;font-weight:850}h1:before{content:\"\";position:absolute;left:0;right:0;top:0;height:1px;background:linear-gradient(90deg,transparent,rgba(255,255,255,.18),transparent)}h2{margin:0 0 16px;font-size:20px;line-height:1.12;font-weight:850}.eyebrow{display:inline-block;order:1;margin:0;padding:6px 12px;border:1px solid rgba(255,255,255,.16);border-radius:999px;background:rgba(255,255,255,.08);box-shadow:0 8px 24px rgba(0,0,0,.24);font-size:32px;line-height:1;font-weight:900;text-decoration:none;backdrop-filter:blur(18px)}.eyebrow:hover,.eyebrow:focus{border-color:#2997ff;background:rgba(0,113,227,.18);outline:none}");
  page += F(".top{display:flex;flex-wrap:wrap;justify-content:space-between;gap:18px;align-items:flex-start;margin:8px 0 22px}.top>div{display:contents}nav{display:flex;order:2;gap:8px;flex-wrap:wrap;padding:5px;background:rgba(36,36,38,.72);border:1px solid rgba(255,255,255,.12);border-radius:999px;box-shadow:0 12px 32px rgba(0,0,0,.34);backdrop-filter:blur(18px)}nav a{position:relative;padding:8px 13px;border-radius:999px;color:#d2d2d7;text-decoration:none;font-size:14px;font-weight:700}nav a.active,nav a:hover{color:#fff;background:#0071e3}.updateDot{display:inline-block;width:8px;height:8px;margin-left:7px;border-radius:50%;background:#30d158;box-shadow:0 0 0 3px rgba(48,209,88,.18),0 0 14px rgba(48,209,88,.7);vertical-align:1px}.updateText{margin-left:6px;color:#30d158;font-size:11px;font-weight:900}");
  page += F(".panel,.status{background:rgba(28,28,30,.78);border:1px solid rgba(255,255,255,.12);border-radius:8px;box-shadow:0 18px 42px rgba(0,0,0,.42);backdrop-filter:blur(20px)}.panel{padding:22px;margin-top:18px}.status{padding:13px 15px;color:#a1a1a6}p{line-height:1.48}label{display:block;margin-top:15px;color:#f5f5f7;font-weight:700;font-size:14px}");
  page += F(".metrics{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px}.metric{padding:15px;background:rgba(44,44,46,.82);border:1px solid rgba(255,255,255,.10);border-radius:8px}.metric span{display:block;color:#a1a1a6;font-size:12px;font-weight:800;text-transform:uppercase}.metric strong{display:block;margin-top:6px;color:#f5f5f7;font-size:22px;letter-spacing:0}.next{margin-bottom:0}");
  page += F("input,select{width:100%;padding:12px 13px;margin-top:7px;font-size:16px;color:#f5f5f7;background:rgba(44,44,46,.9);border:1px solid rgba(255,255,255,.16);border-radius:8px;outline:none}input:focus,select:focus{border-color:#2997ff;box-shadow:0 0 0 4px rgba(41,151,255,.18)}input[type=checkbox]{width:auto;margin-right:9px;accent-color:#0071e3}");
  page += F("button{border:1px solid rgba(255,255,255,.16);border-radius:999px;background:rgba(44,44,46,.9);color:#f5f5f7;padding:11px 15px;font-size:15px;font-weight:800;cursor:pointer;box-shadow:0 8px 20px rgba(0,0,0,.28)}button:hover{border-color:#2997ff;color:#2997ff}.primary{width:100%;margin-top:20px;background:#0071e3;border-color:#0071e3;color:white}.primary:hover{background:#0077ed;color:white}");
  page += F(".resourceLink{display:block;margin-top:12px;padding:12px 13px;background:rgba(44,44,46,.82);border:1px solid rgba(255,255,255,.10);border-radius:8px;color:#2997ff;text-decoration:none;font-weight:800}.resourceLink:hover{background:rgba(0,113,227,.16);border-color:#2997ff}");
  page += F(".window{padding:12px;margin-top:12px;background:rgba(44,44,46,.82);border:1px solid rgba(255,255,255,.10);border-radius:8px}.window label{font-weight:600}.inline{display:flex;gap:12px}.inline label{flex:1}.check{display:flex;align-items:center}.actions{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:14px}.actions form,.profile{margin:0}.actions button,.profile button{width:100%;margin-top:0}");
  page += F(".profile{display:grid;grid-template-columns:1fr auto auto;gap:8px;align-items:end;margin-top:12px;padding:10px;background:rgba(44,44,46,.82);border:1px solid rgba(255,255,255,.10);border-radius:8px}.profile label{margin-top:0}code{color:#2997ff}.percent{font-weight:800;margin-top:8px;color:#2997ff}progress{width:100%;height:24px;margin-top:18px;accent-color:#0071e3}");
  page += F(".hint{display:grid;gap:5px;padding:13px;background:rgba(44,44,46,.82);border:1px solid rgba(255,255,255,.10);border-radius:8px}.hint span,.uploadBox small{color:#a1a1a6;font-size:12px;font-weight:800;text-transform:uppercase}.uploadBox{position:relative;display:block;padding:28px 20px;margin-top:14px;text-align:center;background:rgba(44,44,46,.82);border:1px dashed rgba(255,255,255,.24);border-radius:8px;cursor:pointer}.uploadBox:hover,.uploadBox:focus-within{border-color:#2997ff;background:rgba(0,113,227,.12)}.uploadBox span{display:block;color:#f5f5f7;font-size:18px;font-weight:800}.uploadBox input{position:absolute;inset:0;width:100%;height:100%;margin:0;opacity:0;cursor:pointer}.panelRule{height:1px;margin:18px 0 0;background:linear-gradient(90deg,transparent,rgba(255,255,255,.16),transparent)}.progressBox{margin-top:16px}.stage{margin:10px 0 0;color:#f5f5f7;font-size:18px;font-weight:850}.message{margin:4px 0 0;color:#a1a1a6}.checklist{display:grid;gap:8px}.checklist div{padding:11px;background:rgba(44,44,46,.82);border:1px solid rgba(255,255,255,.10);border-radius:8px;color:#f5f5f7}.badge{display:inline-block;margin-right:8px;padding:2px 7px;border-radius:999px;font-size:11px;font-weight:900}.badge.ok{background:rgba(48,209,88,.16);color:#30d158;border:1px solid rgba(48,209,88,.42)}.badge.notok{background:rgba(255,69,58,.16);color:#ff453a;border:1px solid rgba(255,69,58,.42)}.badge.pending{background:rgba(142,142,147,.18);color:#d2d2d7;border:1px solid rgba(142,142,147,.36)}button:disabled{opacity:.45;cursor:not-allowed}");
  page += F("@media(max-width:620px){.page{padding:22px 14px 36px}h1{font-size:25px}.eyebrow{font-size:28px}nav{justify-content:center}.metrics,.actions{grid-template-columns:1fr}.panel{padding:18px}}");
  page += F("</style></head><body><main class=\"page\">");
  return page;
}

String pageEnd() {
  return F("<script>document.addEventListener('DOMContentLoaded',()=>{const fw=document.querySelector('nav a[href=\"/update\"]'),key='chronoFirmwareUpdate';if(!fw)return;function render(j){if(!j||!j.updateAvailable||fw.querySelector('.updateDot'))return;const dot=document.createElement('span');dot.className='updateDot';dot.title='Firmware update available';fw.appendChild(dot);const text=document.createElement('span');text.className='updateText';text.textContent=j.remoteVersion;fw.appendChild(text);}try{const cached=JSON.parse(localStorage.getItem(key)||'null');if(cached&&Date.now()-cached.checkedAt<600000){render(cached);return;}}catch(e){}fetch('/remote-update-check').then(r=>r.ok?r.json():null).then(j=>{if(j){j.checkedAt=Date.now();localStorage.setItem(key,JSON.stringify(j));render(j);}}).catch(()=>{});});</script></main></body></html>");
}

String htmlEscape(const String &value) {
  String escaped = value;
  escaped.replace("&", "&amp;");
  escaped.replace("\"", "&quot;");
  escaped.replace("<", "&lt;");
  escaped.replace(">", "&gt;");
  return escaped;
}

String jsonEscape(const String &value) {
  String escaped = value;
  escaped.replace("\\", "\\\\");
  escaped.replace("\"", "\\\"");
  escaped.replace("\r", "");
  escaped.replace("\n", "\\n");
  return escaped;
}

String jsStringEscape(const String &value) {
  String escaped = value;
  escaped.replace("\\", "\\\\");
  escaped.replace("'", "\\'");
  escaped.replace("\r", "");
  escaped.replace("\n", " ");
  escaped.replace("</", "<\\/");
  return escaped;
}

void updateDailyTurnCounter() {
  int today = currentDateKey();
  if (today == 0 || today == dailyTurnDate) {
    return;
  }

  dailyTurnDate = today;
  completedTurnsToday = 0;
  autoDailySuppressedByManualStop = false;
  persistDailyTurnCounter();
  Serial.printf("New local day detected. TPD counter reset for %d.\n", dailyTurnDate);
}

int currentDateKey() {
  struct tm timeInfo;
  if (!getLocalTime(&timeInfo, 5)) {
    return dailyTurnDate;
  }
  return (timeInfo.tm_year + 1900) * 10000 + (timeInfo.tm_mon + 1) * 100 + timeInfo.tm_mday;
}

uint16_t remainingTurnsToday() {
  updateDailyTurnCounter();
  if (!shouldUseDailyTurnLimit()) {
    return config.turnsPerBurst;
  }
  if (completedTurnsToday >= config.turnsPerDay) {
    return 0;
  }
  return config.turnsPerDay - completedTurnsToday;
}

void recordCompletedTurns(uint16_t turns) {
  if (!shouldUseDailyTurnLimit()) {
    motor1.setCurrentPosition(0);
    Serial.printf("Completed %u test/manual turn(s). Daily progress unchanged: %u/%u.\n", turns, completedTurnsToday, config.turnsPerDay);
    return;
  }

  uint32_t updatedTurns = (uint32_t)completedTurnsToday + turns;
  completedTurnsToday = updatedTurns > config.turnsPerDay ? config.turnsPerDay : updatedTurns;

  motor1.setCurrentPosition(0);
  Serial.printf("Completed burst of %u turn(s). Daily progress: %u/%u.\n", turns, completedTurnsToday, config.turnsPerDay);

  if (completedTurnsToday % 10 == 0 || completedTurnsToday >= config.turnsPerDay) {
    persistDailyTurnCounter();
  }
}

void persistDailyTurnCounter() {
  preferences.begin("watchwinder", false);
  preferences.putInt("turnDate", dailyTurnDate);
  preferences.putUShort("turnsDone", completedTurnsToday);
  preferences.putBool("autoSupp", autoDailySuppressedByManualStop);
  preferences.end();
}

uint16_t turnsForNextBurst() {
  if (singleTurnTestMode) {
    return 1;
  }
  if (!shouldUseDailyTurnLimit()) {
    return config.turnsPerBurst;
  }
  uint16_t remainingTurns = remainingTurnsToday();
  return config.turnsPerBurst < remainingTurns ? config.turnsPerBurst : remainingTurns;
}

bool shouldUseDailyTurnLimit() {
  if (singleTurnTestMode) {
    return false;
  }
  return !currentOperationManual || config.manualRunUsesDailyLimit;
}

bool dailyTurnLimitReached() {
  return shouldUseDailyTurnLimit() && remainingTurnsToday() == 0;
}

void startConfiguredBurst(uint16_t turns) {
  if (turns == 0) {
    setOperationState(OPERATION_STOP, "daily TPD complete");
    return;
  }

  motor1.enableOutputs();
  motor1.setCurrentPosition(motor1.currentPosition());
  applyMotorMotion(rpmToStepsPerSecond(config.activeRpm));
  long steps = (long)turns * config.stepsPerRotation;
  activeBurstTurns = turns;
  activeBurstStartPosition = motor1.currentPosition();
  bool runCcw = config.direction == DIRECTION_CCW;
  if (config.direction == DIRECTION_BOTH) {
    runCcw = nextBothBurstCcw;
    nextBothBurstCcw = !nextBothBurstCcw;
  }
  if (runCcw) {
    motorPhase = MOTOR_CCW;
    motor1.moveTo(motor1.currentPosition() - steps);
  } else {
    motorPhase = MOTOR_CW;
    motor1.moveTo(motor1.currentPosition() + steps);
  }
  Serial.printf("%s burst started: %u turn(s) at %u RPM. Target %ld. Remaining after burst: %u.\n",
                runCcw ? "Counter clockwise" : "Clockwise", turns, config.activeRpm, motor1.targetPosition(), remainingTurnsToday() > turns ? remainingTurnsToday() - turns : 0);
}

long directionalCenterPosition() {
  long current = motor1.currentPosition();
  long relative = current;
  if (relative % config.stepsPerRotation == 0) {
    return current;
  }

  long centerIndex = relative / config.stepsPerRotation;

  if (relative < 0) {
    centerIndex--;
  }

  long lowerCenter = centerIndex * config.stepsPerRotation;
  long upperCenter = lowerCenter + config.stepsPerRotation;

  if (motorPhase == MOTOR_CCW) {
    return lowerCenter;
  }
  if (motorPhase == MOTOR_CW) {
    return upperCenter;
  }

  long distanceToLower = labs(current - lowerCenter);
  long distanceToUpper = labs(upperCenter - current);
  return distanceToLower <= distanceToUpper ? lowerCenter : upperCenter;
}

long nearestCenterPosition() {
  long current = motor1.currentPosition();
  if (current % config.stepsPerRotation == 0) {
    return current;
  }

  long centerIndex = current / config.stepsPerRotation;
  if (current < 0) {
    centerIndex--;
  }

  long lowerCenter = centerIndex * config.stepsPerRotation;
  long upperCenter = lowerCenter + config.stepsPerRotation;
  long distanceToLower = labs(current - lowerCenter);
  long distanceToUpper = labs(upperCenter - current);
  return distanceToLower <= distanceToUpper ? lowerCenter : upperCenter;
}

void moveToNearestCenter(const char *reason) {
  long centerTarget = nearestCenterPosition();
  motorPhase = MOTOR_IDLE;
  activeBurstTurns = 0;
  motor1.setCurrentPosition(motor1.currentPosition());
  activeBurstStartPosition = motor1.currentPosition();
  motor1.enableOutputs();
  applyMotorMotion(rpmToStepsPerSecond(CENTER_RETURN_RPM));
  motor1.moveTo(centerTarget);
  Serial.printf("%s. Returning to nearest center target %ld from %ld at %u RPM.\n", reason, centerTarget, motor1.currentPosition(), CENTER_RETURN_RPM);
}

void stopWinderNow(const char *reason) {
  autoDailySuppressedByManualStop = true;
  currentOperationManual = false;
  singleTurnTestMode = false;
  setTouchType(NONE, "web action consumed touch");
  setOperationState(OPERATION_STOP, reason);
  moveToNearestCenter(reason);
}

void requestRestart(const char *reason, unsigned long delayMs) {
  if (restartPending) {
    return;
  }

  restartPending = true;
  restartAt = millis() + delayMs;
  firmwareUpdateInProgress = true;
  autoDailySuppressedByManualStop = true;
  currentOperationManual = false;
  singleTurnTestMode = false;
  motorPhase = MOTOR_IDLE;
  activeBurstTurns = 0;
  activeBurstStartPosition = motor1.currentPosition();
  restUntil = 0;
  motor1.moveTo(motor1.currentPosition());
  motor1.disableOutputs();
  Serial.printf("Restart queued in %lu ms: %s\n", delayMs, reason);
}

bool centerAndStopForMaintenance(const char *reason) {
  currentOperationManual = false;
  singleTurnTestMode = false;
  setTouchType(NONE, "maintenance consumed touch");
  restUntil = 0;

  long centerTarget = nearestCenterPosition();
  motorPhase = MOTOR_IDLE;
  activeBurstTurns = 0;
  motor1.setCurrentPosition(motor1.currentPosition());
  activeBurstStartPosition = motor1.currentPosition();
  motor1.enableOutputs();
  applyMotorMotion(rpmToStepsPerSecond(CENTER_RETURN_RPM));
  motor1.moveTo(centerTarget);
  setOperationState(OPERATION_STOP, reason);
  Serial.printf("%s. Returning to center target %ld before maintenance at %u RPM.\n", reason, centerTarget, CENTER_RETURN_RPM);

  unsigned long startedAt = millis();
  while (motor1.distanceToGo() != 0 && millis() - startedAt < 15000UL) {
    motor1.run();
    delay(1);
    yield();
  }

  bool centered = motor1.distanceToGo() == 0;
  if (motor1.distanceToGo() == 0) {
    motor1.setCurrentPosition(0);
    setOperationState(STANDBY, "centered for maintenance");
    Serial.println("Winder centered for maintenance.");
  } else {
    motor1.moveTo(motor1.currentPosition());
    Serial.println("Timed out while centering for maintenance; motor stopped at current position.");
  }

  motor1.disableOutputs();
  motorPhase = MOTOR_IDLE;
  activeBurstTurns = 0;
  activeBurstStartPosition = motor1.currentPosition();
  Serial.println("Winder stopped for maintenance.");
  return centered;
}

void centerAndStopForFirmwareUpdate(const char *reason) {
  firmwareUpdateInProgress = true;
  autoDailySuppressedByManualStop = true;
  bool centered = centerAndStopForMaintenance(reason);
  Serial.println(centered ? "Winder centered for firmware update." : "Winder was not centered for firmware update.");
}

void resetDailyCounter() {
  dailyTurnDate = currentDateKey();
  completedTurnsToday = 0;
  autoDailySuppressedByManualStop = false;
  persistDailyTurnCounter();
  Serial.printf("Daily turn counter reset for %d.\n", dailyTurnDate);
}

float rpmToStepsPerSecond(uint16_t rpm) {
  return (rpm * config.stepsPerRotation) / 60.0;
}

void applyMotorMotion(float maxStepsPerSecond) {
  motor1.setMaxSpeed(maxStepsPerSecond);
  motor1.setAcceleration(400);
}

const char *directionName() {
  if (config.direction == DIRECTION_CCW) {
    return "Counter clockwise";
  }
  if (config.direction == DIRECTION_BOTH) {
    return "Both";
  }
  return "Clockwise";
}

const char *phaseName() {
  switch (motorPhase) {
    case MOTOR_IDLE:
      return "idle";
    case MOTOR_CW:
      return "clockwise";
    case MOTOR_CCW:
      return "counter clockwise";
    case MOTOR_REST:
      return "resting";
  }
  return "unknown";
}

String actionButton(const String &command, const String &label) {
  String html = F("<form method=\"post\" action=\"/action\"><input type=\"hidden\" name=\"command\" value=\"");
  html += htmlEscape(command);
  html += F("\"><button type=\"submit\">");
  html += htmlEscape(label);
  html += F("</button></form>");
  return html;
}

String statusTile(const String &label, const String &value) {
  String html = F("<div class=\"metric\"><span>");
  html += htmlEscape(label);
  html += F("</span><strong>");
  html += htmlEscape(value);
  html += F("</strong></div>");
  return html;
}

String burstProgressText() {
  if (activeBurstTurns == 0 || (motorPhase != MOTOR_CW && motorPhase != MOTOR_CCW)) {
    return "-";
  }

  long movedSteps = labs(motor1.currentPosition() - activeBurstStartPosition);
  uint32_t movedTurns = (movedSteps / config.stepsPerRotation) + 1;
  uint16_t currentTurn = movedTurns > activeBurstTurns ? activeBurstTurns : movedTurns;
  if (motor1.distanceToGo() == 0) {
    currentTurn = activeBurstTurns;
  }
  return String(currentTurn) + " of " + String(activeBurstTurns);
}

String statusText() {
  String text = F("State: ");
  text += operationStateName(opState);
  text += F(", phase: ");
  text += phaseName();
  text += F(". Completed today: ");
  text += String(completedTurnsToday);
  text += F(" / ");
  text += String(config.turnsPerDay);
  text += F(" turns. Automatic daily winding: ");
  text += config.autoDailyWinding ? "on" : "off";
  if (autoDailySuppressedByManualStop) {
    text += F(", paused by manual stop");
  }
  text += F(".");
  return text;
}

String nextActionText() {
  if (opState == OPERATION && motorPhase == MOTOR_REST) {
    unsigned long now = millis();
    unsigned long remainingMs = restUntil > now ? restUntil - now : 0;
    return "Next action: next burst in about " + String((remainingMs + 59999UL) / 60000UL) + " minute(s).";
  }
  if (opState == OPERATION && (motorPhase == MOTOR_CW || motorPhase == MOTOR_CCW)) {
    return "Next action: finish current burst and return to rest.";
  }
  if (opState == OPERATION_STOP) {
    return "Next action: return to center and stop.";
  }
  if (remainingTurnsToday() == 0) {
    return "Next action: daily TPD target is complete.";
  }
  if (config.autoDailyWinding && autoDailySuppressedByManualStop) {
    return "Next action: automatic daily winding is paused by manual stop until tomorrow or until today's turns are reset.";
  }
  if (config.autoDailyWinding) {
    return "Next action: automatic daily winding can run when idle.";
  }
  return "Next action: waiting for manual start.";
}

String profileName(uint8_t index) {
  preferences.begin("watchwinder", true);
  char key[12];
  snprintf(key, sizeof(key), "p%dName", index);
  String name = preferences.getString(key, "Profile " + String(index + 1));
  preferences.end();
  return name;
}

void saveProfile(uint8_t index) {
  preferences.begin("watchwinder", false);
  char key[12];
  snprintf(key, sizeof(key), "p%dDir", index);
  preferences.putUChar(key, config.direction);
  snprintf(key, sizeof(key), "p%dTpd", index);
  preferences.putUShort(key, config.turnsPerDay);
  snprintf(key, sizeof(key), "p%dRpm", index);
  preferences.putUShort(key, config.activeRpm);
  snprintf(key, sizeof(key), "p%dBurst", index);
  preferences.putUShort(key, config.turnsPerBurst);
  snprintf(key, sizeof(key), "p%dRest", index);
  preferences.putUShort(key, config.restMinutes);
  snprintf(key, sizeof(key), "p%dSteps", index);
  preferences.putUShort(key, config.stepsPerRotation);
  preferences.end();
}

void loadProfile(uint8_t index) {
  preferences.begin("watchwinder", true);
  char key[12];
  snprintf(key, sizeof(key), "p%dDir", index);
  config.direction = (RotationDirection)preferences.getUChar(key, config.direction);
  if (config.direction > DIRECTION_BOTH) {
    config.direction = DIRECTION_CW;
  }
  snprintf(key, sizeof(key), "p%dTpd", index);
  config.turnsPerDay = preferences.getUShort(key, config.turnsPerDay);
  snprintf(key, sizeof(key), "p%dRpm", index);
  config.activeRpm = preferences.getUShort(key, config.activeRpm);
  snprintf(key, sizeof(key), "p%dBurst", index);
  config.turnsPerBurst = preferences.getUShort(key, config.turnsPerBurst);
  snprintf(key, sizeof(key), "p%dRest", index);
  config.restMinutes = preferences.getUShort(key, config.restMinutes);
  snprintf(key, sizeof(key), "p%dSteps", index);
  config.stepsPerRotation = preferences.getUShort(key, config.stepsPerRotation);
  preferences.end();
  if (config.automaticBasedOnTpd) {
    applyAutomaticTpdSettings();
  }
}

const char *operationStateName(OperationState state) {
  switch (state) {
    case STANDBY:
      return "STANDBY";
    case CALIBRATION_ENTRY:
      return "CALIBRATION_ENTRY";
    case CALIBRATION:
      return "CALIBRATION";
    case CALIBRATION_STOP:
      return "CALIBRATION_STOP";
    case OPERATION_START:
      return "OPERATION_START";
    case OPERATION:
      return "OPERATION";
    case OPERATION_STOP:
      return "OPERATION_STOP";
  }
  return "UNKNOWN";
}

const char *touchStateName(TouchState state) {
  switch (state) {
    case TOUCH_STANDBY:
      return "TOUCH_STANDBY";
    case TOUCH_START:
      return "TOUCH_START";
    case TOUCH:
      return "TOUCH";
    case TOUCH_STOP:
      return "TOUCH_STOP";
  }
  return "UNKNOWN";
}

const char *touchTypeName(TouchType type) {
  switch (type) {
    case NONE:
      return "NONE";
    case SIMPLE:
      return "SIMPLE";
    case LONG:
      return "LONG";
  }
  return "UNKNOWN";
}

void setOperationState(OperationState nextState, const char *reason) {
  if (opState == nextState) {
    return;
  }
  Serial.printf("Operation: %s -> %s (%s)\n", operationStateName(opState), operationStateName(nextState), reason);
  opState = nextState;
}

void setTouchState(TouchState nextState) {
  if (touchState == nextState) {
    return;
  }
  Serial.printf("Touch state: %s -> %s\n", touchStateName(touchState), touchStateName(nextState));
  touchState = nextState;
}

void setTouchType(TouchType nextType, const char *reason) {
  if (touchType == nextType) {
    return;
  }
  Serial.printf("Touch type: %s -> %s (%s)\n", touchTypeName(touchType), touchTypeName(nextType), reason);
  touchType = nextType;
}

void logButtonEdge(bool isPressed) {
  Serial.printf("GPIO%d debounced %s at %lu ms\n", BUTTON_PIN, isPressed ? "HIGH" : "LOW", millis());
}

void logStartupHeartbeat() {
  unsigned long now = millis();
  if (now > startupLogUntil || now - lastStartupLog < STARTUP_LOG_INTERVAL_MS) {
    return;
  }
  lastStartupLog = now;
  Serial.printf("Startup check: firmware alive at %lu ms, state=%s, touch=%s\n", now, operationStateName(opState), touchTypeName(touchType));
}
