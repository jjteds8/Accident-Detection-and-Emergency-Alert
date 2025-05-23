/**
 * Car Accident Detection & Alert System for LilyGO T-A7670E ESP32
 * - Features:
 *   * High-G and abnormal orientation detection via MPU6050
 *   * Sends SMS via A7670E module on high G, abnormal orientation, or web-based manual trigger
 *   * Web-based dashboard with multipage, simple styling
 *   * Buzzer beeps on close proximity (ultrasonic)
 *   * Sensor sensitivity adjustable in web app
 *   * Fallback: If GPS is not available, SMS includes a randomized location within 5m of Bacolod City Hall
 *   * NEO-6M GPS for Lat/Lon/Speed
 *   * Manual alert via BUTTON is DISABLED to prevent false triggering
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#include <math.h>

// Default fallback coordinates
// 10°39'25.7"N 122°56'52.9"E = 10.657139, 122.948028
const double DEFAULT_LAT = 10.657139;
const double DEFAULT_LON = 122.948028;
// 5 meters in degrees latitude ~ 0.000045°
const double LAT_RADIUS_DEG = 5.0 / 111320.0; // 111,320 meters per deg latitude
const double LON_RADIUS_DEG = 5.0 / (111320.0 * cos(DEFAULT_LAT * M_PI / 180.0));

// Generate a random coordinate within 5m radius of default
void getRandomDefaultLocation(double &lat, double &lon) {
    double theta = 2 * M_PI * (random(0, 10000) / 10000.0);
    double r = sqrt(random(0, 10000) / 10000.0) * 1.0; // 0..1
    double dLat = r * LAT_RADIUS_DEG * cos(theta);
    double dLon = r * LON_RADIUS_DEG * sin(theta);
    lat = DEFAULT_LAT + dLat;
    lon = DEFAULT_LON + dLon;
}

// Pin Definitions (match hardware!)
#define MODEM_TX_PIN        26
#define MODEM_RX_PIN        27
#define BOARD_PWRKEY_PIN    4
#define BOARD_POWERON_PIN   12
#define MODEM_RESET_PIN     5
#define MODEM_BAUDRATE      115200

#define BUZZER_PIN          14
#define BUTTON_PIN          13
#define ULTRASONIC_TRIG     19
#define ULTRASONIC_ECHO     18

#define GPS_RX_PIN          33
#define GPS_TX_PIN          22
#define GPS_BAUDRATE        9600

// --- Global Variables ---
Adafruit_MPU6050 mpu;
bool mpuPresent = false;

// WiFi Access Point
const char* AP_SSID = "CAR_ALERT_HOTSPOT";
const char* AP_PASS = "caralert123";
IPAddress local_ip(192,168,4,1);
IPAddress gateway(192,168,4,1);
IPAddress subnet(255,255,255,0);

WebServer server(80);

// Modem serial
HardwareSerial SerialAT(1);        // UART 1 for modem
// GPS serial and TinyGPS++
HardwareSerial GPS_Serial(2);      // UART 2 for GPS
TinyGPSPlus gps;

// SMS
const char* defaultRecipient = "+639948244158";
String lastRecipient = defaultRecipient;
String smsResult = "";
String smsDebugLog = "";

// Thresholds (modifiable)
float impactGThreshold = 5.0;
float flipAngleThreshold = 60.0;
uint16_t ultrasonicThreshold = 30;

// Sensor state
float lastG = 1.0;
String lastOrientation = "Normal";
float lastAccelSpeed = 0;
double gpsLat = 0, gpsLon = 0;
bool gpsIsValid = false;
float gpsSpeedKmh = 0;
unsigned long lastMPUCheck = 0;
bool lastUltrasonicLow = false;

// Alert logic
bool alertActive = false;
String lastAlertCause = "";
unsigned long lastAlertTime = 0;
bool buzzerOn = false;
bool manualAlert = false; // Only via web, not button
bool buzzerTestActive = false;

// Modem status
String modemSIM = "";
String modemNetwork = "";
String modemSignal = "";
String modemCarrier = "";
String modemVersion = "";

// Filtering
#define G_AVG_WINDOW 5
float gBuffer[G_AVG_WINDOW] = {1.0, 1.0, 1.0, 1.0, 1.0};
int gIndex = 0;

// Function declarations
void modemPowerOn();
String sendAT(const char* cmd, uint32_t timeout = 2000, bool waitForPrompt = false);
String sendSMS(String recipient, String message);
long safeReadUltrasonicCM();
void scanMPU();
void scanGPS();
void updateModemStatus();
void checkAndSendAlert();

// --- Web server page handlers ---
void handleHome();
void handleSensor();
void handleMap();
void handleManual();
void handleSettings();
void handleStatus();
void handleSendSMS();
void handleTestSMS();
void handleSetThresholds();
void handleReset();
void handleManualAlert();
void handleSetRecipient();
void handleTestBuzzer();

String getGoogleMapsWidget(double lat, double lon);
String getCSS();
String htmlHeader(const String& title, const String& nav = "");
String navBar(const String& sel);
String formatTime(unsigned long ms);

// --- Setup ---
void setup() {
    Serial.begin(115200);
    pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(ULTRASONIC_TRIG, OUTPUT);
    pinMode(ULTRASONIC_ECHO, INPUT);

    // Seed random number for fallback GPS
    randomSeed(analogRead(0));

    modemPowerOn();
    SerialAT.begin(MODEM_BAUDRATE, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
    GPS_Serial.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    WiFi.softAPConfig(local_ip, gateway, subnet);
    WiFi.softAP(AP_SSID, AP_PASS);

    // Multipage handlers
    server.on("/", handleHome);
    server.on("/sensor", handleSensor);
    server.on("/map", handleMap);
    server.on("/manual", handleManual);
    server.on("/settings", handleSettings);
    server.on("/status", handleStatus);
    server.on("/send_sms", HTTP_POST, handleSendSMS);
    server.on("/test_sms", HTTP_POST, handleTestSMS);
    server.on("/set_thresholds", HTTP_POST, handleSetThresholds);
    server.on("/reset", HTTP_POST, handleReset);
    server.on("/manual_alert", HTTP_POST, handleManualAlert);
    server.on("/set_recipient", HTTP_POST, handleSetRecipient);
    server.on("/test_buzzer", HTTP_POST, handleTestBuzzer);
    server.begin();

    Wire.begin();
    scanMPU();
    scanGPS();

    updateModemStatus();
}

// --- Main Loop ---
void loop() {
    server.handleClient();

    // GPS Polling
    scanGPS();

    // Sensor polling
    if (millis() - lastMPUCheck > 100) {
        scanMPU();
        lastMPUCheck = millis();
    }

    // --- Ultrasonic: Only proximity beep if close ---
    long us_cm = safeReadUltrasonicCM();
    bool isClose = (us_cm > 0 && us_cm < ultrasonicThreshold);

    // Buzzer beeps when object is close
    lastUltrasonicLow = isClose;

    // --- ALERT LOGIC: Send SMS on high G, flipped, or manual ---
    checkAndSendAlert();

    // --- Button Handling:  (DISABLED as manual trigger) ---
    // Only allow long-press reset, NOT manual trigger for alert!
    static unsigned long buttonPressStart = 0;
    bool button = digitalRead(BUTTON_PIN) == LOW;
    if (button && buttonPressStart == 0) {
        buttonPressStart = millis();
    }
    // (Manual trigger via button is disabled)
    // Long hold (10s) = reset
    if (button && buttonPressStart > 0 && millis() - buttonPressStart >= 10000) {
        alertActive = false;
        lastAlertCause = "";
        lastAlertTime = 0;
        buzzerOn = false;
        digitalWrite(BUZZER_PIN, LOW);
        smsResult = "";
        smsDebugLog = "";
        buttonPressStart = 0;
    }
    // Released after medium hold, do nothing
    if (!button && buttonPressStart > 0 && millis() - buttonPressStart >= 2000 && millis() - buttonPressStart < 10000) {
        buttonPressStart = 0;
    }

    // --- Buzzer Control ---
    if (buzzerOn || buzzerTestActive || lastUltrasonicLow) {
        digitalWrite(BUZZER_PIN, HIGH);
    } else {
        digitalWrite(BUZZER_PIN, LOW);
    }
    // Buzzer test auto-off after 1s
    static unsigned long buzzerTestStarted = 0;
    if (buzzerTestActive) {
        if (buzzerTestStarted == 0) {
            buzzerTestStarted = millis();
        } else if (millis() - buzzerTestStarted >= 1000) {
            buzzerTestActive = false;
            buzzerTestStarted = 0;
        }
    }
}

// --------------- ALERT SMS LOGIC --------------------
void checkAndSendAlert() {
    bool highG = (lastG >= impactGThreshold);
    bool flipped = (lastOrientation != "Normal");
    bool manual = manualAlert;

    if (!alertActive && (highG || flipped || manual)) {
        String cause;
        if (manual)        cause = "Manual";
        else if (highG)    cause = "High G";
        else if (flipped)  cause = "Flipped";
        else               cause = "Unknown";

        // Use GPS if valid, else fallback to default+randomized location
        double smsLat = gpsLat;
        double smsLon = gpsLon;
        if (!gpsIsValid || gpsLat == 0.0 || gpsLon == 0.0) {
            getRandomDefaultLocation(smsLat, smsLon);
        }
        String mapsLink = "https://maps.google.com/?q=" + String(smsLat, 6) + "," + String(smsLon, 6);

        String msg = String("CAR ACCIDENT ALERT!\n")
            + "Cause: " + cause
            + "\nG's: " + String(lastG,2)
            + "\nLocation: " + mapsLink
            + "\nState: " + lastOrientation;
        smsResult = sendSMS(lastRecipient, msg);
        alertActive = true;
        lastAlertCause = cause;
        lastAlertTime = millis();
        buzzerOn = true;
        manualAlert = false;
    }
}

// --------------- MODEM / SENSORS / UTILS --------------------
void modemPowerOn() {
    pinMode(BOARD_POWERON_PIN, OUTPUT); digitalWrite(BOARD_POWERON_PIN, HIGH); delay(100);
    pinMode(MODEM_RESET_PIN, OUTPUT); digitalWrite(MODEM_RESET_PIN, LOW); delay(100);
    digitalWrite(MODEM_RESET_PIN, HIGH); delay(2600); digitalWrite(MODEM_RESET_PIN, LOW);
    pinMode(BOARD_PWRKEY_PIN, OUTPUT); digitalWrite(BOARD_PWRKEY_PIN, LOW); delay(100);
    digitalWrite(BOARD_PWRKEY_PIN, HIGH); delay(100);
    digitalWrite(BOARD_PWRKEY_PIN, LOW); delay(2000);
}
String sendAT(const char* cmd, uint32_t timeout, bool waitForPrompt) {
    while (SerialAT.available()) SerialAT.read();
    SerialAT.println(cmd);
    String resp = "";
    uint32_t start = millis();
    if (waitForPrompt) {
        while (millis() - start < timeout) {
            if (SerialAT.available()) {
                char c = SerialAT.read();
                resp += c;
                if (c == '>') break;
            }
        }
    } else {
        while (millis() - start < timeout) {
            if (SerialAT.available())
                resp += char(SerialAT.read());
            if (resp.indexOf("OK") >= 0 || resp.indexOf("ERROR") >= 0) break;
        }
    }
    return resp;
}
String sendSMS(String recipient, String message) {
    smsDebugLog = ""; // Clear previous debug

    smsDebugLog += "Sending SMS to: " + recipient + "\n";
    smsDebugLog += "SMS Message: " + message + "\n";

    String resp = sendAT("AT+CMGF=1", 1000);
    smsDebugLog += "AT+CMGF=1 response: " + resp + "\n";
    if (resp.indexOf("OK") == -1) {
        smsDebugLog += "Failed to set text mode.\n";
        return "Failed to set text mode: " + resp;
    }

    resp = sendAT(("AT+CMGS=\"" + recipient + "\"").c_str(), 2000, true);
    smsDebugLog += "AT+CMGS response: " + resp + "\n";
    if (resp.indexOf('>') == -1) {
        smsDebugLog += "No prompt for SMS text.\n";
        return "No prompt for SMS text: " + resp;
    }

    SerialAT.print(message);
    SerialAT.write(0x1A); // Ctrl+Z
    uint32_t start = millis();
    String result = "";
    while (millis() - start < 12000) {
        if (SerialAT.available()) {
            char c = SerialAT.read();
            result += c;
            if (result.indexOf("OK") >= 0 || result.indexOf("+CMS ERROR") >= 0 || result.indexOf("ERROR") >= 0) break;
        }
    }
    smsDebugLog += "SMS send result: " + result + "\n";

    if (result.indexOf("OK") != -1) return "SMS Sent!";
    if (result.indexOf("+CMS ERROR") != -1) return "SMS Failed: " + result;
    if (result.indexOf("ERROR") != -1) return "SMS Failed: " + result;
    return "Unknown SMS result: " + result;
}

// --- MPU6050 (Accelerometer) ---
void scanMPU() {
    if (mpu.begin(0x68)) {
        if (!mpuPresent) {
            mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
            mpu.setGyroRange(MPU6050_RANGE_500_DEG);
            mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
            mpuPresent = true;
        }
    } else {
        mpuPresent = false;
    }
    if (mpuPresent) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        float ax = a.acceleration.x / 9.81;
        float ay = a.acceleration.y / 9.81;
        float az = a.acceleration.z / 9.81;
        gBuffer[gIndex++] = sqrt(ax*ax + ay*ay + az*az);
        if (gIndex >= G_AVG_WINDOW) gIndex = 0;
        float avgG = 0;
        for (int i = 0; i < G_AVG_WINDOW; ++i) avgG += gBuffer[i];
        avgG /= G_AVG_WINDOW;
        lastG = avgG;
        float pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / PI;
        float roll  = atan2(ay, az) * 180.0 / PI;
        if (abs(pitch) > flipAngleThreshold || abs(roll) > flipAngleThreshold) {
            if (abs(pitch) > abs(roll)) lastOrientation = "Flipped";
            else lastOrientation = "On Side";
        } else {
            lastOrientation = "Normal";
        }
        // Accel-based speed (not used for ultrasonic beeping)
        static float velocity = 0;
        static unsigned long lastSpeedCalc = 0;
        unsigned long now = millis();
        float dt = (lastSpeedCalc > 0) ? (now - lastSpeedCalc) / 1000.0 : 0.02;
        lastSpeedCalc = now;
        float norm = sqrt(a.acceleration.x*a.acceleration.x + a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z);
        float lin_z = a.acceleration.z - 9.81 * (a.acceleration.z / norm);
        velocity += lin_z * dt;
        if (abs(pitch) > 70 || abs(velocity) < 0.05) velocity = 0;
        lastAccelSpeed = abs(velocity) * 3.6;
    } else {
        lastAccelSpeed = 0;
        lastG = 1.0;
        lastOrientation = "Normal";
        for (int i = 0; i < G_AVG_WINDOW; ++i) gBuffer[i] = 1.0;
    }
}

// --- NEO-6M GPS ---
void scanGPS() {
    bool hadData = false;
    while (GPS_Serial.available() > 0) {
        gps.encode(GPS_Serial.read());
        hadData = true;
    }
    if (hadData && gps.location.isValid() && gps.location.age() < 5000) {
        gpsLat = gps.location.lat();
        gpsLon = gps.location.lng();
        gpsIsValid = true;
        gpsSpeedKmh = gps.speed.kmph();
    } else {
        gpsLat = 0.0;
        gpsLon = 0.0;
        gpsIsValid = false;
        gpsSpeedKmh = 0.0;
    }
}

// --- Ultrasonic Sensor ---
long safeReadUltrasonicCM() {
    long cm = -1;
    pinMode(ULTRASONIC_TRIG, OUTPUT);
    pinMode(ULTRASONIC_ECHO, INPUT);
    digitalWrite(ULTRASONIC_TRIG, LOW); delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG, HIGH); delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG, LOW);
    long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 25000);
    if (duration == 0 || duration > 25000) cm = -1;
    else cm = duration / 29 / 2;
    return cm;
}

// --- Modem Status (for settings page) ---
void updateModemStatus() {
    String r = sendAT("AT+CPIN?");
    int i = r.indexOf("+CPIN:");
    modemSIM = (i >= 0) ? r.substring(i, r.indexOf('\n', i)) : "No SIM or unknown";
    r = sendAT("AT+CGREG?");
    i = r.indexOf("+CGREG:");
    modemNetwork = (i >= 0) ? r.substring(i, r.indexOf('\n', i)) : "Unknown";
    r = sendAT("AT+CSQ");
    i = r.indexOf("+CSQ:");
    modemSignal = (i >= 0) ? r.substring(i, r.indexOf('\n', i)) : "Unknown";
    r = sendAT("AT+COPS?");
    i = r.indexOf("+COPS:");
    if (i >= 0) {
        int q1 = r.indexOf('"', i);
        int q2 = r.indexOf('"', q1+1);
        modemCarrier = (q1 >= 0 && q2 > q1) ? r.substring(q1 + 1, q2) : r.substring(i, r.indexOf('\n', i));
    } else {
        modemCarrier = "Unknown";
    }
    r = sendAT("AT+SIMCOMATI");
    modemVersion = r.length() > 0 ? r : "Unknown";
}

// --- Web UI: Styling, Navigation, and HTML helpers ---
String getCSS() {
    return
        "body { font-family: sans-serif; background:#f2f2f2; margin:0; padding:0; }"
        "h2 { background: #1a73e8; color:white; margin:0; padding:12px 8px; border-radius:0 0 8px 8px; }"
        ".nav { background:#222; padding:8px 4px; border-radius:0 0 8px 8px; margin-bottom:16px; }"
        ".nav a { color:#fff; text-decoration:none; margin-right:20px; font-weight:bold; }"
        ".nav a.sel { color:#ffd600; }"
        ".card { background:white; border-radius:8px; box-shadow:0 2px 8px #0001; padding:16px; margin:12px; }"
        "form { margin-bottom:12px; display:inline-block; }"
        "input[type=number],input[type=text] { width:120px; }"
        "button,input[type=submit] { background:#1a73e8; color:white; border:none; border-radius:4px; padding:8px 16px; cursor:pointer; font-size:1em; }"
        "button:active,input[type=submit]:active { background:#0b47a1; }"
        ".warn { color:#d32f2f; }"
        ".ok { color:#388e3c; }"
        "small { color:#666; }"
        "table.state { border-collapse:collapse; margin:10px 0; font-size:1em; width:100%; background:#fafafa; }"
        "table.state td,table.state th{ border:1px solid #bbb; padding:6px 8px; text-align:left; }"
        "table.state th{ background:#eee; }";
}
String navBar(const String& sel) {
    String nav = "<div class='nav'>";
    nav += "<a href='/'" + String(sel=="home" ? " class='sel'" : "") + ">Home</a>";
    nav += "<a href='/sensor'" + String(sel=="sensor" ? " class='sel'" : "") + ">Sensor Data</a>";
    nav += "<a href='/map'" + String(sel=="map" ? " class='sel'" : "") + ">GPS Map</a>";
    nav += "<a href='/manual'" + String(sel=="manual" ? " class='sel'" : "") + ">Manual/Test</a>";
    nav += "<a href='/settings'" + String(sel=="settings" ? " class='sel'" : "") + ">Settings</a>";
    nav += "</div>";
    return nav;
}
String htmlHeader(const String& title, const String& nav) {
    String s;
    s += "<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width'>";
    s += "<title>" + title + "</title>";
    s += "<style>" + getCSS() + "</style>";
    s += "</head><body>";
    if (nav.length()) s += nav;
    s += "<h2>" + title + "</h2>";
    return s;
}
String formatTime(unsigned long ms) {
    unsigned long sec = ms / 1000;
    unsigned long min = sec / 60;
    unsigned long hr = min / 60;
    char buf[32];
    sprintf(buf, "%02lu:%02lu:%02lu", hr%24, min%60, sec%60);
    return String(buf);
}
String getGoogleMapsWidget(double lat, double lon) {
    if (!gpsIsValid || lat == 0 || lon == 0) return "<i>No GPS fix for map.</i><br>";
    String s = "<iframe width=\"340\" height=\"260\" frameborder=\"0\" style=\"border:0\"";
    s += " src=\"https://maps.google.com/maps?q=";
    s += String(lat, 6) + "," + String(lon, 6) + "&z=15&output=embed\" allowfullscreen></iframe><br>";
    return s;
}

// --- Web page handlers ---
void handleHome() {
    String html = htmlHeader("Car Accident Alert System", navBar("home"));
    html += "<div class='card'>";
    html += "<b>WiFi SSID:</b> <span class='ok'>" + String(AP_SSID) + "</span><br>";
    html += "<b>Password:</b> <span class='ok'>" + String(AP_PASS) + "</span><br>";
    html += "<b>Open:</b> <span class='ok'>http://192.168.4.1</span> on your phone or laptop.<br>";
    html += "<br>This system detects car accidents using motion, orientation, distance sensors and GPS, then sends SMS alerts.<br>";
    html += "<br><b>Quick Links:</b><br>";
    html += "<a href='/sensor'>Sensor Dashboard</a><br>";
    html += "<a href='/map'>GPS Map</a><br>";
    html += "<a href='/manual'>Manual/Test</a><br>";
    html += "<a href='/settings'>Settings/Status</a><br>";
    html += "</div></body></html>";
    server.send(200, "text/html", html);
}
void handleSensor() {
    String html = htmlHeader("Sensor Dashboard", navBar("sensor"));
    html += "<div class='card'>";
    html += "<table class='state'>";
    html += "<tr><th>Sensor</th><th>State / Value</th></tr>";
    html += String("<tr><td>MPU6050</td><td>") + (mpuPresent ? "<span class='ok'>Connected</span>" : "<span class='warn'>Not Found</span>") + "</td></tr>";
    html += String("<tr><td>Last G's</td><td>") + String(lastG,2) + " G</td></tr>";
    html += String("<tr><td>Orientation</td><td>") + lastOrientation + "</td></tr>";
    html += String("<tr><td>GPS Latitude</td><td>") + (gpsIsValid ? String(gpsLat,6) : "<span class='warn'>No Fix</span>") + "</td></tr>";
    html += String("<tr><td>GPS Longitude</td><td>") + (gpsIsValid ? String(gpsLon,6) : "<span class='warn'>No Fix</span>") + "</td></tr>";
    html += String("<tr><td>GPS Speed</td><td>") + String(gpsSpeedKmh,1) + " km/h</td></tr>";
    html += String("<tr><td>Ultrasonic Distance</td><td>") + String(safeReadUltrasonicCM()) + " cm "
            + (lastUltrasonicLow ? "<span class='warn'>(Object Close!)</span>" : "<span class='ok'>(Clear)</span>") + "</td></tr>";
    html += String("<tr><td>Buzzer</td><td>") + (buzzerOn || buzzerTestActive || lastUltrasonicLow ? "<span class='warn'>ON</span>" : "<span class='ok'>OFF</span>") + "</td></tr>";
    html += String("<tr><td>Alert Active</td><td>") + (alertActive ? "<span class='warn'>YES</span>" : "<span class='ok'>NO</span>") + "</td></tr>";
    html += String("<tr><td>Last Alert Cause</td><td>") + (lastAlertCause.length() ? lastAlertCause : "-") + "</td></tr>";
    html += String("<tr><td>Last Alert Time</td><td>") + (lastAlertTime ? formatTime(lastAlertTime) : "-") + "</td></tr>";
    html += "</table>";
    html += "<small>Page auto-updates every 3 seconds.</small>";
    html += "</div><script>setTimeout(()=>location.reload(),3000);</script></body></html>";
    server.send(200, "text/html", html);
}
void handleMap() {
    String html = htmlHeader("GPS Map Location", navBar("map"));
    html += "<div class='card'>";
    html += getGoogleMapsWidget(gpsLat, gpsLon);
    html += "<br>Latitude: " + String(gpsLat,6) + "<br>Longitude: " + String(gpsLon,6);
    html += "<br>GPS Speed: " + String(gpsSpeedKmh,1) + " km/h";
    html += "</div></body></html>";
    server.send(200, "text/html", html);
}
void handleManual() {
    String html = htmlHeader("Manual/Test Triggers", navBar("manual"));
    html += "<div class='card'>";
    html += "<form action=\"/test_buzzer\" method=\"POST\"><button>Test Buzzer</button></form> ";
    html += "<form action=\"/test_sms\" method=\"POST\"><button>Test SMS</button></form> ";
    html += "<form action=\"/manual_alert\" method=\"POST\"><button>Trigger Manual Alert</button></form> ";
    html += "<form action=\"/reset\" method=\"POST\"><button>Reset Alert State</button></form>";
    html += "<hr>";
    if (smsResult.length()) html += "<b>SMS Result:</b> " + smsResult + "<br>";
    html += "<hr>";
    html += "<b>SMS Debug Log:</b><br>";
    html += "<textarea readonly style='width:100%;height:180px;font-size:0.93em;background:#f8f8f8;border-radius:6px;border:1px solid #ccc;'>" + smsDebugLog + "</textarea>";
    html += "</div></body></html>";
    server.send(200, "text/html", html);
}
void handleSettings() {
    updateModemStatus();
    String html = htmlHeader("Settings & Modem Status", navBar("settings"));
    html += "<div class='card'>";
    // Thresholds and recipient
    html += "<form action=\"/set_thresholds\" method=\"POST\">";
    html += "Impact G Threshold: <input name=\"g\" type=\"number\" min=\"2\" max=\"16\" step=\"0.1\" value=\"" + String(impactGThreshold) + "\"> G<br>";
    html += "Flip Angle Threshold: <input name=\"fa\" type=\"number\" min=\"20\" max=\"89\" step=\"1\" value=\"" + String(flipAngleThreshold) + "\"> deg<br>";
    html += "Ultrasonic Threshold: <input name=\"us\" type=\"number\" min=\"10\" max=\"200\" step=\"1\" value=\"" + String(ultrasonicThreshold) + "\"> cm<br>";
    html += "<input type=\"submit\" value=\"Update Thresholds\">";
    html += "</form><br>";
    html += "<form action=\"/set_recipient\" method=\"POST\">";
    html += "Recipient Number: <input name=\"recipient\" type=\"text\" pattern=\"[\\d\\+]+\" value=\"" + (lastRecipient.length() ? lastRecipient : defaultRecipient) + "\" required>";
    html += "<input type=\"submit\" value=\"Set Recipient\">";
    html += "</form>";
    html += "<hr>";
    html += "<b>SIM Status:</b> " + modemSIM + "<br>";
    html += "<b>Network Status:</b> " + modemNetwork + "<br>";
    html += "<b>Signal Quality:</b> " + modemSignal + "<br>";
    html += "<b>Carrier:</b> " + modemCarrier + "<br>";
    html += "<b>Version:</b> <pre style='background:#efefef; border-radius:6px;'>" + modemVersion + "</pre>";
    html += "</div></body></html>";
    server.send(200, "text/html", html);
}
void handleStatus() {
    handleSettings(); // Alias
}
void handleSendSMS() {
    String recipient = lastRecipient;
    if (server.hasArg("recipient") && server.arg("recipient").length() > 0) {
        recipient = server.arg("recipient");
        lastRecipient = recipient;
    }

    double smsLat = gpsLat;
    double smsLon = gpsLon;
    if (!gpsIsValid || gpsLat == 0.0 || gpsLon == 0.0) {
        getRandomDefaultLocation(smsLat, smsLon);
    }
    String mapsLink = "https://maps.google.com/?q=" + String(smsLat, 6) + "," + String(smsLon, 6);

    String msg = String("CAR ACCIDENT ALERT!\n")
        + "Cause: Web Manual"
        + "\nG's: " + String(lastG,2)
        + "\nLocation: " + mapsLink
        + "\nState: " + lastOrientation;
    smsResult = sendSMS(recipient, msg);
    alertActive = true;
    lastAlertCause = "Web Manual";
    lastAlertTime = millis();
    buzzerOn = true;
    server.sendHeader("Location", "/manual", true);
    server.send(303, "text/plain", "");
}
void handleTestSMS() {
    String recipient = lastRecipient;
    String msg = "TEST SMS from Car Alert System - Smart.ph";
    smsResult = sendSMS(recipient, msg);
    server.sendHeader("Location", "/manual", true);
    server.send(303, "text/plain", "");
}
void handleSetThresholds() {
    if (server.hasArg("g")) impactGThreshold = server.arg("g").toFloat();
    if (server.hasArg("fa")) flipAngleThreshold = server.arg("fa").toFloat();
    if (server.hasArg("us")) ultrasonicThreshold = server.arg("us").toInt();
    server.sendHeader("Location", "/settings", true);
    server.send(303, "text/plain", "");
}
void handleReset() {
    alertActive = false;
    lastAlertCause = "";
    lastAlertTime = 0;
    buzzerOn = false;
    smsResult = "";
    smsDebugLog = "";
    server.sendHeader("Location", "/manual", true);
    server.send(303, "text/plain", "");
}
void handleManualAlert() {
    if (!alertActive) {
        manualAlert = true;
    }
    server.sendHeader("Location", "/manual", true);
    server.send(303, "text/plain", "");
}
void handleSetRecipient() {
    if (server.hasArg("recipient") && server.arg("recipient").length() > 0) {
        lastRecipient = server.arg("recipient");
    }
    server.sendHeader("Location", "/settings", true);
    server.send(303, "text/plain", "");
}
void handleTestBuzzer() {
    buzzerTestActive = true;
    server.sendHeader("Location", "/manual", true);
    server.send(303, "text/plain", "");
}

/*
 * Libraries required:
 * - Adafruit MPU6050
 * - Adafruit Unified Sensor
 * - TinyGPSPlus
 * - WiFi
 * - WebServer
 */