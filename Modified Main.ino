// ... [all your code before checkAndSendAlert() remains unchanged]

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

        // --- BEGIN: Randomized fallback coordinates within 5m radius if GPS unavailable ---
        double smsLat = gpsLat;
        double smsLon = gpsLon;
        if (!gpsIsValid || gpsLat == 0.0 || gpsLon == 0.0) {
            // Bacolod City Hall: 10.657139, 122.948028
            // 5m in deg: 5/111320.0 ≈ 0.00004489
            // For longitude correction, cos(latitude) in radians
            const double DEFAULT_LAT = 10.657139;
            const double DEFAULT_LON = 122.948028;
            const double LAT_RADIUS_DEG = 5.0 / 111320.0;
            const double LON_RADIUS_DEG = 5.0 / (111320.0 * cos(DEFAULT_LAT * M_PI / 180.0));
            double theta = 2 * M_PI * (random(0, 10000) / 10000.0);
            double r = sqrt(random(0, 10000) / 10000.0);
            double dLat = r * LAT_RADIUS_DEG * cos(theta);
            double dLon = r * LON_RADIUS_DEG * sin(theta);
            smsLat = DEFAULT_LAT + dLat;
            smsLon = DEFAULT_LON + dLon;
        }
        String mapsLink = "https://maps.google.com/?q=" + String(smsLat,6) + "," + String(smsLon,6);
        // --- END: Fallback random coordinates logic ---

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

// ... [all your code after checkAndSendAlert() remains unchanged]