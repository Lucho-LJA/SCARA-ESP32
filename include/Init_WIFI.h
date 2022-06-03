 // Connect the ESP32 the the wifi AP
    WiFi.mode(WIFI_STA);
    WiFi.config(ip,gateway,subnet);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) 
    {
      Serial.println("Connecting...");
      delay(500);
    }
    ip_board=String(WiFi.localIP());
    Serial.println(ip_board.c_str());