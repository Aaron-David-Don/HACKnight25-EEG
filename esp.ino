#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ESPmDNS.h>

// Wi-Fi credentials
const char* ssid     = "Shout Aaron for PASSWORD!!!";  // Replace with your Wi-Fi SSID
const char* password = "aaronaaron";                     // Replace with your Wi-Fi password

// Create WebSocket server on port 81
WebSocketsServer webSocket = WebSocketsServer(81);

// Define motor B direction pins
#define MOTOR_B_IN1 14
#define MOTOR_B_IN2 16

// Define motor B enable pin (for L298)
#define MOTOR_B_EN    15

// Fixed PWM settings for motor B
#define MOTOR_B_CHANNEL 1
#define PWM_FREQ        1000   // 1 kHz
#define PWM_RES_BITS    8      // 8-bit => duty range 0-255

// >>> SET YOUR FIXED SPEED HERE <<<
#define FIXED_SPEED     128    // e.g. half speed (0=off, 255=full)

// Motor B state variable
bool motorBActive = false;
bool isConnected  = false;  // Flag for tracking WebSocket connection status

// WebSocket event handler
void webSocketEvent(uint8_t clientNum, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT) {
    String msg = String((char*) payload);
    Serial.printf("Message from client: %s\n", msg.c_str());

    // If message == "1", motor B ON; otherwise, motor B OFF
    if (msg == "1") {
      motorBActive = true;
      Serial.println("Motor B: ON (fixed speed)");
    } else {
      motorBActive = false;
      Serial.println("Motor B: OFF");
    }

    // Send a response back to the client
    webSocket.sendTXT(clientNum, "ESP32: Message received");
  }
}

void setup() {
  Serial.begin(115200);

  // Set motor B direction pins as outputs
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);

  // Setup LEDC for PWM on motor B enable pin
  ledcSetup(MOTOR_B_CHANNEL, PWM_FREQ, PWM_RES_BITS);
  ledcAttachPin(MOTOR_B_EN, MOTOR_B_CHANNEL);

  // Initialize motor B off
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
  ledcWrite(MOTOR_B_CHANNEL, 0);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Initialize mDNS
  if (MDNS.begin("esp32")) {
    Serial.println("mDNS responder started");
  } else {
    Serial.println("Error setting up mDNS responder!");
  }

  // Start the WebSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("WebSocket server started on port 81");

  // Mark as connected
  isConnected = true;
}

void loop() {
  // Process WebSocket communication
  webSocket.loop();

  // Only control motor B if device is connected
  if (isConnected) {
    if (motorBActive) {
      // Forward direction
      digitalWrite(MOTOR_B_IN1, HIGH);
      digitalWrite(MOTOR_B_IN2, LOW);
      // Use fixed speed
      ledcWrite(MOTOR_B_CHANNEL, FIXED_SPEED);
    } else {
      // Stop motor B
      digitalWrite(MOTOR_B_IN1, LOW);
      digitalWrite(MOTOR_B_IN2, LOW);
      ledcWrite(MOTOR_B_CHANNEL, 0);
    }
  } else {
    // If not connected, ensure motor B remains off
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, LOW);
    ledcWrite(MOTOR_B_CHANNEL, 0);
  }
}
