/**
 * @file mqtt_servo_subscriber_mkr1000.ino
 * @brief Servo angle control via MQTT (subscriber only) for Arduino MKR1000.
 * @details
 *  - Uses WiFi101 + ArduinoMqttClient.
 *  - Subscribes to ONE public topic and expects ASCII integer degrees.
 *  - Input is clamped to [0..180] and applied via Servo.write().
 *  - No MQTT publish, no LWT, no retained messages.
 */

#include <Arduino.h>
#include <Servo.h>
#include <WiFi101.h>            /* MKR1000 Wi-Fi stack */
#include <ArduinoMqttClient.h>

/* ===== Wi-Fi credentials (from arduino_secrets.h) ===== */
char ssid[] = "ElGuaro";
char pass[] = "lamedusa";

/* ===== MQTT broker (public) & topic ===== */
static const char* const MQTT_HOST   = "test.mosquitto.org";
static const int         MQTT_PORT   = 1883;
static const char* const MQTT_TOPIC  = "alessio/unique/servo/angle"; /* <-- make this truly unique */
static const char* const MQTT_CLIENT = "servo-subscriber-mkr1000-01"; /* client id */

/*===== Serial configuration =====*/
#define BAUD_RATE         (115200U)
/* ===== Servo configuration ===== */
/* MKR1000 supports Servo on several digital pins; D9 is fine in most cases. */
#define SERVO_PIN         (9)
#define SERVO_MIN_US      (500U)
#define SERVO_MAX_US      (2500U)
#define MIN_ANGLE_DEG     (0L)
#define MAX_ANGLE_DEG     (180L)
#define ANGLE_STARTUP_DEG (90L)

/* ===== Globals ===== */
static Servo g_servo;
static WiFiClient g_wifiClient;
static MqttClient g_mqtt(g_wifiClient);

/* ===== Forward declarations ===== */
static void wifi_connect_(void);
static void mqtt_connect_and_sub_(void);
static void onMqttMessage_(int messageSize);
static long clamp_angle_(long v);

/* ===== Helpers ===== */
static long clamp_angle_(long v)
{
  long out = v;
  if (out < MIN_ANGLE_DEG) {
    out = MIN_ANGLE_DEG;
  } else if (out > MAX_ANGLE_DEG) {
    out = MAX_ANGLE_DEG;
  } else {
    /* no-op */
  }
  return out;
}

/* ===== Connectivity ===== */
static void wifi_connect_(void)
{
  Serial.print(F("Connecting to SSID: "));
  Serial.println(ssid);
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    Serial.print(F("."));
    delay(2000);
  }
  Serial.println(F("\nWiFi connected."));
}


/**
 * @brief Parse an ASCII integer from a NUL-terminated buffer (single return).
 * @details
 *  - Accepts base-10 integers; allows trailing spaces only.
 *  - Rejects any other trailing characters.
 *  - On success writes the parsed value into *out.
 *  - On failure, *out is left unmodified.
 *
 * @param[in]  s    NUL-terminated input buffer (must not be NULL).
 * @param[out] out  Parsed value destination (must not be NULL).
 * @return true on valid integer parse; false otherwise.
 */
static bool parse_ascii_int_(const char* s, long* out)
{
  bool status = false;  /* single exit flag */

  if ((s != NULL) && (out != NULL)) {
    char* endptr = NULL;
    long val = strtol(s, &endptr, 10);

    /* Allow only trailing spaces after the numeric part */
    while ((endptr != NULL) && (*endptr == ' ')) {
      ++endptr;
    }

    /* Accept only if:
       - strtol parsed something (endptr not NULL),
       - no conversion error (errno == 0),
       - and we ended exactly at string end. */
    if ((endptr != NULL) && (*endptr == '\0')) {
      *out = val;   /* write output only on success */
      status = true;
    } else {
      /* not ok; *out remains unchanged */
    }
  } else {
    /* invalid pointers; ok remains false */
  }

  return status;
}

static void mqtt_connect_and_sub_(void)
{
  Serial.print(F("Connecting MQTT: "));
  Serial.print(MQTT_HOST);
  Serial.print(F(":"));
  Serial.println(MQTT_PORT);

  while (!g_mqtt.connect(MQTT_HOST, MQTT_PORT)) {
    Serial.print(F("MQTT connect failed, err="));
    Serial.println(g_mqtt.connectError());
    delay(1500);
  }

  g_mqtt.onMessage(onMqttMessage_);
  if (!g_mqtt.subscribe(MQTT_TOPIC)) {
    Serial.println(F("Subscribe failed. Check topic and broker."));
  } else {
    Serial.print(F("Subscribed to: "));
    Serial.println(MQTT_TOPIC);
  }
}

/* ===== MQTT callback (subscriber-only) ===== */
static void onMqttMessage_(int messageSize)
{
  /* ArduinoMqttClient::messageTopic() returns an Arduino String */
  String topic = g_mqtt.messageTopic();
  if (!topic.equals(MQTT_TOPIC)) {
    /* Unexpected topic: drain payload and ignore (no publish) */
    while (g_mqtt.available() > 0) { (void)g_mqtt.read(); }
    return;
  }

  if (messageSize <= 0) {
    return;
  }

  /* Copy payload into a fixed-size buffer and NUL-terminate */
  static const int buffSize = 16;
  char buf[buffSize];
  int i = 0;
  int maxCopy = 0;
  
  if (messageSize < (buffSize - 1)) {
    maxCopy = messageSize;
  } else {
    maxCopy = buffSize - 1;
  }

  while ((g_mqtt.available() > 0) && (i < maxCopy)) {
    buf[i++] = static_cast<char>(g_mqtt.read());
  }
  buf[i] = '\0';

  /* Discard any extra bytes beyond our buffer */
  while (g_mqtt.available() > 0) { (void)g_mqtt.read(); }

  long value = 0U;
  
  if(parse_ascii_int_(buf, &value) != true)
  {
    return;
  }

  const long clamped = clamp_angle_(value);
  g_servo.write(static_cast<int>(clamped));

  /* Optional local log (kept on Serial only, no MQTT publish) */
  Serial.print(F("Angle set: req=")); Serial.print(value);
  Serial.print(F(" -> ")); Serial.println(clamped);
}

/* ===== Arduino lifecycle ===== */
void setup(void)
{
  Serial.begin(BAUD_RATE);
  while (!Serial) { /* for native USB */ }

  g_servo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  g_servo.write(static_cast<int>(ANGLE_STARTUP_DEG));

  wifi_connect_();
  mqtt_connect_and_sub_();

  Serial.println(F("Ready. Publish ASCII degrees to the subscribed topic."));
}

void loop(void)
{
  /* Keep MQTT alive and receive messages */
  g_mqtt.poll();

  /* Basic reconnection logic */
  if (WiFi.status() != WL_CONNECTED) {
    wifi_connect_();
  }
  if (!g_mqtt.connected()) {
    mqtt_connect_and_sub_();
  }
}