#include "config.h"

#include <thread>
#include <atomic>
#include <mutex>

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <EasyButton.h>

#define WIFI_TIMEOUT_MS 20000 // 20 second WiFi connection timeout
#define WIFI_RECOVER_TIME_MS 30000 // Wait 30 seconds after a failed connection attempt

std::atomic<bool> online;
volatile bool station_running[8];

void sprinkler_toggle_station(int s) {
  if (!online) {
    return;
  }

  int state = station_running[s] ? 0 : 1;
  safe_printf("Toggling station %i to %i\n", s, state);

  int start = millis();

  WiFiClient client;
  HTTPClient http;
  char url[200];
  sprintf(url, "%s/cm?pw=%s&sid=%i&en=%i&t=300", BASE_URL, PWD, s, state);
  http.begin(client, url);
  int code = http.GET();
  const char *body = http.getString().c_str();
  safe_printf("TOGGLE (%i ms) => %i - %s\n", millis() - start, code, body);
  http.end();
}

struct Button {
  int led;
  EasyButton button;
  void (*pressed)();
  int station;
};

Button buttons[] = { 
  {5, EasyButton(2), []{sprinkler_toggle_station(0);}, 0},
  {6, EasyButton(3), []{sprinkler_toggle_station(1);}, 1} 
};

void setup() {
  Serial.begin(115200);

  for (Button &b : buttons) {
    pinMode(b.led, OUTPUT);

    b.button.begin();
    b.button.onPressed(b.pressed);
  }

  TaskHandle_t task1;
  xTaskCreatePinnedToCore(sprinkler_led_thread, "sprinkler_led_thread", 10000, NULL, 1, &task1, 0);
  
  TaskHandle_t task2;
  xTaskCreatePinnedToCore(wifi_connect_thread, "wifi_connect_thread", 10000, NULL, 1, &task2, 0);
}

void blink(int port, int intv) {
  digitalWrite(port, (millis() % intv > intv / 2) ? HIGH : LOW);
}

std::mutex serial_lock;

void safe_printf(const char *fmt, ...) {
  char buffer[256];
  va_list args;
  va_start (args, fmt);
  vsprintf (buffer, fmt, args);
  va_end (args);
  
  std::lock_guard<std::mutex> lg{serial_lock};
  Serial.print(buffer);
}

void safe_print(const char * s) {
  std::lock_guard<std::mutex> lg{serial_lock};
  Serial.print(s);
}

void safe_println(const char *s) {
  std::lock_guard<std::mutex> lg{serial_lock};
  Serial.println(s);
}

void blink_leds() {
  for (Button &b : buttons) {
    blink(b.led, 500);
  }
}

void sprinkler_led_loop() {
  if (!online) {
    return;
  }
  
  int start = millis();

  WiFiClient client;
  HTTPClient http;
  char url[200];
  sprintf(url, "%s/js?pw=%s", BASE_URL, PWD);
  http.begin(client, url);

  safe_println("GET...");
  int code = http.GET();
  const char *body = http.getString().c_str();
  safe_printf("GET (%i ms) => %i - %s\n", millis() - start, code, body);
  if (code == 200) {
    StaticJsonDocument<1000> doc;
    DeserializationError error = deserializeJson(doc, body);

    if (error) {
      safe_print("deserializeJson() failed: ");
      safe_println(error.c_str());
    } else {
      // {"sn":[0,0,0,0,0,0,0,0],"nstations":8}
      JsonArray sn = doc["sn"];
      int i = 0;
      for (JsonVariant s : sn) {
        station_running[i] = s.as<int>() ? true : false;
        i++;
      }
    }
  }

  for (Button &b : buttons) {
    digitalWrite(b.led, station_running[b.station] ? HIGH : LOW);
  }
  
  http.end();
}

void sprinkler_led_thread(void *) {
  while (true) {
    sprinkler_led_loop();
    delay(1000);
  }
}

void wifi_connect_thread(void * parameter){
    while (true) {
        online = WiFi.status() == WL_CONNECTED;
      
        if (online) {
            vTaskDelay(10000 / portTICK_PERIOD_MS);
            continue;
        }

        safe_println("Connecting to WiFi");
        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_SSID, WIFI_PWD);

        unsigned long startAttemptTime = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT_MS) {}

        if (WiFi.status() != WL_CONNECTED){
          safe_println("WiFi connect failed");
          vTaskDelay(WIFI_RECOVER_TIME_MS / portTICK_PERIOD_MS);
          continue;
        }

        safe_println(("Connected as " + WiFi.localIP().toString()).c_str());
        online = true;
    }
}

void loop() {
  while (true) {
    for (Button &b : buttons) {
      b.button.read();
    }

    if (!online) {
       blink_leds();
    }
  }
}
