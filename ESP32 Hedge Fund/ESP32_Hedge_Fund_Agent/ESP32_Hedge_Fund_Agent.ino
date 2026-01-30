/*

Abstract:
*   The hedge-fund's process for a decision on a stock
*   in a High Frequency Trading environment.

*/

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

/*  WiFi credentials */
const char* ssid = "--";
const char* password = "-----";

/*  MQTT broker */
const char* mqttServer = "----"; // IP of the Raspberry Pi running your broker
const int mqtt_port = 19000;

/*  Stock symbol to follow */
const char* stockSymbol = "STOCK"; // set this up to be based on what stock comes in from a subscribe.
                                  //  make it to where i can dynamically subscribe to different stocks from SSH

/*  WiFi and MQTT clients */
WiFiClient espClient;
PubSubClient client(espClient);

/*  Strategy threshold */
float priceThreshold = 90.0;  // Example: take action if price drops below this



/*  1.) data organization layer */

typedef struct {
  const char* symbol;
  double price;
  int volume;
  double volatility;
  uint64_t ts_ns;
} Market_Tick;

typedef struct {
  double last_price;       // historic data
  int last_volume;
  
  double ema_price;        // exponential moving average
  double ema_volatility;
  
  double variance;         // running variance estimate
  double stddev;
  
  uint64_t last_ts_ns;     // authoritative market time
  uint64_t tick_count;
} Stock_Delta;


void update_tick_from_JSON(Market_Tick *m, JsonDocument& doc) {
    m->symbol     = doc["symbol"];
    m->price      = doc["price"].as<double>();
    m->volume     = doc["volume"].as<int>();
    m->volatility = doc["volatility"].as<double>();
    m->ts_ns      = doc["ts"].as<uint64_t>();
}

/*   2.) stock characteristics layer  */
void stock_initialization(Stock_Delta *f){]
  memset(f, 0, sizeof(*f));
}

void stock_update(Stock_Delta *f, const Market_Tick *t) {
    const double alpha = 0.05;

    if (f->tick_count == 0) {
        f->ema_price = t->price;
        f->ema_volatility = t->volatility;
        f->variance = 0.0;
    } else {
        double diff = t->price - f->ema_price;

        f->ema_price = alpha * t->price + (1.0 - alpha) * f->ema_price;
        f->ema_volatility = alpha * t->volatility +
                           (1.0 - alpha) * f->ema_volatility;

        f->variance = alpha * (diff * diff) +
                      (1.0 - alpha) * f->variance;
    }

    f->stddev = sqrt(f->variance);
    f->last_price = t->price;
    f->last_volume = t->volume;
    f->last_ts_ns = t->ts_ns;
    f->tick_count++;
}

/*   3.) Stratgy Layer  */

typedef enum {
  SIGNAL_BUY = 1;
  SIGNAL_SELL = - 1;
  SIGNAL_HOLD = 0;
} TradeSignal;


TradeSignal strategy_decide(const Stock_Delta *f) {
    if (f->tick_count < 10)
        return SIGNAL_HOLD;

    if (f->last_price > f->ema_price &&
        f->ema_volatility < 0.3)
        return SIGNAL_BUY;

    if (f->last_price < f->ema_price)
        return SIGNAL_SELL;

    return SIGNAL_HOLD;
}

/*   4.) Logging of final decision per tick via MQTT */
void on_market_tick(const Market_Tick *t) {
    stock_update(&some_stock, t);

    TradeSignal sig = strategy_decide(&some_stock);

    if (sig != SIGNAL_HOLD)
        publish_signal(sig); // MQTT call to publish on the Pi Broker log.
}







/*  Declaration of variables to be used in the MQTT callback. */

Market_Tick tick;
Stock_Delta some_stock;
stock_initialization(some_stock);




/* Connect to WiFi */
void setup_wifi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi!");
}

/*  Callback when message is received */
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  // Copy payload into a null-terminated string
  char msg[length + 1];
  memcpy(msg, payload, length);
  msg[length] = '\0';
  Serial.println(msg);

  // --- Parse JSON ---
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, msg);
  if (error) {
    Serial.print("JSON parse failed: ");
    Serial.println(error.c_str());
    return;
  }

  /*  Update market tick from parsed data   */
  update_tick_from_JSON(tick);
  /*  Update stock with the latest tick's data  */
  stock_update(some_stock, tick);

}

/* Reconnect to MQTT broker if disconnected */
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32-Trader")) {
      Serial.println("connected");
      // Subscribe to cleaned stock feed
      String topic = String("market/clean/stock/") + stockSymbol;
      client.subscribe(topic.c_str());
      Serial.print("Subscribed to: ");
      Serial.println(topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      delay(2000);
    }
  }
}







































void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqttServer, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
