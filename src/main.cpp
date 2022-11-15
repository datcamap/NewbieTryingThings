#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <AsyncWebSocket.h>
#include <Wire.h>
#include <LiquidCrystal.h>

#define tdsPin 35
#define pHPin 34
#define temPin 4 // GPIO where the DS18B20 is connected to
#define testPin 23
#define waterPin 5  // Cảm biến chất lỏng
#define valvePin 18 // Van chống tràn
#define pumpPin 14  // Máy bơm 12V
#define filtPin 15  // Relay bộ lọc
#define VREF 3.3    // analog reference voltage(Volt) of the ADC
#define SCOUNT 30   // sum of sample point
#define MESSAGE_LENGTH 1000
#define LED 2

/* Wifi connection */
const char *ssid = "ThaoAn/2.4G";
const char *password = "123456789";

const char *JSON_MIME = "application/json";
const char *TEXT_MIME = "text/plain";
// Change here to send data
const int deviceId = 4;
String hostname = "http://aqua-iot.pro/api/v1/sensordatas";
String twinurl = "http://aqua-iot.pro/api/v1/devices/" + String(deviceId);

int tdsBuffer[SCOUNT]; // store the TDS values in the array, read from ADC
int _TDS_Index = 0;
int pHBuffer[SCOUNT]; // store the pH values
int _pH_Index = 0;
int temperBuffer[SCOUNT];
int _temper_Index = 0;
int analogBufferTemp[SCOUNT];
int copyIndex = 0;
float pH_samples[10];
float settling[10];
static int sidx = 0;
static int idx = 0;
float averageVoltage = 0;
float tdsValue = 0;
int averagepH = 0;
float dopH = 0;
float temperature = 0; // current temperature for compensation

int _reading_Cycle = 60U;    // Thoi gian doc tin hieu cam bien
int _print_Interval = 2000U; // Thoi gian in ra serial monitor
int timerDelay = 5000U;      // Thoi gian tinh gia tri trung binh tu tap mau va gui len server
int pollingCycle = 2000U;    // Thoi gian request lenh dieu khien tu server
int _switch_Sensors = HIGH;  // will be switched HIGH/LOW to control whether TDS or pH is being measured
static unsigned long analogSampleTimepoint = millis();
static unsigned long printTimepoint = millis();
static unsigned long lastTime = millis();
static unsigned long lastPoll = millis();

OneWire _oneWire(temPin);             // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature sensors(&_oneWire); // Pass our oneWire reference to Dallas Temperature sensor
HTTPClient http;
DynamicJsonDocument doc(MESSAGE_LENGTH);

hw_timer_t *timer = NULL; // khơi tạo timer
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// median filtering algorithm, lọc trung vị
int getMedianNum(int bArray[], int iFilterLen);

void IRAM_ATTR onTimer()
{
  portENTER_CRITICAL_ISR(&timerMux); // vào chế độ tránh xung đột
  _switch_Sensors = !_switch_Sensors;
  digitalWrite(testPin, _switch_Sensors); // đảo giá trị Led
  portEXIT_CRITICAL_ISR(&timerMux); // thoát
}

void setup()
{
  // Start the serial monitor
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.printf("Wifi failed to connect!");
    return;
  }
  Serial.printf("IP Address: ");
  Serial.println(WiFi.localIP());

  // Start the DS18B20 sensor
  sensors.begin();

  // OnLED
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  // Set pin mode
  pinMode(tdsPin, INPUT);
  pinMode(pHPin, INPUT);
  pinMode(waterPin, INPUT);
  pinMode(testPin, OUTPUT);
  pinMode(filtPin, OUTPUT);
  pinMode(valvePin, INPUT);
  pinMode(pumpPin, OUTPUT);

  // khởi tạo timer với chu kì 1us vì thạch anh của ESP chạy 8MHz
  timer = timerBegin(0, 80, true);
  // khởi tạo hàm xử lý ngắt ngắt cho Timer
  timerAttachInterrupt(timer, &onTimer, true);
  // khởi tạo thời gian ngắt cho timer
  timerAlarmWrite(timer, 100000, true);
  // bắt đầu chạy timer
  timerAlarmEnable(timer);
}

bool pumpEN = false;
bool MODE_AUTOMATE = true;
void loop()
{
  if (MODE_AUTOMATE)
  {
    int overflow = digitalRead(valvePin);
    bool lowerflow = digitalRead(waterPin);
    if (overflow)
    {
      pumpEN = false;
      digitalWrite(pumpPin, HIGH); // ngắt bơm (đối với relay kích hoạt low, cài NO)
    }
    if (lowerflow)
    {
      pumpEN = true;
      digitalWrite(pumpPin, LOW); // bật bơm (đối với relay kích hoạt low, cài NO)
    }
    digitalWrite(filtPin, lowerflow);
    Serial.println(lowerflow);
  }

  // read the sensors
  if (millis() - analogSampleTimepoint > _reading_Cycle)
  {
    analogSampleTimepoint = millis();

    // Read temperature
    sensors.requestTemperatures();
    temperature = sensors.getTempCByIndex(0);

    // Read TDS or pH depends on which sensor has been turned on
    if (_switch_Sensors)
    {
      tdsBuffer[_TDS_Index] = analogRead(tdsPin);
      _TDS_Index++;
      if (_TDS_Index == SCOUNT)
      {
        _TDS_Index = 0;
      }
    }
    else
    {
      pHBuffer[_pH_Index] = analogRead(pHPin);
      _pH_Index++;
      if (_pH_Index == SCOUNT)
      {
        _pH_Index = 0;
      }
    }
  }

  if (millis() - printTimepoint > _print_Interval)
  {
    printTimepoint = millis();

    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
    {
      analogBufferTemp[copyIndex] = tdsBuffer[copyIndex];
    }
    // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 4096.0;
    // temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
    // temperature compensation
    float compensationVoltage = averageVoltage / compensationCoefficient;
    // convert voltage value to tds value
    tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;

    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
    {
      analogBufferTemp[copyIndex] = pHBuffer[copyIndex];
    }
    averagepH = getMedianNum(analogBufferTemp, SCOUNT);
    // averagepH = analogBufferTemp[0];
    dopH = averagepH;
    pH_samples[idx] = dopH;
    settling[idx] = pH_samples[idx];
    idx++;

    // print to serial
    if (idx > 10)
    {
      idx = 0;
      float sum = 0;
      for (int i = 0; i < 10; i++)
      {
        sum += pH_samples[i];
      }
      Serial.println();
      Serial.print("---------- Do pH trung binh = ");
      Serial.print(sum / 10.0);
      Serial.println(" ----------");
      settling[sidx] = (int)(sum / 10.0);
      sidx++;
      if (sidx >= 10)
      {
        sidx = 0;
      }
      for (int i = 0; i < 10; i++)
      {
        Serial.print(settling[i]);
        Serial.print(", ");
      }
      Serial.println();
    }

    Serial.print(temperature);
    Serial.print("ºC\t");
    Serial.print("pH_ADC: ");
    Serial.print(dopH);
    Serial.print("\t");
    Serial.print("voltage:");
    Serial.print(averageVoltage, 2);
    Serial.print("V\t");
    Serial.print("TDS Value:");
    Serial.print(tdsValue, 0);
    Serial.println("ppm\t");
  }

  /*** Phần 1: Gửi dữ liệu lên server ***/
  /*Cấu trúc tin nhắn gửi dữ liệu:
    {
      "deviceId": int,
      "data": string (dạng json)
    }
  */
  if (WiFi.status() == WL_CONNECTED)
  {
    if ((millis() - lastTime) > timerDelay)
    {
      lastTime = millis();
      Serial.println("\n[+] Starting post method to send data:");
      // Tạo document để thực hiện serialize và deserialize json, xem thêm tại: https://arduinojson.org/
      String data_str = "";
      // Bước 1: tạo chuỗi JSON cho trường data
      doc["TDS"] = tdsValue;
      doc["temperature"] = temperature;
      doc["pH"] = dopH;
      serializeJson(doc, data_str);
      // Bước 2: tạo tin nhắn dưới dạng JSON
      doc.clear();
      doc["deviceId"] = deviceId;
      doc["data"] = data_str;
      String sensorData = "";
      serializeJson(doc, sensorData);
      // Bước 3: Setup http client, bao gồm tên miền gửi lên và header
      http.begin(hostname.c_str());
      http.addHeader("Content-Type", JSON_MIME);
      http.addHeader("Connection", "keep-alive");
      http.addHeader("Accept", JSON_MIME);
      // Bước 4: Gửi dữ liệu lên, method là POST rồi in ra
      int respCode = http.POST(sensorData);
      Serial.print("[+] Sending: ");
      Serial.println(sensorData);
      if (respCode > 0)
      {
        Serial.print("[+] Http Response code: ");
        Serial.println(respCode);
        String payload = http.getString();
        Serial.print("[+] Payload: ");
        Serial.println(payload);
      }
      else
      {
        Serial.printf("[+] Error code: ");
        Serial.println(respCode);
      }
      http.end();
    }

    /*** Phần 2: Thực hiện nhận chuỗi điều khiển ***/
    /* Cấu trúc của nó có dạng như sau:
      {
        "data": {
          "id": 3,
          "name": "espcong",
          "enabled": "Enabled",
          "connectionState": "Disconnected",
          "deviceToCloudMessages": 131,
          "cloudToDeviceMessages": 0,
          "desired": "{\"hello\": \"whatlad\", \"desired_ph\": \"7\", \"desired_temp\": \"20\", \"desired_humid\": \"85\"}",
          "reported": "{}"
        }
      }
    */
    if (millis() - lastPoll > pollingCycle)
    {
      lastPoll = millis();
      Serial.println("\n[+] Starting get method to receive desired twin:");
      // Bước 1: Setup http client với hostname
      http.begin(twinurl.c_str());
      http.addHeader("Accept", JSON_MIME);
      // Bước 2: thực hiện phương thức GET để lấy dữ liệu
      int httpResponseCode = http.GET();
      // Bước 3: Lấy dữ liệu từ chuỗi JSON nhận về
      if (httpResponseCode > 0)
      {
        Serial.print("[+] HTTP Response code: ");
        Serial.println(httpResponseCode);
        String payload = http.getString();
        Serial.println("[+] Payload: " + payload);
        // Phân tích dữ liệu trả về
        deserializeJson(doc, payload);
        // Lấy dữ liệu desired
        String desired = doc["data"]["desired"];
        Serial.println("[+] Desired string: " + desired);
        doc.clear();
        // Biến dữ liệu desired thành object!
        deserializeJson(doc, desired);
        // Thay doi o day: Dieu khien dua vao du lieu nhan duoc
        boolean led_state = doc["led"];
        Serial.println("\n[+] LED: " + led_state);
        digitalWrite(LED, led_state);
      }
      else
      {
        Serial.print("[+] Error code: ");
        Serial.println(httpResponseCode);
      }
      http.end();
    }
  }
  else
  {
    Serial.println("[+] Wifi disconnected!");
  }
}

int getMedianNum(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1])
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
  {
    bTemp = bTab[(iFilterLen - 1) / 2];
  }
  else
  {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}