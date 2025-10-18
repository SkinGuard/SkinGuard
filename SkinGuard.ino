#include "esp32-hal-adc.h"
#include "esp32-hal-ledc.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "BluetoothSerial.h"
#include <Wire.h>
#include <Adafruit_AHTX0.h>

// --- BLE 标准串口服务 (Nordic UART Service) UUID ---
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
// TX 特征：ESP32 通过这个特征发送数据给手机 (Notify)
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
// RX 特征：ESP32 通过这个特征接收手机发来的数据 (Write)
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

#define CONTACT_THRESHOLD 5.0      // 接触阈值
#define FILTER_WINDOW_SIZE 20      // 滑动窗口大小
// 全局变量
float filterBuffer[FILTER_WINDOW_SIZE];  // 滤波缓冲区
int bufferIndex = 0;                      // 当前缓冲区索引
int validSampleCount = 0;                 // 有效样本计数
// LED灯的闪灯程序
int counter = 0;              // 计数器
bool ledState = false;        // LED状态（false=LOW, true=HIGH）
const int COUNT_THRESHOLD = 50;  // 触发阈值

// 结构体用于返回结果
struct SkinSignalResult {
  bool SkinContact;      // 是否有有效接触
  float SkinHuResult;    // 输出的湿度结果
};


BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false; // 新增一个全局变量来跟踪连接状态
// 服务器连接/断开回调
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Connect!");
    }
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Disconnect restart wait");
      // 客户端断开后，需要手动重启广播
      BLEDevice::startAdvertising(); 
    }
};
// 接收 (RX) 特征的回调
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      // 【修正】正确地获取和打印接收到的数据
      String rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.print("Receive Data: ");
        for (int i = 0; i < rxValue.length(); i++) {
          Serial.print(rxValue[i]);
        }
        Serial.println();
      }
    }
};
#define PIN_ADC0 (0)   //定义引脚宏 
#define KEY1 (1) //KEY1
#define KEY2 (2) //KEY2

int KEY1_STATE = 0;
int KEY2_STATE = 0;

Adafruit_AHTX0 aht;

const int ledPin = 6;  // LED 连接的 GPIO 引脚
const int frequency = 5000;  // PWM 频率 5kHz
const int resolution = 8;    // PWM 分辨率 8 位（0-255）
const int ledChannel = 0;    // PWM 通道号

// LED 延迟闪灯程序
void LEDDisplay() {
  // 你的主程序逻辑在这里
  // ...
  
  // 计数器递增
  counter++;
  
  // 当计数达到50次时，翻转LED状态
  if (counter >= COUNT_THRESHOLD) {
    // 重置计数器
    counter = 0;
    
    // 翻转LED状态
    ledState = !ledState;
    
    // 根据状态设置LED
    if (ledState) {
      digitalWrite(LED_BUILTIN, LOW);   // 内置LED亮
      digitalWrite(12, HIGH);            // 引脚12 LED亮
    } else {
      digitalWrite(LED_BUILTIN, HIGH);  // 内置LED灭
      digitalWrite(12, LOW);             // 引脚12 LED灭
    }
  }
}

// 皮肤接触信号处理函数
SkinSignalResult processSkinSignal(float sampleValue) {
  SkinSignalResult result;
  
  // 判断是否有有效接触
  if (sampleValue < CONTACT_THRESHOLD) {
    // 无有效接触
    result.SkinContact = false;
    result.SkinHuResult = sampleValue;
    
    // 重置滤波器状态
    validSampleCount = 0;
    bufferIndex = 0;
    
  } else {
    // 有有效接触
    result.SkinContact = true;
    
    // 将新采样值加入缓冲区
    filterBuffer[bufferIndex] = sampleValue;
    bufferIndex = (bufferIndex + 1) % FILTER_WINDOW_SIZE;  // 循环索引
    
    // 更新有效样本计数（最多20个）
    if (validSampleCount < FILTER_WINDOW_SIZE) {
      validSampleCount++;
    }
    
    // 计算平均值
    float sum = 0;
    for (int i = 0; i < validSampleCount; i++) {
      sum += filterBuffer[i];
    }
    result.SkinHuResult = sum / validSampleCount;
  }
  
  return result;
}


// the setup function runs once when you press reset or power the board
void setup() {
    //BlueTooth
    
    // initialize digital pin LED_BUILTIN as an output.
    ledcAttach(ledPin, frequency, resolution);
    ledcWrite(ledPin, 128);
    pinMode(PIN_ADC0,INPUT);
    analogReadResolution(12);
    analogSetAttenuation(ADC_6db);
    
    pinMode(KEY1,INPUT);
    pinMode(KEY2,INPUT);

    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(12, OUTPUT);
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);
    // 初始化滤波缓冲区
    for (int i = 0; i < FILTER_WINDOW_SIZE; i++) {
      filterBuffer[i] = 0;
    }

    Wire.begin(4, 5);
    if (! aht.begin()) {
      Serial.println("Could not find AHT? Check wiring");
    }
    Serial.println("Enable BLE Server...");
    BLEDevice::init("SkinSensor"); // 这是你手机搜索时会看到的名字
    BLEServer *pServer = BLEDevice::createServer();
    // *** 关键修改：设置回调 ***
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);
      pTxCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
    pTxCharacteristic->addDescriptor(new BLE2902());

    // 创建RX特征 (用于接收)
    BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
                                            CHARACTERISTIC_UUID_RX,
                                            BLECharacteristic::PROPERTY_WRITE
                                          );
    pRxCharacteristic->setCallbacks(new MyCallbacks());

    pService->start();
     pServer->getAdvertising()->start();
    Serial.println("Enable Done Wait Connect...");
}

// the loop function runs over and over again forever
void loop() {
  sensors_event_t humidity, temp;
  KEY1_STATE = digitalRead(KEY1);
  KEY2_STATE = digitalRead(KEY2);
  delay(10);
  aht.getEvent(&humidity, &temp);
  //Serial.print("Temperature: "); 
  //Serial.print(temp.temperature);
  //Serial.println(" degrees C");
  //Serial.print("Humidity: "); 
  //Serial.print(humidity.relative_humidity); Serial.println("% rH");

  int analogVoltsValue = 0;
  analogVoltsValue = analogReadMilliVolts(PIN_ADC0);
  //Serial.printf("ADC millivolts value = %d ms\n",analogVoltsValue);
  Serial.write("ADC_value = ");
  Serial.println(analogVoltsValue);
  // 一直发送
  // 处理信号
  SkinSignalResult result = processSkinSignal((float)analogVoltsValue/10);

  char adcString[50];
  sprintf(adcString, "SkinH:%3.1f%%", result.SkinHuResult); // 将整数转换为字符串
  pTxCharacteristic->setValue(adcString);
  pTxCharacteristic->notify();
  sprintf(adcString, "EnvH:%4.2f%%",humidity.relative_humidity); // 将整数转换为字符串
  pTxCharacteristic->setValue(adcString);
  pTxCharacteristic->notify();
  sprintf(adcString, "EnvT:%4.2f\r\n",temp.temperature); // 将整数转换为字符串
  pTxCharacteristic->setValue(adcString);
  pTxCharacteristic->notify();
  sprintf(adcString, "SkinH:%3.1f%%,EnvH:%4.2f%%,EnvT:%4.2f\r\n", (float)analogVoltsValue/10,humidity.relative_humidity,temp.temperature); // 将整数转换为字符串
  Serial.println(adcString);

  LEDDisplay();
}
