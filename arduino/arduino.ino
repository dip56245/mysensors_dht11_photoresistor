/*
 * Датчик температуры/влажность/освещенности
 * За основу взят: https://www.mysensors.org/build/humidity
 * 
 * фоторезистор с 10к резистором (делитель напряжения) - A0 пин
 * DHT11 - к 3 пину
 * 
 */

//#define MY_DEBUG
#define MY_RADIO_NRF24 
#include <SPI.h>
#include <MySensors.h>  
#include <DHT.h>

#define DHTPIN 3        // куда подключен DHT
#define DHTTYPE DHT11   // DHT 11
#define SENSOR_TEMP_OFFSET 0
#define LIGHT_PIN A0

const float my_vcc_const = 1.097;    // константа вольтметра (для правильного определения напряжения питания)
int oldBatteryPcnt = 0;
static const uint64_t UPDATE_INTERVAL = 60000;
static const uint8_t FORCE_UPDATE_N_READS = 10;

#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1
#define CHILD_ID_LIGHT 2

float lastTemp;
float lastHum;
uint8_t nNoUpdatesTemp;
uint8_t nNoUpdatesHum;
const bool metric = true;
int oldLight;

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgLight(CHILD_ID_LIGHT, V_LIGHT_LEVEL);
DHT dht(DHTPIN, DHTTYPE);

void presentation(){ 
  sendSketchInfo("TemperatureAndHumidity", "1.1");
  present(CHILD_ID_HUM, S_HUM);
  present(CHILD_ID_TEMP, S_TEMP);  
  present(CHILD_ID_LIGHT, S_LIGHT_LEVEL);
}


void setup(){
  dht.begin();
}

void loop(){  
  //Батарейка
  long voltage = readVcc();
  int batteryPcnt = map(voltage, 3000, 4200, 0, 100);
  batteryPcnt = constrain(batteryPcnt, 0, 100);

  #ifdef MY_DEBUG
    Serial.print("Battery raw: ");
    Serial.println(voltage);
    float batteryV  = voltage / 1000;
    Serial.print("Battery Voltage: ");
    Serial.print(batteryV);
    Serial.println(" V");

    Serial.print("Battery percent: ");
    Serial.print(batteryPcnt);
    Serial.println(" %");
  #endif

  if (oldBatteryPcnt != batteryPcnt) {
    // Power up radio after sleep
    sendBatteryLevel(batteryPcnt);
    oldBatteryPcnt = batteryPcnt;
  }
  
  long lightlevel = map(analogRead(LIGHT_PIN),0, 1024, 0, 100);
  #ifdef MY_DEBUG
    Serial.print("PhotoSensor: ");
    Serial.print(lightlevel);
    Serial.println("%");
  #endif
  if (oldLight!=lightlevel){
    oldLight=lightlevel;
    send(msgLight.set(lightlevel, 1));
  }
  
  float temperature = dht.readTemperature();
  if (isnan(temperature)) {
    Serial.println("Failed reading temperature from DHT!");
  } else if (temperature != lastTemp || nNoUpdatesTemp == FORCE_UPDATE_N_READS) {
    // Only send temperature if it changed since the last measurement or if we didn't send an update for n times
    lastTemp = temperature;
    // Reset no updates counter
    nNoUpdatesTemp = 0;
    temperature += SENSOR_TEMP_OFFSET;
    send(msgTemp.set(temperature, 1));

    #ifdef MY_DEBUG
    Serial.print("T: ");
    Serial.println(temperature);
    #endif
  } else {
    // Increase no update counter if the temperature stayed the same
    nNoUpdatesTemp++;
  }

  // Get humidity from DHT library
  float humidity = dht.readHumidity();
  if (isnan(humidity)) {
    Serial.println("Failed reading humidity from DHT");
  } else if (humidity != lastHum || nNoUpdatesHum == FORCE_UPDATE_N_READS) {
    // Only send humidity if it changed since the last measurement or if we didn't send an update for n times
    lastHum = humidity;
    // Reset no updates counter
    nNoUpdatesHum = 0;
    send(msgHum.set(humidity, 1));
    
    #ifdef MY_DEBUG
    Serial.print("H: ");
    Serial.println(humidity);
    #endif
  } else {
    // Increase no update counter if the humidity stayed the same
    nNoUpdatesHum++;
  }

  // Sleep for a while to save energy
  sleep(UPDATE_INTERVAL); 
}

long readVcc() { //функция чтения внутреннего опорного напряжения, универсальная (для всех ардуин)
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both
  long result = (high << 8) | low;

  result = my_vcc_const * 1023 * 1000 / result; // расчёт реального VCC
  return result; // возвращает VCC
}

