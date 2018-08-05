// Enable debug prints
#define MY_DEBUG

#define NODE_IS_BATTERY_POWERED
#define NODE_HAS_DS18B20_ATTACHED
//#define NODE_HAS_DHT22_ATTACHED
//#define NODE_HAS_BH1750_ATTACHED

// Enable and select radio type attached 
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69
//#define MY_RS485

#if defined(NODE_HAS_DS18B20_ATTACHED) || defined(NODE_HAS_DHT22_ATTACHED) || defined(NODE_HAS_BH1750_ATTACHED)
#include <SPI.h>
#endif

#include <MySensors.h>

#ifdef NODE_HAS_DS18B20_ATTACHED
#include <DallasTemperature.h>
#include <OneWire.h>

#define ONE_WIRE_BUS 3 // Pin where dallase sensor is connected 
#define MAX_ATTACHED_DS18B20 16
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature. 
float lastTemperature[MAX_ATTACHED_DS18B20];
int noTemperatureUpdate[MAX_ATTACHED_DS18B20];
int numSensors=0;
bool receivedConfig = false;
// Initialize temperature message
MyMessage DS18msg(0,V_TEMP);
#endif

#ifdef NODE_HAS_DHT22_ATTACHED
#include <DHT.h>
#define DHT_DATA_PIN 3
#define SENSOR_TEMP_OFFSET 0
#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1
float lastTemp;
float lastHum;
uint8_t nNoUpdatesTemp;
uint8_t nNoUpdatesHum;
MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
DHT dht;
#endif

#if defined(NODE_HAS_DS18B20_ATTACHED) || defined(NODE_HAS_DHT22_ATTACHED)
bool metric = true;
#endif

#ifdef NODE_HAS_BH1750_ATTACHED
#include <BH1750.h>
#include <Wire.h>
#define CHILD_ID_LIGHT 20
BH1750 lightSensor;
MyMessage msgLight(CHILD_ID_LIGHT, V_LIGHT_LEVEL);
uint16_t lastlux;
int noLightUpdate = 0;
#endif


static const uint64_t UPDATE_INTERVAL = 120000;
static const uint8_t FORCE_UPDATE_N_READS = 6;


#ifdef NODE_IS_BATTERY_POWERED
//=========================
// BATTERY VOLTAGE DIVIDER SETUP
// 1M, 470K divider across battery and using internal ADC ref of 1.1V
// Sense point is bypassed with 0.1 uF cap to reduce noise at that point
// ((1e6+470e3)/470e3)*1.1 = Vmax = 3.44 Volts
// 3.44/1023 = Volts per bit = 0.003363075
#define VBAT_PER_BITS 0.003363075
#define VMIN 1.9                                  //  Vmin (radio Min Volt)=1.9V (564v)
#define VMAX 3.0                                  //  Vmax = (2xAA bat)=3.0V (892v)
int batteryPcnt = 0;                              // Calc value for battery %
int batLoop = 0;                                  // Loop to help calc average
int batArray[3];                                  // Array to store value for average calc.
int BATTERY_SENSE_PIN = A0;                       // select the input pin for the battery sense point
//=========================
#endif


void before()
{
#ifdef NODE_HAS_DS18B20_ATTACHED
  sensors.begin();
#endif
}


void presentation()  
{ 
  // Send the sketch version information to the gateway
  sendSketchInfo("TestSensorBoard", "1.1");

#ifdef NODE_HAS_DS18B20_ATTACHED
  // Fetch the number of attached temperature sensors  
  numSensors = sensors.getDeviceCount();

  // Present all sensors to controller
  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {   
     present(i, S_TEMP);
  }
#endif

#ifdef NODE_HAS_DHT22_ATTACHED
  present(CHILD_ID_HUM, S_HUM);
  present(CHILD_ID_TEMP, S_TEMP);
#endif

#ifdef NODE_HAS_BH1750_ATTACHED
  present(CHILD_ID_LIGHT, S_LIGHT_LEVEL);
#endif

#if defined(NODE_HAS_DS18B20_ATTACHED) || defined(NODE_HAS_DHT22_ATTACHED)
  metric = getControllerConfig().isMetric;
#endif
}


void setup()
{
#ifdef NODE_IS_BATTERY_POWERED
  analogReference(INTERNAL);
#endif

  delay(500); // Allow time for radio if power used as reset´

#ifdef NODE_HAS_BH1750_ATTACHED
  lightSensor.begin();
#endif

#ifdef NODE_HAS_DHT22_ATTACHED
  dht.setup(DHT_DATA_PIN); // set data pin of DHT sensor
  if (UPDATE_INTERVAL <= dht.getMinimumSamplingPeriod()) {
    Serial.println("Warning: UPDATE_INTERVAL is smaller than supported by the sensor!");
  }
  // Sleep for the time of the minimum sampling period to give the sensor time to power up
  // (otherwise, timeout errors might occure for the first reading)
  sleep(dht.getMinimumSamplingPeriod());
#endif

#ifdef NODE_HAS_DS18B20_ATTACHED
  sensors.setWaitForConversion(false);
#endif
}


void loop()      
{  
  delay(500); // Allow time for radio if power used as reset

#ifdef NODE_HAS_DS18B20_ATTACHED
  sensors.requestTemperatures();

  // query conversion time and sleep until conversion completed
  int16_t conversionTime = sensors.millisToWaitForConversion(sensors.getResolution());
  // sleep() call can be replaced by wait() call if node need to process incoming messages (or if node is repeater)
  sleep(conversionTime);

  // Read temperatures and send them to controller 
  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {

    // Fetch and round temperature to one decimal
    float temperature = static_cast<float>(static_cast<int>((getControllerConfig().isMetric?sensors.getTempCByIndex(i):sensors.getTempFByIndex(i)) * 10.)) / 10.;

    // Only send data if temperature has changed and no error
    if ((lastTemperature[i] != temperature || noTemperatureUpdate[i] == FORCE_UPDATE_N_READS) && temperature != -127.00 && temperature != 85.00) {
      // Send in the new temperature
      send(DS18msg.setSensor(i).set(temperature,1));
      // Save new temperatures for next compare
      lastTemperature[i]=temperature;
      noTemperatureUpdate[i]=0;
    } else {
      noTemperatureUpdate[i] += 1;
    }
  }
#endif

#ifdef NODE_HAS_DHT22_ATTACHED
  float temperature = dht.getTemperature();
  if (isnan(temperature)) {
    Serial.println("Failed reading temperature from DHT!");
  } else if (temperature != lastTemp || nNoUpdatesTemp == FORCE_UPDATE_N_READS) {
    lastTemp = temperature;

    temperature += SENSOR_TEMP_OFFSET;

    if (!metric) {
      temperature = dht.toFahrenheit(temperature);
    }
    nNoUpdatesTemp = 0;
    send(msgTemp.set(temperature, 1));

    #ifdef MY_DEBUG
    Serial.print("T: ");
    Serial.println(temperature);
    #endif
  } else {
    nNoUpdatesTemp++;
  }

  float humidity = dht.getHumidity();
  if (isnan(humidity)) {
    Serial.println("Failed reading humidity from DHT");
  } else if (humidity != lastHum || nNoUpdatesHum == FORCE_UPDATE_N_READS) {
    lastHum = humidity;
    nNoUpdatesHum = 0;
    send(msgHum.set(humidity, 1));

    #ifdef MY_DEBUG
    Serial.print("H: ");
    Serial.println(humidity);
    #endif
  } else {
    nNoUpdatesHum++;
  }
#endif

#ifdef NODE_HAS_BH1750_ATTACHED
  uint16_t lux = lightSensor.readLightLevel();// Get Lux value
  Serial.println(lux);
  if (lux != lastlux || noLightUpdate == FORCE_UPDATE_N_READS) {
      send(msgLight.set(lux));
      lastlux = lux;
      noLightUpdate = 0;
  } else {
    noLightUpdate++;
  }
#endif

#ifdef NODE_IS_BATTERY_POWERED
  batM();
#endif
  
  // Sleep for a while to save energy
  sleep(UPDATE_INTERVAL); 
}

#ifdef NODE_IS_BATTERY_POWERED
void batM() //The battery calculations
{
  delay(500);
  // Battery monitoring reading
  int sensorValue = analogRead(BATTERY_SENSE_PIN);
  delay(500);

  // Calculate the battery in %
  float Vbat  = sensorValue * VBAT_PER_BITS;
  int batteryPcnt = static_cast<int>(((Vbat - VMIN) / (VMAX - VMIN)) * 100.);
  Serial.print("Battery percent: "); Serial.print(batteryPcnt); Serial.println(" %");

  // Add it to array so we get an average of 3 (3x20min)
  batArray[batLoop] = batteryPcnt;

  if (batLoop > 2) {
    batteryPcnt = (batArray[0] + batArray[1] + batArray[2] + batArray[3]);
    batteryPcnt = batteryPcnt / 3;

    if (batteryPcnt > 100) {
      batteryPcnt = 100;
    }
    sendBatteryLevel(batteryPcnt);
    batLoop = 0;
  }
  else
  {
    batLoop++;
  }
}
#endif
