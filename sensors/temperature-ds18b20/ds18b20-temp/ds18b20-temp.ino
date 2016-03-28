#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 2

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void setup(){
  Serial.begin(9600);
  Serial.println("Temperature Sensor Demo for projectROV");
  
  sensors.begin();
}

void loop(){
  Serial.print("Requesting Temperature...");
   sensors.requestTemperatures();
   Serial.println("Done!");
   
   Serial.print("Temperature is:");
   Serial.print(sensors.getTempCByIndex(0));
}
