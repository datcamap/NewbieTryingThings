#include <Arduino.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

#define tdsPin 34
#define pHPin 35
#define t0Pin 32
#define VREF 3.3              // analog reference voltage(Volt) of the ADC
#define SCOUNT  30            // sum of sample point

LiquidCrystal_I2C lcd(0X27,16,2);

int tdsBuffer[SCOUNT];     // store the TDS values in the array, read from ADC
int pHBuffer[SCOUNT];     // store the pH values
int analogBufferTemp[SCOUNT];
int analogIndex = 0;
int copyIndex = 0;

float averageVoltage = 0;
float tdsValue = 0;
float averagepH = 0;
float dopH = 0;
float temperature = 30;       // current temperature for compensation

// median filtering algorithm
int getMedianNum(int bArray[], int iFilterLen){
  int bTab[iFilterLen];
  for (byte i = 0; i<iFilterLen; i++)
  bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0){
    bTemp = bTab[(iFilterLen - 1) / 2];
  }
  else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}

void setup(){
  Serial.begin(115200);
  lcd.init();                    
  lcd.backlight();

  pinMode(33,OUTPUT);
  digitalWrite(33,HIGH);
  pinMode(tdsPin,INPUT);
  pinMode(pHPin,INPUT);
  pinMode(t0Pin,INPUT);
}

int _readingCycle = 40U;
void loop(){
  static unsigned long analogSampleTimepoint = millis();
  if(millis()-analogSampleTimepoint > _readingCycle){     //every 40 milliseconds,read the analog value from the ADC
    analogSampleTimepoint = millis();
    tdsBuffer[analogIndex] = analogRead(tdsPin);    //read the analog value and store into the buffer
    pHBuffer[analogIndex] = analogRead(pHPin);    
    analogIndex++;
    if(analogIndex == SCOUNT){ 
      analogIndex = 0;
    }
  }   
  
  int _print_Interval = 800U;
  static unsigned long printTimepoint = millis();
  if(millis()-printTimepoint > _print_Interval){
    printTimepoint = millis();
    for(copyIndex=0; copyIndex<SCOUNT; copyIndex++){
      analogBufferTemp[copyIndex] = tdsBuffer[copyIndex];
    }
    // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 4096.0;
    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0)); 
    float compensationCoefficient = 1.0+0.02*(temperature-25.0);
    //temperature compensation
    float compensationVoltage=averageVoltage/compensationCoefficient;
    //convert voltage value to tds value
    tdsValue=(133.42*compensationVoltage*compensationVoltage*compensationVoltage - 255.86*compensationVoltage*compensationVoltage + 857.39*compensationVoltage)*0.5;
    
    for(copyIndex=0; copyIndex<SCOUNT; copyIndex++){
      analogBufferTemp[copyIndex] = pHBuffer[copyIndex];
    }
    //averagepH = getMedianNum(analogBufferTemp, SCOUNT);
    averagepH = analogBufferTemp[0];
    dopH = averagepH;

    // print to LCD
    lcd.setCursor(0,0);
    lcd.print("TDS");
    lcd.setCursor(0,1);
    lcd.print(tdsValue,0);

    lcd.setCursor(5,0);
    lcd.print("pH");
    lcd.setCursor(5,1);
    lcd.print(dopH);

    int tem = analogRead(t0Pin);
    lcd.setCursor(8,0);
    lcd.print(tem);

    // print to serial
    Serial.print("voltage:");
    Serial.print(averageVoltage,2);
    Serial.print("V   ");
    Serial.print("TDS Value:");
    Serial.print(tdsValue,0);
    Serial.println("ppm");
  }
}