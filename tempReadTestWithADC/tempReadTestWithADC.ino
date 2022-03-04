#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads1115; // Construct an ads1115 
const float multiplier = 0.1875F;
//const float fiveTo3Mofier = 5/3.3F;
float inputVoltage = 0;
float tempC = 0;

void setup(void)
{
  


  Serial.begin(115200);
  Serial.println("Hello!");
  
  Serial.println("Getting single-ended readings from AIN0..3");
  Serial.println("ADC Range: )");
  ads1115.begin(0x48);  // Initialize ads1115 at address
}

void loop(void)
{
  int16_t adc0;

  adc0 = ads1115.readADC_SingleEnded(0);
  inputVoltage = adc0 * multiplier; // * fiveTo3Mofier;
  
  Serial.print("Input voltage: "); Serial.println(inputVoltage);

  // Copied from Excel 2nd order interpolation from raw data provided here: https://e3d-online.zendesk.com/hc/en-us/articles/360017153677-E3D-PT100-Amplifier-Guide
  tempC = 1*pow(10,-5)*pow(inputVoltage,2) + 0.0989*inputVoltage - 140.21;

  Serial.print("Temp approx: "); Serial.print(tempC); Serial.println(" deg C");
  
  delay(500);
}
