// This example just provide basic function test;
// For more informations, please vist www.heltec.cn or mail to support@heltec.cn

#include <Wire.h>
//https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series
#include "OLED.h"
//https://github.com/adafruit/Adafruit_CCS811
#include "Adafruit_CCS811.h"
//https://github.com/sparkfun/BMP180_Breakout
#include "SFE_BMP180.h"
//https://github.com/wemos/WEMOS_SHT3x_Arduino_Library.git
#include "WEMOS_SHT3X.h"

typedef enum {
  RET_SUCCESS = 0,
  RET_NOT_SUPPORT,
  RET_NOT_IMPLEMENT,
  RET_NOT_INITIAL,
  RET_INVALID_PARAM,
  RET_DATA_SIZE_NOT_MATCH,
  RET_BUF_TOO_SMALL,
  RET_TIMEOUT,
  RET_HW_ERROR,
} RET_STATUS;

/* Module Pin define + */
const int SDA_PIN = 4;		/* GPIO4	D2 */
const int SCL_PIN = 5;		/* GPIO5	D1 */
const int RST_OLED = 16;	/* GPIO16	D0 */
/* Module Pin define - */

/* Module class define + */
OLED oled_disp(SDA_PIN, SCL_PIN);
Adafruit_CCS811 ccs;
SFE_BMP180 bmp180;
SHT3X sht30(0x44);
/* Module class define - */

/* Module DataStruct + */
#define I2C_TAITLE_LEN 4
#define NOW_TEMP 26.0

uint8 I2CAdr[5];

struct CCS811_struct {
	uint8    Status;
	double   Temp;
	uint16   CO2;
	uint16   TVOC;
} CCS811_Data;

struct BMP180_struct {
	uint8	Status;
	double	Baseline;
	double	Pressure;
	double	Altitude;
	double	Temperature;

} BMP180_Data;

/* Module DataStruct - */

void I2CScan(void)
{
	byte error, address;
	int nDevices;
	char Tempstr[3];

	//Serial.println("Scanning...");
	for(uint8 idx=0;idx<sizeof(I2CAdr);idx++)
		I2CAdr[idx] = 0x00;

	nDevices = 0;
	for(address = 1; address < 127; address++ )
	{
		// The i2c_scanner uses the return value of
		// the Write.endTransmisstion to see if
		// a device did acknowledge to the address.
		Wire.beginTransmission(address);
		error = Wire.endTransmission();

		if (error == 0)
		{
			/*
			Serial.print("I2C device found at address 0x");
			if (address<16)
				Serial.print("0");
			Serial.print(address, HEX); Serial.println("  !");
			*/
			I2CAdr[nDevices] = address;

			nDevices++;
		}
		else if (error==4)
		{
			Serial.print("Unknown error at address 0x");
			if (address<16)
				Serial.print("0");
			Serial.println(address, HEX);
		}
	}
	/*
	if (nDevices == 0)
		Serial.println("No I2C devices found\n");
	else
		Serial.println("done\n");
	*/
}

void basic_setup(void)
{
	pinMode(RST_OLED, OUTPUT);
	Serial.begin(74880);
	Serial.println("Serial init OK!!!");
	Wire.begin();
	Serial.println("Wire init OK!!!");

	Serial.println("Reset OLED");
	digitalWrite(RST_OLED, LOW);   // turn D2 low to reset OLED
	delay(50);
	digitalWrite(RST_OLED, HIGH);    // while OLED is running, must set D2 in high
}

void showColumn(void)
{
	char Tempstr[2];

	for(uint8 idx=0;idx<0x10;idx++)
	{
		sprintf(Tempstr, "%x", idx);
		oled_disp.print(Tempstr, 0, idx);
	}
}

void showRow(void)
{
	char Tempstr[2];

	for(uint8 idx=0;idx<0x04;idx++)
	{
		sprintf(Tempstr, "%x", idx);
		oled_disp.print(Tempstr, idx);
	}
}

double getPressure()
{
  char status;

  // You must first get a temperature measurement to perform a pressure reading.

  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = bmp180.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = bmp180.getTemperature(BMP180_Data.Temperature);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = bmp180.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = bmp180.getPressure(BMP180_Data.Pressure, BMP180_Data.Temperature);
        if (status != 0)
        {
          return(BMP180_Data.Pressure);
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
}

void oled_setup(void)
{
	// Initialize display
	oled_disp.begin();
	oled_disp.clear();

	showColumn();
	showRow();

	oled_disp.print("Bob" , 1, 2);
	oled_disp.print("Kara", 2, 5);
	delay(1000);

	// Test display OFF
	oled_disp.off();
	delay(300);

	// Test display ON
	oled_disp.on();
	delay(1000);

	oled_disp.clear();
}

void ccs_setup(void)
{
	if(!ccs.begin())
	{
		CCS811_Data.Status = RET_NOT_SUPPORT;
		Serial.println("Failed to start sensor! Please check your wiring.");
		//while(1);
	}
	else
	{
		CCS811_Data.Status = RET_SUCCESS;
		//calibrate temperature sensor
		while(!ccs.available());

		ccs.setTempOffset(ccs.calculateTemperature() - NOW_TEMP);
	}
}

void BMP180_setup(void)
{
	char Tempstr[50];

	if (!bmp180.begin())
	{
		// Oops, something went wrong, this is usually a connection problem,
		// see the comments at the top of this sketch for the proper connections.
		BMP180_Data.Status = RET_NOT_SUPPORT;
		Serial.println("BMP180 init fail (disconnected?)\n\n");
	}
	else
	{
		BMP180_Data.Status = RET_SUCCESS;
		Serial.println("BMP180 init success");
		BMP180_Data.Baseline = getPressure();

		sprintf(Tempstr, "Baseline pressure:  %f hPa", BMP180_Data.Baseline);
		Serial.println(Tempstr);
	}
}

void setup()
{
	basic_setup();
	oled_setup();
	ccs_setup();
	BMP180_setup();
}

void ccs_loop(void)
{
	char Tempstr[40];

	if(ccs.available())
	{
		if(!ccs.readData())
		{
			CCS811_Data.Temp = ccs.calculateTemperature();
			CCS811_Data.CO2  = ccs.geteCO2();
			CCS811_Data.TVOC = ccs.getTVOC();

			Serial.print("Temp: ");	Serial.print(CCS811_Data.Temp);
			Serial.print("CO2: ");	Serial.print(CCS811_Data.CO2);  	Serial.print("ppm, ");
			Serial.print("TVOC: ");	Serial.print(CCS811_Data.TVOC);		Serial.println("ppb, ");

			sprintf(Tempstr, "Temp: %02d, CO2: %04dppm, TVOC: %03dppb\n",
						(uint16)CCS811_Data.Temp, (uint16)CCS811_Data.CO2, (uint16)CCS811_Data.TVOC);

			Serial.println(Tempstr);
		}
		else
		{
			Serial.println("ERROR!");
			//while(1);
		}
	}
}

void bmp180_loop(void)
{
	char Tempstr[50];

	// Get a new pressure reading:
	BMP180_Data.Pressure = getPressure();

	// Show the relative altitude difference between
	// the new reading and the baseline reading:
	sprintf(Tempstr, "Temperature: %d, Pressure: %d hPa", (int32)BMP180_Data.Temperature, (int32)BMP180_Data.Pressure);
	Serial.println(Tempstr);

	BMP180_Data.Altitude = bmp180.altitude(BMP180_Data.Pressure, BMP180_Data.Baseline);
	sprintf(Tempstr, "Relative altitude: %d meters (%d feet).", (int32)BMP180_Data.Altitude, (int32)(3.28084*BMP180_Data.Altitude));
	Serial.println(Tempstr);
}

void sht30_loop(void)
{
	char Tempstr[120];

	if(sht30.get()==0)
	{
		Serial.print("Temperature in Celsius : ");
		Serial.println();

		sprintf(Tempstr, "Temperature in Celsius: %f, Temperature in Fahrenheit: %f, Relative Humidity: %f",
							sht30.cTemp, sht30.fTemp, sht30.humidity);
		Serial.println(Tempstr);
	}
	else
	{
		Serial.println("Error!");
		oled_disp.print("Error!", 2);
	}
}

void showdata(void)
{
	char i2cadr[3], Tempstr[20];

	oled_disp.clear();
	/*
	for(uint8 idx_R=0;idx_R<0x04;idx_R++)
		//for(uint8 idx_C=0;idx_C<0x10;idx_C++)
		oled_disp.print("                ", idx_R);
	*/

	sprintf(Tempstr, "");
	oled_disp.print("I2C:");
	for(uint8 idx=0;I2CAdr[idx]!=0x00;idx++)
	{
		sprintf(i2cadr, " %x", I2CAdr[idx]);
		strncat(Tempstr, i2cadr, 3);
	}
	Serial.println(Tempstr);
	oled_disp.print(Tempstr, 0, I2C_TAITLE_LEN);

	sprintf(Tempstr, "BT:%dC ", (int32)BMP180_Data.Temperature);
	Serial.println(Tempstr);
	oled_disp.print(Tempstr, 1);

	sprintf(Tempstr, " P:%dhPa A:%dm", (int32)BMP180_Data.Pressure, (int32)BMP180_Data.Altitude);
	Serial.println(Tempstr);
	oled_disp.print(Tempstr, 2);

	sprintf(Tempstr, "ST:%dC %dF H:%d", (int32)sht30.cTemp, (int32)sht30.fTemp, (int32)sht30.humidity);
	Serial.println(Tempstr);
	oled_disp.print(Tempstr, 3);
}

void loop()
{
	I2CScan();

	if(CCS811_Data.Status == RET_SUCCESS)
		ccs_loop();

	if(BMP180_Data.Status == RET_SUCCESS)
		bmp180_loop();

	sht30_loop();
	showdata();

	Serial.println("-------");
	delay(1000);
}
