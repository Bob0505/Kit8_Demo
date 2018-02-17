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
// https://github.com/adafruit/DHT-sensor-library
#include <DHT.h>
#include <DHT_U.h>

#include <ESP8266WiFi.h>
//https://github.com/JChristensen/Timer
#include "Timer.h"
#include "basic.h"

/* Module class define + */
OLED oled_disp(SDA_PIN, SCL_PIN);
Adafruit_CCS811 ccs;
SFE_BMP180 bmp180;
SHT3X sht30(0x44);
DHT_Unified DHT_A(DHT_PIN_A, DHT22);	// DHT 22 (AM2302)

Timer Tasks;
/* Module class define - */

#ifdef	EN_I2C_SCAN
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
#endif

void basic_setup(void)
{
	pinMode(RST_OLED, OUTPUT);
	Serial.begin(SERIAL_BAUD);
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

	oled_disp.print((char*)"Bob" , 1, 2);
	oled_disp.print((char*)"Kara", 2, 5);
	delay(250);

	// Test display OFF
	oled_disp.off();
	delay(100);

	// Test display ON
	oled_disp.on();
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

void DHT22_setup(DHT_Unified dht)
{
	// Initialize device.
	dht.begin();
	Serial.println("DHT22 Unified Sensor Example");
	// Print temperature sensor details.
	sensor_t sensor;
	dht.temperature().getSensor(&sensor);
	Serial.println("------------------------------------");
	Serial.println("Temperature");
	Serial.print  ("Sensor:       "); Serial.println(sensor.name);
	Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
	Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
	Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" *C");
	Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" *C");
	Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" *C");  
	Serial.println("------------------------------------");
	// Print humidity sensor details.
	dht.humidity().getSensor(&sensor);
	Serial.println("------------------------------------");
	Serial.println("Humidity");
	Serial.print  ("Sensor:       "); Serial.println(sensor.name);
	Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
	Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
	Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println("%");
	Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println("%");
	Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println("%");  
	Serial.println("------------------------------------");
	// Set delay between sensor readings based on sensor details.
	DHT22_Data.min_delay_ms = sensor.min_delay / 1000;
	Serial.print  ("DHT22_Data.min_delay_ms:   "); Serial.println(DHT22_Data.min_delay_ms);
	Serial.println("------------------------------------");
}

void Wifi_setup(void)
{
    // Connecting to a WiFi network
    Serial.print("Connect to ");
    Serial.println( SSID );
    WiFi.begin( SSID, PASS );

    // wait connect WiFi SSID
    while( WiFi.status() != WL_CONNECTED )
    {
        delay(500);
        Serial.print( "." );
    }
    Serial.println( "" );

    Serial.println( "WiFi connected" );
    Serial.println( "IP address: " );
    Serial.println( WiFi.localIP() );

    oled_disp.print((char*)"WiFi connected", 3, 1);
    delay(1000);
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
	//char Tempstr[50];

	// Get a new pressure reading:
	BMP180_Data.Pressure = getPressure();

	// Show the relative altitude difference between
	// the new reading and the baseline reading:
	//sprintf(Tempstr, "Temperature: %d, Pressure: %d hPa", (int32)BMP180_Data.Temperature, (int32)BMP180_Data.Pressure);
	//Serial.println(Tempstr);

	BMP180_Data.Altitude = bmp180.altitude(BMP180_Data.Pressure, BMP180_Data.Baseline);
	//sprintf(Tempstr, "Relative altitude: %d meters (%d feet).", (int32)BMP180_Data.Altitude, (int32)(3.28084*BMP180_Data.Altitude));
	//Serial.println(Tempstr);
}

void sht30_loop(void)
{
	//char Tempstr[120];

	if(sht30.get()==0)
	{
		//sprintf(Tempstr, "Temperature in Celsius: %f, Temperature in Fahrenheit: %f, Relative Humidity: %f",
		//					sht30.cTemp, sht30.fTemp, sht30.humidity);
		//Serial.println(Tempstr);
	}
	else
	{
		Serial.println("Error!");
		oled_disp.print((char*)"Error!", 2);
	}
}

void DHT22_loop(DHT_Unified dht, float* temp, float* humi)
{
	//RET_STATUS Status = RET_SUCCESS;
	// Delay between measurements.
	//delay(DHT22_Data.min_delay_ms);

	// Get temperature event and print its value.
	sensors_event_t event;
	dht.temperature().getEvent(&event);
	if (isnan(event.temperature))
	{
		Serial.println("Error reading temperature!");
		//SET_BIT(Status, b_RET_DHT22_TEMP_ERROR);
		//Status|= (1 << b_RET_DHT22_TEMP_ERROR);
	}
	else {
		//Serial.print("Temperature: ");
		//Serial.print("Temp: ");
		//Serial.print(event.temperature);
		//Serial.println(" *C");
		*temp = event.temperature;
	}
	// Get humidity event and print its value.
	dht.humidity().getEvent(&event);
	if (isnan(event.relative_humidity))
	{
		Serial.println("Error reading humidity!");
		//SET_BIT(Status, b_RET_DHT22_HUMI_ERROR);
		//Status|= (1 << b_RET_DHT22_HUMI_ERROR);
	}
	else
	{
		//Serial.print("Humidity: ");
		//Serial.print("Humi: ");
		//Serial.print(event.relative_humidity);
		//Serial.println("%");
		*humi = event.relative_humidity;
	}

	//return Status;
}

void DHT22_loop(void)
{
	//Serial.println("[A]");
	DHT22_loop(DHT_A, &DHT22_Data.Temp, &DHT22_Data.Humi);
	//Serial.print("  Temp: ");	Serial.print(DHT22_Data.Temp);	Serial.println(" *C");
	//Serial.print("  Humi: ");	Serial.print(DHT22_Data.Humi);	Serial.println(" %");
}

void UpdateIoTData(void)
{
	IoT_Data.DHT22_Temp 	= (int32)DHT22_Data.Temp;
	IoT_Data.DHT22_Humi 	= (int32)DHT22_Data.Humi;
	IoT_Data.SHT30_Temp		= (int32)sht30.cTemp;
	IoT_Data.SHT30_Humi		= (int32)sht30.fTemp;
	IoT_Data.BMP180_Temp	= (int32)BMP180_Data.Temperature;
	IoT_Data.BMP180_Pres	= (int32)BMP180_Data.Pressure;
}

void wifi_loop(void)
{
	//RET_STATUS Status = RET_SUCCESS;

	// setting ESP8266 as Client
	WiFiClient client;
	if( !client.connect( HOST, PORT ) )
	{
		Serial.println( "connection failed" );
		//SET_BIT(Status, b_RET_WIFI_HUMI_ERROR);
		//Status|= (1 << b_RET_WIFI_CNT_ERROR);
	}
	else
	{
		String getStr = GET +   "&field1=" + String(IoT_Data.DHT22_Temp) +
								"&field2=" + String(IoT_Data.DHT22_Humi) +
								"&field3=" + String((float)IoT_Data.SHT30_Temp) +
								"&field4=" + String((float)IoT_Data.SHT30_Humi) +
								"&field5=" + String(IoT_Data.BMP180_Temp) +
								"&field6=" + String(IoT_Data.BMP180_Pres) +
								" HTTP/1.1\r\n";;
		client.print( getStr );
		client.print( "Host: api.thingspeak.com\n" );
		client.print( "Connection: close\r\n\r\n" );

		delay(10);
		client.stop();
	}

	//return Status;
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
#ifdef	EN_I2C_SCAN
/* Show I2C device address + */
	oled_disp.print((char*)"I2C:");
	for(uint8 idx=0;I2CAdr[idx]!=0x00;idx++)
	{
		sprintf(i2cadr, " %x", I2CAdr[idx]);
		strncat(Tempstr, i2cadr, 3);
	}
	Serial.println(Tempstr);
	oled_disp.print(Tempstr, 0, I2C_TAITLE_LEN);
/* Show I2C device address - */
#else
/* Show DHT22  data + */
	sprintf(Tempstr, "DT:%dC     H:%d%", (int32)DHT22_Data.Temp, (int32)DHT22_Data.Humi);
	Serial.println(Tempstr);
	oled_disp.print(Tempstr);
/* Show DHT22 data - */
#endif
/* Show SHT30 data + */
	sprintf(Tempstr, "ST:%dC %dF H:%d%", (int32)sht30.cTemp, (int32)sht30.fTemp, (int32)sht30.humidity);
	Serial.println(Tempstr);
	oled_disp.print(Tempstr, 1);
/* Show SHT30  data - */

/* Show BMP180 data + */
	sprintf(Tempstr, "BT:%dC ", (int32)BMP180_Data.Temperature);
	Serial.println(Tempstr);
	oled_disp.print(Tempstr, 2);

	sprintf(Tempstr, " P:%dhPa A:%dm", (int32)BMP180_Data.Pressure, (int32)BMP180_Data.Altitude);
	Serial.println(Tempstr);
	oled_disp.print(Tempstr, 3);
/* Show BMP180 data - */

}

void task_1s(void)
{
#ifdef	EN_I2C_SCAN
	I2CScan();
#endif
	if(CCS811_Data.Status == RET_SUCCESS)
		ccs_loop();

	if(BMP180_Data.Status == RET_SUCCESS)
		bmp180_loop();

	sht30_loop();
	DHT22_loop();
	showdata();

	Serial.println("-------");
}

void task_30s(void)
{
	UpdateIoTData();
	wifi_loop();
}

void task_setup(void)
{
	Tasks.every(1*1000,  task_1s);
	Tasks.every(30*1000, task_30s);

	//do once
	task_1s();
	task_30s();
}

void setup()
{
	basic_setup();
	oled_setup();
	ccs_setup();
	BMP180_setup();
	DHT22_setup(DHT_A);
	Wifi_setup();

	task_setup();
}

void loop()
{
	Tasks.update();
}
