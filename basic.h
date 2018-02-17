#ifndef _BASIC_H_
#define _BASIC_H_


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

//typedef enum {
//	b_RET_ERROR_UNKNOWN = 0,	/*  0 something shouldn't happened */
//	b_RET_NOT_SUPPORT,			/*  1 */
//	b_RET_DHT22_TEMP_ERROR,		/*  2 */
//	b_RET_DHT22_HUMI_ERROR,		/*  3 */
//	b_RET_WIFI_CNT_ERROR,		/*  4 */
//	b_SYSTEM_REBOOT		= 15	/* 15 */
//} RET_STATUS_bits;
//typedef uint16_t RET_STATUS;

#define SERIAL_BAUD 74880

/* Module Pin define + */
const int SDA_PIN = 4;		/* GPIO4	D2 */
const int SCL_PIN = 5;		/* GPIO5	D1 */
const int RST_OLED = 16;	/* GPIO16	D0 */
const int DHT_PIN_A = 0;	/* GPIO0	D3 */
/* Module Pin define - */

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

struct DHT22_struct {
	uint8	Status;
	uint32	min_delay_ms;
	float	Temp;
	float	Humi;
} DHT22_Data;

/* Module DataStruct - */


#endif	/*_BASIC_H_*/
