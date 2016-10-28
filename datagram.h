#ifndef DATAGRAM
#define DATAGRAM

typedef struct
{
	double x, y, z;
} POINT3D;

//маяки - передатчики
//#define TAGS_NUMBER 15

//якоря - премники/передатчики
#define ANCHORS_NUMBER 8

#define SPEED_OF_LIGHT      (299702547.0)       // in m/s in air
#define ANC_REP_CODE   0x23						// Data packet signature (35)
#define DWT_TIME_UNITS      (1.0/499.2e6/128.0) //!< = 15.65e-12 s
#define  MAX_T5   (double)0x010000000000		// 5 byte time marks maximum + 1
#define  HALF_T5  (double)(MAX_T5 / 2)

typedef unsigned char uint8;
typedef long long     int64;

// gyroscope and accelerometer sensor data
typedef struct
{
    short ax, ay, az;  // accelerometer
    short gx, gy, gz;  // gyroscope
}G_A_DATA;

//маяки - передатчики
#define TAGS_NUMBER 15

// anchor report
typedef struct
{
  uint8 code;			//ANC_REP_CODE
  uint8 addr;      		//anchor address 0..7
  uint8 sync_n;			// sync series #
  uint8 length;         // data length, 75 bytes or more

  uint8  time_mark[TAGS_NUMBER][5];         // 75 bytes
// optional, if sensor data present
  uint8  reserv[3];					   //
  uint8  sd_tag;					   // sensor data (accel+gyro) tag #
  uint8  sd_tags;					   // number of sensor data blocks
  G_A_DATA  sens_data[TAGS_NUMBER];         // max, in reality packet contains data for 2..3 tags (sd_tag, sd_tag+1, ..)
} ANC_MSG;

#define XY_DIMENSION 3.04
#define Z_DIMENSION 2.41
#define Z_LOW_COORD 0.20

#define MARKS_NUMBER 37 //частота опроса датчика, применяется для задания размера массива данных

#endif // DATAGRAM

