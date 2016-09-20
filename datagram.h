#ifndef DATAGRAM
#define DATAGRAM

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

#endif // DATAGRAM

