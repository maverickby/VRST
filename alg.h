#ifndef ALG_H
#define ALG_H

/*#include <QByteArray>
#include <QMainWindow>

#include "mainwindow.h"

class MainWindow;*/

#include "mainwindow.h"

class MainWindow;


#include "datagram.h"

typedef struct
{
    double x, y, z;
} POINT3D;

//маяки - передатчики
//#define TAGS_NUMBER 15

//якоря - премники/передатчики
#define ANCHORS_NUMBER 8

#define SPEED_OF_LIGHT      (299702547.0)       // in m/s in air
#define ANC_REP_CODE   0x23						// Data packet signature
#define DWT_TIME_UNITS      (1.0/499.2e6/128.0) //!< = 15.65e-12 s
#define  MAX_T5   (double)0x010000000000		// 5 byte time marks maximum + 1
#define  HALF_T5  (double)(MAX_T5 / 2)

typedef unsigned char uint8;
typedef long long     int64;

/*
// gyroscope and accelerometer sensor data
typedef struct
{
    short ax, ay, az;  // accelerometer
    short gx, gy, gz;  // gyroscope
}G_A_DATA;*/

/*
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
} ANC_MSG;*/

/*POINT3D ancor_dflt[ANCHORS_NUMBER] = {				//default ancor positions
                           {0, 0, 2.5},
                           {0, 0, 0},
                           {0, 5, 2.5},
                           {0, 5, 0},
                           {5, 5, 2.5},
                           {5, 5, 0},
                           {5, 0, 2.5},
                           {5, 0, 0}
                          };*/

/*POINT3D ancor_dflt[ANCHORS_NUMBER] = {				//default ancor positions
                           {0, 0, 2.41},
                           {3.02, 0, 2.41},
                           {3.02, 3.02, 2.41},
                           {0, 3.02, 2.41},
                           {0, 0, 0},
                           {3.02, 0, 0},
                           {3.02, 3.02, 0},
                           {0, 3.02, 0}
                          };

double ant_delay[ANCHORS_NUMBER] ={0.5,0.52,1.55,0.45,1,0.96,0.75,0.69};*/


/*
 * This class is intended for math processing of the input messages from 8 anchors as series of time delays in picoseconds
*/
class Alg: public QObject
{
    Q_OBJECT
public:
    //Alg();
    Alg(MainWindow* wnd = 0);
    virtual ~Alg();

    void init();

    POINT3D* DirectCalculationMethod(int tag);

    int getTAi1(int i);
    int getAnyJinAExcludeAi(int i);
    bool getarrJinAExcludeAs(int s);
    int getTA_JK(int j,int k);
    bool Pair_Analyzing(const POINT3D* pt1,const POINT3D* pt2, POINT3D* ptRet);
    bool ProcessAnchorDatagram(const ANC_MSG* datagram, POINT3D* retPoint);
    void process_nav(const ANC_MSG* datagram, POINT3D* retPoint);
    int prepare_data(int tag);
    double find_max_m(void);
    void anc_dist(void);
private:
    int64 t_marks[TAGS_NUMBER][ANCHORS_NUMBER];		//time delay marks array 15x8 (picoseconds)
    double  m_marks[ANCHORS_NUMBER];                   // prepared deltas for one tag
    POINT3D anchor[ANCHORS_NUMBER];						// anchor positions (m)
    POINT3D tag[TAGS_NUMBER];							// calculated tags coordinates

    int a[4];//массив для хранения текущей комбинации 4 из 8 (номера четырех текущих приемников сигнала)
             //(4-х элементное подмножество из множества чисел {1...8})
    int a_[4];//дополнительный массив
    double arrT[ANCHORS_NUMBER];//массив времен задержек
    int arrJ[3];//массив для хранения индексов массива a[] за исключением индекса элемента a[s]
    int sync_series;// series control
    int adj;// adjustment mode to set anchors antennas delays
    double anc0dist[ANCHORS_NUMBER];//distance ancor[i] to ancor[0]

    //координаты приемников
    double u[ANCHORS_NUMBER];
    double v[ANCHORS_NUMBER];
    double w[ANCHORS_NUMBER];

    POINT3D* pt1;
    POINT3D* pt2;
    POINT3D* ptRet;
    POINT3D* p3d;
    MainWindow* mainWindow;
};

#endif // ALG_H
