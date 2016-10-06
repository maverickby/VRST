#ifndef ALG2_H
#define ALG2_H

#include "alg.h"
//#include "mainwindow.h"

//class MainWindow;
//class Alg;
//#include "datagram.h"


/*
* This class is intended for math processing of the input messages from 8 anchors as series of time delays in picoseconds
*/
class Alg2 : public Alg
{
	Q_OBJECT
public:
	Alg2();
	Alg2(MainWindow* wnd = 0);
	virtual ~Alg2();

	void init_();

	POINT3D* MatrixMethod(int tag);
	
	int R_A_JK(int j, int k);//получить произвольное время задержки из массива r[] (arrAnchVal)

	virtual bool ProcessAnchorDatagram(const ANC_MSG* datagram, POINT3D* retPoint);
	virtual void process_nav(const ANC_MSG* datagram, POINT3D* retPoint);
	int processMarkFilter(int tag_number);
	double mark_filter(int tag, int anc, double d);
	virtual int prepare_data(int tag);
	double find_max_m(void);
	void anc_dist(void);
	bool InverseMatrix(double A[][3], double A_[][3]);
	int QuadraticEquationAnalyzing(double alpha, double beta, double gamma);
private:
	bool c[8][3];//массив для поиска точек, НЕ лежащих на одной плоскости 
	double M[3][3];
	double B_[3][3];
	double K[4];
	bool s;
	int Ans;
	//double r[ANCHORS_NUMBER];//массив РАЗНОСТЕЙ расстояний приемников сигналов

	QString file_line_string;

	//MainWindow* mainWindow;
};

#endif // ALG2_H
