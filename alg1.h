#ifndef ALG1_H
#define ALG1_H

#include "alg.h"
//#include "mainwindow.h"

//class MainWindow;
//class Alg;
//#include "datagram.h"
class Alg;

/*
* This class is intended for math processing of the input messages from 8 anchors as series of time delays in picoseconds
*/
class Alg1 : public Alg
{
	Q_OBJECT
public:
	Alg1(MainWindow* wnd = 0);
	virtual ~Alg1();

	void init_();

	POINT3D* DirectCalculationMethod(int tag);

	int getAnyJinAExcludeAi(int i);
	bool getarrJinAExcludeAs(int s);
	int getTA_JK(int j, int k);
	virtual bool ProcessAnchorDatagram(const ANC_MSG* datagram, POINT3D* retPoint);
	virtual void process_nav(const ANC_MSG* datagram, POINT3D* retPoint);
	int processMarkFilter(int tag_number);
	double mark_filter(int tag, int anc, double d);
	virtual int prepare_data(int tag);
	double find_max_m(void);
	void anc_dist(void);
private:
	int a_[4];//дополнительный массив	
	int arrJ[3];//массив для хранения индексов массива a[] за исключением индекса элемента a[s]
	QString file_line_string;

	//MainWindow* mainWindow;
};

#endif // ALG1_H
