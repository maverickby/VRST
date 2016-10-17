#include "Alg2.h"
#include <cmath>
#include <QDebug>

#define BUFFER_SIZE 128

Alg2::Alg2(MainWindow* wnd):Alg(wnd)
{
	//mainWindow = wnd;
	//init();
	init_();
}

void Alg2::init_()
{
	c[0][0] = 0; c[0][1] = 0; c[0][2] = 1;
	c[1][0] = 1; c[1][1] = 0; c[1][2] = 1;
	c[2][0] = 1; c[2][1] = 1; c[2][2] = 1;
	c[3][0] = 0; c[3][1] = 1; c[3][2] = 1;
	c[4][0] = 0; c[4][1] = 0; c[4][2] = 0;
	c[5][0] = 1; c[5][1] = 0; c[5][2] = 0;
	c[6][0] = 1; c[6][1] = 1; c[6][2] = 0;
	c[7][0] = 0; c[7][1] = 1; c[7][2] = 0;
	
	//clear matrix
	for (int i = 0; i < 3; i++)
	{
		K[i] = 0;
		for (int j = 0; j < 3; j++)
		{
			M[i][j] = 0;
			B_[i][j] = 0;
		}
	}
	memset(arrAnchVal, 0, sizeof(arrAnchVal));//clear trash in the array
}

Alg2::~Alg2()
{

}

int Alg2::processMarkFilter(int tag_number)
{
	int i;
	int j = 0;

	for (i = 0; i < ANCHORS_NUMBER; i++)
	{
		//if (fabs(m_marks[i]) < XY_DIMENSION)   // if delta > XY_DIMENSION  -  ignore it
		{
			m_marks[i] = mark_filter(tag_number, i, m_marks[i]);

			j++;
		}
	}
	return(j);
}

//фильтр первого порядка (RC), фильтр отметок
// marks filter
double Alg2::mark_filter(int tag, int anc, double d)
{
	static double f[TAGS_NUMBER][ANCHORS_NUMBER];
	double k = 20.0;
	f[tag][anc] = f[tag][anc] + d - (f[tag][anc] / k);
	return(f[tag][anc] / k);
}

bool Alg2::ProcessAnchorDatagram(const ANC_MSG* datagram, POINT3D* retPoint)
{
	int i;
	if (datagram->addr == 0)  // first packet in the series, process previous data
	{
		//disp_series(datagram->sync_n);		   //display sync_n
		process_nav(datagram, retPoint); //prepare data and do navigation procedure
		memset(t_marks, 0, sizeof(t_marks));	   //clear marks
		sync_series = datagram->sync_n;		   //new #
	}
	// get new data for the anchor
	if (datagram->sync_n == sync_series)
	{
		// copy marks
		for (i = 0; i < TAGS_NUMBER; i++)
			memcpy((char *)&t_marks[i][datagram->addr], datagram->time_mark[i], 5);
	}
	//check for sensors data, NOT IMPLEMENTED YET
	if (datagram->length >(TAGS_NUMBER * 5))
	{/*
	 n = datagram->sd_tag;
	 for(i = 0; i < datagram->sd_tags; i++)
	 {
	 memcpy(&accel_gyro[n], &datagram->sens_data[i], 12);
	 disp_accel_gyro(n);
	 n ++;
	 if(n >= TAGS_NUMBER)
	 n = 0;
	 }*/
	}

	//retPoint = DirectCalculationMethod(double t11,double t21,double t31,double t41,double t51,double t61,double t71,double t81);
	return true;
}

void Alg2::process_nav(const ANC_MSG* datagram, POINT3D* retPoint)
{
	int i, count_anchors_ret;
	for (i = 0; i < TAGS_NUMBER; i++)         // 0..14
	{
		if (prepare_data(i))
		{
			//make mark filter data processing
			count_anchors_ret = processMarkFilter(i);

			if (count_anchors_ret<4)//wrong situation !
				return;

			//call navigation Algorithm here
			retPoint = MatrixMethod(i);

			//show data
			mainWindow->SetOutput2(tr("Anchor X: %1").arg(retPoint->x),tr("Anchor Y: %1").arg(retPoint->y), tr("Anchor Z: %1").arg(retPoint->z));

			//write results to the file
			WriteToFile(datagram, retPoint,mainWindow->getCoordFile2());
		}
	}
}

// distance anchor i to sync anchor 0 in DWT_TIME_UNITS
void Alg2::anc_dist(void)
{
	int i;
	for (i = 0; i < ANCHORS_NUMBER; i++)
		anc0dist[i] = sqrt(pow(anchor[i].x - anchor[0].x, 2) +
			pow(anchor[i].y - anchor[0].y, 2) +
			pow(anchor[i].z - anchor[0].z, 2)) / SPEED_OF_LIGHT / DWT_TIME_UNITS;
}

// ************  Time marks normalization functions

// to find maximum in m_marks array
double Alg2::find_max_m(void)
{
	double a = m_marks[0];
	int i;
	for (i = 1; i < ANCHORS_NUMBER; i++)
		if (m_marks[i] > a)
			a = m_marks[i];
	return(a);
}

// copy t_mark for selected tag to m_mark
// check and fix overflow
// change time from DWT_TIME_UNITS to METERS
// change absolute mark value to delta (markN - mark0)
// return 0 if no data for master anchor (#0)
int Alg2::prepare_data(int tag)
{
	int i;
	double a, d0;
	d0 = 0;
	if (t_marks[tag][0] == 0)  //no data for master anchor
		return(0);
	// fix time marks using distance to master	(work)
	for (i = 0; i < ANCHORS_NUMBER; i++)
	{
		if (t_marks[tag][i] != 0)
		{
			m_marks[i] = (double)t_marks[tag][i];
			if (adj == 0)     // work mode
				m_marks[i] += anc0dist[i];
		}
		else
			m_marks[i] = 0;  // no data
	}
	// check and fix 5 bytes numbers overflow
	a = find_max_m();
	for (i = 0; i < ANCHORS_NUMBER; i++)
	{
		if ((m_marks[i]) && ((a - m_marks[i]) > HALF_T5))
			m_marks[i] += MAX_T5;
		if (m_marks[i])	   // 0 = no data
		{
			// to meters
			// array ant_delay  - meters
			//здесь разницы расстояний сигнала от маяка до i-го передатчика относительно
			//первого передатчика, в метрах
			m_marks[i] = m_marks[i] * SPEED_OF_LIGHT * DWT_TIME_UNITS + ant_delay[i];
			if (i == 0)
				d0 = m_marks[0];
			//change absolute marks  to delta (mark[i] - mark[0])
			m_marks[i] = m_marks[i] - d0;
		}
		//else
		//    m_marks[i] = 10;   // set to 10m - will be removed
	}
	return(1);
}

POINT3D* Alg2::MatrixMethod(int tag)
{
	double x, y, z;
	int l, r_, r1,r2, p, i,j;//r_ используется вместо r
	double alpha, beta, gamma;
	double A, B, C, D, E, F;
	double zl, xl, yl, xMinus, xPlus, yMinus, yPlus, zMinus, zPlus;	

	//здесь разницы расстояний от маяка до i-го передатчика относительно
	//первого передатчика, в метрах
	arrAnchVal[0] = m_marks[0];
	arrAnchVal[1] = m_marks[1];
	arrAnchVal[2] = m_marks[2];
	arrAnchVal[3] = m_marks[3];
	arrAnchVal[4] = m_marks[4];
	arrAnchVal[5] = m_marks[5];
	arrAnchVal[6] = m_marks[6];
	arrAnchVal[7] = m_marks[7];
	
	zl = xl = yl = xMinus = xPlus = yMinus = yPlus = zMinus = zPlus = 0;
	x = y = z = 0;

	a[0] = 0; a[1] = 1; a[2] = 2; a[3] = 3;//индексы приемников, начиная с 0
	l = 0; p = 3;

	//11, begin of the main cycle
	while (p >= 0)
	{
		s = 0;
		for (j=0;j<3;j++)
		{
			for (i=0;i<=3;i++)
			{
				s = s ^ c[a[i]][j];
			}			
			if (s!=0)						
				goto step22;			
		}
		goto step78;
step22:
		for (i = 1; i < 4; i++)//1..3
		{
			M[i - 1][0] = u[a[i]] - u[a[0]];
			M[i - 1][1] = v[a[i]] - v[a[0]];
			M[i - 1][2] = w[a[i]] - w[a[0]];
			K[i] = (u[a[i]] - u[a[0]])*(u[a[i]] - u[a[0]]) + (v[a[i]] - v[a[0]])*(v[a[i]] - v[a[0]]) + (w[a[i]] - w[a[0]])*(w[a[i]] - w[a[0]]);
		}

		//28
		if (InverseMatrix(M, B_) == false)
			continue;
		A = -( B_[0][0] * R_A_JK(a[1], a[0]) + B_[0][1] * R_A_JK(a[2], a[0]) + B_[0][2] * R_A_JK(a[3], a[0]) );
		C = -( B_[1][0] * R_A_JK(a[1], a[0]) + B_[1][1] * R_A_JK(a[2], a[0]) + B_[1][2] * R_A_JK(a[3], a[0]) );
		E = -( B_[2][0] * R_A_JK(a[1], a[0]) + B_[2][1] * R_A_JK(a[2], a[0]) + B_[2][2] * R_A_JK(a[3], a[0]));
		B = 0.5* (B_[0][0] * (K[1] - R_A_JK(a[1], a[0])*R_A_JK(a[1], a[0])) + B_[0][1] * (K[2] - R_A_JK(a[2], a[0])*R_A_JK(a[2], a[0])) + B_[0][2] * (K[3] - R_A_JK(a[3], a[0])*R_A_JK(a[3], a[0])) );		
		D = 0.5* (B_[1][0] * (K[1] - R_A_JK(a[1], a[0])*R_A_JK(a[1], a[0])) + B_[1][1] * (K[2] - R_A_JK(a[2], a[0])*R_A_JK(a[2], a[0])) + B_[1][2] * (K[3] - R_A_JK(a[3], a[0])*R_A_JK(a[3], a[0])));
		F = 0.5* (B_[2][0] * (K[1] - R_A_JK(a[1], a[0])*R_A_JK(a[1], a[0])) + B_[2][1] * (K[2] - R_A_JK(a[2], a[0])*R_A_JK(a[2], a[0])) + B_[2][2] * (K[3] - R_A_JK(a[3], a[0])*R_A_JK(a[3], a[0])));
		alpha = A*A + C*C + E*E - 1;
		beta = A*B + C*D + E*F;
		gamma = B*B + D*D + F*F;
		//38
		Ans = QuadraticEquationAnalyzing(alpha, beta, gamma);
		if (Ans == 0)
			goto step78;
		if (Ans == 1)
		{
			l = l + 1;
			r_ = -gamma / (2 * beta);
			xl = u[a[0]] + A*r_ + B;
			yl = v[a[0]] + C*r_ + D;
			zl = w[a[0]] + E*r_ + F;
			x = x + xl;
			y = y + yl;
			z = z + zl;
			goto step78;
		}
		//48
		if (Ans == 2)
		{
			l = l + 1;
			r_ = -beta / alpha;
			xl = u[a[0]] + A*r_ + B;
			yl = v[a[0]] + C*r_ + D;
			zl = w[a[0]] + E*r_ + F;
			x = x + xl;
			y = y + yl;
			z = z + zl;
			goto step78;
		}
		if (Ans == 3)
		{
			l = l + 1;
			r_ = (-beta - sqrt(beta*beta-alpha*gamma))/ alpha;
			xl = u[a[0]] + A*r_ + B;
			yl = v[a[0]] + C*r_ + D;
			zl = w[a[0]] + E*r_ + F;
			x = x + xl;
			y = y + yl;
			z = z + zl;
			goto step78;
		}
		if (Ans == 4)
		{
			l = l + 1;
			r_ = (-beta + sqrt(beta*beta - alpha*gamma)) / alpha;
			xl = u[a[0]] + A*r_ + B;
			yl = v[a[0]] + C*r_ + D;
			zl = w[a[0]] + E*r_ + F;
			x = x + xl;
			y = y + yl;
			z = z + zl;
			goto step78;
		}
		if (Ans == 5)
		{
			l = l + 1;
			r1 = (-beta - sqrt(beta*beta - alpha*gamma)) / alpha;
			r2 = (-beta + sqrt(beta*beta - alpha*gamma)) / alpha;
			xMinus = u[a[0]] + A*r1 + B;
			yMinus = v[a[0]] + C*r1 + D;
			zMinus = w[a[0]] + E*r1 + F;
			xPlus = u[a[0]] + A*r2 + B;
			yPlus = v[a[0]] + C*r2 + D;
			zPlus = w[a[0]] + E*r2 + F;

			pt1->x = xMinus;
			pt1->y = yMinus;
			pt1->z = zMinus;
			pt2->x = xPlus;
			pt2->y = yPlus;
			pt2->z = zPlus;
			ptRet->x = 0;
			ptRet->y = 0;
			ptRet->z = 0;

			if (Pair_Analyzing(pt1, pt2, ptRet) == 0)
				goto step78;
			l = l + 1;
			Pair_Analyzing(pt1, pt2, ptRet);

			x += ptRet->x;
			y += ptRet->y;
			z += ptRet->z;
			goto step78;
		}
		//78
	step78: if (a[3] == 7)
				p--;
			else
			   p = 3;
			if (p >= 0)
			{
				for (int i = 3; i >= p; i--)
					a[i] = a[p] + i - p + 1;
			}

	}//end while

	x = x / l;
	y = y / l;
	z = z / l;

	p3d->x = x; p3d->y = y; p3d->z = z;
	return p3d;
}

bool Alg2::InverseMatrix(double A[][3], double A_[][3])
{
	double d1, d2, d3, det;

	d1 = A[0][0] * (A[1][1] * A[2][2] - A[2][1] * A[1][2]);
	d2 = A[0][1] * (A[2][0] * A[1][2] - A[1][0] * A[2][2]);
	d3 = A[0][2] * (A[1][0] * A[2][1] - A[2][0] * A[1][1]);
	det = d1 + d2 + d3;
	if (det == 0)
		return false;	
	A_[0][0] = (A[1][1] * A[2][2] - A[2][1] * A[1][2]) / det;
	A_[0][1] = (A[2][0] * A[1][2] - A[1][0] * A[2][2]) / det;
	A_[0][2] = (A[1][0] * A[2][1] - A[2][0] * A[1][1]) / det;
	A_[1][0] = (A[2][1] * A[0][2] - A[0][1] * A[2][2]) / det;
	A_[1][1] = (A[0][0] * A[2][2] - A[2][0] * A[0][2]) / det;
	A_[1][2] = (A[2][0] * A[0][1] - A[0][0] * A[2][1]) / det;
	A_[2][0] = (A[0][1] * A[1][2] - A[1][1] * A[0][2]) / det;
	A_[2][1] = (A[1][0] * A[0][2] - A[0][0] * A[1][2]) / det;
	A_[2][2] = (A[0][0] * A[1][1] - A[1][0] * A[0][1]) / det;
	return true;
}

int Alg2::QuadraticEquationAnalyzing(double alpha, double beta, double gamma)
{
	double r,r1,r2,D;

	if (alpha == 0)
	{
		if (beta == 0)
			return true;
		if (beta != 0)
		{
			r = -gamma / (2 * beta);
			if (r > 0)
				return 1;
			else
				return 0;
		}
	}
	//13
	if (alpha != 0)
	{
		D = beta*beta - alpha*gamma;
		if (D<0)
			return 0;
		if (D == 0)
		{
			r = -beta / alpha;
			if (r > 0)
				return 2;
			else
				return 0;
		}
		if (D > 0)
		{
			r1 = (-beta - sqrt(D)) / alpha;
			r2 = (-beta + sqrt(D)) / alpha;
		}
		if (r1 > 0 && r2 < 0)
			return 3;
		if (r1 < 0 && r2 > 0)
			return 4;
		if (r1 > 0 && r2 > 0)
			return 5;
		else
			return 0;
	}
}

//получить произвольное время задержки из массива r[] (arrAnchVal)
int Alg2::R_A_JK(int j, int k)
{
	if (j<0 || j>7 || k<0 || k>7)
	{
		qDebug("R_A_JK: wrong index !");
		return -1;
	}
	else
	{
		return arrAnchVal[a[j]] - arrAnchVal[a[k]];
	}
}