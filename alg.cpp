#include "alg.h"
#include <cmath>
#include <QDebug>

POINT3D ancor_dflt[ANCHORS_NUMBER] = {				//default ancor positions
                           {0, 0, 2.41},
                           {3.02, 0, 2.41},
                           {3.02, 3.02, 2.41},
                           {0, 3.02, 2.41},
                           {0, 0, 0},
                           {3.02, 0, 0},
                           {3.02, 3.02, 0},
                           {0, 3.02, 0}
                          };

double ant_delay[ANCHORS_NUMBER] ={0.5,0.52,1.55,0.45,1,0.96,0.75,0.69};

/*Alg::Alg()
{
    init();
}*/

Alg::Alg(MainWindow* wnd)
{
    mainWindow = wnd;
    init();
}

void Alg::init()
{
    for (int i=0;i<4;i++)
    {
        arrJ[i]=0;
    }

    u[0]=0;u[1]=0;u[2]=3.02;u[3]=3.02;
    u[4]=0;u[5]=0;u[6]=3.02;u[7]=3.02;
    v[0]=0;v[1]=3.02;v[2]=3.02;v[3]=0;
    v[4]=0;v[5]=3.02;v[6]=3.02;v[7]=0;
    w[0]=0;w[1]=0;w[2]=0;w[3]=0;
    w[4]=2.41;w[5]=2.41;w[6]=2.41;w[7]=2.41;

    pt1 = new POINT3D();
    pt1->x=pt1->y=pt1->z=0;
    pt2 = new POINT3D();
    pt2->x=pt2->y=pt2->z=0;
    ptRet = new POINT3D();
    ptRet->x=ptRet->y=ptRet->z=0;
    p3d = new POINT3D();
    p3d->x=p3d->y=p3d->z=0;
    sync_series = -1;
    adj = 0;
    memset(t_marks, 0, sizeof(t_marks));//clear trash in the array (marks)
    memset(m_marks, 0, sizeof(m_marks));//clear trash in the array
    memset(anc0dist, 0, sizeof(anc0dist));//clear trash in the array

    memset(anchor, 0, sizeof(anchor));//clear trash in the array
    memset(tag, 0, sizeof(tag));//clear trash in the array
    memset(a, 0, sizeof(a));//clear trash in the array
    memset(a_, 0, sizeof(a_));//clear trash in the array
    memset(arrT, 0, sizeof(arrT));//clear trash in the array
}

Alg::~Alg()
{
   delete pt1;
   delete pt2;
   delete ptRet;
   delete pt1;
   delete pt2;
   delete p3d;
}

bool Alg::ProcessAnchorDatagram(const ANC_MSG* datagram, POINT3D* retPoint)
{
    int i, n;
     if(datagram->addr == 0)  // first packet in the series, process previous data
     {
         //disp_series(datagram->sync_n);		   //display sync_n
         process_nav(retPoint);					   //prepare data and do navigation procedure
         memset(t_marks, 0, sizeof(t_marks));	   //clear marks
         sync_series = datagram->sync_n;		   //new #
     }
// get new data for the ancor
     if(datagram->sync_n == sync_series)
     {
       // copy marks
       for(i = 0; i < TAGS_NUMBER; i++)
         memcpy((char *)&t_marks[i][datagram->addr], datagram->time_mark[i], 5);
     }
//check for sensors data, NOT IMPLEMENTED YET
     if(datagram->length > (TAGS_NUMBER * 5))
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

void Alg::process_nav(POINT3D* retPoint)
{
    int i, j;
    for(i = 0; i < TAGS_NUMBER; i++)         // 0..14
    {
       if(prepare_data(i))
       {
         //disp_data(i, m_marks);   // tag - 0, 1..
         //call navigation algorithm here
         //if(bancroft(i) > 0)
            //disp_loacation(i, tag[i].x, tag[i].y, tag[i].z);
           retPoint = DirectCalculationMethod(i);
       }
    }
}

// distance ancor i to sync ancor 0 in DWT_TIME_UNITS
void Alg::anc_dist(void)
{
    int i;
    for(i = 0; i < ANCHORS_NUMBER; i++)
        anc0dist[i] = sqrt(pow(anchor[i].x - anchor[0].x, 2) +
                           pow(anchor[i].y - anchor[0].y, 2) +
                           pow(anchor[i].z - anchor[0].z, 2)) / SPEED_OF_LIGHT / DWT_TIME_UNITS;
}

// ************  Time marks normalisation functions

// to find maximum in m_marks array
double Alg::find_max_m(void)
{
    double a = m_marks[0];
    int i;
    for(i = 1; i < ANCHORS_NUMBER; i ++)
        if(m_marks[i] > a)
            a = m_marks[i];
    return(a);
}

// copy t_mark for selected tag to m_mark
// check and fix overflow
// change time from DWT_TIME_UNITS to meters
// change absolute mark value to delta (markN - mark0)
// return 0 if no data for master ancor (#0)
int Alg::prepare_data(int tag)
{
    int i;
    double a, d0;
    if(t_marks[tag][0] == 0)  //no data for master anchor
        return(0);
// fix time marks using distance to master	(work)
    for(i = 0; i < ANCHORS_NUMBER; i++)
    {
       if(t_marks[tag][i] != 0)
       {
           m_marks[i] = (double)t_marks[tag][i];
           if(adj == 0)     // work mode
                m_marks[i] += anc0dist[i];
       }
       else
           m_marks[i] = 0;  // no data
    }
// check and fix 5 bytes numbers overflow
    a = find_max_m();
    for(i = 0; i < ANCHORS_NUMBER; i++)
     {
        if((m_marks[i]) && ((a - m_marks[i]) > HALF_T5))
            m_marks[i] += MAX_T5;
         if(m_marks[i])	   // 0 = no data
         {
            // to seconds
            // array ant_delay  - meters
           //здесь разницы времен получения сигнала от маяка до i-го передатчика относительно
           //первого передатчика, в секундах (пикосекундах)
           m_marks[i] = m_marks[i] * /*SPEED_OF_LIGHT **/ DWT_TIME_UNITS + ant_delay[i]/SPEED_OF_LIGHT;
           if(i == 0)
             d0 = m_marks[0];
// change absolute marks  to delta (mark[i] - mark[0])
           m_marks[i] = m_marks[i] - d0;
         }
         else
             m_marks[i] = 10;   // set to 10m - will be removed
     }
    return(1);
}

POINT3D* Alg::DirectCalculationMethod(int tag)
{
    double x,y,z;
    //POINT3D p3d;
    int l,p,i;    
    double t11,t21,t31,t41,t51,t61,t71,t81;
    int s,k;
    int count;
    int index_anyJ;
    double u21,u31,u41,v21,v31,v41,w21,w31,w41;
    double delta21,delta31,delta41,tau12,tau13,tau14,tau32,tau42;
    double alpha1,alpha2,beta1,beta2,gamma1,gamma2,g1,g2,c;
    double A,B,C,D,E,F,G,H,I;
    double zl,xl,yl,xMinus,xPlus,yMinus,yPlus,zMinus,zPlus;

    //int64    t_marks[N_TAGS][N_ANCORS];		//time marks
    /*t11 = (double)t_marks[tag][0];
    t21 = (double)t_marks[tag][1];
    t31 = (double)t_marks[tag][2];
    t41 = (double)t_marks[tag][3];
    t51 = (double)t_marks[tag][4];
    t61 = (double)t_marks[tag][5];
    t71 = (double)t_marks[tag][6];
    t81 = (double)t_marks[tag][7];*/

    //здесь разницы времен получения сигнала от маяка до i-го передатчика относительно
    //первого передатчика, в секундах (пикосекундах)
    t11 = m_marks[0];
    t21 = m_marks[1];
    t31 = m_marks[2];
    t41 = m_marks[3];
    t51 = m_marks[4];
    t61 = m_marks[5];
    t71 = m_marks[6];
    t81 = m_marks[7];
    c=SPEED_OF_LIGHT;

    arrT[0]=t11;
    arrT[1]=t21;
    arrT[2]=t31;
    arrT[3]=t41;
    arrT[4]=t51;
    arrT[5]=t61;
    arrT[6]=t71;
    arrT[7]=t81;

    if (t11 == 0 && t21 == 0 && t31 ==0 && t41 ==0 && t51 ==0  && t61 ==0 && t71 == 0 && t81 == 0)
    {
        x = 1.51; y = 1.51; z = 1.205;
        p3d->x = x;p3d->y = y;p3d->z = z;
        return p3d;
    }

    a[0] = 0; a[1] = 1; a[2] = 2; a[3] = 3;//индексы приемников, начиная с 0
    l = 0; p = 3;

    //7, begin of the main cycle
    while(p>=0)
    {
        for (i=0;i<=3;i++)
        {
            index_anyJ = getAnyJinAExcludeAi(i);
            if(arrT[a[i]] != arrT[index_anyJ])//getTAi1(i)
            {
                s = i;
                goto step15;
            }
         }
        goto step102;
step15: a_[0]=a[s];
        k=1;

        //17
        //для каждого j∈{a[1]; a[2]; a[3]; a[4]}\{a[s]}
        getarrJinAExcludeAs(s);// get arrJ[] here
        for( int x=0;x<3;x++)//0..2 потому что множество элементов a[] минус a[s]
        {
            a_[k]=arrJ[x];
                k++;           
        }
        //21
        u21=u[a_[1]]-u[a_[0]];
        u31=u[a_[2]]-u[a_[0]];
        u41=u[a_[3]]-u[a_[0]];
        v21=v[a_[1]]-v[a_[0]];
        v31=v[a_[2]]-v[a_[0]];
        v41=v[a_[3]]-v[a_[0]];
        w21=w[a_[1]]-w[a_[0]];
        w31=w[a_[2]]-w[a_[0]];
        w41=w[a_[3]]-w[a_[0]];
        delta21=u[a_[1]]*u[a_[1]]+v[a_[1]]*v[a_[1]]+w[a_[1]]*w[a_[1]]-
                (u[a_[0]]*u[a_[0]]+v[a_[0]]*v[a_[0]]+w[a_[0]]*w[a_[0]]);
        delta31=u[a_[2]]*u[a_[2]]+v[a_[2]]*v[a_[2]]+w[a_[2]]*w[a_[2]]-
                (u[a_[0]]*u[a_[0]]+v[a_[0]]*v[a_[0]]+w[a_[0]]*w[a_[0]]);
        delta41=u[a_[3]]*u[a_[3]]+v[a_[3]]*v[a_[3]]+w[a_[3]]*w[a_[3]]-
                (u[a_[0]]*u[a_[0]]+v[a_[0]]*v[a_[0]]+w[a_[0]]*w[a_[0]]);
        //33
        tau12= arrT[a_[0]]- arrT[a_[1]];
        tau13= arrT[a_[0]]- arrT[a_[2]];
        tau14= arrT[a_[0]]- arrT[a_[3]];
        tau32= arrT[a_[2]]- arrT[a_[1]];
        tau42= arrT[a_[3]]- arrT[a_[1]];

        alpha1=tau12*u31-tau13*u21;
        alpha2=tau12*u41-tau14*u21;
        beta1=tau12*v31-tau13*v21;
        beta2=tau12*v41-tau14*v21;

        if( (alpha1*beta2-alpha2*beta1) == 0)
            goto step102;
        //45
        gamma1=tau12*w31-tau13*w21;
        gamma2=tau12*w41-tau14*w21;
        g1=(c*c*tau12*tau13*tau32 + tau12*delta31 - tau13*delta21)/2;
        g2=(c*c*tau12*tau14*tau42 + tau12*delta41 - tau14*delta21)/2;
        A=(beta1*gamma2-beta2*gamma1)/(alpha1*beta2-alpha2*beta1);
        B=(beta2*g1-beta1*g2)/(alpha1*beta2-alpha2*beta1);
        C=(alpha2*gamma1-alpha1*gamma2)/(alpha1*beta2-alpha2*beta1);
        D=(alpha1*g2-alpha2*g1)/(alpha1*beta2-alpha2*beta1);
        E=(u21*A+v21*C+w21)/(c*tau12);
        F=(c*tau12)/2 + (2*(u21*B+v21*D)-delta21)/(c*tau12);
        G=A*A+C*C+1-E*E;
        H=2*(A*(B-u[a_[0]])+C*(D-v[a[0]])-w[a_[0]]-E*F);
        I=(B-u[a_[0]])*(B-u[a_[0]]) + (D-v[a_[0]])*(D-v[a_[0]])+
                w[a_[0]]*w[a_[0]]-F*F;
        //58
        if(G==0 && H!=0)
        {
            l++;
            zl=-I/H;
            xl=A*zl+B;
            yl=C*zl+D;
            //надо ли ? надо !
            x+=xl;
            y+=yl;
            z+=zl;
            /////////
            goto step102;
        }
        //65
        if(H==0 || (((H/(2*G))*(H/(2*G))-I/G)<0) )
            goto step102;
        zMinus=-(H/(2*G))-sqrt(((H/(2*G))*(H/(2*G))-I/G));
        zPlus=-(H/(2*G))+sqrt(((H/(2*G))*(H/(2*G))-I/G));
        //70
        if( zPlus<0 || zMinus>2.5 || (zMinus<0 && zPlus>2.5) )
            goto step102;
        //73
        xMinus=A*zMinus+B;
        xPlus=A*zPlus+B;
        //75
        if( (xMinus<0 && xPlus<0) || (xMinus>10 && xPlus>10) ||
                (xMinus<0 && xPlus>10) || (xPlus<0 && xMinus>10) )
            goto step102;

        yMinus=C*zMinus+D;
        yPlus=C*zPlus+D;
        if( (yMinus<0 && yPlus<0) || (yMinus>10 && yPlus>10) ||
                (yMinus<0 && yPlus>10) || (yPlus<0 && yMinus>10) )
            goto step102;
        //83
        if( xMinus<0 || xMinus>10 || yMinus<0 || yMinus>10 ||
                zMinus<0 || xMinus>2.5 )
        {
            l++;
            xl=xPlus;
            yl=yPlus;
            zl=zPlus;
            //надо ли ?
            x+=xl;
            y+=yl;
            z+=zl;
            /////////
            goto step102;
        }
        //90
        if( xPlus<0 || xPlus>10 || yPlus<0 || yPlus>10 ||
                zPlus<0 || zPlus>2.5 )
        {
            l++;
            xl=xMinus;
            yl=yMinus;
            zl=zMinus;
            //надо ли ?
            x+=xl;
            y+=yl;
            z+=zl;
            /////////
            goto step102;
        }
        //97
        pt1->x=xMinus;
        pt1->y=yMinus;
        pt1->z=zMinus;
        pt2->x=xPlus;
        pt2->y=yPlus;
        pt2->z=zPlus;
        ptRet->x=0;
        ptRet->y=0;
        ptRet->z=0;

        if(Pair_Analyzing(pt1,pt2,ptRet)==0)
            goto step102;

        l++;
        Pair_Analyzing(pt1,pt2,ptRet);

        x+=ptRet->x;
        y+=ptRet->y;
        z+=ptRet->z;

step102: if(a[3]==8)
            p--;
        else
            p=3;
        if(p>=0)
        {
            for(int i=3;i>=p;i--)
                a[i]=a[p]+i-p+1;
        }

    }//end while

    //for(int i=0;i<l;i++)
      //x+=x[i];
    x=x/l;
    //for(int i=0;i<l;i++)
      //y+=y[i];
    y=y/l;
    //for(int i=0;i<l;i++)
      //z+=z[i];
    z=z/l;

    p3d->x = x;p3d->y = y;p3d->z = z;
    return p3d;
}

bool Alg::Pair_Analyzing(const POINT3D* pt1,const POINT3D* pt2, POINT3D* ptRet)
{
    int s1,s2;
    double di1,dj1,di2,dj2;
    double tij;

    s1=0;s2=0;

    for(int i=0;i<7;i++)
    {
        for(int j=i+1;i<8;i++)
        {
            di1=sqrt( (u[i]-pt1->x)*(u[i]-pt1->x) + (v[i]-pt1->y)*(v[i]-pt1->y) +
                  (w[i]-pt1->z)*(w[i]-pt1->z) );
            dj1=sqrt( (u[j]-pt1->x)*(u[j]-pt1->x) + (v[j]-pt1->y)*(v[j]-pt1->y) +
                  (w[j]-pt1->z)*(w[j]-pt1->z) );
            tij=arrT[i]- arrT[j];
            if( ((tij>=0) && di1<dj1) || (tij<0 && di1>=dj1) )
            {
                s1=1;
                goto step12;
            }
        }
    }

    step12:
    for(int i=0;i<7;i++)
    {
        for(int j=i+1;i<8;i++)
        {
            di2=sqrt( (u[i]-pt2->x)*(u[i]-pt2->x) + (v[i]-pt2->y)*(v[i]-pt2->y) +
                  (w[i]-pt2->z)*(w[i]-pt2->z) );
            dj2=sqrt( (u[j]-pt2->x)*(u[j]-pt2->x) + (v[j]-pt2->y)*(v[j]-pt2->y) +
                  (w[j]-pt2->z)*(w[j]-pt2->z) );
            tij=arrT[i]- arrT[j];
            if( ((tij>=0) && di2<dj2) || (tij<0 && di2>=dj2) )
            {
                s2=1;
                goto step22;
            }
        }
    }
    step22:
    if( (s1==1 && s2==1) || (s1==0 && s2==0))
        return false;
    if(s1==1 && s2==0)
    {
        ptRet->x=pt2->x;
        ptRet->y=pt2->y;
        ptRet->z=pt2->z;
        return true;
    }
    if(s1==0 && s2==1)
    {
        ptRet->x=pt1->x;
        ptRet->y=pt1->y;
        ptRet->z=pt1->z;
        return true;
    }
}

//получить время задержки относительно передатчика 1 из массива arrT,
//используя индекс i из массива a[]
int Alg::getTAi1(int i)
{
    if(i<0 || i>3)
    {
        qDebug("wrong index i !");
        return -1;
    }
    else
        return arrT[a[i]];//брать a[i] как индекс для arrT !
}

//получить произвольное время задержки из массива a_[]
int Alg::getTA_JK(int j,int k)
{
    if(j<0 || j>3 || k<0 || k>3)
    {
        qDebug("wrong index !");
        return -1;
    }
    else
    {
        return arrT[a_[j]]- arrT[a_[k]];
    }
}

//вернуть j ∈{a[1]; a[2]; a[3]; a[4]} \ {a[i]}
int Alg::getAnyJinAExcludeAi(int i)
{
    if(i<0 || i>3)
    {
        qDebug("getAnyJinAExcludeAi: wrong index !");
        return -1;
    }
    else
    {
        for(int l=0;l<4;l++)
           if(a[l]!=a[i])
                return a[l];
    }
}


bool Alg::getarrJinAExcludeAs(int s)
{
    int indJ=0;
    if(s<0 || s>3)
    {
        qDebug("getarrJinAExcludeAs: wrong index !");
        return false;
    }
    else
    {
        for(int i=0;i<4;i++)
           if(a[i]!=a[s])
           {
               arrJ[indJ]=a[i];
               indJ++;
           }
        return true;
    }
}


