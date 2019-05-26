#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>

#define PI 3.14159265

//atstumai centimetrais

#define kampas1 85
#define kampas2 55
#define kampas3 22
#define kampas4 -20
#define kampas5 -58
#define kampas6 -80
#define tarpasX1 0
#define tarpasY1 60
#define tarpasX2 50
#define tarpasY2 70
#define tarpasX3 80
#define tarpasY3 30
#define tarpasX4 80
#define tarpasY4 -30
#define tarpasX5 50
#define tarpasY5 -70
#define tarpasX6 0
#define tarpasY6 -60
#define k 300

#define MaxSensor 200 //pakeisti pagal realu max atstuma



double koor1[2];
double koor2[2];
double koor3[2];
double koor4[2];
double koor5[2];
double koor6[2];
double kampas;

double INTERCEPT, SLOPE;
double intercept[5];
double slope[5];
double ATSTUMAS;
double atstumas[5];
float tiksloKoor[5];
bool diskrNeig;
int errorNr0; //kai deltaY=0 - horizontali tiese
int errorNr1; //kai deltaX=0 - vertikali tiese
int errorNr2; //kai alpha = 90;


void ilgisIkoordinate(uint16_t ilgis1, uint16_t ilgis2, uint16_t ilgis3, uint16_t ilgis4, uint16_t ilgis5, uint16_t ilgis6)
{
    double x1,x2,y1,y2,x3,x4,y4,y3,x5,x6,y5,y6;
    x1 = tarpasX1 + ilgis1 * cos(kampas1 * PI / 180);
    x2 = tarpasX2 + ilgis2 * cos(kampas2 * PI / 180);
    x3 = tarpasX3 + ilgis3 * cos(kampas3 * PI / 180);
    x4 = tarpasX4 + ilgis4 * cos(kampas4 * PI / 180);
    x5 = tarpasX5 + ilgis5 * cos(kampas5 * PI / 180);
    x6 = tarpasX6 + ilgis6 * cos(kampas6 * PI / 180);
    y1 = tarpasY1 + ilgis1 * sin(kampas1 * PI / 180);
    y2 = tarpasY2 + ilgis2 * sin(kampas2 * PI / 180);
    y3 = tarpasY3 + ilgis3 * sin(kampas3 * PI / 180);
    y4 = tarpasY4 + ilgis4 * sin(kampas4 * PI / 180);
    y5 = tarpasY5 + ilgis5 * sin(kampas5 * PI / 180);
    y6 = tarpasY6 + ilgis6 * sin(kampas6 * PI / 180);

    koor1[0] = x1;
    koor1[1] = y1;
    koor2[0] = x2;
    koor2[1] = y2;
    koor3[0] = x3;
    koor3[1] = y3;
    koor4[0] = x4;
    koor4[1] = y4;
    koor5[0] = x5;
    koor5[1] = y5;
    koor6[0] = x6;
    koor6[1] = y6;
}


void sienosLygtis1_2()
{
    double deltaY, deltaX, atstumasX, atstumasY, slopePerpendicular;
    koor2[0] = (double)round(koor2[0]*1000)/1000;
    koor1[0] = (double)round(koor1[0]*1000)/1000;
    koor2[1] = (double)round(koor2[1]*1000)/1000;
    koor1[1] = (double)round(koor1[1]*1000)/1000;
    deltaX = koor2[0] - koor1[0];
    deltaY = koor2[1] - koor1[1];
    if(deltaX!=0 && deltaY!=0)
    {
        slope[0] = deltaY / deltaX;
        intercept[0] = (koor1[1] - slope[0] * koor1[0]);
        slopePerpendicular = -1 / slope[0];
        atstumasX = intercept[0] / (slopePerpendicular - slope[0]);
        atstumasY = slopePerpendicular * atstumasX;
        atstumas[0] = sqrt(atstumasX*atstumasX+atstumasY*atstumasY);
    }
    else if(deltaX==0 && deltaY!=0)
    {
        atstumas[0]=koor1[0];
        errorNr1 = 1;
        printf("errorNr1 pirmos tieses\n");
    }
    else if(deltaY==0 && deltaX!=0)
    {
        atstumas[0]=koor1[1];
        errorNr0 = 1;
        printf("errorNr0 pirmos tieses\n");
    }
}


void sienosLygtis2_3()
{
    double deltaY, deltaX, atstumasX, atstumasY, slopePerpendicular;
    koor3[0] = (double)round(koor3[0]*1000)/1000;
    koor2[0] = (double)round(koor2[0]*1000)/1000;
    koor3[1] = (double)round(koor3[1]*1000)/1000;
    koor2[1] = (double)round(koor2[1]*1000)/1000;
    deltaX = koor3[0] - koor2[0];
    deltaY = koor3[1] - koor2[1];
    if(deltaX!=0 && deltaY!=0)
    {
        slope[1] = deltaY / deltaX;
        intercept[1] = (koor2[1] - slope[1] * koor2[0]);
        slopePerpendicular = -1 / slope[1];
        atstumasX = intercept[1] / (slopePerpendicular - slope[1]);
        atstumasY = slopePerpendicular * atstumasX;
        atstumas[1] = sqrt(atstumasX*atstumasX+atstumasY*atstumasY);
    }
    else if(deltaX==0 && deltaY!=0)
    {
        atstumas[1]=koor2[0];
        errorNr1 = 1;
        printf("errorNr1 antros tieses\n");
    }
    else if(deltaY==0 && deltaX!=0)
    {
        atstumas[1]=koor2[1];
        errorNr0 = 1;
        printf("errorNr0 antros tieses\n");
    }
}


void sienosLygtis3_4()
{
    double deltaY, deltaX, atstumasX, atstumasY, slopePerpendicular;
    koor4[0] = (double)round(koor4[0]*1000)/1000;
    koor3[0] = (double)round(koor3[0]*1000)/1000;
    koor4[1] = (double)round(koor4[1]*1000)/1000;
    koor3[1] = (double)round(koor3[1]*1000)/1000;
    deltaX = koor4[0] - koor3[0];
    deltaY = koor4[1] - koor3[1];
    if(deltaX!=0 && deltaY!=0)
    {
        slope[2] = deltaY / deltaX;
        intercept[2] = (koor3[1] - slope[2] * koor3[0]);
        slopePerpendicular = -1 / slope[2];
        atstumasX = intercept[2] / (slopePerpendicular - slope[2]);
        atstumasY = slopePerpendicular * atstumasX;
        atstumas[2] = sqrt(atstumasX*atstumasX+atstumasY*atstumasY);
    }
    else if(deltaX==0 && deltaY!=0)
    {
        atstumas[2]=koor3[0];
        errorNr1 = 1;
        printf("errorNr1 trecios tieses\n");
    }
    else if(deltaY==0 && deltaX!=0)
    {
        atstumas[2]=koor3[1];
        errorNr0 = 1;
        printf("errorNr0 trecios tieses\n");
    }
}


void sienosLygtis4_5()
{
    double deltaY, deltaX, atstumasX, atstumasY, slopePerpendicular;
    koor5[0] = (double)round(koor5[0]*1000)/1000;
    koor4[0] = (double)round(koor4[0]*1000)/1000;
    koor5[1] = (double)round(koor5[1]*1000)/1000;
    koor4[1] = (double)round(koor4[1]*1000)/1000;
    deltaX = koor5[0] - koor4[0];
    deltaY = koor5[1] - koor4[1];
    if(deltaX!=0 && deltaY!=0)
    {
        slope[3] = deltaY / deltaX;
        intercept[3] = (koor4[1] - slope[3] * koor4[0]);
        slopePerpendicular = -1 / slope[3];
        atstumasX = intercept[3] / (slopePerpendicular - slope[3]);
        atstumasY = slopePerpendicular * atstumasX;
        atstumas[3] = sqrt(atstumasX*atstumasX+atstumasY*atstumasY);
    }
    else if(deltaX==0 && deltaY!=0)
    {
        atstumas[3]=koor4[0];
        errorNr1 = 1;
        printf("errorNr1 ketvirtos tieses\n");
    }
    else if(deltaY==0 && deltaX!=0)
    {
        atstumas[3]=koor4[1];
        errorNr0 = 1;
        printf("errorNr0 ketvirtos tieses\n");
    }
}


void sienosLygtis5_6()
{
    double deltaY, deltaX, atstumasX, atstumasY, slopePerpendicular;
    koor6[0] = (double)round(koor6[0]*1000)/1000;
    koor5[0] = (double)round(koor5[0]*1000)/1000;
    koor6[1] = (double)round(koor6[1]*1000)/1000;
    koor5[1] = (double)round(koor5[1]*1000)/1000;
    deltaX = koor6[0] - koor5[0];
    deltaY = koor6[1] - koor5[1];
    if(deltaX!=0 && deltaY!=0)
    {
        slope[4] = deltaY / deltaX;
        intercept[4] = (koor5[1] - slope[4] * koor5[0]);
        slopePerpendicular = -1 / slope[4];
        atstumasX = intercept[4] / (slopePerpendicular - slope[4]);
        atstumasY = slopePerpendicular * atstumasX;
        atstumas[4] = sqrt(atstumasX*atstumasX+atstumasY*atstumasY);
    }
    else if(deltaX==0 && deltaY!=0)
    {
        atstumas[4]=koor5[0];
        errorNr1 = 1;
        printf("errorNr1 penktos tieses\n");
    }
    else if(deltaY==0 && deltaX!=0)
    {
        atstumas[4]=koor5[1];
        errorNr0 = 1;
        printf("errorNr0 penktos tieses\n");
    }
}


void menamaSienosLygtis()
{
    //dadeti atstuma k nuo sienos
    //randamas kampas ir pagal ji atstumas k atidedamas nuo y.
    double alpha, tmpr;
    alpha = atan(SLOPE) * 180.0 / PI;
    tmpr = k / cos(alpha * PI / 180); //cos niekada nebus 0 nes jei tiese vertikali sita funkcija nebus kvieciama, ziureti main
    if(INTERCEPT>0)//jeigu siena robotui is kaires
    {
        INTERCEPT = INTERCEPT - tmpr;
    }
    else if(INTERCEPT<=0)//jeigu siena is deznes
    {
        INTERCEPT = INTERCEPT + tmpr;
    }
}


void rastiTaska(double a_tieses, double b_tieses)
{
    double a, b, c, D, x1, x2, xP, yP;
    //y=slope*x+intercept -tiese; x^2+y^2=k^2 -apskritimas;
    //ax^2+bx+c=0 -saknim rasti reikalinga lygtis
    //tiese su apskritimu sulygine gauname
    a = a_tieses * a_tieses + 1;
    b = 2 * a_tieses * b_tieses;
    c = b_tieses * b_tieses - k * k;
    D = b * b - 4 * a *c;
    if(D>0)
    {
        x1 = (-b + sqrt(D)) / (2 * a); //vardiklis niekada nebus 0, ziureti auksciau
        x2 = (-b - sqrt(D)) / (2 * a);
        //if tam kad pasirinkti i judejimo taska esanti toliau roboto priekyje
        if(x1>x2)
        {
            xP = x1;
        }
        else
        {
            xP = x2;
        }
        yP = a_tieses * xP + b_tieses;
    }
    else if(D<0)
    {
        diskrNeig=true;
        D = 0;
        xP = -b / (2 * a);
        yP = a_tieses * xP + b_tieses;
        //cia rastas artimiausias taskas tieseje, bet reikia rasti artimiausia taska apskritimo,
        //esanti salia to tasko
        if(xP==0 && yP==0)
        {
            printf("erorras: x ir y = 0");
        }
        else
        {
            xP = xP * k / (sqrt(xP*xP+yP*yP));
            yP = yP * k / (sqrt(xP*xP+yP*yP));
        }
    }
    else if(D=0)
    {
        xP = -b / (2 * a);
        yP = a_tieses * xP + b_tieses;
    }
    tiksloKoor[0] = xP;
    tiksloKoor[1] = yP;
}

float *gautiKoordinates(uint16_t l1,uint16_t l2,uint16_t l3,uint16_t l4,uint16_t l5,uint16_t l6)
{
    double xP, yP;
    errorNr0 = 0;
    errorNr1 = 0;
    errorNr2 = 0;
    diskrNeig=false;
    //nuskaitome lazerio jutiklius: grazina 6 skaicius parodancius atsuma iki seinos
   
    //konvertuojame atstumus i koordinates
    ilgisIkoordinate(l1,l2,l3,l4,l5,l6);
    //printf("Pirma koordinate: %0.2lf,%0.2lf \nAntra koordinate: %0.2lf,%0.2lf \nTrecia koordinate: %0.2lf,%0.2lf \n", koor1[0], koor1[1], koor2[0], koor2[1], koor3[0], koor3[1]);
   // printf("Ketvirta koordinate: %0.2lf,%0.2lf \nPenkta koordinate: %0.2lf,%0.2lf \nSesta koordinate: %0.2lf,%0.2lf \n", koor4[0], koor4[1], koor5[0], koor5[1], koor6[0], koor6[1]);
    //rasti atstuma iki tiesiu
    sienosLygtis1_2();
  //  printf("\n*** atstumas iki 1_2 tieses: %0.2lf ***\n", atstumas[0]);
    sienosLygtis2_3();//siena roboto kaireje
  //  printf("\n*** atstumas iki 2_3 tieses: %0.2lf ***\n", atstumas[1]);
    sienosLygtis3_4();
  //  printf("\n*** atstumas iki 3_4 tieses: %0.2lf ***\n", atstumas[2]);
    sienosLygtis4_5();
  //  printf("\n*** atstumas iki 4_5 tieses: %0.2lf ***\n", atstumas[3]);
    sienosLygtis5_6();
  //  printf("\n*** atstumas iki 5_6 tieses: %0.2lf ***\n", atstumas[4]);
    //rasti trumpiausia atstuma iki tieses - arciausia esancia
    ATSTUMAS = 99999;
    int i;
    for(i = 0; i<5; i++)
    {
        if(ATSTUMAS>atstumas[i])
        {
            ATSTUMAS = atstumas[i];
            INTERCEPT = intercept[i];
            SLOPE = slope[i];
        }
    }
    //if(ATSTUMAS>k){
    //    ATSTUMAS=ATSTUMAS/2;
    //}
   // printf("\nSienos Lygtis: Y=%0.2lfx +b %0.2lf\n", SLOPE, INTERCEPT);

    //nuo sienos k cm esancios menamos tieses generavimas ir suradimas tikslo koordinates
    //pagal du atvejus: tiese normali, tiese visiskai vertikali

    if(errorNr1!=1 && errorNr2!=1) //siena nera visiskai vertikali
    {
    //    printf("sienos lygtis y = %0.3lf*x + %0.3lf \n",SLOPE,INTERCEPT);

        //surandame menama sienos linija
        menamaSienosLygtis();
        
      //  printf("menama sienos lygtis y = %0.3lf*x + %0.3lf \n",SLOPE,INTERCEPT);
        //dabar paziureime ar susikerta roboto nustatyas apskritimo perimetras k su minima linija vaziavimo
        // jeigu taip, vaziuoti i priekyje roboto esanti taska (x asyje teigiama), jeigu ne, vaziuoti iki artimiausio
        // t y prilyginame apskiritimo funkcija su gauta linijos funkcija
        rastiTaska(SLOPE,INTERCEPT);
      //  printf("\n******rezultatas******\nJudeti i taska x = %0.3lf,y = %0.3lf; \n",tiksloKoor[0],tiksloKoor[1]);
        if(diskrNeig=true){ 
            kampas=0; 
            tiksloKoor[0] = tiksloKoor[0]/2 ;
            tiksloKoor[1] = tiksloKoor[1]/2;
            diskrNeig=false;
        }
        kampas=atan(SLOPE)*180/3.14159;
        tiksloKoor[2] = kampas;
        tiksloKoor[3] = (float)SLOPE;
        tiksloKoor[4]= (float)INTERCEPT;
        // printf("\n******rezultatas******\nJudeti i taska x = %0.3lf,y = %0.3lf; \n",tiksloKoor[0],tiksloKoor[1]);
        //printf("slope: %f intercept %f\n",SLOPE,INTERCEPT);
    }
    else if(errorNr1==1) // sienos linija vertikali
    {
    double x1, y1;
        x1 = ATSTUMAS;
        //pataisyta tiese pagal k;
        x1=x1-k;
     //   printf("sienos lygtis x = %0.3lf\n",x);
        //gauname lygti y^2 + a(tai yra x)^2 = k^2 is cia
        double n;
        n = k*k - x1*x1;
        if(n>0)
        {
            y1 = sqrt(n);
            //kadangi reikia pasirinkti viena is tasku, judame i DESINE roboto prieko atzivlgiu
            y1 = -y1;
        }
        else if(n==0 || n <0) //jeigu menama siena eina kaip liestine per roboto judejimo perimetra
                              //arba yra priekyje jo, tada judeti i prieki
        {
            y1 = 0;
        }
        
        kampas=atan(SLOPE)*180/3.14159;
        tiksloKoor[0] = x1;
        tiksloKoor[1] = y1;
        tiksloKoor[2] = kampas;
        tiksloKoor[3] = (float)SLOPE;
        tiksloKoor[4]= (float)INTERCEPT;
        // printf("\n******rezultatas******\nJudeti i taska x = %0.3lf,y = %0.3lf; \n",tiksloKoor[0],tiksloKoor[1]);
       // printf("slope: %f intercept %f\n",SLOPE,INTERCEPT);
    }
    errorNr0=0;
    errorNr1=0;
    errorNr2=0;
    return tiksloKoor;
}
