/* C program for soundscape generation. (C) P.B.L. Meijer 1996 */
/* hificode.c modified for camera input using OpenCV. (C) 2013 */
/* Last update: December 29, 2014; released under the Creative */
/* Commons Attribution 4.0 International License (CC BY 4.0),  */
/* see http://www.seeingwithsound.com/im2sound.htm for details */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <pthread.h>
#include <semaphore.h>
#include "opencv/highgui.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <windows.h>
#include <mmsystem.h>
#include <time.h>
#include <fstream>
#include <iostream>


using namespace cv;
using namespace std;

#define FNAME "hificode.wav"    /* User-defined parameters   */

#define BUF_SIZE 1
#define FL   500   /* Lowest  frequency (Hz) in soundscape   */
#define FH  5000   /* Highest frequency (Hz)                 */
#define FS 44100   /* Sample  frequency (Hz)                 */
#define T   0.90   /* Image to sound conversion time (s)     */
#define D      1   /* Linear|Exponential=0|1 distribution    */
#define HIFI   1   /* 8-bit|16-bit=0|1 sound quality         */
#define STEREO 0   /* Mono|Stereo=0|1 sound selection        */
#define DELAY  1   /* Nodelay|Delay=0|1 model   (STEREO=1)   */
#define FADE   1   /* Relative fade No|Yes=0|1  (STEREO=1)   */
#define DIFFR  1   /* Diffraction No|Yes=0|1    (STEREO=1)   */
#define BSPL   0   /* Rectangular|B-spline=0|1 time window   */
#define BW     0   /* 16|2-level=0|1 gray format in *P[]     */
#define CAM    1   /* Use OpenCV camera input No|Yes=0|1     */
#define VIEW   0   /* Screen view for debugging No|Yes=0|1   */

#define C_ALLOC(number, type) ((type *) calloc((number),sizeof(type)) )
#define TwoPi 6.283185307179586476925287
#define HIST  (1+HIFI)*(1+STEREO)
#define WHITE 1.00
#define BLACK 0.00



/* Soundscape resolution M rows x N columns */
#if CAM
/* 176 x 64 for live camera view */
#define M     64
#define N    136
#else
/* 64 x 64 for hard-coded image */
#define M     64
#define N     64
#endif


void sleep(float delay)
{

    time_t timer0,timer1;

    time(&timer0);
    do
    {

        time(&timer1);
    }
    while( (timer1-timer0) < delay);

}

FILE *fp, *ola;
unsigned long ir=0L, ia=9301L, ic=49297L, im=233280L;

void wi(unsigned int i)
{
    int b1,b0;
    b0=i%256;
    b1=(i-b0)/256;
    putc(b0,fp);
    putc(b1,fp);

}

void wl(long l)
{
    unsigned int i1,i0;
    i0=l%65536L;
    i1=(l-i0)/65536L;
    wi(i0);
    wi(i1);
}


double rnd(void)
{
    ir = (ir*ia+ic) % im;
    return ir / (1.0*im);
}


void Produtor1 ()
{

    /* OpenCV variables */
    VideoCapture cap;
    Mat gray, frame;

    int i, j, d=D, ss, key=0;
    long k=0L, l, ns=2L*(long)(0.5*FS*T), m=ns/N,sso=HIFI?0L:128L, ssm=HIFI?32768L:128L;
    double **A, a, t, dt=1.0/FS, *w, *phi0, s, y, yp, z, tau1, tau2, x, theta,
                      scale=0.5/sqrt((double)M), q, q2, r, sl, sr, tl, tr, yl, ypl, yr, ypr,
                      zl, zr, hrtf, hrtfl, hrtfr, v=340.0,  /* v = speed of sound (m/s) */
                                                  hs=0.20;  /* hs = characteristic acoustical size of head (m) */

    w    = C_ALLOC(M, double);
    phi0 = C_ALLOC(M, double);
    A    = C_ALLOC(M, double *);

    for (i=0; i<M; i++) A[i] = C_ALLOC(N, double);  /* M x N pixel matrix */

    /* Set lin|exp (0|1) frequency distribution and random initial phase */
    if (d)
        for (i=0; i<M; i++)
            w[i] = TwoPi * FL * pow(1.0* FH/FL,1.0*i/(M-1));
    else
        for (i=0; i<M; i++)
            w[i] = TwoPi * FL + TwoPi * (FH-FL)   *i/(M-1) ;

    for (i=0; i<M; i++) phi0[i] = TwoPi * rnd();

    int cam_id = 0;  /* First available OpenCV camera */
    /* Optionally override ID from command line parameter: prog.exe cam_id */

    cap.open(cam_id);
    if (!cap.isOpened())
    {
        // printf("resenha");
        fprintf(stderr,"Could not open camera %d\n", cam_id);
        exit(1);
    }
    printf("abriu camera\n");
    /* Setting standard capture size, may fail; resize later */

    cap.read(frame);  /* Dummy read needed with some devices */
    //cap.set(CV_CAP_PROP_FRAME_WIDTH , 176);
    //cap.set(CV_CAP_PROP_FRAME_HEIGHT, 144);
    cap.set(CV_CAP_PROP_FRAME_WIDTH , 128);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 128);

    if (VIEW)    /* Screen views only for debugging */
    {
        namedWindow("Large", CV_WINDOW_AUTOSIZE);
        namedWindow("Small", CV_WINDOW_AUTOSIZE);
    }
    int count = 0;
    bool bSuccess;
    while (key != 27)    /* Escape key */
    {
        bSuccess=true;
       for (int i=0; i<5 && bSuccess; i++)
			bSuccess = cap.read(frame);

        cap.read(frame);

        if (frame.empty())
        {
            /* Sometimes initial frames fail */
            fprintf(stderr, "Capture failed\n");
            key = waitKey((int)(100));
            continue;
        }
        printf("capturou frame\n");

        Mat tmp;
        cvtColor(frame,tmp,CV_BGR2GRAY);
        if (frame.rows != M || frame.cols != N)
            resize(tmp, gray, Size(N,M));

        else gray=tmp;

        if (VIEW)    /* Screen views only for debugging */
        {
            /* imwrite("hificodeLarge.jpg", frame); */
            imshow("Large", frame);
            /* imwrite("hificodeSmall.jpg", gray); */
            imshow("Small", gray);
        }

        key = waitKey((int)(10));

        if (CAM)    /* Set live camera image */
        {
            for (i=0; i<M; i++)
            {
                for (j=0; j<N; j++)
                {
                    int mVal=gray.at<uchar>(M-1-i,j)/16;

                    if (mVal == 0)
                        A[i][j]=0;
                    else
                        A[i][j]=pow(10.0,(mVal-15)/10.0);  /* 2dB steps */
                }
            }
        }



        /* Write 8/16-bit mono/stereo .wav file */
        fp = fopen(FNAME,"wb");
        fprintf(fp,"RIFF");         //chunkID
        wl(ns*HIST+36L);            //chunkSize
        fprintf(fp,"WAVEfmt ");     //format + subChunk1ID
        wl(16L);                    //subChunk1Size
        wi(1);                      //audioFormat  1 = PCM
        wi(STEREO?2:1);             //numChannels
        wl(0L+FS);                  //sampleRate
        wl(0L+FS*HIST);             //byteRate
        wi(HIST);                   //blockAlign
        wi(HIFI?16:8);              //bitsPerSample
        fprintf(fp,"data");         //subChunk2ID
        wl(ns*HIST);                //subChunk2Size

        //printf("arquivo setado\n");

        tau1 = 0.5 / w[M-1];
        tau2 = 0.25 * tau1*tau1;

        y = yl = yr = z = zl = zr = 0.0;

        /* Not optimized for speed */
        while (k < ns && !STEREO)
        {
            j = k / m;
            if (j>N-1)
                j=N-1;

            s = 0.0;
            t = k * dt;

            if (k < ns/(5*N))
            {
                s = (2.0*rnd()-1.0) / scale;  /* "click" */
            }
            else
            {
                for (i=0; i<M; i++)
                {
                    a = A[i][j];  /* Rectangular time window */
                    s += a * sin(w[i] * t + phi0[i]);
                }
            }

            yp = y;
            y = tau1/dt + tau2/(dt*dt);
            y  = (s + y * yp + tau2/dt * z) / (1.0 + y);
            z = (y - yp) / dt;
            l  = sso + 0.5 + scale * ssm * y; /* y = 2nd order filtered s */
            if (l >= sso-1+ssm) l = sso-1+ssm;
            if (l < sso-ssm) l = sso-ssm;
            ss = (unsigned int) l;

            if (HIFI)
                wi(ss);
            else
                putc(ss,fp);


            k++;
        }


        fclose(fp);

        printf("a\n");
        PlaySound(TEXT("hificode.wav"), NULL, SND_ASYNC );  /* Play the soundscape */
        printf("b\n");

        k=0;  /* Reset sample count */
    }


}


int main(int argc, char *argv[])
{

    Produtor1();


    return(0);
}
