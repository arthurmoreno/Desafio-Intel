#include <cstdlib>
#include <iostream>
#include <signal.h>
#include <sstream>
#include "opencv/highgui.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "stereoFunction.hpp"


using namespace std;
using namespace cv;

#define limiarCentimetros 100


typedef enum {CALIBRANDO, IMAGEMSTEREO, NAVEGANDO, STANDBY, BATERIA, DESLIGANDO} State;
bool hasCamera1 = true, hasCamera2 = true;
double dWidth, dHeight;

int botaoLigarIsPressed()
{

    cout << "Verifica se o botao de StandBy foi pressionado" << endl;
    //cout << "Se foi pressionado - retorna 1\nSe nao retorna 0" << endl;
    return 0;

}

int botaoStandByIsPressed()
{

    cout << "Verifica se o botao de StandBy foi pressionado" << endl;
    //cout << "Se foi pressionado - retorna 1\nSe nao retorna 0" << endl;
    return 0;
}

int botaoBateriaIsPressed()
{

    cout << "Verifica se o botao da Bateria foi pressionado" << endl;
    //cout << "Se foi pressionado - retorna 1\nSe nao retorna 0" << endl;
    return 0;
}

void emiteAlertaSonoroDistancia()
{


    cout << "Emite alerta sonoro quando a distancia esta proxima" << endl;

}

void emiteAlertaNivelDaBateria()
{

    cout << "Emite sinal sonoro informando nivel da bateria" << endl;

}

void emiteAlertaStandBy()
{

    cout << "Emite sinal sonoro informando que entrou no standBy" << endl;

}

void emiteAlertaDesligar()
{

    cout << "Emite sinal sonoro informando que o Sistema ira desligar" << endl;


}

#define OPENCV_BUFFER_SIZE 5
Mat takePicture(VideoCapture *cap)
{
    Mat frame;

    if (hasCamera1)
    {
        static int pictureCount = 1;
        cout << "\rTaking picture " << pictureCount << "...";
        bool bSuccess = true;
        //Clear the video buffer in order to get the a recent frame
        //only required for exporadic captures

        //for (int i=0; i<OPENCV_BUFFER_SIZE && bSuccess; i++)
        //	bSuccess = cap->read(frame);

        // read a new frame from video
        bSuccess = cap->read(frame);

        //now "frame" stores our image, writ it to the file.
        if (bSuccess)
        {
            cout << "ok.\n";
            pictureCount++;
        }
        else
            cout << "error\n";
    }


    return frame;
}

void wait(){

       for(int j = 0; j < 1200000000; j++); //sleep


}

VideoCapture openCamera(int n)
{

    VideoCapture cap(n); // open the video camera no. 0
    if (cap.isOpened())
    {
        hasCamera1 = hasCamera1 and true;
        cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
        dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
        dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
        cout << "Camera " << n <<" frame size : " << dWidth << " x " << dHeight << endl;
    }
    else
    {
        cout << "Cannot open the video cam" << endl;
        hasCamera1 = false;
        exit(1);
    }

    return cap;
}

void stereoControl(){





}

int main()
{
    bool botaoStandBy = false, botaoDesligar = false;
    State estadoAtual = CALIBRANDO, estadoAnterior;
    float distanciaObstaculo;

    Mat leftFrame, rightFrame, disparidade;

    VideoCapture rightCap = openCamera(0);  //Open rigth Camera
    //VideoCapture leftCap = openCamera(1);  //Open left Camera

    doStereo();

    while(!botaoDesligar)
    {

        if(botaoStandByIsPressed())
        {
            botaoStandBy = not botaoStandBy; //A cada vez que apertar o botao ativa/desativa

            if(botaoStandBy == true)
                estadoAtual = STANDBY;      //talvez tenha que deixar uma thread rodando com essa funcao
        }

        if(botaoBateriaIsPressed())
        {

            estadoAnterior = estadoAtual;
            estadoAtual = BATERIA;
        }

        if(botaoLigarIsPressed())
        {
            estadoAtual = DESLIGANDO;
        }

        switch(estadoAtual)
        {

        case CALIBRANDO:
            cout << "Verifica se as cameras estão ok !" << endl;
            cout << "Executa o algoritmo de calibragem das cameras" <<endl;
            cout << "Cameras calibradas"<<endl;

            estadoAtual = IMAGEMSTEREO;
            break;

        case IMAGEMSTEREO:
            cout << "Captura Imagem esquerda"<<endl;
            //leftFrame = takePicture(&leftCap);
            cout << "Captura Imagem direita" << endl;
            rightFrame = takePicture(&rightCap);
            cout << "Executando algoritmo stereoImage" << endl;
            disparidade = doStereo();



            if(distanciaObstaculo < limiarCentimetros)
                emiteAlertaSonoroDistancia();

            cout << "Imagem obtida" << endl;

            estadoAtual = NAVEGANDO;

            break;

        case NAVEGANDO:

            cout << "Executa Algoritmo de conversao da imagem em frequencia"<< endl;
            cout << "Converte frequencia em som      --" << endl;
            cout << "Converte frequencia em vibracao -- "<< endl;   //PARALELIZAR

            estadoAtual = IMAGEMSTEREO;
            break;

        case STANDBY:
            emiteAlertaStandBy();
            cout << "Desativando alguns componentes" << endl;

            if(botaoStandBy == false) // Se o botao foi apertado novamente -> saiu de true para false
                estadoAtual = IMAGEMSTEREO;

            break;

        case BATERIA:

            emiteAlertaNivelDaBateria();
            estadoAtual = estadoAnterior;
            break;

        case DESLIGANDO:
            emiteAlertaDesligar();
            cout << "Desligando componentes" << endl;
            botaoDesligar = true;
            break;

        }

        wait();
        //break;
    }

    return 0;
}



