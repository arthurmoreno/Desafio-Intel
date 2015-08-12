/*
 * File:   main.cpp
 * Author: hans
 *
 * Created on 4 de Junho de 2015, 20:25
 */

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <signal.h>
#include <sstream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <string.h>


#include "math.h"
#include <wiringPi.h>
#include "robotAPI/Pins.h"
#include "robotAPI/Sonar.h"
#include "robotAPI/Motor.h"
#include "robotAPI/Encoder.h"
#include "robotAPI/KBAsync.h"
#include "robotAPI/Odometry.h"

#define nl "\n\r"
#define pi 3.1415
//#define DEBUG
using namespace std;
using namespace cv;
bool running;
bool hasCamera = true;
float distancia = 0;
float fdist = 20;

typedef enum {AVALIA, VERMELHO, VERDE, NENHUM} TRobotState;
typedef enum {AVANCA, ESTOURA, CONFERE, PERDEU} TAniquilaState;
typedef struct SRetornoPython {
	int identificador;
	float angulo;
}TRetornoPython;

Sonar sonar;
Encoder encoderL, encoderR;
Motor motorL, motorR;

float posInter[3];


//Detect ^C
void handleCTRLC(int s);
void setupCTRLCHandle();
string takePicture(VideoCapture *cap);
TRobotState updateStateRobot (int identificador);
TAniquilaState updateStateAniquila (TAniquilaState aniquilaState, VideoCapture *cap);
void getDataImg (TRetornoPython *retornoPython, VideoCapture *cap);
float to_positive_angle(float angle);
float menorAngulo(float target, float source);
void girarGraus(int grau);
void andarFrente(float distance);
void diferencial(float goal[3]);
//Funcoes para testes e debug
#ifdef DEBUG
TAniquilaState testeStateAniquile(VideoCapture *cap);
#endif

int procImagem(char tipo, VideoCapture *cap){
	
	
	string ss;
	ss = "kkkkkkkk";
	
	ss = takePicture(cap);
	
    int iLowH; 
	int iHighH;

	int iLowS; 
	int iHighS;

	int iLowV;
	int iHighV;

	Mat imgOriginal(320,240, -1);
    imgOriginal = imread(ss);
	if(!imgOriginal.data) {
		cout << "picture nao abriu!" <<endl;
		exit(0);
	}

    //namedWindow("Control", WINDOW_AUTOSIZE); //create a window called "Control"

 if(tipo == 'G'){
	iLowH = 47;  // Green
	iHighH = 92;

	iLowS = 82; 
	iHighS = 255;

	iLowV = 0;
	iHighV = 255;

 }else if(tipo == 'R'){
	iLowH = 170;  //Vermelho
	iHighH = 179;

	 iLowS = 150; 
	iHighS = 255;

	iLowV = 60;
	 iHighV = 255;
}

	int i, j, limiarDePixels = 1000;
	int countEsquerda = 0, countDireita = 0;
	int retorno=0;


		Mat imgHSV;
		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

		Mat imgThresholded;
		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
		
		//imshow("Thresholded Image", imgThresholded); //show the thresholded image
		//imshow("Original", imgOriginal); //show the original image
	
		for(i=0; i< imgThresholded.rows; i++){
			for(j=0; j<imgThresholded.cols/2; j++){
				//cout << imgThresholded.at<uchar>(i,j) << endl;
				if((int)imgThresholded.at<uchar>(i,j) > 200)
					countEsquerda += 1;
			}	
		}

		for(i=0; i <imgThresholded.rows ; i++){
			for(j=imgThresholded.cols/2; j<imgThresholded.cols; j++){
				//cout << imgThresholded.at<uchar>(i,j) << endl;
				if((int)imgThresholded.at<uchar>(i,j) > 200){
					countDireita += 1;
				
				}
			}	
		}
	
		cout << "Esquerda: " << countEsquerda << " Direita: " << countDireita << endl;
		
		if(countEsquerda - countDireita > limiarDePixels){
			//va para esquerda
			//cout << "Esquerda" << endl;
			retorno = 1;
		}
		else if(countDireita - countEsquerda > limiarDePixels){
			//va para direita
			//cout << "Direita" << endl;
			retorno = 2;
		}
		else if(abs(countDireita - countEsquerda) < limiarDePixels && (countDireita > 3500 || countEsquerda > 3500)) {
			//va para frente
			//cout << "Frente" << endl;
			retorno = 0;
		}
		else{
			//cout << "Nao existe" << endl;
			retorno = -1;
		}
	

	return retorno;   
}

int *controlProcImage(VideoCapture *cap){

	int *retorno;
	retorno = (int*)malloc(2*sizeof(int));
	
  	int retornoVerde = procImagem('G',cap);
  	int retornoVermelho = procImagem('R',cap);

	if(retornoVerde == 0){
		retorno[0] = 1;
		retorno[1] = 0;
	}
	else if(retornoVerde == 2 ){
		retorno[0] = 1; //direita
		retorno[1] = -8;
	}
	else if(retornoVerde == 1){
		retorno[0] = 1; //esquerda
		retorno[1] = 8;
	}
	else{
		if(retornoVermelho == 2){
			retorno[0] = 2;
			retorno[1] = 0;
		}
		else{
			retorno[0] = 0;
			retorno[1] = 0;
		}
	}

	return retorno;
}


int main(int argc, char** argv) {

	int *retornoProcIm;
	retornoProcIm = (int*)malloc(2*sizeof(int));
	TRobotState currentState = AVALIA;
	TAniquilaState aniquilaState = CONFERE;
	running = true;
	KBAsync kb;
	String path;
	int key;
	int contador_voltas = 0;
	int contador_voltas2 = 0;
	//float angulo;
	//int identificador;
	ifstream myfile;
	TRetornoPython dadosImagem;

	setupCTRLCHandle();

	if (PIN_MODE==PIN_BCM) {
		cout << "Pins in BCM mode." << nl;
		if (wiringPiSetupGpio()<0) {
			cout << "Could not setup GPIO pins" << nl;
			return -1;
		}
	} else {
		cout << "Pins in wiringPi mode." << nl;
		wiringPiSetup();
	}


	encoderL.setup(ENCODER_LEFT, LEFT_SIDE);
	encoderR.setup(ENCODER_RIGHT, RIGHT_SIDE);
	sonar.setup(SONAR_TRIGGER, SONAR_ECHO);
	motorL.setup(MOTOR_LEFT_A, MOTOR_LEFT_B, MOTOR_LEFT_E, &encoderL);
	motorR.setup(MOTOR_RIGHT_A, MOTOR_RIGHT_B, MOTOR_RIGHT_E, &encoderR);
	motorL.setK(1);
	motorR.setK(1);

	VideoCapture cap(0); // open the video camera no. 0
	if (cap.isOpened()) {
		hasCamera = true;
		cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
		cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
		double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
		double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
		cout << "Camera frame size : " << dWidth << " x " << dHeight << nl;
	} else {
		cout << "Cannot open the video cam" << nl;
		hasCamera = false;
	} 


//Caso nao esteja em modo debug, executa o codigo correto
#ifndef DEBUG
	while ((key=kb.getKey())!='q') {
		
		//cout << "\r" << flush << "Distance:" << fdist << "cm" << nl;
		switch (currentState) {
			case AVALIA:								//tira foto do ambiente ao redor e 
				cout << "AVALIA" << nl;										//define proxima acao
				retornoProcIm = controlProcImage(&cap);
				cout << "Retorno Proc Im ID: " << retornoProcIm[0] << "Retorno Proc Im ANGLE: " << retornoProcIm[1] << nl;
				currentState = updateStateRobot (retornoProcIm[0]); 	//a partir do angulo, define a próxima acao
			break;
			case NENHUM: 								// balao a vista.
				cout << "NENHUM" << nl;
				girarGraus(45);
				cout << "GIROU 45 GRAUS" << nl;
				if(contador_voltas2 > 7){
					cout << "Saindo do lugar\ndistancia do sonar: " << sonar.measureDistance() << nl;
					if(sonar.measureDistance() > 35){
						posInter[0] = 40;
						posInter[1] = 0;
						posInter[2] = 0;
						diferencial(posInter);
						contador_voltas2=0;
					}
				}
				
				currentState = AVALIA;
				
			break;
			
			case VERDE:									//apenas um balao verde, ataca.
				cout << "\nVISUALIZOU O VERDE\n" << nl;
				while (aniquilaState != PERDEU){
					cout << "\nANIQUILA STATE\n" << nl;
					aniquilaState = updateStateAniquila (aniquilaState,&cap);
				}
				currentState = AVALIA;					//nao ve mais verde, retorna para procurar
				aniquilaState = CONFERE;				//reinicia a maquina de estados 
			break;
			
			case VERMELHO:								//so baloes vermelhos, gira.
				cout << "VERMELHO" << nl;
				girarGraus(45);
				if(contador_voltas > 7){
					cout << "Saindo do lugar\ndistancia do sonar: " << sonar.measureDistance() << nl;
					if(sonar.measureDistance() > 35){
						posInter[0] = 45;
						posInter[1] = 0;
						posInter[2] = 0;
						diferencial(posInter);
						contador_voltas=0;
					}
					
				}
				contador_voltas++;
				currentState = AVALIA;
			break;

		}

		motorL.controlSpeed();
		motorR.controlSpeed();

		delayMicroseconds(100000);
	}

	motorL.stop();
	motorR.stop();
	cout << "\r\nExiting...\r\n";
#endif 

//Para testes e debug
#ifdef DEBUG	
	TAniquilaState estado;
	estado = testeStateAniquile(&cap);
	if (estado == CONFERE) cout << "\r\nConfere\r\n";
	else cout << "\r\nOutro\r\n";
#endif

	return 0;
}
//Funcoes para debug e testes
#ifdef DEBUG
TAniquilaState testeStateAniquile(VideoCapture *cap){
	
	TRetornoPython pyData;
	int *retornoProcIm;
	retornoProcIm = controlProcImage();
	TAniquilaState state = CONFERE;

	//getDataImg(&pyData,cap);
	cout << pyData.angulo;
	cout << "\r\n";
	cout << pyData.identificador;
	cout << "\r\n";
	float distance = sonar.measureDistance();
	cout << distance << nl;
	return state;
}
#endif

float to_positive_angle(float angle)
{
	angle = fmod(angle, 2*pi);
	while(angle < 0)
		angle = angle + 2*pi;

	return angle;
}

float menorAngulo(float target, float source)
{
	float a = to_positive_angle(target) - to_positive_angle(source);   
	if (a > pi)
		a = a - 2*pi;
	else if (a < -pi)
		a = a + 2*pi;
	return a;
}

void diferencial(float goal[3])
{	
	
	KBAsync kb;
	
	int state = 0;
	
	float pos[3] = {0,0,0};
	//float goal[3] = {-150,-40,4.71};
	cout << "x: " << goal[0] << "y: " << goal[1] << nl;
	float k_rho = 0.2;
	float k_alpha = 1.7;
	float k_beta = -0.8;
	
	int key;
	float l = 7.25;
	float rLeft = 3.30;
	float rRight = 3.30;
	
	float k = 1.3;
	float velLeft = 10;
	float velRigth = 10;
	float minPower = 10;
	
	Encoder encoderL, encoderR;
	Motor motorL, motorR;

	encoderL.setup(ENCODER_LEFT, LEFT_SIDE);
	encoderR.setup(ENCODER_RIGHT, RIGHT_SIDE);
	motorL.setup(MOTOR_LEFT_A, MOTOR_LEFT_B, MOTOR_LEFT_E, &encoderL);
	motorR.setup(MOTOR_RIGHT_A, MOTOR_RIGHT_B, MOTOR_RIGHT_E, &encoderR);

	Odometry odometry(l, rLeft, rRight, &encoderL, &encoderR);

	motorL.setK(k);
	motorR.setK(k);
	motorL.setMinPower(minPower);
	motorR.setMinPower(minPower);
	int contador = 0;
		
	while ((key=kb.getKey())!='q') {
		if(contador <10){
			k_rho=0.1;
		}
		else{
			k_rho=0.18;
		}
		contador++;
		
		delayMicroseconds(100000);
		
		float dx = goal[0] - pos[0];
		float dy = goal[1] - pos[1];
		float theta = pos[2];
		
		float dtheta = goal[2] - pos[2];
		float rho = sqrt(dx*dx + dy*dy);
				
		float atg = odometry.to180range(atan2(dy,dx));
		float alpha = odometry.to180range(menorAngulo(atg,theta));
		float beta = odometry.to180range(goal[2]-pos[2] - alpha);
		
		float v = k_rho*rho;
		float omega = k_alpha*alpha + k_beta*beta;
		
		float wR = 2*v + l*omega;
		float wL = wR - 2 *l*omega;
		
		float velLeft = wL/rLeft;
		float velRight = wR/rRight;
		
		if (rho<15 || state>0){
			if (abs(dtheta) > 0.15 && state != 2){
				state = 1;
				if(dtheta>=0){
					motorL.setPower(-(3.8)*minPower);
					motorR.setPower((3.8)*minPower);	
				}
				else{
					motorL.setPower((3.8)*minPower);
					motorR.setPower(-(3.8)*minPower);
				}
				
			}
			else{
				velRight = 0;
				velLeft = 0;
				state = 2;
				motorL.setTargetSpeed(velLeft);
				motorR.setTargetSpeed(velRight);
				break;
			}	
		}
		
		//cout << "Iteracao: " << contador << " rho: " << rho << " dtheta: "<<dtheta<< nl;
		//cout <<"State: "<< state << " VelLeft: " << velLeft << " VelRight: " << velRight << nl;
		
		odometry.updatePosition(pos);
		
		motorL.setTargetSpeed(velLeft);
		motorR.setTargetSpeed(velRight);
		
		motorL.controlSpeed();
		motorR.controlSpeed();
		
		//cout << " x: " << pos[0] << " y: " << pos[1] << " theta: " << pos[2] << nl;//*180/M_PI << nl;
	}

	motorL.stop();
	motorR.stop();
	//std::cout << nl << "Exiting..." << nl;
}

void girarGraus(int grau)
{
	float pos[3];
	float radiano = grau*pi/180.0;
	pos[0] = 0;
	pos[1] = 0;
	pos[2] = radiano;
	cout << "vai girar" << nl;
	diferencial(pos);
}


void getDataImg (TRetornoPython *retornoPython, VideoCapture *cap){
	/*
	ifstream myfile;
	std::string filename = "/home/pi/crobot1/ImagensRobotica-2.py";
	std::string cmd = "python ";
	cmd += filename;

	takePicture(cap); 						//tira foto
	system (cmd.c_str());					//executa python para analisar a imagem
	myfile.open ("angulos.txt");				//abre arquivo gerado pelo py
	if (myfile.is_open()){					//se arquivo aberto sem problemas
		myfile >> retornoPython->angulo;					//le o valor do angulo
		myfile >> retornoPython->identificador ;			//indica qual a cor encontrada
											//que sera usada para definir proxima acao
	}										//
	myfile.close();							//fecha arquivo
	*/
	cout << nl << "Angulo: " << nl;
	cin >> retornoPython->angulo;
	cout << nl << "ID: " << nl;
	cin >> retornoPython->identificador;

}


#define ANGULO_MINIMO 7
TAniquilaState updateStateAniquila (TAniquilaState aniquilaState,VideoCapture *cap){
	
	TRetornoPython pyData;
	TAniquilaState newState;
	newState = aniquilaState;
	int *retornoProcIm;
	retornoProcIm = (int*)malloc(2*sizeof(int));
	
	float filterWeight = 0.6;

	//cout << "\r" << flush << "Distance:" << fdist << "cm" << nl;
	
	switch (newState){
		case CONFERE:
			cout << "CONFERINDO" << nl;
			//getDataImg(&pyData,cap);
			retornoProcIm = controlProcImage(cap);
			cout << "Retorno Proc Im ID: " << retornoProcIm[0] << "Retorno Proc Im ANGLE: " << retornoProcIm[1] << endl;
			if (abs(retornoProcIm[1]) < ANGULO_MINIMO && retornoProcIm[0] /*pyData.identificador*/ == 1){
				
				distancia = sonar.measureDistance();
				fdist = filterWeight*fdist + (1-filterWeight)*distancia;
				cout << "distancia sonar: " << fdist << nl;
				if(fdist > 20){						
					newState = AVANCA;
				}
				else {
					newState = ESTOURA;
				}	
			}
			else{
				if (abs(retornoProcIm[1]) >= ANGULO_MINIMO) {
					girarGraus(retornoProcIm[1]);
				}
				newState = PERDEU;
			}
			
		break;
		case AVANCA:
			//avanca rapido ~10cm
			posInter[0] = 35;
			posInter[1] = 0;
			posInter[2] = 0;
			diferencial(posInter);
			newState = CONFERE;
			cout << "AVANCA" << nl;
		break;
		case ESTOURA:
			//avanca lentamente ~1cm
			posInter[0] = 25;
			posInter[1] = 0;
			posInter[2] = 0;
			diferencial(posInter);
			newState = CONFERE;
			cout << "ESTOURA" << nl;
		break;
		case PERDEU:
			newState = PERDEU;
			cout << "PERDEU" << nl;
		break;
	}
	return newState;
}

TRobotState updateStateRobot (int identificador){
	TRobotState newState;
	switch (identificador){
		case 1:						//apenas um balao verde
			newState = VERDE;
		break;
		
		case 2:						//apenas vermelhos
			newState = VERMELHO;
		break;
		default:
			newState = NENHUM;
		break;
		
	}
	return newState;
}


#define OPENCV_BUFFER_SIZE 5
string takePicture(VideoCapture *cap) {
   string retorno;
   stringstream ss;
   if (hasCamera) {
		static int pictureCount = 1;
		//cout << "\rTaking picture " << pictureCount << "...";
		
		ss << pictureCount << ".png";

		Mat frame;
		bool bSuccess = true;

		//Clear the video buffer in order to get the a recent frame
		//only required for exporadic captures
      for (int i=0; i<OPENCV_BUFFER_SIZE && bSuccess; i++)
			bSuccess = cap->read(frame);

		// read a new frame from video
		bSuccess = cap->read(frame); 
		
		//now "frame" stores our image, writ it to the file.
		if (bSuccess && imwrite(ss.str(), frame)){
			cout << "ok." << nl;
			//cout << "path: " << ss.str() << nl;
			pictureCount++;
		} else
			cout << "error." << nl;
	}
		retorno = ss.str();
		return retorno;
}


//Detect ^C
void handleCTRLC(int s) {
  running=false;
}

void setupCTRLCHandle() {
	struct sigaction sigIntHandler;

   sigIntHandler.sa_handler = handleCTRLC;
   sigemptyset(&sigIntHandler.sa_mask);
   sigIntHandler.sa_flags = 0;

   sigaction(SIGINT, &sigIntHandler, NULL);
}

