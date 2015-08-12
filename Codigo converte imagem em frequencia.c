/*
while (existe pixel não convertido)
	pegar valor do pixel
	a partir da posição do pixel colocar em uma frequencia adequada
	a partir do valor do pixel(escala de cinza)
	
	
links das bibliotecas:
	The Synthesis ToolKit: https://ccrma.stanford.edu/software/stk/
	The Synthesis ToolKit Wiki: https://en.wikipedia.org/wiki/Synthesis_Toolkit
	
*/
void colocaPixelAudio(int valorPixel,int i,int j){
	/*
		Aqui deve ser feita a conversão utilizando o stk
		uma opção para unir cada frequencia de cada pixel é utilizar a serie de furrier
		ou seja criar varios sianis senoidais com a stk e depois junta los de alguma maneira.
		
		*pesquisar função (de preferencia na stk) que junte dois sinais senoidais em um sinal apenas.
		
		Outra maneira seria setar a amplitude de cada frequencia desejada (Caso isso seja possivel).
	*/
}

void converteImagem(VideoCapture *cap){
	
	string ss;	
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
		for(j=0; j<imgThresholded.cols; j++){
			//cout << imgThresholded.at<uchar>(i,j) << endl;
			colocaPixelAudio((int)imgThresholded.at<uchar>(i,j),i,j) //função que vai posicionar o pixel 
		}	
	}
}