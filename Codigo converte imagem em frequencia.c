#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <al.h>
#include <alc.h>
#include <time.h>

#define CASE_RETURN(err) case (err): return "##err"
const char* al_err_str(ALenum err) {
    switch(err) {
        CASE_RETURN(AL_NO_ERROR);
        CASE_RETURN(AL_INVALID_NAME);
        CASE_RETURN(AL_INVALID_ENUM);
        CASE_RETURN(AL_INVALID_VALUE);
        CASE_RETURN(AL_INVALID_OPERATION);
        CASE_RETURN(AL_OUT_OF_MEMORY);
    }
    return "unknown";
}
#undef CASE_RETURN

#define __al_check_error(file,line) \
    do { \
        ALenum err = alGetError(); \
        for(; err!=AL_NO_ERROR; err=alGetError()) { \
            std::cerr << "AL Error " << al_err_str(err) << " at " << file << ":" << line << std::endl; \
        } \
    }while(0)

#define al_check_error() \
    __al_check_error(__FILE__, __LINE__)


void init_al() {
    ALCdevice *dev = NULL;
    ALCcontext *ctx = NULL;

    const char *defname = alcGetString(NULL, ALC_DEFAULT_DEVICE_SPECIFIER);
    std::cout << "Default device: " << defname << std::endl;

    dev = alcOpenDevice(defname);
    ctx = alcCreateContext(dev, NULL);
    alcMakeContextCurrent(ctx);
}

void exit_al() {
    ALCdevice *dev = NULL;
    ALCcontext *ctx = NULL;
    ctx = alcGetCurrentContext();
    dev = alcGetContextsDevice(ctx);

    alcMakeContextCurrent(NULL);
    alcDestroyContext(ctx);
    alcCloseDevice(dev);
}


void geraAudio(float mat_media_p [][]){
	/*
		Aqui deve ser feita a conversão utilizando o stk
		uma opção para unir cada frequencia de cada pixel é utilizar a serie de furrier
		ou seja criar varios sianis senoidais com a stk e depois junta los de alguma maneira.

		*pesquisar função (de preferencia na stk) que junte dois sinais senoidais em um sinal apenas.

		Outra maneira seria setar a amplitude de cada frequencia desejada (Caso isso seja possivel).
	*/

	/* initialize OpenAL */
    init_al();

    srand(time(NULL));

    /* Create buffer to store samples */
    ALuint buf;
    alGenBuffers(1, &buf);
    al_check_error();

    /* Fill buffer with Sine-Wave */
    float freq = 400.f;
    int seconds = 4;
    unsigned sample_rate = 8000;
    size_t buf_size = seconds * sample_rate;

    //os vetores de frequencia e amplitude serão recebidos
    //a partir da imagem de profundidade
    //o tamanho de frequencias eh igual ao numero de pixels
    int freq_size = 256;
    float *frequencias;
    frequencias = new float[freq_size];
    float freq_aux;
    for(int i=0; i<256; ++i) {
        if (i == 0)
            frequencias[i] = 100;
        else
            frequencias[i] = frequencias[i-1] + 40;
    }

    //o tamanho de amplitudes eh igual ao numero de pixels
    int amp_size = 256; //igual ao de frequencias
    float *amplitudes;
    amplitudes = new float[amp_size];
    k = 0;
    //passa os valores da matriz pro vetor de amplitudes
    for(i=0;i<16;i++){
        for(j=0;j<16;j++){
            amplitudes[k] = mat_media_p[i][j];
            k++;
        }
    }

    short *samples;
    samples = new short[buf_size];

    for(int i=0; i<buf_size; ++i) {
        //samples[i] = 32760 * sin( (2.f*float(M_PI)*freq)/sample_rate * i );
        samples[i] = 0;
        for (int j=0; j<freq_size; ++j){
            samples[i] = samples[i] + 32760 * amplitudes[j] * sin( (2.f*float(M_PI)*frequencias[j])/sample_rate * i );
        }
        //samples[i] = 32760 * sin( (2.f*float(M_PI)*float(rand()%300+100))/sample_rate * i );
    }

    /* Download buffer to OpenAL */
    alBufferData(buf, AL_FORMAT_MONO16, samples, buf_size, sample_rate);
    al_check_error();


    /* Set-up sound source and play buffer */
    ALuint src = 0;
    alGenSources(1, &src);
    alSourcei(src, AL_BUFFER, buf);
    alSourcePlay(src);

    /* While sound is playing, sleep */
    al_check_error();
    //sleep(seconds);
    for(int i = 0; i< 1000000000; i++);
    /* Dealloc OpenAL */
    exit_al();
    al_check_error();
}

void converteImagemEmAudio(Mat imgOriginal){


	if(!imgOriginal.data) {
		cout << "picture nao abriu!" <<endl;
		exit(0);
	}

	int i, j, limiarDePixels = 1000;
	int countEsquerda = 0, countDireita = 0;
	int retorno=0;

	float mat_media_p [16][16];

    for (i=0;i<16;i++){
       for (j=0;j<16;j++){
           for (x=(i*20);x<(i*20+20);x++){
               for(y=(j*15);y<(j*15+15);y++){
                   mat_media_p[i][j] = mat_media_p[i][j] + (float)imgOriginal.at<uchar>(x,y);
               }
           }
           mat_media_p[i][j] = mat_media_p[i][j] / 300;
       }
    }

    geraAudio(mat_media_p);
}

int main(int argc, char* argv[]) {
    Mat imgOriginal(320,240, -1);

    img = imread("imagem.");
    converteImagemEmAudio(VideoCapture *cap);
}
