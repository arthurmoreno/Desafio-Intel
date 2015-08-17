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

int main(int argc, char* argv[]) {
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
            frequencias[i] = frequencias[i-1] + 15;
    }

    //o tamanho de amplitudes eh igual ao numero de pixels
    int amp_size = 256; //igual ao de frequencias
    float *amplitudes;
    amplitudes = new float[amp_size];
    for(int i=0; i<256; ++i) {
        amplitudes[i] = float(rand()%4) / 2;
        //amplitudes[i] = 1;
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
    std::cout << "dasdasdas";

    /* While sound is playing, sleep */
    al_check_error();
    //sleep(seconds);
    for(int i = 0; i< 1000000000; i++);
    /* Dealloc OpenAL */
    exit_al();
    al_check_error();
    return 0;
}
