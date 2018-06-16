#ifndef AERCTL_H_
#define AERCTL_H_

#define AER_BUFFER_SIZE 64000

#define PLASTIC 0
#define NONPLASTIC 256
#define VIRTUAL 512
#define EXCITATORY 1
#define INHIBITORY 0
#define EXC 1
#define INH 0
#define ENABLE 1
#define DISABLE 0

struct Bias {
    uint32_t id;
    uint32_t coarse;
    uint32_t fine;
    uint32_t high;
    uint32_t type;
    uint32_t cascode;
    uint32_t enable;
}; 

struct Synapse {
    uint32_t mode;
    uint32_t recurrent;
    uint32_t source;
    uint32_t target;
    uint32_t type;
    uint32_t wgt1;
    uint32_t wgt2;
		Synapse(uint32_t mode, uint32_t recurrent, uint32_t source, uint32_t target, uint32_t type, uint32_t wgt1, uint32_t wgt2) :
			mode(mode), recurrent(recurrent), source(source), target(target), type(type), wgt1(wgt1), wgt2(wgt2) {}
};

int aerOpenDevice();
int aerFindHead();
void aerWriteBits(uint32_t bits);
uint32_t aerReadMem(uint32_t address);

void aerResetAll(uint32_t type);
void aerStimSyn(int type, int n, int i);
void aerMonSoma(uint32_t n);
int aerSetConn(Synapse *synapse);

#endif
