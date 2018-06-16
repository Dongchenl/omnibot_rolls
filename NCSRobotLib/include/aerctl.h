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

#include <stdint.h>

typedef struct {
    int32_t id;
    int32_t coarse;
    int32_t fine;
    int32_t high;
    int32_t type;
    int32_t cascode;
    int32_t enable;
} Bias;

typedef struct {
    uint32_t mode;
    uint32_t recurrent;
    uint32_t source;
    uint32_t target;
    uint32_t type;
    uint32_t wgt1;
    uint32_t wgt2;
} Synapse;

extern int aerOpenDevice();
extern int aerFindHead();
extern void aerWriteBits(uint32_t bits);
extern uint32_t aerReadMem(uint32_t address);
extern void aerClose(int aer_dev);
extern void aerSetSynHi(uint32_t n3, uint32_t s3);
#ifdef DEVICE_MN256R1
		extern void aerStimSyn(int type, int n, int i);
    extern void aerResetAll(uint32_t type);
    extern void aerMonSoma(uint32_t n);
    extern void aerSetTau1(uint32_t n);
    extern void aerResetTau2();
    extern int aerSetConn(Synapse *synapse);
    extern void aerMonSomaSyn(uint32_t n, uint32_t s);
    extern void aerSetSynLo(uint32_t n2, uint32_t s2);
    extern void aerSetBias(Bias *bias);
   // extern void aerSetSynHi(uint32_t n3, uint32_t s3);
  #endif
  
#endif
