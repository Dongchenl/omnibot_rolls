
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/types.h>
#include <errno.h>
#include <paex.h>
#include "aerctl.h"

#include <cstring>
#include <sys/ioctl.h>

static int aer;


int aerOpenDevice()
{
    aer = open("/dev/paex", O_RDONLY, 0);
    if (aer < 0) {
        fprintf(stderr,"error opening /dev/paex: %s\n",strerror(errno));
        return -1;
    }
    return aer;
}

void aerClose(int aer_dev)
{
	if (aer_dev >= 0)
		close(aer_dev);
}

static uint32_t aerRead(int cmd, uint32_t address)
{
        int retval;
        aer_mem_t mem;
        mem.address = address;
        retval = ioctl(aer, cmd, &mem);
        if (retval < 0) {
                fprintf(stderr,"ioctl error: %s\n", strerror(errno));
                exit(-1);
        }
        return mem.value;
}

static void aerWrite(int cmd, uint32_t address, uint32_t value)
{
        int retval;
        aer_mem_t mem;
        mem.address = address;
        mem.value = value;
        retval = ioctl(aer, cmd, &mem);
        if (retval < 0) {
                fprintf(stderr,"ioctl error: %s\n", strerror(errno));
                exit(-1);
        }
}

uint32_t aerReadMem(uint32_t address)
{
        return aerRead(AER_READ, address);
}

static void aerWriteMem(uint32_t value)
{
        aerWrite(AER_WRITE, 0, value);
}

static void BGSet(uint32_t value)
{
        aerWrite(AER_SET_BIAS, 0, value);
}

int aerFindHead()
{
        int i = 0;
        while (i < AER_BUFFER_SIZE) {
                uint32_t r = aerReadMem(2*i);
                if (r == 0xffffffff) {
                        return i;
                }
                i++;
        }
        return 0;
}

void aerWriteBits(uint32_t bits) {
	printf ("Setting bits %d\n", bits);
        aerWriteMem(bits);
}

void BGSetFromFile (const char* fname)
{
	printf ("setting biases from file: %s\n", fname);
	FILE* file = fopen (fname, "r");
	uint32_t value;
	int k = 0;

  fscanf(file, "%08x", &value);
	while (!feof (file))
	{
		printf("setting bias %d: %08x\n", k, value);
		fscanf(file, "%08x", &value);
		k++;
		BGSet(value);
	}
	fclose (file);
}


//device-specific code starts here

#ifdef DEVICE_MN256R1
    static uint32_t convert_bias(Bias *bias) {
        uint32_t reverse_3bits[8] = {0,4,2,6,1,5,3,7};
        //check whether all necessary data is available
        if(bias->id == -1 ||
           bias->coarse == -1 ||
           bias->fine == -1 ||
           bias->high == -1 ||
           bias->type == -1 ||
           bias->cascode == -1 ||
           bias->enable == -1) {
           fprintf (stderr, "ERROR: Some parameters were not provided.\n");
           return -1; }
        if(bias->coarse > 7) {
           fprintf (stderr, "ERROR: coarseCurrent must be 3 bits wide.\n");
           return -1; }
        //construct output bits
        uint32_t bits = 0;
        bits += bias->enable << 0;
        bits += bias->type << 1;
        bits += bias->cascode << 2;
        bits += bias->high << 3;
        bits += bias->fine << 4;
        if (bias->id == 65 || bias->id == 66)
            bits += 63 << 10;
        else 
            bits += reverse_3bits[bias->coarse] << 12;
        bits += bias->id << 16;
        return bits;
    }

    void aerSetBias(Bias *bias) {
        int32_t res = convert_bias(bias);
        if (res != -1) {
            printf("Setting bias: %d\n", res);
            BGSet(res); }
        else
            printf("ERROR: Could not convert bias.\n");
    }

//original version from Jonathan
//int aerSetConn(Synapse *synapse) {
//  uint32_t bits;
//  uint32_t offset;
//  uint32_t latches[5];
//	int num_latches;
//	int i;
//	if (synapse->type == NONPLASTIC) {
//            latches[0] = synapse->recurrent; //recurrent
//            latches[1] = synapse->mode; //exc/inh
//            latches[2] = synapse->wgt1; //wgt1
//            latches[3] = synapse->wgt2; //wgt2
//            latches[4] = 0; //broadcast not used at the moment
//	    			num_latches = 5;
//            offset = 133632;
//	}
//	else if (synapse->type == PLASTIC) {
//            uint32_t latches[2];
//            latches[0] = synapse->recurrent; //recurrent
//            latches[1] = 0; //broadcast not used at the moment
//	          num_latches = 2;
//            offset = 1051136;
//	}
//	else return -1;

//	for (i=0; i<num_latches; i++) {
//		bits = offset;
//		bits += synapse->target;
//		bits += (synapse->source) << 8;
//		bits += latches[i] << 16;
//		bits += i << 17;
//		//printf ("Setting bits %d\n", bits);
//		aerWriteMem(bits);
//	}
//	usleep(10);
//	return 0;
//}


void aerStimSyn(int type, int n, int i) {
        uint32_t bits = n;       //neuron
        bits += (type + i) << 8; //synapse
        aerWriteMem(bits);
}

//set only connectivity between neurons
int aerSetConn(Synapse *synapse) {
  uint32_t bits;
  uint32_t offset;
  uint32_t latches[5];
	int num_latches;
	int i;
	int plastic_w;
	if (synapse->type == NONPLASTIC) {
            latches[0] = synapse->recurrent; //recurrent
            latches[1] = synapse->mode; //exc/inh
            latches[2] = synapse->wgt1; //wgt1
            latches[3] = synapse->wgt2; //wgt2
            latches[4] = 0; //broadcast not used at the moment
	    num_latches = 5;
            offset = 133632;
	    for (i=0; i<num_latches; i++) {
		bits = offset;
		bits += synapse->target;
		bits += (synapse->source) << 8;
		bits += latches[i] << 16; //sr     //in case the selected option (latch) is high set the sr to 1, otherwise 0
		bits += i << 17;					//latch  //the value of the index equals the latch value 
	        aerWriteMem(bits); //you can not set several latches at the same time only one  by one
	     }
	     usleep(10);
	}
	else if (synapse->type == PLASTIC) {
	//setting connection
            uint32_t latches[2];
            latches[0] = synapse->recurrent; //recurrent
            latches[1] = 0; //broadcast not used at the moment
	    num_latches = 2;
            offset = 788992;
	    for (i=0; i<num_latches; i++) 
	    {
		bits = offset;
		bits += synapse->target;
		bits += (synapse->source) << 8;
		bits += latches[i] << 16; //sr     //in case the selected option (latch) is high set the sr to 1, otherwise 0
		bits += i << 17;					//latch //the value of the index equals the latch value 
		aerWriteMem(bits); //you can not set several latches at the same time only one  by one
	     }
	     usleep(10);

	//setting existing plastic connection to high/low
       		plastic_w = synapse->wgt1; //wgt1 , wgt2 ignored (1bin weight encoding)
        	offset = 1051136;
		bits = offset;
		bits += synapse->target;
		bits += (synapse->source) << 8;
		bits += plastic_w << 16;	//hl //high or low weight
        	// printf ("target: %d source: %d weight: %d  = Setting bits %d\n",synapse->target, synapse->source, plastic_w, bits);
		aerWriteMem(bits);
		usleep(10);
	}	
	else return -1;

	return 0;
}

void aerMonSoma(uint32_t n) {
        aerWriteBits(1249024);     //assert all reset lines
        usleep(100*1000);
        aerWriteBits(1249025);     //deassert all reset lines
        usleep(100*1000);
        aerWriteBits(1249284 + n); //monitor neuron n
        usleep(100*1000);
        aerWriteBits(1249025);     //deassert all reset lines
        usleep(100*1000);
        aerWriteBits(1249284 + n); //monitor neuron n
    }

void aerSetTau1(uint32_t n) {
	aerWriteBits(1249026+n);     //set to tau1
	aerWriteBits(1249283); 		 // fix a bug
	aerWriteBits(1249026+n);	 //set to tau1
	}

void aerResetTau2() {
	aerWriteBits(1249282);       // assert reset
	aerWriteBits(1249283); 		 // deassert reset
	}

 void aerMonSomaSyn(uint32_t n, uint32_t s) {
         aerWriteBits(1249024);     //assert all reset lines
         usleep(100*1000);
         aerWriteBits(1249025);     //deassert all reset lines
         usleep(100*1000);
         aerWriteBits(1249284 + n); //monitor neuron n
         usleep(100*1000);
         aerWriteBits(1182208 + (s*256) + n); //monitor synapse s of neuron n
         usleep(100*1000);
         aerWriteBits(1249025);     //deassert all reset lines
         usleep(100*1000);
         aerWriteBits(1249284 + n); //monitor neuron n
         usleep(100*1000);
         aerWriteBits(1182208 + (s*256) + n); //monitor synapse s of neuron n
     }
 
 void aerSetSynHi(uint32_t n2, uint32_t s2) {
         aerWriteBits(1249024);     //assert all reset lines
         usleep(100*1000);
         aerWriteBits(1249025);     //deassert all reset lines
         usleep(100*1000);
         aerWriteBits(1051136 + (1*256*256) + (s2*256) + n2); //set synapse s of neuron n high
         usleep(100*1000);
         aerWriteBits(1249284 + n2); //monitor neuron n
         usleep(100*1000);
         aerWriteBits(1182208 + (s2*256) + n2); //monitor synapse s of neuron n
         usleep(100*1000);
         aerWriteBits(1249025);     //deassert all reset lines
         usleep(100*1000);
         aerWriteBits(1051136 + (1*256*256) + (s2*256) + n2); //set synapse s of neuron n high
         usleep(100*1000);
         aerWriteBits(1249284 + n2); //monitor neuron n
         usleep(100*1000);
         aerWriteBits(1182208 + (s2*256) + n2); //monitor synapse s of neuron n
 }
 
 void aerSetSynLo(uint32_t n3, uint32_t s3) {
         aerWriteBits(1249024);     //assert all reset lines
         usleep(100*1000);
         aerWriteBits(1249025);     //deassert all reset lines
         usleep(100*1000);
         aerWriteBits(1051136 + (0*256*256) + (s3*256) + n3); //set synapse s of neuron n low
         usleep(100*1000);
         aerWriteBits(1249284 + n3); //monitor neuron n
         usleep(100*1000);
         aerWriteBits(1182208 + (s3*256) + n3); //monitor synapse s of neuron n
         usleep(100*1000);
         aerWriteBits(1249025);     //deassert all reset lines
         usleep(100*1000);
         aerWriteBits(1051136 + (0*256*256) + (s3*256) + n3); //set synapse s of neuron n low
         usleep(100*1000);
         aerWriteBits(1249284 + n3); //monitor neuron n
         usleep(100*1000);
         aerWriteBits(1182208 + (s3*256) + n3); //monitor synapse s of neuron n
     }

void aerResetAll(uint32_t type) {
     int i,j;
     for (i=0; i<256; i++) {
        for (j=0; j<256; j++) {
    	    //printf("resetting synapse (%d,%d)\n", i, j);
    	  Synapse synapse;
    	  synapse.type = type;
    	  synapse.recurrent = DISABLE;
          synapse.mode = EXCITATORY;
          synapse.source = j;
          synapse.target = i;
          synapse.wgt1 = DISABLE;
          synapse.wgt2 = DISABLE;
    	    
          if(aerSetConn(&synapse) < 0)
    		printf("an error occured while setting synapse\n");
        }
     }
}
#endif


//add other devices here....

#ifdef DEVICE_DUMMY
    //Your device-specific code
#endif

