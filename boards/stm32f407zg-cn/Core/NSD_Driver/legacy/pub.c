/**
 * @file pub.c
 * @brief Implementation of stub variables for legacy network code
 */
#include "pub.h"

/* Power measurement variables (stubs - not actively used) */
float P = 0.0f;
float Q = 0.0f;
float Pav = 0.0f;
float Qav = 0.0f;

/* Network TX/RX buffers */
unsigned char ucQTx[12] = {0};
unsigned char ucMornitor[256][256] = {0};

/* Network state variables */
int Net_Monitor_C = 0;
int Net_Monitor_Index = 0;
int ms_C = 0;
int Monitor_Upload_Flag = 0;

/* Byte to float conversion stub */
void Byte_To_Float(float* data, unsigned char* b)
{
    /* Simple IEEE 754 conversion (big-endian assumed) */
    union {
        float f;
        unsigned char bytes[4];
    } u;

    u.bytes[0] = b[3];
    u.bytes[1] = b[2];
    u.bytes[2] = b[1];
    u.bytes[3] = b[0];

    *data = u.f;
}
