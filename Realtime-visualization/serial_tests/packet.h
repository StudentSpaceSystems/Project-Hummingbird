#ifndef PACKET_H_
#define PACKET_H_

typedef struct {
   char * stream;
   short length;
} byte_array;

byte_array ArrayFloatToByte(float * data, short length);

#endif
