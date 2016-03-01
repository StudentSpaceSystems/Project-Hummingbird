#include <stdbool.h>
#include "packet.h"

int main() {
    float returnStuff[] = {2.3, 1.2, 1.1};
    ArrayFloatToByte(returnStuff, 2);
    return 0;
}

byte_array ArrayFloatToByte(float * data, short length)  {
    byte_array bytes;
    bytes.stream = (char *) data;
    bytes.length = length;
}

void writePacket(char channel_number, char num_streams, char data[], char sig_flags, char number_bytes, bool evil_bit)  {
    /**
    * char* writePacket(char channel_number, char num_streams, float data[], char sig_flags, char number_bytes, bool evil_bit)
    * channel_number   -- channel identification number (ranges from 0 to 15)
    * sig_flag         -- 3-bit general purpose flags (byte-value from 0 to 7 but can be set by bitwise operations) (last bit implement optional packet-counter)
    * number_bytes     -- number of bytes per stream instance (ranges from 1 to 8)
    * float data[]     -- float array of data points to be passed
    * num_streams      -- number of elements in data[] dimension must be between 1 and 16
    * boolean evil_bit -- implemented partial-extension of RFC 3514. I don't write the rules; I just follow them.
    *
    * packet format
    * [SOH][channel_number(4)|number_of_streams(4)][flags(4)|evil_bit(1)|number_of_bytes(3)] <stream_1_data> <stream_2_data> .. <stream_n_data> [ETB]
    * packet length will be 4 + num_stream * number_bytes
    */
    char start_of_header = 0x01;
    char end_transmission_block = 0x17;
    short packet_length = 4 + num_streams*number_bytes;
    char byte_stream[packet_length];
    char format_byte = (sig_flags << 4) | (evil_bit << 3) | (number_bytes - 1) & 7;
    char stream_byte = (channel_number << 4) | (num_streams - 1) & 15;
    *(byte_stream + 0) = start_of_header;
    *(byte_stream + 1) = stream_byte;
    *(byte_stream + 2) = format_byte;
    char * b = &(((char *) data)[0]);
    for (short i = 0; i < packet_length - 4 ; i++)  {
        *(byte_stream + 3 + i) = *(b + i);
    }
    *(byte_stream + packet_length - 1) = end_transmission_block;
    byte_array _packet;
    _packet.stream = byte_stream;
    _packet.length = packet_length;
    return _packet;
}
