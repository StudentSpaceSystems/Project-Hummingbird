void writePacket(byte channel_number, byte num_streams, byte number_bytes, byte data[], byte sig_flags, bool evil_bit)  {
  /**
  * COBS-encoded packet-writing:
  * byte* writePacket(byte channel_number, byte num_streams, float data[], byte sig_flags, byte number_bytes, bool evil_bit)
  * channel_number   -- channel identification number (ranges from 0 to 15)
  * sig_flag         -- 3-bit general purpose flags (byte-value from 0 to 7 but can be set by bitwise operations) (last bit implement optional packet-counter)
  * number_bytes     -- number of bytes per stream instance (ranges from 1 to 8)
  * float data[]     -- float array of data points to be passed
  * num_streams      -- number of elements in data[] dimension must be between 1 and 16
  * boolean evil_bit -- implemented partial-extension of RFC 3514. I don't write the rules; I just follow them.
  *
  * packet format pre-encoding
  * [channel_number(4)|number_of_streams(4)][flags(4)|evil_bit(1)|number_of_bytes(3)] <stream_1_data> <stream_2_data> .. <stream_n_data>
  * packet length will be 4 + num_stream * number_bytes
  */
  byte end_transmission_block = 0x17;
  word packet_length = 2 + num_streams*number_bytes;
  byte byte_stream[packet_length];
  byte format_byte = (sig_flags << 4) | (evil_bit << 3) | (number_bytes - 1) & B00000111;
  byte stream_byte = (channel_number << 4) | (num_streams - 1) & B00001111;
  *(byte_stream + 0) = stream_byte;
  *(byte_stream + 1) = format_byte;
  byte * b = &(((byte *) data)[0]);
  for (word i = 0; i < packet_length - 4 ; i++)  {
      *(byte_stream + 2 + i) = *(b + i);
  }
  /**
   * Beginning of COBS encoding (consideration put towards COBS-R encoding but due to latency, abandoned for standard COBS
   * 254-byte long packets
   */
  byte packet[packet_length + packet_length/254 + 1]; // +1 to include our 0x00 terminator
  word lastIndex = 0;
  word index = 0;
  word stopIndex = packet_length + packet_length/254 + 1;
  while (index < stopIndex) {
    
    if ( *(byte_stream + index) != 0) {
      *(packet + index + 1) = *(byte_stream + index);
    }
    else  {
      *(packet + lastIndex) = index - lastIndex;
      lastIndex = index;
    }
    
    index++;
  }
  Serial.write(packet, stopIndex);
}



void setup() {
  Serial.begin(38400);
  float test_data[] = {1.0, -9.5, 2.3};
  byte channel = 0;
  byte streams = 3;
  writePacket(channel, streams, 4, (byte *) test_data, 0, false);
}

void loop() {
}
