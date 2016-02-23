byte* writePacket(byte channel_number, byte num_streams, float data[], byte sig_flags=0, byte number_bytes=4, boolean evil_bit=false)  {
  /*
   * WARNING -- Insecure implementation; DO NOT USE FOR SECURE SYSTEMS
   * byte* writePacket(byte channel_number, byte num_streams, byte stream_size, float[] data, boolean evil_bit=false)
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
  byte start_of_header = 0x01;
  byte end_transmission_block = 0x17;
  int packet_length = 4+num_streams*number_bytes;
  byte byte_stream[packet_length];
  byte format_byte = (sig_flags << 4) | (evil_bit << 3) | (number_bytes - 1) & 7;
  byte stream_byte = (channel_number << 4) | (num_streams - 1) & 15;
  *(byte_stream + 0) = start_of_header;
  *(byte_stream + 1) = stream_byte;
  *(byte_stream + 2) = format_byte;
  byte* b = &(((byte *) data)[0]);
  for (int i = 0; i < packet_length-4 ; i++)  {
    *(byte_stream + 3 + i) = *(b+i);
  }
  *(byte_stream + packet_length-1) = end_transmission_block;
  cobsr_encode (byte_stream, packet_length)
  //Serial.write(byte_stream, packet_length);
}



void setup() {
  Serial.begin(38400);
  float test_data[] = {1.0, -9.5, 2.3};
  byte channel = 0;
  byte streams = 3;
  writePacket(channel, streams, test_data);
}

void loop() {
}
