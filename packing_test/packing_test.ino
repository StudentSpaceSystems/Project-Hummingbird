void thing(float c[]) {
  byte *b = &((byte *)c )[0];
  for (int i = 0; i < sizeof(c); i++)  {
    Serial.write(*(b+i));
  }
  Serial.println("DONE");
  Serial.println(c[0]);
}

void setup() {
  Serial.begin(38400);
  float a[5] = {1,2,3,4,5};
  thing(a);
}

void loop() {
  // put your main code here, to run repeatedly:

}
