boolean readFlag = false;
int intBuffer[3];
String stringBuffer = "";
void setup() {
  Serial.begin(19200);
}

void loop() {
  if (readFlag and Serial.available()) {
    char c;
    c = Serial.read();
    if (c == 59) {   // Semicolon
      int commaIndex = stringBuffer.indexOf(',');
      int secondCommaIndex = stringBuffer.indexOf(',', commaIndex+1);
      intBuffer[0] = stringBuffer.substring(0, commaIndex).toInt();
      intBuffer[1] = stringBuffer.substring(commaIndex+1, secondCommaIndex).toInt();
      intBuffer[2] = stringBuffer.substring(secondCommaIndex+1).toInt();
      Serial.print(":: ");Serial.print(intBuffer[0]);Serial.print(" ");Serial.print(intBuffer[1]);Serial.print(" ");Serial.println(intBuffer[2]);
      stringBuffer = "";
      readFlag = false;
    }
    else  {
      stringBuffer += c;
    }
  }
  else if (Serial.available()) {
    if (Serial.peek() == 33)  {   // Exclamation mark
      stringBuffer = "";
      Serial.read();
      readFlag = true;
    }
  }
}
