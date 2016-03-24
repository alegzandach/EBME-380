byte commandByte;
byte noteByte;
byte velocityByte;

void setup(){
  Serial.begin(31250);
}

void checkMIDI(){
  do{
    if (Serial.available()){
      commandByte = Serial.read();//read first byte
      noteByte = Serial.read();//read next byte
      velocityByte = Serial.read();//read final byte
    }
  }
  while (Serial.available() > 2);//when at least three bytes available
}
    

void loop(){
  checkMIDI();
}