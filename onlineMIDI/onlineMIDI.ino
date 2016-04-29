byte commandByte;
byte noteByte;
byte velocityByte;

void setup(){
  Serial1.begin(60);
  Serial.begin(9600);
}

void checkMIDI(){
  do{
    if (Serial1.available()){
      commandByte = Serial1.read();//read first byte
      noteByte = Serial1.read();//read next byte
      velocityByte = Serial1.read();//read final byte
      Serial.print("Command: ");
      Serial.println(commandByte, BIN);
      Serial.print("Note: ");
      Serial.println(noteByte, BIN);
      Serial.print("Velocity: ");
      Serial.println(velocityByte, BIN);
      
    }
  }
  while (Serial1.available() > 2);//when at least three bytes available
}
    

void loop(){
  checkMIDI();
}
