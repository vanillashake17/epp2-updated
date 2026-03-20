void setup() {}

void loop() {
  // put your main code here, to run repeatedly:

  forward(255); // takes values from 0 to 255
  delay(3000);

  stop();
  delay(2000);

  ccw(255); // takes values from 0 to 255
  delay(10000);


  //backward(200); // takes values from 0 to 255
  delay(10000);

  stop();
  delay(2000);

  cw(255); 
  delay(10000);
  delay(1000000);
}
