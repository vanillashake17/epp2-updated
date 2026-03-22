void setup() {}

void loop() {
  // put your main code here, to run repeatedly:

  forward(200); // takes values from 0 to 255
  delay(3000);

  stop();
  delay(2000);

  ccw(200); // takes values from 0 to 255
  delay(3000);


  backward(200); // takes values from 0 to 255
  delay(3000);

  stop();
  delay(2000);

  cw(200); 
  delay(3000);
  delay(1000000);
}
