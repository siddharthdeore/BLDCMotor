// Open Loop SVPWM test
#define PIN_U 9
#define PIN_V 10
#define PIN_W 11

#define sineArraySize 360

const uint8_t pwmArray[sineArraySize] = {128, 132, 136, 140, 143, 147, 151, 155, 159, 162, 166, 170, 174, 178, 181, 185, 189, 192, 196, 200, 203, 207, 211, 214, 218, 221, 225, 228, 232, 235, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 235, 232, 228, 225, 221, 218, 214, 211, 207, 203, 200, 196, 192, 189, 185, 181, 178, 174, 170, 166, 162, 159, 155, 151, 147, 143, 140, 136, 132, 128, 124, 120, 116, 113, 109, 105, 101, 97, 94, 90, 86, 82, 78, 75, 71, 67, 64, 60, 56, 53, 49, 45, 42, 38, 35, 31, 28, 24, 21, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21, 24, 28, 31, 35, 38, 42, 45, 49, 53, 56, 60, 64, 67, 71, 75, 78, 82, 86, 90, 94, 97, 101, 105, 109, 113, 116, 120, 124};

uint8_t svpwm(int angle);

float direction = 1;  // direction -1/+1
int index = 0;        // angle index
uint32_t t_prev;

void setup(){
  pinMode(PIN_U,OUTPUT);
  pinMode(PIN_V,OUTPUT);
  pinMode(PIN_W,OUTPUT);

  pinMode(A0,INPUT);
  
  Serial.begin(115200);
}
void loop()
{
  float omega = analogRead(A0);
  direction = 1;

  double dt=(1000)*TWO_PI/omega;
  
  if(millis()-t_prev>dt){
    analogWrite(PIN_U,svpwm(index));
    analogWrite(PIN_V,svpwm(index-120));
    analogWrite(PIN_W,svpwm(index+120));

    index=index+direction;

    index = index % 360;
    if (index < 0)
      index += 360;
    
    t_prev=millis();

    Serial.print(svpwm(index));
    Serial.print(",");
    Serial.print(svpwm(index-120));
    Serial.print(",");
    Serial.println(svpwm(index+120));
  }
}

// fast svpwm reference table
uint8_t svpwm(int angle){
    angle = angle % 360;
    if (angle < 0)
        angle += 360;
    return pwmArray[angle];
}
