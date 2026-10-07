int AbsMap[5] = { 9, 10, 11, 12, 13 };
int QuadMap[2] = { 8, 9 };
uint8_t absOut = 0b00000000;
unsigned long starttime = 0;
float angleNow = 0;
float anglePrev = 0;
float angleDiff = 0;
bool clockwise = 0;
//-------------------------------------------------------
float Errors[10] = {};
int errIdx = 0;
int count = 0;
//-------------------------------------------------------
float deg = 45;  // Rotation degree
float s = 0;     //Encoder counts
int sm1 = 0;     //Built-in chanel 1
int sm2 = 0;     //Built-in chanel 2
int r = 0;       //indicator for reading builtin encoder to avoid the reading redundancy
float er;        //Proportional error for PI controller
float eri;       //Integral error for PI controller




int t = 0;   //time in ms
int t0 = 0;  //memory for time in ms




int finish = 0;  //finish indicator
int rep = 1;     //Repetition indicator
//-------------------------------------------------------
uint8_t G2B(uint8_t G) {
  uint8_t M = G;
  while (M > 0) {
    M >>= 1;
    G ^= M;
  }
  return G;
}
float getAngle(){
    float angle=0;
    uint8_t reg=0;
    for (int i = 4; i >= 0; i--) {
        reg |= !digitalRead(AbsMap[i]) << i;  // write new bits
    }
    angle = G2B(reg)*11.25;
    return angle;
}



void setup() {

  Serial.begin(250000);  //Baud rate of communication
  Serial.println("Enter the desired rotation in degree.");

  while (Serial.available() == 0)  //Obtaining data from user
  {} //Wait for user input

  deg = Serial.readString().toFloat();  //Reading the Input string from Serial port.
  if (deg < 0) {
      analogWrite(3, 255);  //change the direction of rotation by applying voltage to pin 3 of arduino
  }
  deg = abs(deg);


  anglePrev = getAngle();
}



float kp = .6 * 90 / deg;  //proportional gain of PI
float ki = .02;            //integral gain of PI

bool summaryDone = 0;

void loop() {

  //-----------------------------------------------------------
  t = millis();                       //reading time
  t0 = t;                             //sving the current time in memory
  while (t < t0 + 4000 && rep <= 10)  //let the code to ran for 4 seconds each with repetitions of 10
  {
    if (t % 10 == 0)  //PI controller that runs every 10ms
    {
      if (s < deg * 114 * 2 / 360) {
        er = deg - s * 360 / 228;
        eri = eri + er;
        analogWrite(6, kp * er + ki * eri);
      }

      if (s >= deg * 228 / 360) {
        analogWrite(6, 0);
        eri = 0;
      }
      delay(1);
    }

    sm1 = digitalRead(7);  //reading chanel 1
    sm2 = digitalRead(5);  //reading chanel 2

    if (sm1 != sm2 && r == 0) {  //counting the number changes for both chanels
      s = s + 1;
      r = 1;  // this indicator wont let this condition, (sm1 != sm2), to be counted until the next condition, (sm1 == sm2), happens
    }
    if (sm1 == sm2 && r == 1) {
      s = s + 1;
      r = 0;  // this indicator wont let this condition, (sm1 == sm2), to be counted until the next condition, (sm1 != sm2), happens
    }

    t = millis();  //updating time
    finish = 1;    //cghanging finish indicator
  }

  if (finish == 1) {  //this part of the code is for displaying the result
    delay(500);       //half second delay
    rep = rep + 1;    // increasing the repetition indicator

    angleNow = getAngle();
    //get change with ccw as +, decompose to direction and angle, convert to opposite if smaller
    //----------------------------------------------
    angleDiff = angleNow-anglePrev;
    if(angleDiff < 0 ){ //ACW-positive, determine rotation direction
      clockwise = 1;
    } else{
      clockwise = 0;
    }
    angleDiff = abs(angleDiff); //decompose to magnitude
    if(angleDiff>180){ // flip direction and get opposite angle if smaller.
      angleDiff = abs(360-angleDiff);
      clockwise = !clockwise;
    } 
    //----------------------------------------------
    anglePrev = angleNow;//set old to current


    Serial.print("shaft position from optical absolute sensor from home position: ");
    Serial.println(angleNow);

    Serial.print("shaft displacement from optical absolute sensor: ");
    Serial.print(angleDiff);
    if (clockwise) {
      Serial.println(" cw");
    } else {
      Serial.println(" ccw");
    }
    

    Serial.print("Shaft displacement from motor's builtin encoder: ");
    Serial.println(s * 360 / 228);  //every full Revolution of the shaft is associated with 228 counts of builtin
                                    //encoder so to turn it to degre we can use this formula (s * 360 / 228), "s" is the number of  built-in encoder counts

    float Error = angleDiff - s * 360 / 228; // ours minus theirs
    Errors[errIdx++] = Error;
    Serial.print("Error :");
    Serial.println(Error);  //displaying error
    Serial.println();
    s = 0;
    finish = 0;
    ++count;
  }
  analogWrite(6, 0);  //turning off the motor
  //-----------------------------------------------------------

  //write a summary if over the time and set stop flag
  if((count>10)&&(summaryDone == 0)){
    int largest1Idx = 0;
    int largest2Idx = 1;
    for (int i=2; i<10; i++){
      if(abs(Errors[i])>=abs(Errors[largest1Idx])){
        largest2Idx = largest1Idx;
        largest1Idx = i;
      } 
      else if (abs(Errors[i])>=abs(Errors[largest2Idx])){
        largest2Idx = i;
      }
    }

    float sum = 0;
    for (int i = 0; i<10; i++){
      if ((i!=largest1Idx)&&(i!=largest2Idx)){
        sum += abs(Errors[i]);
      }
    }
    Serial.print("largest error index: ");
    Serial.println(largest1Idx);
    Serial.print("2nd largest error index: ");
    Serial.println(largest2Idx);
    Serial.print("Average of magnitudes of best 8 errors: ");
    Serial.println(sum/8);
    summaryDone = 1;
  }
}
