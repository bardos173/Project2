//Note, Quad and Abs code can be added to the folder and called indepedantly, void loop and void set up name may need to be changed
//Possibly a switch function which clears and enables speficic variables added, this will be inplace of the respestive void setups
//Additions to void setup will need to be made



enum State {
START,
QUAD,
ABS
};

State currentState = START; 

void setup() {
  Serial.begin(250000); 
  PrintBorder();  
  Serial.println("Press 'Q' for Quadrature Mode and 'A' for Absoulute Mode");


}

void loop() {

while (currentState == QUAD){

check_change();
}

while (currentState == ABS){
check_change();
}
check_change();

}









void check_change (){

  if (Serial.available() > 0){

 char incomingChar = Serial.read();

    if (incomingChar == 'Q' | incomingChar == 'q') {
      currentState = QUAD;
      PrintBorder();
      Serial.println("Quadrature Encoder Mode Entered");

    } else if (incomingChar == 'A' | incomingChar == 'a') {
      currentState = ABS;
      PrintBorder();
      Serial.println("Absoulte Encoder Mode Entered");
    }
  }
}


void PrintBorder(){
  Serial.println(" ");
  Serial.println("─────────────────────────────────── °∘❉∘° ───────────────────────────────────");
  Serial.println(" ");
}

