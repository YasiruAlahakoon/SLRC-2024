#include <Servo.h>

#define Open 20
#define Close 110
#define Small 108
#define Medium 87
#define Large 50

Servo gripper;


void pick(int x){

  gripper.write(x);  
}

void release(){

  gripper.write(Open);
}
void setup() {
  // put your setup code here, to run once:
    gripper.attach(10); // gripper pin
    gripper.write(120);
    pick(0);

}

void loop() {
  // put your main code here, to run repeatedly:
  
    


}