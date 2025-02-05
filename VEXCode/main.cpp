#include "robot-config.h"

/*
        

*/

//Creates a competition object that allows access to Competition methods.
vex::competition Competition;

//FUNCTIONS
void pre_auton(void);       //SETTINGS FOR MOTORS, SCREEN ETC
void setUpMotor(motor(M));

//VARIABLES
double adjSpeed = 1.0;   //adjust drive sensitivity
double adjLift = 1.0;    //adjust arm's sensitivity
int deadBand = 10;      //range below which joystick is ignored
double adjField = 1.0;  //adjust automouse drive() degrees match conditions
bool clawClosed = 0;

int fwdDir = 1;

//MOVEMENT / CONTROL
int tLift();                                  //arm-relatecd tasks / buttons
void rDrive(double, double, int, int, bool);  //DRIVE BY RELATIVE DISTANCE
void rLift(double, int, bool);                //ARM BY RELATIVE DISTANCE 
void aLift(double, int, bool);                //ARM BY ABSOLUTE DISTANCE
void toggleClaw(void);

void autonomous( void ) {
    /*
	PUT AUTONOMOUS CODE HERE
    */
}//end autonomous

void usercontrol( void ) {
    //ONE TIME COMMANDS RAN B4 USER CONTROL
    
    //START TASK for LIFT
    vex::task t(tLift); //start task which updates controller screen
    t.setPriority(6);  //medium priority
    
    while (1){  
      //DRIVE
        L_Rear_Drive.spin(directionType::fwd, (Controller1.Axis1.value() + Controller1.Axis2.value())*adjSpeed, velocityUnits::pct);
        R_Rear_Drive.spin(directionType::fwd, (Controller1.Axis1.value() - Controller1.Axis2.value())*adjSpeed, velocityUnits::pct);
        L_Front_Drive.spin(directionType::fwd, (Controller1.Axis1.value() + Controller1.Axis2.value())*adjSpeed, velocityUnits::pct);
        R_Front_Drive.spin(directionType::fwd, (Controller1.Axis1.value() - Controller1.Axis2.value())*adjSpeed, velocityUnits::pct);
        vex::task::sleep(100); //Sleep the task for a short amount of time to prevent wasted resources. 
        
    }//end while
}


void toggleClaw(void){ //
  if(clawClosed){ //OPEN CLAW
        Claw.rotateFor(360, vex::rotationUnits::deg,50, vex::velocityUnits::pct, false);
        clawClosed = 0;
        vex::task::sleep(200);
  }
  else {
        Claw.rotateFor(-360, vex::rotationUnits::deg,50, vex::velocityUnits::pct, false);
        clawClosed = 1;
        vex::task::sleep(200);
  }
}//end vClaw

int tLift(void){ //ARM & CLAW TASK
    while(1){
      if(abs(Controller1.Axis3.value())>deadBand){  
        L_Lift.spin(vex::directionType::fwd, Controller1.Axis3.value()*adjLift, vex::velocityUnits::rpm);
        R_Lift.spin(vex::directionType::fwd, Controller1.Axis3.value()*adjLift, vex::velocityUnits::rpm);
      }else if(Controller1.ButtonL1.pressing()){    
          Claw.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct);
      }else if(Controller1.ButtonL2.pressing()){
          Claw.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
      }else if(Controller1.ButtonUp.pressing()){
          // do something like rDrive(50,20,1); 
      }else if(Controller1.ButtonR1.pressing()){
          //do something like call a function()
      }else{
          L_Lift.stop(brakeType::hold);
          R_Lift.stop(brakeType::hold);
          Claw.stop(brakeType::hold);
      }
      vex::task::sleep(20);
    }   
    return 0;
}//end tArm

void rDrive(double lDeg, double rDeg, int l, int r, bool b){  // drive by relative distance
    lDeg = lDeg * adjField; //adjust distance based on field
    rDeg = rDeg * adjField;
    L_Rear_Drive.rotateFor(lDeg * adjField, vex::rotationUnits::deg,l, vex::velocityUnits::pct, false); //This command must be non blocking.
    L_Front_Drive.rotateFor(lDeg* adjField, vex::rotationUnits::deg,l, vex::velocityUnits::pct, false); //This command must be non blocking.
    R_Rear_Drive.rotateFor(rDeg* adjField, vex::rotationUnits::deg,r, vex::velocityUnits::pct, false); //This command must be non blocking.
    R_Front_Drive.rotateFor(rDeg* adjField, vex::rotationUnits::deg,r, vex::velocityUnits::pct,b); //This command is blocking so the program will wait here until the right motor is done.  

    if(b){  // b = tue means wait for motors stop spinning or timeout
      while(R_Rear_Drive.isSpinning()||L_Rear_Drive.isSpinning())
        {}
    }
}//end eDrive

void aLift(double deg, int s, bool b){
    L_Lift.rotateTo(deg, vex::rotationUnits::deg,s, vex::velocityUnits::pct, false); //This command must be non blocking.
    R_Lift.rotateTo(deg, vex::rotationUnits::deg,s, vex::velocityUnits::pct, b); //This command is blocking so the program will wait here until the right motor is done.  
    
    if(b){  // b = tue means wait for motors stop spinning or timeout
      while(R_Lift.isSpinning()||L_Lift.isSpinning())
        {}
    }
}//end aArm

void rLift(double deg, int s, bool b){
    L_Lift.rotateFor(deg, vex::rotationUnits::deg,s, vex::velocityUnits::pct, false); //This command must be non blocking.
    R_Lift.rotateFor(deg, vex::rotationUnits::deg,s, vex::velocityUnits::pct,b); //This command is blocking so the program will wait here until the right motor is done.  
    
    if(b){  // b = tue means wait for motors stop spinning or timeout
      while(R_Lift.isSpinning()||L_Lift.isSpinning())
        {}
    }
}//end eArm


//SYSTEM SET-UP
void pre_auton(void) {  
  //set motor defayults 
	setUpMotor(L_Front_Drive);
	setUpMotor(L_Rear_Drive);
	setUpMotor(R_Front_Drive);
  setUpMotor(R_Rear_Drive);  
  setUpMotor(L_Lift);
  setUpMotor(R_Lift);
  setUpMotor(Claw);
    
    //clear controller display
    Controller1.Screen.clearScreen();
} //end pre_auton  

//DEFAULT MOTOR SETTINGS
void setUpMotor(motor(M)){
    M.setMaxTorque(90,percentUnits::pct); //set torque
    M.setBrake(brakeType::hold);          //braking
    M.setTimeout(3,timeUnits::sec);       //set motor timeout
    M.resetRotation();                    //reset encoders
}

// Main will set up the competition functions and callbacks.
int main() {    
    //Run the pre-autonomous function. 
    pre_auton();
    //Set up callbacks for autonomous and driver control periods.
    Competition.autonomous( autonomous );
    Competition.drivercontrol( usercontrol );
    //Prevent main from exiting with an infinite loop.                        
    while(1) {
      vex::task::sleep(100);//Sleep the task for a short amount of time to prevent wasted resources.
    } 
}
