/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Bobby                                                     */
/*    Created:      1/10/2024, 10:52:32 AM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"


using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
#define PI 3.14159265

//Controller and Brain Defined
controller Controller1 = controller(primary);
brain Brain;


//Motor Defined
motor RF = motor(PORT15, ratio6_1, false);

motor RM = motor(PORT5, ratio6_1, true);

motor RB = motor(PORT4, ratio6_1, false);

motor LF = motor(PORT16, ratio6_1, true);

motor LM = motor(PORT6, ratio6_1, false);

motor LB = motor(PORT7, ratio6_1, true);

motor LWing = motor(PORT17, ratio18_1, true);

motor RWing = motor(PORT14, ratio18_1, false);

//Sensors Defined

inertial Inertial1 = inertial(PORT10);

rotation BTrack = rotation(PORT1, true);

rotation ATrack = rotation(PORT11, false);

pot Rpot = pot(Brain.ThreeWirePort.A);

pot Lpot = pot(Brain.ThreeWirePort.H);

distance Fvis = distance(PORT3);

distance Bvis = distance(PORT2);


//Odometry Task-------------------------------------------

//BOT INITIAL POSITION AND DIRECTION VALUES

//front left corner (the tip of the piece of channel with the wing joint) is 24 inches from the wall.
//Back left corner of bot 12.5 in from wall
float initialDir = 18.25; //0 when facing Red Goal
//(x,y) == (0,0) at bottom right when facing Red goal
float initialX = 21.5; //increases as you approach Red Goal
float initialY = 18.5; //increases from right to left when facing red goal

float Dir = initialDir; //Live Direction
float x = initialX; //Live X
float y = initialY; //Live Y
float truex = 0; //Position of center of bot
float truey = 0; //Position of center of bot
float prevA = 0; //sideways
float prevB = 0; //forward
float prevDir = initialDir;

//Robot Physical Constants
float ADFC = 5.625; //A distance from center in inches
float BDFC = -.125;//B distance from center in inches
float SWC = (3.25 * PI); //small wheel circumference

int MyGyroTask()
{
Inertial1.calibrate();
Inertial1.setRotation(initialDir, degrees);                   //Set Gyro to initial
Inertial1.setHeading(initialDir, degrees);
ATrack.setPosition(0,degrees);
BTrack.setPosition(0,degrees);
while(1)
{
 //Gyro math//
 Dir = Inertial1.rotation(degrees) * (360/358.725); //Because the gyro is off
 //Position math//
 float A = ATrack.position(turns) * SWC;
 float B = BTrack.position(turns) * SWC;
 x += -sin(((Dir + prevDir)/2)*PI/180) * (A - prevA) + cos(((Dir + prevDir)/2)*PI/180) * (B - prevB);
 y += -cos(((Dir + prevDir)/2)*PI/180) * (A - prevA) + -sin(((Dir + prevDir)/2)*PI/180) * (B - prevB);
 truey = y + ((sin(Dir * PI / 180) * ADFC) + (-cos(Dir*PI/180) * BDFC));
 truex = x + ((-cos(Dir * PI / 180) * ADFC) + (-sin(Dir*PI/180) * BDFC));
 prevA = A;
 prevB = B;
 prevDir = Dir;
 //Direction math//

 vex::task::sleep(10);                              //cuts cpu usage
}
return(0);
}
vex::task t(MyGyroTask);


//Print Task--------------------------------------------------------
float ReadVar1 = 0;
int PrintTask(){
  while(1){

    // printf("RV1 %.2f", ReadVar1);
    // printf(" fvis %.2f", Fvis.objectDistance(inches));
    // printf(" bvis %.2f", Bvis.objectDistance(inches));
    printf(" lpot %.2f", Lpot.angle(degrees));
    printf("Gyro %.2f", Dir);
    // // printf(" xTrack %.2f", x);
    // // printf(" yTrack %.2f", y);
    printf(" xtrue %.2f", truex);
    printf(" ytrue %.2f\n", truey);
    // printf(" angle %.2f\n", LeftScoop.position(degrees));

    // Controller1.Screen.clearScreen();
    Brain.Screen.clearScreen();
    vex::task::sleep(100);
  }
  return(0);
}
vex::task p(PrintTask);

/*-----------------------------------------------------------------------------
                        FUNCTIONS FOR AUTONOUMOUS PERIOD

-----------------------------------------------------------------------------*/

//DRIVE FUNCTIONS~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//Turn Function-------------------------
//turn to face a certain direction


//Tuning Values
float Turngain = .1;
float TurnDgain = .5;
//```````````````````

void Turn(float Ang){
  float ATurnSpeed = 0;
  bool TurnLoop = true;
  int stillCounter = 0;
  float prevAngle = Dir;
  int msecPassed = 0;
  float error = 0;
  float prevError = 0;

  while(TurnLoop){

    error = Ang - Dir;
    ATurnSpeed = error * Turngain;
    ATurnSpeed += (error - prevError) * TurnDgain;

    if(ATurnSpeed > 12){
      ATurnSpeed = 12;
    }
    if(ATurnSpeed < -12){
      ATurnSpeed = -12;
    }


    LF.spin(forward, ATurnSpeed, volt);
    LM.spin(forward, ATurnSpeed, volt);
    LB.spin(forward, ATurnSpeed, volt);
    
    RF.spin(reverse, ATurnSpeed, volt);
    RM.spin(reverse, ATurnSpeed, volt);
    RB.spin(reverse, ATurnSpeed, volt);

    if(fabs(Dir - prevAngle) < .05 && msecPassed > 150){
      stillCounter ++;
    } else {
      stillCounter = 0;
    }
    if(stillCounter > 10){
      TurnLoop = false;
    }
    prevAngle = Dir;
    prevError = error;

    msecPassed += 10;
    wait(10, msec);
  }
}

//Point-Turn Function------------------------------
//turns to face a specific point on the field

//Similar Tuning ^
float Tgain = .1;
float TDgain = .5;
//``````````````````````

void PTurn(float tx, float ty, bool fwd = true){
  float ATurnSpeed = 0;
  bool TurnLoop = true;
  int stillCounter = 0;
  float prevAngle = Dir;
  int msecPassed = 0;
  float error;
  float prevError;
  float tarAng;
  float DirDrive;

  while(TurnLoop){
    DirDrive = fmod(Dir, 360.0);
    if(!fwd){
      DirDrive += 180;
      if(DirDrive > 360){
        DirDrive -= 360;
      }
    }

    tarAng = (atan((tx - truex)/(ty - truey))) * 180 / PI;
    if(truey <= ty) {
      tarAng += 270;
    }else{
      tarAng += 90;
    }

    error = tarAng - DirDrive;
    if(error > 180){
      error -= 360;
    }
    ATurnSpeed = error * Tgain;
    ATurnSpeed += (error - prevError) * TDgain;

    if(ATurnSpeed > 12){
      ATurnSpeed = 12;
    }
    if(ATurnSpeed < -12){
      ATurnSpeed = -12;
    }


    LF.spin(forward, ATurnSpeed, volt);
    LM.spin(forward, ATurnSpeed, volt);
    LB.spin(forward, ATurnSpeed, volt);
    
    RF.spin(reverse, ATurnSpeed, volt);
    RM.spin(reverse, ATurnSpeed, volt);
    RB.spin(reverse, ATurnSpeed, volt);

    if(fabs(Dir - prevAngle) < .05 && msecPassed > 150){
      stillCounter ++;
    } else {
      stillCounter = 0;
    }
    if(stillCounter > 10){
      TurnLoop = false;
    }
    prevAngle = Dir;
    prevError = error;

    msecPassed += 10;
    wait(10, msec);
  }
}

//Point-Drive function------------------------Tuned-ish, could be tuned more
//Drives to a point

float ktf = 0;//turn correction feed forward constant
float ktp = .03;//turn correction proportional constant
float ktd = 0; //suspicion that it only works well one way
float turnBand = 0;
float kti = 0;
float kp = .6;//proportional constant
float kd = .4;


void PDrive(float tx, float ty, float Speed = .95, float timeOut = 15000, bool fwd = true){
  float PIDinputVolt;
  float Gerror; //direction error
  float prevGerror;
  float Derror; //distance error
  float prevDerror;
  float compensate = ktf;
  float Lcomp = 1;
  float Rcomp = 1;
  float Itotal = 0;
  int msecPassed = 0;
  int stillCounter = 0;
  bool PIDEnabled = true;
  
  while(PIDEnabled){

    int DirDrive = fmod(Dir, 360.0);
    if(!fwd){
      DirDrive += 180;
      if(DirDrive > 360){
        DirDrive -= 360;
      }
    }

    int tarAng = (atan((tx - truex)/(ty - truey))) * 180 / PI;
    if(truey <= ty) {
      tarAng += 270;
    }else{
      tarAng += 90;
    }

    Gerror = tarAng - DirDrive;
    if(Gerror > 180){
      Gerror -= 360;
    }

    Derror = sqrt((truex - tx) * (truex - tx) + (truey - ty) * (truey - ty));
    Derror *= cos(Gerror * PI / 180);

    if(Gerror > 90){
      Gerror -= 180;
    } else if(Gerror < -90){
      Gerror += 180;
    }


    compensate = ktf;
    compensate -= ((prevGerror - Gerror) * ktd);
    compensate += ((Gerror) * ktp);
    compensate += Itotal * kti;
    if(Derror < 12 && Derror > -12){
      compensate *= (fabs(Derror) / 12);
    }

    if(compensate < 0 && Derror > 0){
      Lcomp = 1 + compensate;
      Rcomp = 1;
    }
    if(compensate > 0 && Derror > 0){
      Rcomp = 1 - compensate;
      Lcomp = 1;
    }
    if(compensate < 0 && Derror < 0){
      Rcomp = 1 + compensate;
      Lcomp = 1;
    }
    if(compensate > 0 && Derror < 0){
      Lcomp = 1 - compensate;
      Rcomp = 1;
    }

    PIDinputVolt = Derror * kp;
    PIDinputVolt += (Derror - prevDerror) * kd;

    
    if(PIDinputVolt > 12){
      PIDinputVolt = 12;
    }
    if(PIDinputVolt < -12){
      PIDinputVolt = -12;
    }

    LF.spin(forward, PIDinputVolt * Speed * Lcomp, volt);
    LM.spin(forward, PIDinputVolt * Speed * Lcomp, volt);
    LB.spin(forward, PIDinputVolt * Speed * Lcomp, volt);
    RF.spin(forward, PIDinputVolt * Speed * Rcomp, volt);
    RM.spin(forward, PIDinputVolt * Speed * Rcomp, volt);
    RB.spin(forward, PIDinputVolt * Speed * Rcomp, volt);  

    if(fabs(Derror - prevDerror) < .01 && msecPassed > 100){
      stillCounter ++;
    } else {
      stillCounter = 0;
    }
    if(stillCounter > 13){
      PIDEnabled = false;
    }

    ReadVar1 = Derror;

    if(fabs(Gerror) > turnBand){
      Itotal += (Gerror) * Derror;
    }
    if(msecPassed > timeOut){
      PIDEnabled = false;
    }


    prevDerror = Derror;
    prevGerror = Gerror;

    msecPassed += 10;
    wait(10, msec);
  }
  LF.stop(brake);
  LM.stop(brake);
  LB.stop(brake);
  RF.stop(brake); 
  RM.stop(brake);
  RB.stop(brake);
}

//Through-Drive function------------------------NOT tuned or checked Edits to be made
//Drives through an array of points

float Aktf = 0;//turn correction feed forward constant
float Aktp = .02;//turn correction proportional constant
float Aktd = .024;
float AktDis = 0;//.0085;
float AturnBand = .05;
float Akti = 0;
float Akp = .6;//proportional constant
float Akd = .7;
float Glidekp = .055; //less is faster

void ThrDrive(float XCoordArray[], float YCoordArray[], int arraySize = 1, float Speed = .95, float timeOut = 15000, bool fwd = true){
  
  

  for(int i = 0; i < arraySize; i++ ){  
    float PIDinputVolt;
    float Gerror; //direction error
    float prevGerror;
    float Derror; //distance error
    float prevDerror;
    float compensate = ktf;
    float beginx = truex;
    float beginy = truey;
    float Lcomp = 1;
    float Rcomp = 1;
    float Itotal = 0;
    int msecPassed = 0;
    int stillCounter = 0;
    bool PIDEnabled = true;

    //Calculate exit
    //We need find a line that goes through the end point and is perpendicular to the line that goes through starting point and through the end point
    float slope;
    if(i != 0){
      slope = (XCoordArray[i] - XCoordArray[i-1])/(YCoordArray[i] - YCoordArray[i-1]);//BUG WITH THIS LINE IF BOTH Y COORDS ARE SAME
    } else {
      slope = (XCoordArray[i] - beginx)/(YCoordArray[i] - beginy);
    }


    while(PIDEnabled){
      int DirDrive = fmod(Dir, 360.0);
      if(!fwd){
        DirDrive += 180;
        if(DirDrive > 360){
          DirDrive -= 360;
        }
      }

      int tarAng = (atan((XCoordArray[i] - truex)/(YCoordArray[i] - truey))) * 180 / PI;
      if(truey <= YCoordArray[i]) {
        tarAng += 270;
      }else{
        tarAng += 90;
      }

      Gerror = tarAng - DirDrive;
      if(Gerror >= 180){
        Gerror -= 360;
      } else if(Gerror <= -180){ //JUST ADDED MAY NEED TO ADD TO OTHER
        Gerror += 360;
      }

      Derror = sqrt((truex - XCoordArray[i]) * (truex - XCoordArray[i]) + (truey - YCoordArray[i]) * (truey - YCoordArray[i]));
      if(Derror < 2){
        Derror *= cos(Gerror * PI / 180);
      }

      compensate = Aktf;
      compensate += (fabs(Gerror - prevGerror) * -Aktd);
      compensate += (fabs(Gerror) * Aktp);
      compensate += (fabs(Itotal * Akti));
      compensate += Derror * -AktDis;

      if(compensate > 2){
        compensate = 2;
      }
      if(PIDinputVolt < 0){
        compensate = 0;
      }

      if(Gerror > 0){
        Rcomp = 1 - compensate;
        Lcomp = 1;
      }
      if(Gerror < 0){
        Lcomp = 1 - compensate;
        Rcomp = 1;
      }
      
      if(i == arraySize - 1){
        PIDinputVolt = Derror * Akp;
        PIDinputVolt += (Derror - prevDerror) * Akd;
      } else {
        PIDinputVolt = 12;
        PIDinputVolt -= (fabs(Gerror) * Glidekp);
      }
      
      if(PIDinputVolt > 12){
        PIDinputVolt = 12;
      }
      if(PIDinputVolt < -12){
        PIDinputVolt = -12;
      }



      if(i == arraySize - 1){
        if(fabs(Derror - prevDerror) < .01 && msecPassed > 100){
          stillCounter ++;
        } else {
          stillCounter = 0;
        }
        if(stillCounter > 13){
          PIDEnabled = false;
        }
      } else {
        if(i == 0){
          if(y + (slope * x) > (slope * XCoordArray[i]) + YCoordArray[i] && YCoordArray[i] > beginy) {
            PIDEnabled = false;
          } else if(y + (slope * x) < (slope * XCoordArray[i]) + YCoordArray[i] && YCoordArray[i] < beginy) {
            PIDEnabled = false;
          }
        } else {
          if(y + (slope * x) > (slope * XCoordArray[i]) + YCoordArray[i] && YCoordArray[i] > YCoordArray[i-1]) {
            PIDEnabled = false;
          } else if(y + (slope * x) < (slope * XCoordArray[i]) + YCoordArray[i] && YCoordArray[i] < YCoordArray[i-1]) {
            PIDEnabled = false;
          }
        }

      }

      ReadVar1 = Gerror;

      if(fabs(Gerror) > AturnBand){
        Itotal += (Gerror) * Derror;
      }
      
      if(msecPassed > timeOut){
        PIDEnabled = false;
      }


      prevDerror = Derror;
      prevGerror = Gerror;

      LF.spin(forward, PIDinputVolt * Speed * Lcomp, volt);
      LM.spin(forward, PIDinputVolt * Speed * Lcomp, volt);
      LB.spin(forward, PIDinputVolt * Speed * Lcomp, volt);
      RF.spin(forward, PIDinputVolt * Speed * Rcomp, volt);
      RM.spin(forward, PIDinputVolt * Speed * Rcomp, volt);
      RB.spin(forward, PIDinputVolt * Speed * Rcomp, volt);

      msecPassed += 10;
      wait(10, msec);
    }
    LF.stop(brake);
    LM.stop(brake);
    LB.stop(brake);
    RF.stop(brake); 
    RM.stop(brake);
    RB.stop(brake);
  }
}

void PowerDrive(float timew, float vlt){
      LF.spin(forward, vlt, volt);
      LM.spin(forward, vlt, volt);
      LB.spin(forward, vlt, volt);
      RF.spin(forward, vlt, volt);
      RM.spin(forward, vlt, volt);
      RB.spin(forward, vlt, volt);
      wait(timew, seconds);
      LF.stop(brake);
      LM.stop(brake);
      LB.stop(brake);
      RF.stop(brake); 
      RM.stop(brake);
      RB.stop(brake);
}

//open wings function
void OpenRWing(){
  bool stopWings = false;
  bool limitPressed = false;
  RWing.spin(forward, 12, volt);
  while(!stopWings){
    float angel = Rpot.angle(degrees);
    if(angel > 58){
      limitPressed = true;
    }
    if(limitPressed && angel <= 50){
      stopWings = true;
    }

    wait(10, msec);
  }
  RWing.stop(brake);
}

void CloseRWing(){
  RWing.spin(forward, 12, volt);
  while(Rpot.angle(degrees) < 55){
    wait(10, msec);
  }
  RWing.stop(brake);
}

void OpenLWing(){
  bool stopWings = false;
  bool limitPressed = false;
  LWing.spin(forward, 12, volt);
  while(!stopWings){
    float angel = Lpot.angle(degrees);
    if(angel > 58){
      limitPressed = true;
    }
    if(limitPressed && angel <= 50){
      stopWings = true;
    }

    wait(10, msec);
  }
  LWing.stop(brake);
}

void CloseLWing(){
  LWing.spin(forward, 12, volt);
  while(Lpot.angle(degrees) < 55){
    wait(10, msec);
  }
  LWing.stop(brake);
}


void Smack(float pause = 0){
  RWing.spin(forward, 12, volt);
  while(Rpot.angle(degrees) < 57){
    wait(10, msec);
  }
  RWing.stop(brake);
  wait(pause, msec);

  bool stopWings = false;
  bool limitPressed = false;
  RWing.spin(forward, 12, volt);
  while(!stopWings){
    float angel = Rpot.angle(degrees);
    if(angel > 58){
      limitPressed = true;
    }
    if(limitPressed && angel <= 50){
      stopWings = true;
    }

    wait(10, msec);
  }
  RWing.spin(forward, 6, volt);
  wait(200, msec);
  RWing.stop(brake);
}


void Clench(){
  RF.stop(hold);
  LF.stop(hold);
  RM.stop(hold);
  LM.stop(hold);
  RB.stop(hold);
  LB.stop(hold);
}

void Relief(){
  RF.setStopping(coast);
  LF.setStopping(coast);
  RM.setStopping(coast);
  LM.setStopping(coast);
  RB.setStopping(coast);
  LB.setStopping(coast);
}







/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  //FUNCTIONS
  //void Turn(float Ang)
  //void PTurn(float tx, float ty, bool fwd = true)
  //void PDrive(float tx, float ty, float Speed = .95, float timeOut = 15000, bool fwd = true)
  //void ThrDrive(float XCoordArray[], float YCoordArray[], int arraySize, float Speed = .95, float timeOut = 15000, bool fwd = true)
  //void Smack(float pause = 0)
  //void CloseRWing()
  //void OpenRWing()

  wait(1.5, seconds);//for gyro to calibrate can probably be removed for comp
  
  //Launching Triballs ~35 seconds

  /*OpenLWing();*/
  Clench();
  for (int i = 0; i < 25; i++)
  {
    Smack(200);
  }
  Relief();

  //Push triballs passed line

  CloseRWing();

  PDrive(36, 8);
  
  while(x < 54){
    PTurn(60, 8);
    PDrive(60, 8);
  }
  
  while(x > 36){
    PTurn(36, 14, false);
    PDrive(36, 14);
  }

  /*CloseLWing()*/

  //Push balls over barrier and become wall

  PTurn(36, 36);
  PDrive(36, 36);
  PTurn(66, 36);
  PDrive(66, 36); 
  PDrive(60, 36);
  PTurn(48, 90);
  OpenRWing();
  /*OpenLWing()*/
  PDrive(48, 90);
  wait(2, seconds);

  //return to alley and push balls down alley

  CloseRWing();
  /*CloseLWing()*/
  PDrive(48, 72);
  PTurn(36, 10);
  PDrive(36, 10);

  PTurn(84, 10); //possibly get rid of this turn and substitute for 0 turn and do wall check
  PDrive(84, 10);
  /*OpenLWing()*/
  PTurn(108, 12);
  PDrive(108, 12);
  

  //push balls in side

  PTurn(132, 30);
  PDrive(132, 30);
  PTurn(132, 48);
  PowerDrive(1, 12);
  PTurn(135, 30);
  PDrive(135, 30);
  PowerDrive(1, 12);
  PTurn(135, 30);
  PDrive(135, 30);
  
  //corral balls to middle and push in

  PTurn(108, 36);
  PDrive(108, 36);
  OpenRWing();
  PTurn(96, 54);
  PDrive(96, 54);
  PTurn(120, 54);
  PowerDrive(1, 12);
  CloseRWing();
  /*CloseLWing();*/
  PDrive(84, 60);
  PTurn(120, 60);
  PowerDrive(2, 12);
  PDrive(84, 60);
  PTurn(120, 60);
  PowerDrive(2, 12);








}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop

  float rightDriveVelocity = 0;
  float leftDriveVelocity = 0;

  while (1) {

    Controller1.ButtonA.pressed(OpenRWing);
    Controller1.ButtonB.pressed(CloseRWing);

    if(Controller1.ButtonL1.pressing()){
      LWing.spin(forward, 12, volt);
    }
    if(Controller1.ButtonL2.pressing()){
      LWing.stop(brake);
    }
    
    if(Controller1.ButtonR1.pressing()){
      RWing.spin(forward, 12, volt);
    }
    if(Controller1.ButtonR2.pressing()){
      RWing.stop(brake);
    }

    rightDriveVelocity = Controller1.Axis2.position();
    leftDriveVelocity = Controller1.Axis3.position();

    RF.setVelocity(rightDriveVelocity, percent);
    RM.setVelocity(rightDriveVelocity, percent);
    RB.setVelocity(rightDriveVelocity, percent);
    LF.setVelocity(leftDriveVelocity, percent);
    LM.setVelocity(leftDriveVelocity, percent);
    LB.setVelocity(leftDriveVelocity, percent);

    RF.spin(forward);
    RM.spin(forward);
    RB.spin(forward);
    LF.spin(forward);
    LM.spin(forward);
    LB.spin(forward);

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
