#include "nuke.h"

/* IK Engine */
ik_req_t endpoints[LEG_COUNT];
float bodyRotX = 0;             // body roll (rad)
float bodyRotY = 0;             // body pitch (rad)
float bodyRotZ = 0;             // body rotation (rad)
int bodyPosX = 0;               // body offset (mm)
int bodyPosY = 0;               // body offset (mm)
int Xspeed;                     // forward speed (mm/s)
int Yspeed;                     // sideward speed (mm/s)
float Rspeed;                   // rotation speed (rad/s)

/* Gait Engine */
int gaitLegNo[LEG_COUNT];       // order to step through legs
ik_req_t gaits[LEG_COUNT];      // gait engine output
int pushSteps;                  // how much of the cycle we are on the ground
int stepsInCycle;               // how many steps in this cycle
int step;                       // current step
int tranTime;
int liftHeight;
float cycleTime;                // cycle time in seconds (adjustment from speed to step-size)


/* Setup the starting positions of the legs. */
void setupIK() {
  endpoints[RIGHT_FRONT].x = 40;
  endpoints[RIGHT_FRONT].y = 100; 
  endpoints[RIGHT_FRONT].z = 40;  

  endpoints[RIGHT_REAR].x = -40;
  endpoints[RIGHT_REAR].y = 100;
  endpoints[RIGHT_REAR].z = 40;

  endpoints[LEFT_FRONT].x = 40;
  endpoints[LEFT_FRONT].y = -100;
  endpoints[LEFT_FRONT].z = 40;

  endpoints[LEFT_REAR].x = -40;
  endpoints[LEFT_REAR].y = -100;
  endpoints[LEFT_REAR].z = 40;

  liftHeight = 25;
  stepsInCycle = 1;
  step = 0;
}

#include "gaits.h"

/* Convert radians to servo position offset. */
int radToServo(float rads) {
  float val = rads * 195.56959407132f;
  return (int) val;
}

/* Body IK solver: compute where legs should be. */
ik_req_t bodyIK(int X, int Y, int Z, int Xdisp, int Ydisp, float Zrot) {
  ik_req_t ans;

  float cosB = cos(bodyRotX);       // 1
  float sinB = sin(bodyRotX);       // 0
  float cosG = cos(bodyRotY);       // 1
  float sinG = sin(bodyRotY);       // 0
  float cosA = cos(bodyRotZ + Zrot); // 1
  float sinA = sin(bodyRotZ + Zrot); // 0

  int totalX = X + Xdisp + bodyPosX;
  int totalY = Y + Ydisp + bodyPosY;

  ans.x = totalX - int(totalX * cosG * cosA + totalY * sinB * sinG * cosA + Z * cosB * sinG * cosA - totalY * cosB * sinA + Z * sinB * sinA) + bodyPosX;
  ans.y = totalY - int(totalX * cosG * sinA + totalY * sinB * sinG * sinA + Z * cosB * sinG * sinA + totalY * cosB * cosA - Z * sinB * cosA) + bodyPosY;
  ans.z = Z - int(-totalX * sinG + totalY * sinB * cosG + Z * cosB * cosG);

  return ans;
}

/* Simple 3dof leg solver. X,Y,Z are the length from the Coxa rotate to the endpoint. */
ik_sol_t legIK(int X, int Y, int Z) {
  ik_sol_t ans;

  // first, make this a 2DOF problem... by solving coxa
  ans.coxa = radToServo(atan2(X, Y));
  long trueX = sqrt(sq((long)X) + sq((long)Y)) - L_COXA;
  long im = sqrt(sq((long)trueX) + sq((long)Z));  // length of imaginary leg

  // get femur angle above horizon...
  float q1 = -atan2(Z, trueX);
  long d1 = sq(L_FEMUR) - sq(L_TIBIA) + sq(im);
  long d2 = 2 * L_FEMUR * im;
  float q2 = acos((float)d1 / (float)d2);
  ans.femur = radToServo(q1 + q2);

  // and tibia angle from femur...
  d1 = sq(L_FEMUR) - sq(im) + sq(L_TIBIA);
  d2 = 2 * L_TIBIA * L_FEMUR;
  ans.tibia = radToServo(acos((float)d1 / (float)d2) - 1.57);
  return ans;
}

void doIK() {
  int pos;
  ik_req_t req, gait;
  ik_sol_t sol;

  gaitSetup();

  // right front leg
  gait = gaitGen(RIGHT_FRONT);
  req = bodyIK(endpoints[RIGHT_FRONT].x + gait.x, endpoints[RIGHT_FRONT].y + gait.y, endpoints[RIGHT_FRONT].z + gait.z, X_COXA, Y_COXA, gait.r);
  sol = legIK(endpoints[RIGHT_FRONT].x + req.x + gait.x, endpoints[RIGHT_FRONT].y + req.y + gait.y, endpoints[RIGHT_FRONT].z + req.z + gait.z);

  pos = 190 - map(sol.coxa, 0, 1023, 0, 300);
  servo(servoID[0][0], pos);
  //Serial.print(pos); Serial.println(" FR0 ");

  pos = 130 - map(sol.femur, 0, 1023, 0, 300);
  servo(servoID[0][1], pos);
  // Serial.print(pos); Serial.println(" FR1 ");

  pos = 135 + map(sol.tibia, 0, 1023, 0, 300);
  servo(servoID[0][2], pos);
  //Serial.print(pos); Serial.println(" FR2 ");


  // right rear leg
  gait = gaitGen(RIGHT_REAR);
  req = bodyIK(endpoints[RIGHT_REAR].x + gait.x, endpoints[RIGHT_REAR].y + gait.y, endpoints[RIGHT_REAR].z + gait.z, -X_COXA, Y_COXA, gait.r);
  sol = legIK(-endpoints[RIGHT_REAR].x - req.x - gait.x, endpoints[RIGHT_REAR].y + req.y + gait.y, endpoints[RIGHT_REAR].z + req.z + gait.z);

  pos = 105 + map(sol.coxa, 0, 1023, 0, 300);
  servo(servoID[1][0], pos);
  //Serial.print(pos); Serial.println(" RR0 ");

  pos = 130 - map(sol.femur, 0, 1023, 0, 300);
  servo(servoID[1][1], pos);

  pos = 135 + map(sol.tibia, 0, 1023, 0, 300);
  servo(servoID[1][2], pos);


  // left front leg
  gait = gaitGen(LEFT_FRONT);
  req = bodyIK(endpoints[LEFT_FRONT].x + gait.x, endpoints[LEFT_FRONT].y + gait.y, endpoints[LEFT_FRONT].z + gait.z, X_COXA, -Y_COXA, gait.r);
  sol = legIK(endpoints[LEFT_FRONT].x + req.x + gait.x, -endpoints[LEFT_FRONT].y - req.y - gait.y, endpoints[LEFT_FRONT].z + req.z + gait.z);

  pos = 105 + map(sol.coxa, 0, 1023, 0, 300);
  servo(servoID[2][0], pos);
  //Serial.print(pos); Serial.println(" LF0 ");

  pos = 130 - map(sol.femur, 0, 1023, 0, 300);
  servo(servoID[2][1], pos);

  pos = 135 + map(sol.tibia, 0, 1023, 0, 300);
  servo(servoID[2][2], pos);

  // left rear leg
  gait = gaitGen(LEFT_REAR);
  req = bodyIK(endpoints[LEFT_REAR].x + gait.x, endpoints[LEFT_REAR].y + gait.y, endpoints[LEFT_REAR].z + gait.z, -X_COXA, -Y_COXA, gait.r);
  sol = legIK(-endpoints[LEFT_REAR].x - req.x - gait.x, -endpoints[LEFT_REAR].y - req.y - gait.y, endpoints[LEFT_REAR].z + req.z + gait.z);

  pos = 190 - map(sol.coxa, 0, 1023, 0, 300);
  servo(servoID[3][0], pos);
  //Serial.print(pos); Serial.println(" LR0 ");

  pos = 130 - map(sol.femur, 0, 1023, 0, 300);
  servo(servoID[3][1], pos);

  pos = 135 + map(sol.tibia, 0, 1023, 0, 300);
  servo(servoID[3][2], pos);

  delay(30); //25
  step = (step + 1) % stepsInCycle;
}
