//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#include "configure.h"
#include "robot_stewart.h"

#if MACHINE_STYLE == STEWART

#include "Vector3.h"

#define NUM_ARMS  (6)

struct EndEffector  {
  Vector3 up;
  Vector3 left;
  Vector3 forward;
  Vector3 pos;
  Vector3 relative;
  float r, p, y; // roll, pitch, yaw
};

struct StewartArm {
  Vector3 shoulder;
  Vector3 elbow;
  Vector3 shoulder_to_elbow;
  Vector3 wrist;

  float angle;
};

struct StewartPlatform {
  StewartArm arms[NUM_ARMS];
  EndEffector ee;
};


StewartPlatform robot;


void stewartDemo() {
  float pos[NUM_AXIES] = {0,0,0,0,0,0};
  int i,j;

  // linear moves
  for(j=0;j<3;++j) {
    for(i=0;i<5;++i) {
      pos[j]=2;
      lineSafe(pos,feed_rate);
      pos[j]=-2;
      lineSafe(pos,feed_rate);
    }
    pos[j]=0;
    lineSafe(pos,feed_rate);
  }
  
  // tilting
  for(j=4;j<6;++j) {
    for(i=0;i<5;++i) {
      pos[j]=10;
      lineSafe(pos,feed_rate);
      pos[j]=-10;
      lineSafe(pos,feed_rate);
    }
    pos[j]=0;
    lineSafe(pos,feed_rate);
  }
  
  // combos
  for(j=0;j<3;++j) {
    for(i=0;i<360*5;i+=10) {
      pos[ j     ]=2*sin(((float)i/180.0)*PI);
      pos[(j+1)%3]=2*cos(((float)i/180.0)*PI);
      lineSafe(pos,feed_rate);
    }
    pos[ j     ]=0;
    pos[(j+1)%3]=0;
    lineSafe(pos,feed_rate);
  }

  // combos rotation
  for(j=4;j<6;++j) {
    for(i=0;i<360*5;i+=10) {
      pos[    j        ]=10.0*sin(((float)i/180.0)*PI);
      pos[3+((j-3+1)%3)]=10.0*cos(((float)i/180.0)*PI);
      lineSafe(pos,feed_rate);
    }
    pos[    j        ]=0;
    pos[3+((j-3+1)%3)]=0;
    lineSafe(pos,feed_rate);
  }

  // combos random
  for(j=0;j<50;++j) {
    pos[0]=2*cos(((float)random(360)/180.0)*PI);
    pos[1]=2*cos(((float)random(360)/180.0)*PI);
    pos[2]=1.5*cos(((float)random(360)/180.0)*PI);
    pos[3]=10.0*cos(((float)random(360)/180.0)*PI);
    pos[4]=10.0*cos(((float)random(360)/180.0)*PI);
    pos[5]=10.0*cos(((float)random(360)/180.0)*PI);
    lineSafe(pos,feed_rate);
  }
  for(i=0;i<NUM_AXIES;++i) {
    pos[i]=0;
  }
  lineSafe(pos,feed_rate);
}

/**
   Update the end effector according to the desired motion
   @input mov final end effector position
   @input rpy final end effector roll pitch yaw (relative to base)
*/
void stewart_update_endeffector(Vector3 &mov, Vector3 &rpy) {
  // translation
  robot.ee.pos = mov;

  // roll pitch & yaw
  robot.ee.r = radians(rpy.x);
  robot.ee.p = radians(rpy.y);
  robot.ee.y = radians(rpy.z);
  robot.ee.up.set(0, 0, 1);
  robot.ee.forward.set(1, 0, 0);
  robot.ee.left.set(0, 1, 0);

  // roll
  Vector3 axis;
  axis.set(1, 0, 0);
  robot.ee.up.rotate(axis, robot.ee.r);
  robot.ee.forward.rotate(axis, robot.ee.r);
  robot.ee.left.rotate(axis, robot.ee.r);

  // pitch
  axis.set(0, 1, 0);
  robot.ee.up.rotate(axis, robot.ee.p);
  robot.ee.forward.rotate(axis, robot.ee.p);
  robot.ee.left.rotate(axis, robot.ee.p);

  // yaw
  axis.set(0, 0, 1);
  robot.ee.up.rotate(axis, robot.ee.y);
  robot.ee.forward.rotate(axis, robot.ee.y);
  robot.ee.left.rotate(axis, robot.ee.y);
}


/**
   Update the wrist positions according to the end effector
*/
void stewart_update_wrists() {
  Vector3 n1, o1;
  float c, s;
  int i;
  for (i = 0; i < NUM_ARMS / 2; ++i) {
    StewartArm &arma = robot.arms[i * 2 + 0];
    StewartArm &armb = robot.arms[i * 2 + 1];

    c = cos(i * PI * 2.0f / 3.0f);
    s = sin(i * PI * 2.0f / 3.0f);

    n1 = robot.ee.forward * c + robot.ee.left * s;
    o1 = robot.ee.forward * -s + robot.ee.left * c;

    arma.wrist = robot.ee.pos + robot.ee.relative + n1 * T2W_X - o1 * T2W_Y + robot.ee.up * T2W_Z;
    armb.wrist = robot.ee.pos + robot.ee.relative + n1 * T2W_X + o1 * T2W_Y + robot.ee.up * T2W_Z;
  }

#if VERBOSE > 0
  for (i = 0; i < NUM_ARMS; ++i) {
    StewartArm &arm = robot.arms[i];
    Serial.print(i);
    Serial.print("\twrist =");
    Serial.print(arm.wrist.x);
    Serial.print(F(","));
    Serial.print(arm.wrist.y);
    Serial.print(F(","));
    Serial.println(arm.wrist.z);
  }
#endif
}


/**
   update the elbow positions according to the wrists and shoulders, then find shoulder angle.
*/
void stewart_update_shoulder_angles() {
  Vector3 ortho, w, wop, temp, r;
  float a, b, d, r1, r0, hh, y, x;

  int i;
  for (i = 0; i < NUM_ARMS; ++i) {
    StewartArm &arm = robot.arms[i];

#if VERBOSE > 0
    Serial.print(i);
#endif

    // project wrist position onto plane of bicep (wop)
    ortho.x = cos((i / 2) * PI * 2.0f / 3.0f);
    ortho.y = sin((i / 2) * PI * 2.0f / 3.0f);
    ortho.z = 0;

    w = arm.wrist - arm.shoulder;

    a = w | ortho; //endeffector' distance
    wop = w - (ortho * a);
    //arm.wop.pos=wop + arm.shoulder;  // e' location
    //vector_add(arm.wop,wop,arm.shoulder);

    // because wop is projected onto the bicep plane, wop-elbow is not the same as wrist-elbow.
    // we need to find wop-elbow to calculate the angle at the shoulder.
    b = sqrt(FOREARM_LENGTH * FOREARM_LENGTH - a * a); // e'j distance

    // use intersection of circles to find elbow point.
    //a = (r0r0 - r1r1 + d*d ) / (2 d)
    r1 = b; // circle 1 centers on e'
    r0 = BICEP_LENGTH; // circle 0 centers on shoulder
    d = wop.Length();
    // distance from shoulder to the midpoint between the two possible intersections
    a = ( r0 * r0 - r1 * r1 + d * d ) / ( 2 * d );

#if VERBOSE > 0
    Serial.print("\tb =");
    Serial.println(b);
    Serial.print("\td =");
    Serial.println(d);
    Serial.print("\ta =");
    Serial.println(a);
#endif

    // normalize wop
    wop /= d;
    // find the midpoint
    temp = arm.shoulder + (wop * a);
    // with a and r0 we can find h, the distance from midpoint to intersections.
    hh = sqrt(r0 * r0 - a * a);
    // get a normal to the line wop in the plane orthogonal to ortho
    r = ortho ^ wop;
    if (i % 2 == 0) arm.elbow = temp + r * hh;
    else       arm.elbow = temp - r * hh;

    // use atan2 to find theta
    temp = arm.elbow - arm.shoulder;
    y = -temp.z;
    temp.z = 0;
    x = temp.Length();
    if ( ( arm.shoulder_to_elbow | temp ) < 0 ) x = -x;
    arm.angle = degrees(atan2(-y, x));
#if VERBOSE > 0
    Serial.print(i);
    Serial.print("\tangle =");
    Serial.println(arm.angle);
#endif
  }
}


/**
   Inverse Kinematics turns XY coordinates into step counts from each motor
   @param axies the cartesian coordinate
   @param motorStepArray a measure of each belt to that plotter position
*/
void IK(const float *const cartesian, long *motorStepArray) {
  float x = cartesian[0];
  float y = cartesian[1];
  float z = cartesian[2];
  float u = cartesian[3];
  float v = cartesian[4];
  float w = cartesian[5];

  Vector3 mov(x, y, z);
  Vector3 rpy(u, v, w);

  stewart_update_endeffector(mov, rpy);
  stewart_update_wrists();
  stewart_update_shoulder_angles();

  int i;
  for (i = 0; i < NUM_ARMS; ++i) {
    motorStepArray[i] = robot.arms[i].angle * MICROSTEP_PER_DEGREE;
  }
}


/**
   Forward Kinematics - turns step counts into XY coordinates
   @param motorStepArray a measure of each belt to that plotter position
   @param axies the resulting cartesian coordinate
   @return 0 if no problem, 1 on failure.
*/
int FK(long *motorStepArray, float *axies) {
  return 0;
}


/**
   Build a virtual model of the stewart shoulders for calculating angles later.
*/
void stewart_build_shoulders() {
  Vector3 n, o, n1, o1;
  float c, s;
  int i;
  for (i = 0; i < 3; ++i) {
    StewartArm &arma = robot.arms[i * 2 + 0];
    StewartArm &armb = robot.arms[i * 2 + 1];

    c = cos(i * PI * 2.0f / 3.0f);
    s = sin(i * PI * 2.0f / 3.0f);

    n = robot.ee.forward;
    o = robot.ee.up ^ robot.ee.forward;
    o.Normalize();

    n1 = n * c + o * s;
    o1 = n * -s + o * c;

    arma.shoulder = n1 * B2S_X - o1 * B2S_Y + robot.ee.up * B2S_Z;
    armb.shoulder = n1 * B2S_X + o1 * B2S_Y + robot.ee.up * B2S_Z;
    arma.elbow = n1 * B2S_X - o1 * (B2S_Y + BICEP_LENGTH) + robot.ee.up * B2S_Z;
    armb.elbow = n1 * B2S_X + o1 * (B2S_Y + BICEP_LENGTH) + robot.ee.up * B2S_Z;
    arma.shoulder_to_elbow = -o1;
    armb.shoulder_to_elbow = o1;
  }

#if VERBOSE > 0
  for (i = 0; i < 6; ++i) {
    StewartArm &arm = robot.arms[i];
    Serial.print(i);
    Serial.print("\ts =");
    Serial.print(arm.shoulder.x);
    Serial.print(F(","));
    Serial.print(arm.shoulder.y);
    Serial.print(F(","));
    Serial.print(arm.shoulder.z);

    Serial.print("\te =");
    Serial.print(arm.elbow.x);
    Serial.print(F(","));
    Serial.print(arm.elbow.y);
    Serial.print(F(","));
    Serial.println(arm.elbow.z);
  }
#endif
}


void robot_setup() {
}


void robot_findHome() {
  wait_for_empty_segment_buffer();
  motor_engage();

  findStepDelay();

  Serial.println(F("Finding..."));

  uint8_t i, hits;
  // back up until all switches are hitG
  do {
    hits = 0;
    // for each stepper,
    for (i = 0; i < NUM_MOTORS; ++i) {
      digitalWrite(motors[i].dir_pin, HIGH);
      // if this switch hasn't been hit yet
      if (digitalRead(motors[i].limit_switch_pin) == HIGH) {
        // move "down"
        //Serial.print('|');
        digitalWrite(motors[i].step_pin, HIGH);
        digitalWrite(motors[i].step_pin, LOW);
      } else {
        ++hits;
        //Serial.print('*');
      }
    }
    //Serial.println();
    pause(step_delay);
  } while (hits < NUM_MOTORS);

  // The arms are 19.69 degrees from straight down when they hit the switcrobot.
  // @TODO: This could be better customized in firmware.
  uint32_t steps_to_zero = SWITCH_ANGLE * MICROSTEP_PER_DEGREE;
  
  Serial.println(F("Homing..."));
#if VERBOSE > 0
  Serial.print("steps=");
  Serial.println(steps_to_zero);
#endif
  for (uint32_t j = 0; j < steps_to_zero; ++j) {
    for (i = 0; i < NUM_MOTORS; ++i) {
      digitalWrite(motors[i].dir_pin, LOW);
      digitalWrite(motors[i].step_pin, HIGH);
      digitalWrite(motors[i].step_pin, LOW);
    }
    //Serial.println(steps_to_zero-j,DEC);
    pause(step_delay);
  }

  // set robot to home position

  Vector3 zero(0, 0, 0);
  stewart_update_endeffector(zero, zero);
  stewart_build_shoulders();
  stewart_update_wrists();

  // find the starting height of the end effector at home position
  // @TODO: project wrist-on-bicep to get more accurate distance
  Vector3 el = robot.arms[0].elbow;
  Vector3 wr = robot.arms[0].wrist;
  float aa = (el.y - wr.y);
  float cc = FOREARM_LENGTH;
  float bb = sqrt((cc * cc) - (aa * aa));
  aa = el.x - wr.x;
  cc = bb;
  bb = sqrt((cc * cc) - (aa * aa));
  robot.ee.relative.set(0, 0, bb + B2S_Z - T2W_Z);

  float zeros[6] = {0, 0, 0, 0, 0, 0};
  teleport(zeros);
}



#endif
