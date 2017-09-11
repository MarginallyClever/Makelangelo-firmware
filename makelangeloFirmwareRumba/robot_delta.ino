//------------------------------------------------------------------------------
// Makelangelo - a mural drawing robot
// dan@marginallycelver.com 2013-12-26
// Copyright at end of file.  Please see
// http://www.github.com/MarginallyClever/Makelangelo for more information.
//------------------------------------------------------------------------------

#if MACHINE_STYLE == DELTA


struct Joint {
  Vector3 pos;
  Vector3 relative;
};


struct Arm {
  Vector3 shoulder;
  Joint elbow;
  Joint wrist;
  Joint wop;
  
  float angle;

  // for motors
  int new_step;
  
  int delta;
  int absdelta;
  int dir;
  int over;

  Vector3 plane_ortho;
  Vector3 plane_normal;
};


struct DeltaRobot {
  Arm arms[NUM_AXIES];
  Vector3 ee;  // current position of the end effector
  float e;  // rotation of the 4th axis
  Joint base;
  float default_height;
  int reverse;
};


DeltaRobot robot;

/**
 * Inverse Kinematics turns XY coordinates into step counts from each motor
 * @param axies the cartesian coordinate
 * @param motorStepArray a measure of each belt to that plotter position
 */
void IK(float *axies, long *motorStepArray) {
  float x = axies[0];
  float y = axies[1];
  float z = axies[2];
  
  int i;

  // update wrist positions
  for(i=0;i<NUM_AXIES;++i) {
    Arm &arm=robot.arms[i];

    arm.wrist.pos = robot.ee + arm.wrist.relative;
  }

  // update elbows
  float a,b,c,r1,r0,d,h;
  Vector3 r,p1,mid,wop,w,n;
  for(i=0;i<NUM_AXIES;++i) {
    Arm &arm=robot.arms[i];

    // get wrist position on plane of bicep
    w = arm.wrist.pos - arm.shoulder;
    
    a = w | arm.plane_normal;  // ee' distance
    wop = w - arm.plane_normal * a;
    arm.wop.pos = wop + arm.shoulder;
    
    // use intersection of circles to find two possible elbow points.
    // the two circles are the bicep (shoulder-elbow) and the forearm (elbow-arm.wop.pos)
    // the distance between circle centers is wop.Length()
    //a = (r0r0 - r1r1 + d*d ) / (2 d) 
    r1=sqrt(ELBOW_TO_WRIST*ELBOW_TO_WRIST-a*a);  // circle 1 centers on wop
    r0=SHOULDER_TO_ELBOW;  // circle 0 centers on shoulder
    d=wop.Length();
    c = ( r0 * r0 - r1 * r1 + d*d ) / ( 2*d );
    // find the midpoint
    n=wop;
    n/=d;
    mid = arm.shoulder+(n*c);
    // with c and r0 we can find h, the distance from midpoint to the intersections.
    h=sqrt(r0*r0-c*c);
    // the distance h on a line orthogonal to n and plane_normal gives us the two intersections.
    r = arm.plane_normal ^ n;
    p1 = mid - r * h;
    //p2 = mid + r * h;
    
    arm.elbow.pos.set(p1.x,p1.y,p1.z);
  }

  // update shoulder angles
  Vector3 temp;
  float x2,y2,new_angle,nx;
  
  for(i=0;i<NUM_AXIES;++i) {
    Arm &arm=robot.arms[i];
    
    // get the angle of each shoulder
    // use atan2 to find theta
    temp = arm.elbow.pos - arm.shoulder;

    y2 = temp.z;
    temp.z = 0;
    x2 = temp.Length();
        
    if( ( arm.elbow.relative | temp ) < 0 ) x2=-x2;

    new_angle = degrees( atan2(-y2,x2) );
    // cap the angle
    //if(new_angle>90) new_angle=90;
    //if(new_angle<-90) new_angle=-90;

    // we don't care about elbow angle, but we could find it here if we needed it.

    //Serial.print(i==0?"\tAng=":",");
    //Serial.print(new_angle,2); 
    //if(i==2) Serial.print("\n"); 

    // update servo to match the new IK data
    // 2013-05-17 http://www.marginallyclever.com/forum/viewtopic.php?f=12&t=4707&p=5103#p5091
    nx = ( (robot.reverse==1) ? new_angle : -new_angle );
  /*
    if(nx>MAX_ANGLE) {
      Serial.println("over max");
      nx=MAX_ANGLE;
    }
    if(nx<MIN_ANGLE) {
      Serial.println("under min");
      nx=MIN_ANGLE;
    }
  */
    //arm.angle=nx;
    motorStepArray[i] = nx;
  }

#if VERBOSE > 0
  Serial.print(i);
  Serial.print('=');
  Serial.print(robot.ee.x);
  Serial.print(',');
  Serial.print(robot.ee.y);
  Serial.print(',');
  Serial.print(robot.ee.z);
  Serial.print(',');
  Serial.print(robot.arms[0].angle);
  Serial.print(',');
  Serial.print(robot.arms[1].angle);
  Serial.print(',');
  Serial.print(robot.arms[2].angle);
  #if NUM_AXIES >=4
  Serial.print(',');
  Serial.print(robot.arms[3].angle);
  #endif
  Serial.println();
#endif
}


/** 
 * Forward Kinematics - turns step counts into XY coordinates
 * @param motorStepArray a measure of each belt to that plotter position
 * @param axies the resulting cartesian coordinate
 * @return 0 if no problem, 1 on failure.
 */
int FK(long *motorStepArray,float *axies) {
  float sqrt3 = sqrt(3.0);
  float sin120 = sqrt3/2.0;
  float cos120 = -0.5;
  float tan60 = sqrt3;
  float sin30 = 0.5;
  float tan30 = 1.0/sqrt3;

  float t = (CENTER_TO_SHOULDER-EFFECTOR_TO_WRIST)*tan30/2.0;
  float dtr = PI/180.0;

  float theta1 = radians(motorStepArray[0]);
  float theta2 = radians(motorStepArray[1]);
  float theta3 = radians(motorStepArray[2]);

  float y1 = -(t + SHOULDER_TO_ELBOW*cos(theta1));
  float z1 = -SHOULDER_TO_ELBOW*sin(theta1);

  float y2 = (t + SHOULDER_TO_ELBOW*cos(theta2))*sin30;
  float x2 = y2*tan60;
  float z2 = -SHOULDER_TO_ELBOW*sin(theta2);

  float y3 = (t + SHOULDER_TO_ELBOW*cos(theta3))*sin30;
  float x3 = -y3*tan60;
  float z3 = -SHOULDER_TO_ELBOW*sin(theta3);

  float dnm = (y2-y1)*x3-(y3-y1)*x2;

  float w1 = y1*y1 + z1*z1;
  float w2 = x2*x2 + y2*y2 + z2*z2;
  float w3 = x3*x3 + y3*y3 + z3*z3;

  // x = (a1*z + b1)/dnm
  float a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
  float b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;

  // y = (a2*z + b2)/dnm;
  float a2 = -(z2-z1)*x3+(z3-z1)*x2;
  float b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;

  // a*z^2 + b*z + c = 0
  float a = a1*a1 + a2*a2 + dnm*dnm;
  float b = 2.0*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
  float c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - ELBOW_TO_WRIST*ELBOW_TO_WRIST);

  // discriminant
  float d = b*b - 4.0*a*c;
  if (d < 0.0) return 1; // no intersection.
  
  float z0 = -0.5*(b+sqrt(d))/a;
  float x0 = (a1*z0 + b1)/dnm;
  float y0 = (a2*z0 + b2)/dnm;

  axies[0]=x0;
  axies[1]=y0;
  axies[2]=z0;
  return 0;
}

#endif
