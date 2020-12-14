/* 
Algorithm that use distance numbers that came from thre ultrasonic sensors to determine and classify 
the situation into one of many possible scenarios. The robot has three distance sensors:
left, center, and right, and each sensor will measure the distance from the
center of the robot to the wall in mm. There will be a single reference point on the
robot, and the three distances will be measured from that common reference.

Author: Joaquin Sopena
Date: Nov, 2020
*/
#include <stdint.h>
enum scenario {
    Error = 0,
    LeftTooClose = 1,
    RightTooClose = 2,
    LeftAndRightTooClose = 3,
    CenterTooClose = 4,
    CenterAndLeftTooClose = 5,
    CenterAndRightTooClose = 6,
    CenterAndLeftAndRightTooClose = 7,
    Straight = 8,
    LeftTurn = 9,
    RightTurn = 10,
    TeeJoint = 11,
    LeftJoint = 12,
    RightJoint = 13,
    CrossRoad = 14,
    Blocked = 15
};
typedef enum scenario scenario_t;

#define SIDEMAX 354    // largest side distance to wall in mm
#define SIDEMIN 212    // smallest side distance to wall in mm
#define CENTEROPEN 600 // distance to wall between open/blocked
#define CENTERMIN 150  // min distance to wall in the front
#define MINDIST 50     // min distance the sensor can read in mm
#define MAXDIST 800    // max distance the sensor can read in mm
scenario_t Classify(int32_t Left, int32_t Center, int32_t Right){
  scenario_t result = Error;
  //ERROR CASES
  if(Left < MINDIST || Center < MINDIST || Right < MINDIST ||
     Left > MAXDIST || Center > MAXDIST || Right > MAXDIST)
  {
      result = Error;
  }

  //DANGER CASES

  else if(Left < SIDEMIN && Right < SIDEMIN && Center < CENTERMIN )
  {
      result = CenterAndLeftAndRightTooClose;
  }

  else if(Center < CENTERMIN && Right < SIDEMIN)
    {
      result = CenterAndRightTooClose;
    }

  else if(Center < CENTERMIN && Left < SIDEMIN)
      {
        result = CenterAndLeftTooClose;
      }

  else if(Right < SIDEMIN && Left < SIDEMIN)
        {
          result = LeftAndRightTooClose;
        }

  else if(Center < CENTERMIN)
      {
        result = CenterTooClose;
      }

  else if(Left <SIDEMIN)
      {
        result = LeftTooClose;
      }

  else if(Right <SIDEMIN)
        {
          result = RightTooClose;
        }

  //NORMAL OPERATION CASES
  else if(Center >= CENTEROPEN)
  {
      if((SIDEMIN <= Left) && (Left < SIDEMAX) && (SIDEMIN <= Right) && (Right < SIDEMAX))
      {
          result = Straight;
      }

      if((Right >= SIDEMAX) && (Left >= SIDEMAX))
      {
          result = CrossRoad;
      }
      if((Left >= SIDEMAX) && (SIDEMIN <= Right) && (Right < SIDEMAX))
      {
          result = LeftJoint;
      }

      if((Right >= SIDEMAX) && (SIDEMIN <= Left) && (Left < SIDEMAX))
      {
          result = RightJoint;
      }
  }

  else if(CENTERMIN <= Center < CENTEROPEN)
  {
      if((Right >= SIDEMAX) && (Left >= SIDEMAX))
      {
          result = TeeJoint;
      }

      if((Left >= SIDEMAX) && (SIDEMIN <= Right) && (Right < SIDEMAX))
      {
          result = LeftTurn;
      }

      if((Right >= SIDEMAX) && (SIDEMIN <= Left) && (Left < SIDEMAX))
      {
          result = RightTurn;
      }

      if((SIDEMIN <= Left) && (Left < SIDEMAX) && (SIDEMIN <= Right) && (Right < SIDEMAX))
      {
          result = Blocked;
      }
  }

  return result;
}

/* 
Algorithm that converts raw 14-bit ADC data to distance in mm, that came from
the GP2Y0A21YK0F ultrasonic sensor

Author: Joaquin Sopena
Date: Nov, 2020
*/

#define IRSlope 1195172
#define IROffset -1058
#define IRMax 2552

int32_t Convert(int32_t n){
    int32_t D;                  //D local, dynamic allocation
    if(n <= IRMax){             //The maximum measurement distance for the sensor is 800 mm -> ADC value > 2552
        D = 800;
    }
    else{
        D = IRSlope/(n+IROffset);
    }
  return D;
}

