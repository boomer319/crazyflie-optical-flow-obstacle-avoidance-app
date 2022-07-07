/*
 * wall_follower_multi_ranger_onboard.c
 *
 *  Created on: Aug 7, 2018
 *      Author: knmcguire
The same wallfollowing strategy was used in the following paper:
 @article{mcguire2019minimal,
  title={Minimal navigation solution for a swarm of tiny flying robots to explore an unknown environment},
  author={McGuire, KN and De Wagter, Christophe and Tuyls, Karl and Kappen, HJ and de Croon, Guido CHE},
  journal={Science Robotics},
  volume={4},
  number={35},
  year={2019},
  publisher={Science Robotics}
 */

#include "wallfollowing_multiranger_onboard.h"
#include <math.h>

// variables
static float maxForwardSpeed = 0.2f;
// static float maxTurnRate = 0.5f;
static float firstRun = false;
static float prevHeading = 0.0f;
static bool aroundCornerBackTrack = false;
static float stateStartTime;
static const float inCornerAngle = 0.8f;
static const float waitForMeasurementSeconds = 1.0f;

static StateWF stateWF = forward;
float timeNow = 0.0f;

int count = 0;
int heightBit = 0;

void wallFollowerInit(float maxForwardSpeed_ref, StateWF initState)
{
  maxForwardSpeed = maxForwardSpeed_ref;
  firstRun = true;
  stateWF = initState;
}

static void commandHover(float *cmdVelX, float *cmdVelY, float *cmdAngW)
{
  *cmdVelX = 0.0f;
  *cmdVelY = 0.0f;
  *cmdAngW = 0.0f;
}
static StateWF transition(StateWF newState)
{
  stateStartTime = timeNow;
  return newState;
}

StateWF wallFollower(float *cmdVelX, float *cmdVelY, float *cmdAngW, float *cmdHeight, float currentHeading, float timeOuter, double yawRate)
{
  timeNow = timeOuter;

  if (firstRun)
  {
    prevHeading = currentHeading;
    aroundCornerBackTrack = false;
    firstRun = false;
  }

  /***********************************************************
  * Handle state transitions
  ***********************************************************/
  switch (stateWF)
  {

  case forward:
    break;

  case hover:
    break;

  default:
    stateWF = transition(hover);
  }

  /***********************************************************
   * Handle state actions
   ***********************************************************/

  float cmdVelXTemp = 0.0f;
  float cmdVelYTemp = 0.0f;
  float cmdAngWTemp = 0.0f;
  float cmdHeightTemp = 0.0f;

  switch (stateWF)
  {
  case forward:
    cmdVelXTemp = maxForwardSpeed;
    cmdVelYTemp = 0.0f;
    if (yawRate > 0.1 ||  yawRate < -0.1)
    {
      cmdAngWTemp = yawRate;
    }
    else
    {
      cmdAngWTemp = 0;
    }

    count += 1;

    if (count > 100){
      heightBit ^= 1;
      count = 0;
    }

    if (heightBit){
      cmdHeightTemp = 0.6f;
    }
    else{
      cmdHeightTemp = 0.4f;
    }
    
    break;

  case hover:
    commandHover(&cmdVelXTemp, &cmdVelYTemp, &cmdAngWTemp);
    break;

  default:
    //State does not exist so hover!!
    commandHover(&cmdVelXTemp, &cmdVelYTemp, &cmdAngWTemp);
  }

  *cmdVelX = cmdVelXTemp;
  *cmdVelY = cmdVelYTemp;
  *cmdAngW = cmdAngWTemp;
  *cmdHeight = cmdHeightTemp;

  return stateWF;
}