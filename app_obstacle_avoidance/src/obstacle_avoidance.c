#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "commander.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

#include "log.h"
#include "param.h"
#include <math.h>
#include "usec_time.h"

#include "uart1.h"

static uint8_t count;
static uint8_t byte;
static int8_t yaw_command;

void appMain()
{
  vTaskDelay(M2T(1000));
  uart1Init(115200);
  vTaskDelay(M2T(1000));
  // // Getting the Logging IDs of the state estimates
  // logVarId_t idStabilizerYaw = logGetVarId("stabilizer", "yaw");
  // logVarId_t idHeightEstimate = logGetVarId("stateEstimate", "z");

  // // Getting Param IDs of the deck driver initialization
  // paramVarId_t idAIdeck = paramGetVarId("deck", "bcAI");

  // // Intialize the setpoint structure
  // setpoint_t setpoint;

  DEBUG_PRINT("Waiting for activation ...\n");

  count = 0;

  while (1)
  {
    vTaskDelay(M2T(10));

    // Check if AI deck is properly mounted
    // uint8_t aiInit = paramGetUint(idAIdeck);

    count += 1;
    uart1GetDataWithTimeout(&byte, portMAX_DELAY);

    yaw_command = byte;
  }
}

// PARAM_GROUP_START(app)
// PARAM_ADD(PARAM_UINT8, goLeft, &goLeft)
// PARAM_ADD(PARAM_FLOAT, distanceWall, &distanceToWall)
// PARAM_ADD(PARAM_FLOAT, maxSpeed, &maxForwardSpeed)
// PARAM_GROUP_STOP(app)

LOG_GROUP_START(app)
LOG_ADD(LOG_UINT8, number_counter, &count)
LOG_ADD(LOG_INT8, yaw_command, &yaw_command)
LOG_GROUP_STOP(app)
