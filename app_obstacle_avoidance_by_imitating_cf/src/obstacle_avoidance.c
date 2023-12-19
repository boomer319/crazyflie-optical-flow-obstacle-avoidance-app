/* This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * based on obstacle_avoidance.c - App layer for optical flow-based obstacle avoidance.
 *
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

// for using abs()
#include <stdlib.h>


#include "app.h"

// #include "commander.h"

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "OBAV"
#include "debug.h"

#include "log.h"
#include "param.h"

#include <math.h>
#include "usec_time.h"

#include "deck.h"

#include "uart1.h"

#include "system.h"

#include "obstacle_avoidance.h"

static uint8_t byte;
float yaw_command;

// static int8_t prev_yaw_command;
// float delta_yaw_command;

// float diff_yaw;
float obs_distance_x;
float obs_distance_y;

int obstacle_trigger = 0;
int obstacle_trigger_new_pos = 0;

static float yaw_approx_obstacle;

// Calculate the vector to the obstacle (to be added to the crazyflies position)
// The vector is added to the position in collision_avoidance.c (because asking
// for position from log before there is Mocap data causes the crazyflie not to start up)

// Place the initial obstacle position far away
Vector calculatedVector = {10.0f, 0.0f};

Vector calculateVector(float angle, float distance_x, float distance_y) {
    float radians = angle * (3.14159265f / 180.0f);
    calculatedVector.x = distance_x * cosf(radians);
    calculatedVector.y = distance_y * sinf(radians);
    return calculatedVector;
}

static void receiveByteTask(void *param)
{
  systemWaitStart();
  vTaskDelay(M2T(10));

  // Read out the byte the Gap8 sends and immediately send it to the console.
  while (1)
  {
    uart1GetDataWithTimeout(&byte, portMAX_DELAY);
    // DEBUG_PRINT("Data received from flo_proc on GAP8: %d\n", byte);
  }
}

void appMain()
{
  vTaskDelay(M2T(1000));
  uart1Init(115200);
  vTaskDelay(M2T(1000));

  DEBUG_PRINT("Waiting for activation ...\n");

  // Distance of obstacle to drone when triggered
  // Slightly less than the ellipsoid x-y-diameter,
  // so the cf goes away from the obstacle in frontal collision scenario
  obs_distance_x = 0.29f;
  obs_distance_y = 0.29f;

  // Pull reset for GAP8/ESP32
  pinMode(DECK_GPIO_IO4, OUTPUT);
  digitalWrite(DECK_GPIO_IO4, LOW);

  xTaskCreate(receiveByteTask, "AI DECK UART READOUT", AI_DECK_TASK_STACKSIZE, NULL, AI_DECK_TASK_PRI, NULL);

  vTaskDelay(M2T(100));

  // Release reset for GAP8/ESP32
  digitalWrite(DECK_GPIO_IO4, HIGH);
  pinMode(DECK_GPIO_IO4, INPUT_PULLUP);

  vTaskDelay(M2T(100));

  // Get yaw from cf
  logVarId_t idYaw = logGetVarId("stateEstimate", "yaw");
  int yaw = 0.0f;

  while (1)
  {
    vTaskDelay(M2T(250));

    // Get yaw from cf
    yaw = logGetFloat(idYaw);
    // DEBUG_PRINT("cf's yaw is now: %f deg\n", (double)yaw);

    yaw_command = byte; // 0 -> 255

    // Conversion from 0 -> 255 to -128 to 128
    if (yaw_command >= 128)
    {
      yaw_command -= 256;
    }
    // Conversion from -128 -> 128 to roughly -45 -> 45
    yaw_command = yaw_command * (45.0f/128.0f);

    // // Derivative of yaw_command
    // delta_yaw_command = prev_yaw_command - yaw_command;
    // prev_yaw_command = yaw_command;

    // // Difference between cf's yaw and yaw_command
    // diff_yaw = abs(yaw - yaw_command);

    // Triggering obstacle in collision avoidance
    // Value determined through testing
    // 11: too high runs into obstacles
    // 8: too high
    // 7.5: too high
    // 7.3: false tirggers only when flying in one direction in the cage
    //      only regard the working direction (abs(yaw) > 160)
    // 7: too low
    // 6: triggers with normal flying
    if (abs(yaw_command) >= 7.3 && abs(yaw) > 160) 
    {
      yaw_approx_obstacle = yaw + yaw_command;
      calculateVector(yaw_approx_obstacle, obs_distance_x, obs_distance_y);

      obstacle_trigger_new_pos = 1;
      obstacle_trigger = 1; // triggers collision avoidance

      DEBUG_PRINT("calculatedVector.x: %f\n", (double)calculatedVector.x);
      DEBUG_PRINT("calculatedVector.y: %f\n", (double)calculatedVector.y);

      // Wait for cf to navigate around obstacle before overwriting it
      vTaskDelay(M2T(10000));

      // Reset obstacle trigger to disregard the obstacle
      obstacle_trigger = 0;
    }
    else
    {
      obstacle_trigger = 0;
    }

    // send to console
    // DEBUG_PRINT("byte: %d\n", byte);
    DEBUG_PRINT("yaw_command: %f deg\n", (double)yaw_command);
    // DEBUG_PRINT("delta_yaw_command: %f\n", (double)delta_yaw_command);
    // DEBUG_PRINT("diff_yaw: %f\n", (double)diff_yaw);
    // DEBUG_PRINT("yaw_approx_obstacle: %f\n", (double)obstacle_trigger);
    // DEBUG_PRINT("\n");
  }
}

// PARAM_GROUP_START(obAv)
// PARAM_ADD(PARAM_FLOAT, max_yaw_angle, &max_yaw_angle)
// PARAM_ADD(PARAM_FLOAT, obs_distance_x, &obs_distance_x)
// PARAM_ADD(PARAM_FLOAT, obs_distance_y, &obs_distance_y)
// PARAM_GROUP_STOP(obAv)