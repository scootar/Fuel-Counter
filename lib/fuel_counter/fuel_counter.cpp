/*******************************************************************************
 * fuel_counter.cpp — Implementation of core counting logic
 ******************************************************************************/
#include "fuel_counter.h"

bool processLaneReading(Lane& lane, uint16_t distance_mm,
                        uint32_t now_ms, uint32_t lockout_ms)
{
  if (!lane.sensorOk) return false;

  lane.lastDistance_mm = distance_mm;

  switch (lane.state) {

    case LaneState::IDLE:
      if (distance_mm < lane.threshold_mm) {
        lane.state = LaneState::BALL_PRESENT;
      }
      break;

    case LaneState::BALL_PRESENT:
      // Ball has fully passed when distance rises above the clear threshold
      if (distance_mm > lane.clearThresh_mm) {
        lane.count++;
        lane.state        = LaneState::LOCKOUT;
        lane.lockoutStart = now_ms;
        return true;   // <— ball counted
      }
      break;

    case LaneState::LOCKOUT:
      if (now_ms - lane.lockoutStart >= lockout_ms) {
        lane.state = LaneState::IDLE;
      }
      break;
  }

  return false;
}

void calculateThresholds(Lane& lane, uint16_t baseline_mm,
                         uint16_t detection_delta_mm,
                         uint16_t clear_hysteresis_mm)
{
  lane.baseline_mm    = baseline_mm;
  lane.threshold_mm   = baseline_mm - detection_delta_mm;
  lane.clearThresh_mm = lane.threshold_mm + clear_hysteresis_mm;
}

void resetLanes(Lane lanes[], uint8_t num_lanes, uint32_t& totalCount)
{
  for (uint8_t i = 0; i < num_lanes; i++) {
    lanes[i].count = 0;
    lanes[i].state = LaneState::IDLE;
  }
  totalCount = 0;
}
