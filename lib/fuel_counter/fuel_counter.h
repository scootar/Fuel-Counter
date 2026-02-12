/*******************************************************************************
 * fuel_counter.h — Core counting logic for the Hub Fuel Counter
 *
 * Pure C++ (no Arduino dependencies) so it can be unit-tested natively.
 ******************************************************************************/
#pragma once
#include <stdint.h>

// ── Number of lanes ─────────────────────────────────────────────────────
static const uint8_t FC_NUM_LANES = 4;

// ── Lane state machine ──────────────────────────────────────────────────
enum class LaneState : uint8_t { IDLE, BALL_PRESENT, LOCKOUT };

struct Lane {
  uint16_t  baseline_mm     = 0;       // calibrated empty-lane distance
  uint16_t  threshold_mm    = 0;       // baseline − delta  (ball present below this)
  uint16_t  clearThresh_mm  = 0;       // threshold + hysteresis (ball cleared above this)
  uint32_t  count           = 0;
  LaneState state           = LaneState::IDLE;
  uint32_t  lockoutStart    = 0;
  bool      sensorOk        = false;   // false = sensor init failed / offline
  uint16_t  lastDistance_mm = 0;       // latest raw reading
};

// ── Pure-logic functions (hardware-independent) ─────────────────────────

/**
 * Process a single ToF distance reading for one lane.
 *
 * State machine:
 *   IDLE ──(dist < threshold)──► BALL_PRESENT
 *   BALL_PRESENT ──(dist > clearThresh)──► LOCKOUT  (count++)
 *   LOCKOUT ──(elapsed ≥ lockout_ms)──► IDLE
 *
 * @param lane          Lane struct (modified in place)
 * @param distance_mm   Current distance reading from sensor
 * @param now_ms        Current timestamp (millis)
 * @param lockout_ms    Dead-time after a count before re-arming
 * @return true if a ball was counted on this call
 */
bool processLaneReading(Lane& lane, uint16_t distance_mm,
                        uint32_t now_ms, uint32_t lockout_ms);

/**
 * Compute threshold_mm and clearThresh_mm from a known baseline.
 */
void calculateThresholds(Lane& lane, uint16_t baseline_mm,
                         uint16_t detection_delta_mm,
                         uint16_t clear_hysteresis_mm);

/**
 * Reset all lane counts and states; zero the total.
 */
void resetLanes(Lane lanes[], uint8_t num_lanes, uint32_t& totalCount);
