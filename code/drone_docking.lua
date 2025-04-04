--[[
Autonomous Drone-to-Drone Docking System: Lua Script with PD Controller
This script implements a Proportional-Derivative (PD) controller for correcting the XY position error during docking.
The aggressiveness of the XY corrections is scaled by the relative altitude error (the closer the drones, the gentler the correction).
The Kp and Kd gains for the XY controller are dynamically tuned via RC channels connected to potentiometers.
To use this script:
1. Save it as "drone_docking.lua" in the APM/scripts directory on your SD card.
2. Enable scripting in Ardupilot by setting SCR_ENABLE = 1.
3. Set SCR_HEAP_SIZE to at least 100000 (recommended).
4. Reboot the flight controller.
--]]

-- Script information
local SCRIPT_NAME = "Drone Docking"
local SCRIPT_VERSION = "1.1"

-- Configuration parameters
local UPDATE_RATE_HZ = 10  -- Script execution rate in Hz
local DOCKING_CHANNEL = 7  -- RC channel to activate docking (adjust as needed)
local DOCKING_THRESHOLD = 1800  -- RC value threshold to activate docking

-- Default PD Controller parameters for XY (used as fallback)
local Kp_xy_default = 0.5  -- Default proportional gain for XY position
local Kd_xy_default = 0.2  -- Default derivative gain for XY position

-- Position error limits (in meters)
local MAX_XY_ERROR = 2.0  -- Maximum allowed XY error

-- Control output limits (in m/s)
local MAX_XY_SPEED = 0.5  -- Maximum allowed XY speed

-- Docking completion parameters
local DOCKING_COMPLETE_DISTANCE = 0.1  -- Distance threshold for docking completion (m)
local DOCKING_TIMEOUT = 60             -- Timeout for docking procedure (seconds)

-- RC channels for tuning PD controller via potentiometers (adjust these channel numbers as needed)
local RC_KP_CHANNEL = 8  -- Channel for tuning Kp
local RC_KD_CHANNEL = 9  -- Channel for tuning Kd

-- Global variables
local docking_active = false
local docking_start_time = 0
local last_update_time = 0
local last_position_error = Vector3f()
local current_position_error = Vector3f()
local target_position = Vector3f()
local last_vision_position_time = 0
local vision_position_timeout = 1000  -- Timeout for vision position data (ms)

-- Mapping function: maps RC channel values (typically 1000-2000) to a desired gain range
function map_value(x, in_min, in_max, out_min, out_max)
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
end

-- Initialize the script
function init()
  gcs:send_text(0, string.format("%s v%s initialized", SCRIPT_NAME, SCRIPT_VERSION))
  return update, UPDATE_RATE_HZ
end

-- Main update function
function update()
  if not arming:is_armed() then
    return update, UPDATE_RATE_HZ
  end

  local current_time_ms = millis()
  check_docking_activation()

  if docking_active then
    if current_time_ms - docking_start_time > DOCKING_TIMEOUT * 1000 then
      gcs:send_text(0, "Docking timeout reached, aborting")
      docking_active = false
      return update, UPDATE_RATE_HZ
    end

    if current_time_ms - last_vision_position_time > vision_position_timeout then
      gcs:send_text(3, "No recent vision position data")
      return update, UPDATE_RATE_HZ / 2
    end

    local dt = (current_time_ms - last_update_time) / 1000.0  -- in seconds
    if dt > 0 then
      apply_pd_controller(dt)
    end

    last_update_time = current_time_ms

    if check_docking_complete() then
      gcs:send_text(0, "Docking completed successfully")
      docking_active = false
    end
  end

  return update, UPDATE_RATE_HZ
end

-- Check if docking should be activated/deactivated based on RC channel input
function check_docking_activation()
  local rc_value = rc:get_channel(DOCKING_CHANNEL)
  if not docking_active and rc_value > DOCKING_THRESHOLD then
    docking_active = true
    docking_start_time = millis()
    last_update_time = docking_start_time
    gcs:send_text(0, "Docking procedure activated")
    last_position_error = Vector3f()
    current_position_error = Vector3f()
  elseif docking_active and rc_value <= DOCKING_THRESHOLD then
    docking_active = false
    gcs:send_text(0, "Docking procedure deactivated")
  end
end

-- Apply PD controller for XY position using dynamic RC tuning and scaling by relative altitude error
function apply_pd_controller(dt)
  -- Read RC channel values for tuning Kp and Kd
  local raw_Kp = rc:get_channel(RC_KP_CHANNEL)
  local raw_Kd = rc:get_channel(RC_KD_CHANNEL)

  -- Map RC values to desired gain ranges (adjust these ranges as needed)
  local tuned_Kp_xy = map_value(raw_Kp, 1000, 2000, 0.1, 1.0)
  local tuned_Kd_xy = map_value(raw_Kd, 1000, 2000, 0.05, 0.5)

  -- Get current altitude and compute altitude difference (error in relative altitude)
  local current_altitude = vehicle:get_altitude()
  local altitude_difference = math.max(0.1, target_position:z() - current_altitude)

  -- Calculate scale factor: as drones get closer (smaller altitude difference), movements become slower
  local scale_factor = calculate_scale_factor(altitude_difference)

  -- Calculate proportional and derivative terms for XY error only
  local p_term_x = tuned_Kp_xy * current_position_error:x() * scale_factor
  local p_term_y = tuned_Kp_xy * current_position_error:y() * scale_factor

  local d_term_x = tuned_Kd_xy * (current_position_error:x() - last_position_error:x()) / dt * scale_factor
  local d_term_y = tuned_Kd_xy * (current_position_error:y() - last_position_error:y()) / dt * scale_factor

  -- Calculate control outputs and constrain them
  local vx = constrain(p_term_x + d_term_x, -MAX_XY_SPEED, MAX_XY_SPEED)
  local vy = constrain(p_term_y + d_term_y, -MAX_XY_SPEED, MAX_XY_SPEED)
  local vz = 0  -- No vertical (Z) correction

  if vehicle:get_mode() == 4 then  -- GUIDED mode
    vehicle:set_velocity_ned(vx, vy, vz, false)  -- false = do not adjust yaw
    gcs:send_text(6, string.format("PD: vx=%.2f vy=%.2f scale=%.2f", vx, vy, scale_factor))
  else
    gcs:send_text(3, "Vehicle not in GUIDED mode, cannot send velocity commands")
  end

  -- Update last_position_error for next iteration (only for XY)
  last_position_error:x(current_position_error:x())
  last_position_error:y(current_position_error:y())
end

-- Calculate scale factor based on altitude difference
function calculate_scale_factor(altitude_difference)
  -- Scale factor decreases as drones get closer: more aggressive when far, gentler when near.
  local max_altitude_difference = 5.0  -- meters
  return math.max(0.1, math.min(1.0, altitude_difference / max_altitude_difference))
end

-- Check if docking is complete based solely on XY distance
function check_docking_complete()
  local distance_xy = math.sqrt(
    current_position_error:x() * current_position_error:x() +
    current_position_error:y() * current_position_error:y()
  )
  return distance_xy < DOCKING_COMPLETE_DISTANCE
end

-- Constrain a value between min and max
function constrain(value, min, max)
  if value < min then return min end
  if value > max then return max end
  return value
end

-- MAVLink message handler for VISION_POSITION_ESTIMATE
function mavlink_message_handler(msg)
  if msg:id() == 102 then  -- VISION_POSITION_ESTIMATE message ID
    local x = msg:getf(0)  -- X position (m)
    local y = msg:getf(4)  -- Y position (m)
    local z = msg:getf(8)  -- Z position (m)

    -- Update target position (retain Z for altitude difference)
    target_position:x(x)
    target_position:y(y)
    target_position:z(z)

    -- Update current position error for XY only; ignore Z error for PD correction
    current_position_error:x(x)
    current_position_error:y(y)
    current_position_error:z(0)

    last_vision_position_time = millis()
    gcs:send_text(6, string.format("Vision pos: x=%.2f y=%.2f alt=%.2f", x, y, z))
    return true
  end

  return false
end

-- Register the MAVLink message handler
mavlink:register_message_handler(mavlink_message_handler)

return init()
