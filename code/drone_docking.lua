--[[
Autonomous Drone-to-Drone Docking System: Lua Script with PD Controller
This script implements a Proportional-Derivative (PD) controller for position correction
during drone-to-drone docking. It receives position data from the ESP32 via MAVLink
and generates control commands scaled by altitude difference.

To use this script:
1. Save it as "drone_docking.lua" in the APM/scripts directory on your SD card
2. Enable scripting in ArduPilot by setting SCR_ENABLE = 1
3. Set SCR_HEAP_SIZE to at least 100000 (recommended)
4. Reboot the flight controller

The script will automatically run when the flight controller starts.
--]]

-- Script information
local SCRIPT_NAME = "Drone Docking"
local SCRIPT_VERSION = "1.0"

-- Configuration parameters
local UPDATE_RATE_HZ = 10  -- Script execution rate in Hz
local DOCKING_CHANNEL = 7  -- RC channel to activate docking (adjust as needed)
local DOCKING_THRESHOLD = 1800  -- RC value threshold to activate docking

-- PD Controller parameters
local Kp_xy = 0.5  -- Proportional gain for XY position
local Kd_xy = 0.2  -- Derivative gain for XY position
local Kp_z = 0.3   -- Proportional gain for Z position
local Kd_z = 0.1   -- Derivative gain for Z position

-- Position error limits (in meters)
local MAX_XY_ERROR = 2.0  -- Maximum allowed XY error
local MAX_Z_ERROR = 1.0   -- Maximum allowed Z error

-- Control output limits (in m/s)
local MAX_XY_SPEED = 0.5  -- Maximum allowed XY speed
local MAX_Z_SPEED = 0.3   -- Maximum allowed Z speed

-- Docking completion parameters
local DOCKING_COMPLETE_DISTANCE = 0.1  -- Distance threshold for docking completion (m)
local DOCKING_TIMEOUT = 60            -- Timeout for docking procedure (seconds)

-- Global variables
local docking_active = false
local docking_start_time = 0
local last_update_time = 0
local last_position_error = Vector3f()
local current_position_error = Vector3f()
local target_position = Vector3f()
local last_vision_position_time = 0
local vision_position_timeout = 1000  -- Timeout for vision position data (ms)

-- Initialize the script
function init()
  gcs:send_text(0, string.format("%s v%s initialized", SCRIPT_NAME, SCRIPT_VERSION))
  return update, UPDATE_RATE_HZ
end

-- Main update function
function update()
  -- Check if vehicle is armed and in a suitable flight mode
  if not arming:is_armed() then
    return update, UPDATE_RATE_HZ
  end
  
  -- Get current time
  local current_time_ms = millis()
  
  -- Check if docking should be activated/deactivated
  check_docking_activation()
  
  -- If docking is active, perform docking procedure
  if docking_active then
    -- Check for timeout
    if current_time_ms - docking_start_time > DOCKING_TIMEOUT * 1000 then
      gcs:send_text(0, "Docking timeout reached, aborting")
      docking_active = false
      return update, UPDATE_RATE_HZ
    end
    
    -- Check if we have recent vision position data
    if current_time_ms - last_vision_position_time > vision_position_timeout then
      gcs:send_text(3, "No recent vision position data")
      -- Continue with docking but at reduced rate
      return update, UPDATE_RATE_HZ / 2
    end
    
    -- Calculate time delta for derivative term
    local dt = (current_time_ms - last_update_time) / 1000.0  -- Convert to seconds
    if dt > 0 then
      -- Apply PD controller
      apply_pd_controller(dt)
    end
    
    -- Update last update time
    last_update_time = current_time_ms
    
    -- Check if docking is complete
    if check_docking_complete() then
      gcs:send_text(0, "Docking completed successfully")
      docking_active = false
    end
  end
  
  return update, UPDATE_RATE_HZ
end

-- Check if docking should be activated/deactivated
function check_docking_activation()
  -- Check RC channel for docking activation
  local rc_value = rc:get_channel(DOCKING_CHANNEL)
  
  if not docking_active and rc_value > DOCKING_THRESHOLD then
    -- Activate docking
    docking_active = true
    docking_start_time = millis()
    last_update_time = docking_start_time
    gcs:send_text(0, "Docking procedure activated")
    
    -- Reset position errors
    last_position_error = Vector3f()
    current_position_error = Vector3f()
  elseif docking_active and rc_value <= DOCKING_THRESHOLD then
    -- Deactivate docking
    docking_active = false
    gcs:send_text(0, "Docking procedure deactivated")
  end
end

-- Apply PD controller based on position error
function apply_pd_controller(dt)
  -- Get current altitude
  local current_altitude = vehicle:get_altitude()
  
  -- Calculate altitude difference (assuming target drone is higher)
  local altitude_difference = math.max(0.1, target_position:z() - current_altitude)
  
  -- Calculate scaling factor based on altitude difference
  local scale_factor = calculate_scale_factor(altitude_difference)
  
  -- Calculate proportional term
  local p_term_x = Kp_xy * current_position_error:x() * scale_factor
  local p_term_y = Kp_xy * current_position_error:y() * scale_factor
  local p_term_z = Kp_z * current_position_error:z() * scale_factor
  
  -- Calculate derivative term
  local d_term_x = Kd_xy * (current_position_error:x() - last_position_error:x()) / dt * scale_factor
  local d_term_y = Kd_xy * (current_position_error:y() - last_position_error:y()) / dt * scale_factor
  local d_term_z = Kd_z * (current_position_error:z() - last_position_error:z()) / dt * scale_factor
  
  -- Calculate control outputs
  local vx = constrain(p_term_x + d_term_x, -MAX_XY_SPEED, MAX_XY_SPEED)
  local vy = constrain(p_term_y + d_term_y, -MAX_XY_SPEED, MAX_XY_SPEED)
  local vz = constrain(p_term_z + d_term_z, -MAX_Z_SPEED, MAX_Z_SPEED)
  
  -- Send velocity commands to the vehicle
  if vehicle:get_mode() == 4 then  -- GUIDED mode
    vehicle:set_velocity_ned(vx, vy, vz, false)  -- false = not use yaw
    
    -- Debug output
    gcs:send_text(6, string.format("PD: vx=%.2f vy=%.2f vz=%.2f sf=%.2f", vx, vy, vz, scale_factor))
  else
    gcs:send_text(3, "Vehicle not in GUIDED mode, cannot send velocity commands")
  end
  
  -- Store current error for next iteration
  last_position_error = current_position_error:copy()
end

-- Calculate scale factor based on altitude difference
function calculate_scale_factor(altitude_difference)
  -- Scale factor decreases as drones get closer
  -- More aggressive when far, gentler when close
  local max_altitude_difference = 5.0  -- meters
  return math.max(0.1, math.min(1.0, altitude_difference / max_altitude_difference))
end

-- Check if docking is complete
function check_docking_complete()
  -- Calculate distance to target
  local distance = math.sqrt(
    current_position_error:x() * current_position_error:x() +
    current_position_error:y() * current_position_error:y() +
    current_position_error:z() * current_position_error:z()
  )
  
  -- Check if distance is below threshold
  return distance < DOCKING_COMPLETE_DISTANCE
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
    -- Extract position data
    local x = msg:getf(0)  -- X position (m)
    local y = msg:getf(4)  -- Y position (m)
    local z = msg:getf(8)  -- Z position (m)
    
    -- Update target position (assuming message contains relative position to target)
    target_position:x(x)
    target_position:y(y)
    target_position:z(z)
    
    -- Update current position error
    current_position_error:x(x)
    current_position_error:y(y)
    current_position_error:z(z)
    
    -- Update last vision position time
    last_vision_position_time = millis()
    
    -- Debug output (low priority)
    gcs:send_text(6, string.format("Vision pos: x=%.2f y=%.2f z=%.2f", x, y, z))
    
    return true
  end
  
  -- Return false to allow other scripts to process this message
  return false
end

-- Register the MAVLink message handler
mavlink:register_message_handler(mavlink_message_handler)

-- Return the initialization function to start the script
return init()
