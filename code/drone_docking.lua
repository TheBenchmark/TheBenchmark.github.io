--[[
Autonomous Drone-to-Drone Docking System: Lua Script with Tunable PD Controller
This script implements a Proportional-Derivative (PD) controller for position correction
during drone-to-drone docking. P and D gains are mapped from two RC potentiometers,
and horizontal corrections are softened as the drones get vertically closer.

Setup:
1. Save as "drone_docking.lua" in APM/scripts on your SD card.
2. SCR_ENABLE = 1, SCR_HEAP_SIZE ≥ 100000.
3. Wire your P gain poti to RC channel 8 and your D gain poti to RC channel 9.
4. Tune MAX_* gains to suit your dynamics.
--]]

--[[
Update: 
Autonomous Drone-to-Drone Docking System: Lua Script with Change‑Only PD Messages
to save bandwidth
--]]

-- Script info
local SCRIPT_NAME    = "Drone Docking"
local SCRIPT_VERSION = "1.2"

-- Execution and activation
local UPDATE_RATE_HZ     = 10
local DOCKING_CHANNEL    = 7
local DOCKING_THRESHOLD  = 1800

-- RC channels for gain tuning
local KP_CHANNEL = 8
local KD_CHANNEL = 9

-- Max PD gains
local MAX_KP_XY = 0.5
local MAX_KD_XY = 0.2
local MAX_KP_Z  = 0.3
local MAX_KD_Z  = 0.1

-- RC input bounds
local RC_MIN, RC_MAX = 1000, 2000

-- Altitude scaling
local MAX_ALT_DIFF = 5.0
local function calculate_scale_factor(alt)
  return math.max(0.1, math.min(1.0, alt / MAX_ALT_DIFF))
end

-- Docking thresholds
local DOCKING_COMPLETE_DISTANCE = 0.1
local DOCKING_TIMEOUT            = 60

-- Speed limits
local MAX_XY_SPEED = 0.5
local MAX_Z_SPEED  = 0.3

-- Last‑seen gains (force initial print)
local last_Kp_xy, last_Kd_xy = -1, -1
-- Minimum change to consider "different"
local gain_change_threshold = 0.001

-- State
local docking_active         = false
local docking_start_time     = 0
local last_update_time       = 0
local last_position_error    = Vector3f()
local current_position_error = Vector3f()
local target_position        = Vector3f()
local last_vision_position_time = 0
local vision_position_timeout   = 1000

function init()
  gcs:send_text(0, SCRIPT_NAME.." v"..SCRIPT_VERSION.." initialized")
  return update, UPDATE_RATE_HZ
end

function update()
  if not arming:is_armed() then
    return update, UPDATE_RATE_HZ
  end

  local now = millis()
  check_docking_activation()

  if docking_active then
    if now - docking_start_time > DOCKING_TIMEOUT * 1000 then
      gcs:send_text(0, "Docking timeout, aborting")
      docking_active = false
      return update, UPDATE_RATE_HZ
    end

    if now - last_vision_position_time > vision_position_timeout then
      gcs:send_text(3, "No vision data — slowing loop")
      return update, UPDATE_RATE_HZ / 2
    end

    local dt = (now - last_update_time) / 1000
    if dt > 0 then
      apply_pd_controller(dt)
    end
    last_update_time = now

    if check_docking_complete() then
      gcs:send_text(0, "Docking complete")
      docking_active = false
    end
  end

  return update, UPDATE_RATE_HZ
end

function check_docking_activation()
  local rc_val = rc:get_channel(DOCKING_CHANNEL)
  if not docking_active and rc_val > DOCKING_THRESHOLD then
    docking_active      = true
    docking_start_time  = millis()
    last_update_time    = docking_start_time
    last_position_error = Vector3f()
    current_position_error = Vector3f()
    gcs:send_text(0, "Docking activated")
  elseif docking_active and rc_val <= DOCKING_THRESHOLD then
    docking_active = false
    gcs:send_text(0, "Docking deactivated")
  end
end

function apply_pd_controller(dt)
  -- 1) Altitude‐based horizontal scaling
  local alt      = vehicle:get_altitude()
  local alt_diff = math.max(0.1, target_position:z() - alt)
  local scale_xy = calculate_scale_factor(alt_diff)

  -- 2) Read knobs and normalize
  local rc_kp = rc:get_channel(KP_CHANNEL)
  local rc_kd = rc:get_channel(KD_CHANNEL)
  local s_kp  = math.min(math.max((rc_kp - RC_MIN)/(RC_MAX - RC_MIN), 0), 1)
  local s_kd  = math.min(math.max((rc_kd - RC_MIN)/(RC_MAX - RC_MIN), 0), 1)

  -- 3) Dynamic gains
  local Kp_xy = s_kp * MAX_KP_XY
  local Kd_xy = s_kd * MAX_KD_XY
  local Kp_z  = s_kp * MAX_KP_Z
  local Kd_z  = s_kd * MAX_KD_Z

  -- 4) Only print when Kp_xy or Kd_xy changed beyond threshold
  if math.abs(Kp_xy - last_Kp_xy) > gain_change_threshold
  or math.abs(Kd_xy - last_Kd_xy) > gain_change_threshold then
    gcs:send_text(6, string.format("Gains→ Kp=%.3f  Kd=%.3f", Kp_xy, Kd_xy))
    last_Kp_xy, last_Kd_xy = Kp_xy, Kd_xy
  end

  -- 5) Compute P/D terms
  local p_x = Kp_xy * current_position_error:x() * scale_xy
  local p_y = Kp_xy * current_position_error:y() * scale_xy
  local p_z = Kp_z  * current_position_error:z()

  local d_x = Kd_xy * (current_position_error:x() - last_position_error:x())/dt * scale_xy
  local d_y = Kd_xy * (current_position_error:y() - last_position_error:y())/dt * scale_xy
  local d_z = Kd_z  * (current_position_error:z() - last_position_error:z())/dt

  -- 6) Sum & clamp
  local vx = constrain(p_x + d_x, -MAX_XY_SPEED, MAX_XY_SPEED)
  local vy = constrain(p_y + d_y, -MAX_XY_SPEED, MAX_XY_SPEED)
  local vz = constrain(p_z + d_z, -MAX_Z_SPEED,  MAX_Z_SPEED)

  -- 7) Send velocity
  if vehicle:get_mode() == 4 then  -- GUIDED
    vehicle:set_velocity_ned(vx, vy, vz, false)
  else
    gcs:send_text(3, "Not in GUIDED mode")
  end

  last_position_error = current_position_error:copy()
end

function check_docking_complete()
  local dx,dy,dz = current_position_error:x(), current_position_error:y(), current_position_error:z()
  return math.sqrt(dx*dx + dy*dy + dz*dz) < DOCKING_COMPLETE_DISTANCE
end

function constrain(v, mn, mx)
  if v<mn then return mn end
  if v>mx then return mx end
  return v
end

function mavlink_message_handler(msg)
  if msg:id()==102 then
    local x,y,z = msg:getf(0), msg:getf(4), msg:getf(8)
    target_position:x(x); target_position:y(y); target_position:z(z)
    current_position_error:copy(target_position)
    last_vision_position_time = millis()
    return true
  end
  return false
end

mavlink:register_message_handler(mavlink_message_handler)
return init()
