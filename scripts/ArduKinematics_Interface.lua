-- ArduKinematics_Interface.lua
-- ArduPilot bridge: pulls targets, runs IK, applies gap corrections and sends PWM.

local core = require("ArduKinematics_Core")

-- Link lengths (meters) pulled from Gazebo SDF offsets
local FL = 0.135
local AL = 0.175

-- Joint limits (radians)
local SHOULDER_MIN = -1.57
local SHOULDER_MAX = 1.57
local ELBOW_MIN = 0.0
local ELBOW_MAX = 3.14

-- Helpers
local function clamp(x, lo, hi)
    if x < lo then
        return lo
    elseif x > hi then
        return hi
    end
    return x
end

local function angle_to_pwm(angle, min_angle, max_angle, pwm_min, pwm_max)
    local span = max_angle - min_angle
    if span == 0 then
        return math.floor(0.5 * (pwm_min + pwm_max) + 0.5)
    end
    local t = (angle - min_angle) / span
    t = clamp(t, 0, 1)
    return math.floor(pwm_min + t * (pwm_max - pwm_min) + 0.5)
end

-- Script parameter table
local PARAM_TABLE_KEY = 187
local PARAM_TABLE_PREFIX = "IK_"

local function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format("could not find %s parameter", name))
    return p
end

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format("could not add param %s", name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 10), "could not add IK param table")

local IK_SH_FUNC = bind_add_param("SHFN",   1, 0)     -- servo function id for shoulder
local IK_EL_FUNC = bind_add_param("ELFN",   2, 0)     -- servo function id for elbow
local IK_SH_GAP  = bind_add_param("SHGAP",  3, 0.0)   -- shoulder gap correction (rad)
local IK_EL_GAP  = bind_add_param("ELGAP",  4, 0.0)   -- elbow gap correction (rad)
local IK_PWM_MIN = bind_add_param("PWM_MIN",5, 1000)
local IK_PWM_MAX = bind_add_param("PWM_MAX",6, 2000)
local IK_TX      = bind_add_param("TX",     7, 0.15)  -- target X (m)
local IK_TZ      = bind_add_param("TZ",     8, 0.05)  -- target Z (m)
local IK_HZ      = bind_add_param("HZ",     9, 10)    -- loop frequency (Hz)
local IK_LOG     = bind_add_param("LOG",    10,1)     -- 0 = silent, 1 = gcs texts

local last_notice = nil
local function notify_once(key, message)
    if IK_LOG:get() < 1 then
        return
    end
    if last_notice ~= key then
        last_notice = key
        if gcs and message then
            gcs:send_text(0, message)
        end
    end
end

local function loop_period_ms()
    local hz = IK_HZ:get()
    hz = clamp(hz, 1, 100)
    return math.floor(1000 / hz + 0.5)
end

local function get_target()
    return IK_TX:get(), IK_TZ:get()
end

-- Keep last commanded PWM to avoid sudden jumps when errors occur
local last_pwm_shoulder = angle_to_pwm(0, SHOULDER_MIN, SHOULDER_MAX, IK_PWM_MIN:get(), IK_PWM_MAX:get())
local last_pwm_elbow = angle_to_pwm(0, ELBOW_MIN, ELBOW_MAX, IK_PWM_MIN:get(), IK_PWM_MAX:get())

local function push_outputs(sh_pwm, el_pwm)
    local sh_fn = IK_SH_FUNC:get()
    local el_fn = IK_EL_FUNC:get()
    if sh_fn > 0 then
        SRV_Channels:set_output_pwm(sh_fn, sh_pwm)
    end
    if el_fn > 0 then
        SRV_Channels:set_output_pwm(el_fn, el_pwm)
    end
end

local function update()
    local period = loop_period_ms()
    local x, z = get_target()

    local sa, ea = core.solve_2dof_ik(x, z, FL, AL)
    if not sa then
        notify_once("unreachable", string.format("IK unreachable x=%.3f z=%.3f", x, z))
        push_outputs(last_pwm_shoulder, last_pwm_elbow)
        return update, period
    end

    local sa_adj = sa - IK_SH_GAP:get() -- gap correction
    local ea_adj = ea - IK_EL_GAP:get()

    if sa_adj < SHOULDER_MIN or sa_adj > SHOULDER_MAX or ea_adj < ELBOW_MIN or ea_adj > ELBOW_MAX then
        notify_once("limits", string.format("IK limit sa=%.3f ea=%.3f", sa_adj, ea_adj))
        push_outputs(last_pwm_shoulder, last_pwm_elbow)
        return update, period
    end

    local pwm_min = IK_PWM_MIN:get()
    local pwm_max = IK_PWM_MAX:get()
    local sh_pwm = angle_to_pwm(sa_adj, SHOULDER_MIN, SHOULDER_MAX, pwm_min, pwm_max)
    local el_pwm = angle_to_pwm(ea_adj, ELBOW_MIN, ELBOW_MAX, pwm_min, pwm_max)

    push_outputs(sh_pwm, el_pwm)
    last_pwm_shoulder = sh_pwm
    last_pwm_elbow = el_pwm

    notify_once("ok", string.format("IK ok x=%.3f z=%.3f", x, z))
    return update, period
end

notify_once("init", "ArduKinematics interface loaded")
return update()
