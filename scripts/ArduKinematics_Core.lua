local M = {}

-- Clamp a value into [-1, 1] to keep acos numerically safe.
local function clamp_unit(x)
    if x > 1 then
        return 1
    elseif x < -1 then
        return -1
    end
    return x
end

-- target (x, z) with link lengths FL (forearm) and AL (arm).
-- Returns shoulder angle (SA) and elbow angle (EA) in radians, or nil and a reason string when unreachable.
function M.solve_2dof_ik(x, z, FL, AL)
    if x == nil or z == nil or FL == nil or AL == nil then
        return nil, "invalid_input"
    end
    if FL <= 0 or AL <= 0 then
        return nil, "invalid_links"
    end

    local L = math.sqrt(x * x + z * z)

    -- Reachability check: target must be within the annulus |FL-AL| <= L <= FL+AL.
    local reach_far = FL + AL
    local reach_near = math.abs(FL - AL)
    if L > reach_far or L < reach_near then
        return nil, "unreachable"
    end

    -- Elbow angle via Law of Cosines.
    local cos_EA = (FL * FL + AL * AL - L * L) / (2 * FL * AL)
    cos_EA = clamp_unit(cos_EA)
    local EA = math.acos(cos_EA)

    -- Shoulder angle: target bearing minus internal angle from Law of Cosines.
    local Va = math.atan(z, x) -- atan2(z, x)
    local cos_Vb = (FL * FL + L * L - AL * AL) / (2 * FL * L)
    cos_Vb = clamp_unit(cos_Vb)
    local Vb = math.acos(cos_Vb)
    local SA = Va - Vb

    return SA, EA
end

return M
