local quatMath = require("physEngine/quaternions")
require("physEngine/vectors")

--[=============================================================================]--

local ForceGenerators = {}
local forceRegistry = {}

function ForceGenerators.gravityForceGen(gravityAcc)
	return function(body, dt)
		-- Disallow infinite mass
		if body.inverseMass == 0 then return end
		body:addForceAtCenter(gravityAcc/body.inverseMass)
	end
end

function ForceGenerators.register(body, generator)
	local body_generator = {body, generator}
	forceRegistry[body_generator] = true
	return body_generator
end

function ForceGenerators.remove(body_generator)
	forceRegistry[body_generator] = nil
end

function ForceGenerators.updateAllForces(dt)
	for data in next, forceRegistry do
		data[2](data[1], dt)
	end
end

return ForceGenerators