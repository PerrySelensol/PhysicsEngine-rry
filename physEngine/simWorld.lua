local ForceGenerators = require("physEngine/forceGens")

--[=============================================================================]--

local simWorld = {}

local simWorldPart = models:newPart("simWorldPart", "World")--:pos(16*vec(-86, 163, -207))

local renderName = host:isHost() and "world_render" or "render"
events[renderName] = function(delta)
	--drint(simWorld[1].ori_, simWorld[1].ori)
	for i, body in ipairs(simWorld) do
		body:render(delta)
	end
end

-- In Figura, tick is running at constant speed,
-- but we can change the duration to frame time if needed
local TIME_STEP_DURATION = 1/20

function events.tick()
	for _, body in ipairs(simWorld) do
		ForceGenerators.updateAllForces(TIME_STEP_DURATION)
		body:integrate(TIME_STEP_DURATION)

		--body:solvePosition()
		--body:recalculateMotion(TIME_STEP_DURATION)
		--body:solveVelocity()
	end
end



return simWorld, simWorldPart