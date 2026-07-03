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
	for i, body in ipairs(simWorld) do
		ForceGenerators.updateAllForces(TIME_STEP_DURATION)
		body:integrate(TIME_STEP_DURATION)
	end
	--simWorld[1]:addForce(vec( 1,0,0), vec(0,1,0))
	--simWorld[1]:addForce(vec(-1,0,0), vec(0,-1,0))
	--simWorld[1]:addForce(vec( 1,0,0), vec(0,0,1))
	--simWorld[1]:addForce(vec(-1,0,0), vec(0,0,-1))
	--simWorld[1]:addForce(vec(0,1,0), vec(-1,-1,-1))
	--simWorld[1]:addForce(vec(0,-1,0), vec(1,1,1))
	--simWorld[2]:addForce(vec(0,1,0), vec(-1,-1,-1))
	--simWorld[2]:addForce(vec(0,-1,0), vec(1,1,1))
	--simWorld[1]:addForceAtBodyPoint(vec(0,1,0), vec(1,0,0))
	--simWorld[2]:addForceAtBodyPoint(vec(0,1,0), vec(1,0,0))
end



return simWorld, simWorldPart