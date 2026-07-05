local ForceGenerators = require("physEngine/forceGens")
local CollisionSolver = require("physEngine/collisionSolver")
local ContactGenerators = require("physEngine/contacts")

--[=============================================================================]--

local simWorld = {}

local simWorldPart = models:newPart("simWorldPart", "World")--:pos(16*vec(-233, 63, 165))

local renderName = host:isHost() and "world_render" or "render"
events[renderName] = function(delta)
	--drint(simWorld[1].ori_, simWorld[1].ori)
	for i, body in ipairs(simWorld) do
		if not body.noRender then body:render(delta) end
	end
end

-- In Figura, tick is running at constant speed,
-- but we can change the duration to frame time if needed
local TIME_STEP_DURATION = 1/20

function events.tick()
	for _, body in ipairs(simWorld) do
		if not body.colliderOnly then
			ForceGenerators.updateAllForces(TIME_STEP_DURATION)
			body:integrate(TIME_STEP_DURATION)
		end
	end

	ContactGenerators.boxToHalfSpaceContacts(simWorld[2], simWorld[1])

	CollisionSolver:solve(TIME_STEP_DURATION)
end



return simWorld, simWorldPart