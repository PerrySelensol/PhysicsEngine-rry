local ForceGenerators = require("physEngine/forceGenerators/forceGens")
local CollisionSolver = require("physEngine/collisionSolver")
local ContactGenerators = require("physEngine/contacts/contacts")

--[=============================================================================]--

local simWorld = {}
local simRunning = true

local simWorldPart = models:newPart("simWorldPart", "World")--:pos(16*vec(-481, 128, -304))

local renderName = host:isHost() and "world_render" or "render"
events[renderName] = function(delta)
	for i, body in ipairs(simWorld) do
		if not body.noRender then body:render(simRunning and delta or 1) end
	end
end

-- In Figura, tick is running at constant speed,
-- but we can change the duration to frame time if needed
local TIME_STEP_DURATION = 1/20

local function step()
	for _, body in ipairs(simWorld) do
		if not body.colliderOnly then
			ForceGenerators.updateAllForces(TIME_STEP_DURATION)
			body:integrate(TIME_STEP_DURATION)
		end
	end

	-- Manual contact generation :skull:
	ContactGenerators.boxToHalfSpaceContacts(simWorld[2], simWorld[1])

	CollisionSolver:solve(TIME_STEP_DURATION)

end

events.tick:register(step)
keybinds:newKeybind("step", "key.keyboard.page.up"):onPress(function()
	events.tick[simRunning and "remove" or "register"](events.tick, step)
	simRunning = not simRunning
end)
keybinds:newKeybind("step", "key.keyboard.end"):onPress(step)

return simWorld, simWorldPart