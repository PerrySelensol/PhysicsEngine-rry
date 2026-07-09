local ForceGenerators = require("./forceGenerators/forceGens")
local CollisionSolver = require("./collisionSolver")
local ContactGenerators = require("./contacts/init")

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

local typeOrder = {
	box = 1,
	halfSpace = 2,
}

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

	-- Currently uses narrow phase only
	for i = 1, #simWorld do for j = i+1, #simWorld do
		local typeA, typeB = simWorld[i].type, simWorld[j].type
		if typeOrder[typeA] <= typeOrder[typeB] then
			ContactGenerators[typeA .. typeB](CollisionSolver, simWorld[i], simWorld[j])
		else
			ContactGenerators[typeB .. typeA](CollisionSolver, simWorld[j], simWorld[i])
		end
	end end

	CollisionSolver:solve(TIME_STEP_DURATION)

end

events.tick[simRunning and "register" or "remove"](events.tick, step)
keybinds:newKeybind("step", "key.keyboard.page.up"):onPress(function()
	simRunning = not simRunning
	events.tick[simRunning and "register" or "remove"](events.tick, step)
end)
keybinds:newKeybind("step", "key.keyboard.end"):onPress(step)

return simWorld, simWorldPart