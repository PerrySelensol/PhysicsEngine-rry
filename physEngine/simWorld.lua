local ForceGenerators = require("./forceGenerators/forceGens")
local CollisionSolver = require("./collisionSolver")
local ContactGenerators = require("./contacts/init")

--[=============================================================================]--

local simWorld = {}
local simRunning = true

local simWorldPart = models:newPart("simWorldPart", "World")--:pos(16*vec(50, 259, 21))

local renderName = host:isHost() and "world_render" or "render"
events[renderName] = function(delta)
	for _, body in ipairs(simWorld) do
		if not body.noRender then body:render(simRunning and delta or 1) end
	end
end

local typeOrder = {
	box = 1,
	halfSpace = 2,
}

-- In Figura, tick is running at constant speed,
-- but we can change the duration to frame time if needed
local TIME_STEP_DURATION = 1/60
local SUBSTEPS = 1

local dt = TIME_STEP_DURATION/SUBSTEPS

local function step()
	for _, body in ipairs(simWorld) do
		if not body.colliderOnly then
			body.render_pos = body.pos
			body.render_ori = body.ori
		end
	end

	for _ = 1, SUBSTEPS do
		ForceGenerators.updateAllForces(dt)
		
		for _, body in ipairs(simWorld) do
			if not body.colliderOnly then
				body:integrateVelocity(dt)
				body:calculateDerivedData()
			end
		end

		-- Currently uses narrow phase only
		for i = 1, #simWorld do for j = i+1, #simWorld do
			local typeA, typeB = simWorld[i].type, simWorld[j].type
			if typeA == "halfSpace" and typeB == "halfSpace" then goto endOfLoop end
			if typeOrder[typeA] <= typeOrder[typeB] then
				ContactGenerators[typeA .. typeB](CollisionSolver, simWorld[i], simWorld[j])
			else
				ContactGenerators[typeB .. typeA](CollisionSolver, simWorld[j], simWorld[i])
			end
			::endOfLoop::
		end end

		CollisionSolver:solve(dt)

		for _, body in ipairs(simWorld) do
			if not body.colliderOnly then
				body:integratePosition(dt)
			end
		end
	end
end

events.tick[simRunning and "register" or "remove"](events.tick, step)
keybinds:newKeybind("step", "key.keyboard.page.up"):onPress(function()
	simRunning = not simRunning
	events.tick[simRunning and "register" or "remove"](events.tick, step)
end)
keybinds:newKeybind("step", "key.keyboard.end"):onPress(step)

return simWorld, simWorldPart