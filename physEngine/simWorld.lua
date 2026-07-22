local ForceGenerators = require("./forceGenerators/forceGens")
local ContactGenerators = require("./contacts/init")

local Solvers = require("physEngine/solvers/init")

--[=============================================================================]--

local BODY_ORDER = {
	box = 1,
	halfSpace = 2,
}

local simWorld = {
	isRunning = true,
	worldPart = models:newPart("simWorldPart", "World"),

	rigidBodies = {},
	constraints = {},
	solver = "pgs",

	-- In Figura, tick is running at constant speed,
	-- but we can change the duration to frame time if needed
	stepDuration = 1/20,
	worldSubsteps = 2,

	velocityIterations = 4,
	positionIterations = 2
}
--simWorld.worldPart:pos(16*vec(50, 259, 21))

function simWorld:render(delta)
	for _, body in next, self.rigidBodies do
		if not body.noRender then body:render(self.isRunning and delta or 1) end
	end
end

function simWorld:addRigidBody(body) table.insert(self.rigidBodies, body) return body end

function simWorld:integrateBodyPositions(dt)
	for _, body in ipairs(self.rigidBodies) do
		if not body.colliderOnly then
			body:integratePosition(dt)
			body:calculateDerivedData()
		end
	end
end
function simWorld:integrateBodyVelocities(dt)
	ForceGenerators.updateAllForces(dt)
	for _, body in ipairs(self.rigidBodies) do
		if not body.colliderOnly then
			body:integrateVelocity(dt)
		end
	end
end

function simWorld:addConstraint(data)
	assert(data.type, "no type")
	assert(data.A, "no A")
	if data.type == "contact" then
		assert(data.contactPointA, "no contactPointA")
		assert(data.contactPointB, "no contactPointB")
		assert(data.contactNormal, "no normal")
		assert(data.friction, "no friction")
		assert(data.restitution, "no restitution")
	end

	table.insert(self.constraints, data)
end

function simWorld:step(manualStep)
	if not (self.isRunning or manualStep) then return end

	local dt = self.stepDuration/self.worldSubsteps
	local rigidBodies = self.rigidBodies

	for _, body in ipairs(rigidBodies) do
		if not body.colliderOnly then
			body.render_pos = body.pos
			body.render_ori = body.ori
		end
	end

	for _ = 1, self.worldSubsteps do

		-- Currently uses narrow phase only
		for i = 1, #rigidBodies do for j = i+1, #rigidBodies do
			local typeA, typeB = rigidBodies[i].type, rigidBodies[j].type
			if typeA == "halfSpace" and typeB == "halfSpace" then goto endOfLoop end
			if BODY_ORDER[typeA] <= BODY_ORDER[typeB] then
				ContactGenerators[typeA .. typeB](self, rigidBodies[i], rigidBodies[j])
			else
				ContactGenerators[typeB .. typeA](self, rigidBodies[j], rigidBodies[i])
			end
			::endOfLoop::
		end end

		Solvers[self.solver](self)

	end
end

--[=============================================================================]--

function events.tick() simWorld:step() end
local renderName = host:isHost() and "world_render" or "render"
events[renderName] = function(delta) simWorld:render(delta) end

keybinds:newKeybind("pause/play", "key.keyboard.page.up"):onPress(function()
	simWorld.isRunning = not simWorld.isRunning
end)
keybinds:newKeybind("step", "key.keyboard.end"):onPress(function()
	simWorld:step(true)
end)

return simWorld