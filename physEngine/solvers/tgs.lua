local ForceGenerators = require("physEngine/forceGenerators/forceGens")
local quatMath = require("physEngine/libs/quaternions")
require("physEngine/libs/vectors")
local common = require("./common")

--[=============================================================================]--

local SLOW_CLOSING_VELOCITY_LIMIT = 0.1
local function solveContact(contact, dt, useBias)
	-- Convert contact point to world orientation, but local position
	local contactPointA = contact.A.oriMat*contact.contactPointA
	local contactPointB = contact.B and contact.B.oriMat*contact.contactPointB or contact.B_oriMat*contact.contactPointB

	local penetration = contact.contactNormal ..
		((contactPointB + (contact.B and contact.B.pos or contact.B_pos)) - (contactPointA + contact.A.pos))

	local separatingVel = common.getSeparatingVel(
		contact.A, contact.B,
		contactPointA, contactPointB,
		contact.contactMatrix
	)

	-- Skip contact pairs that aren't penetrating (with small tolerance)
	if penetration < -0.005 then return end
	--point(self.A.oriMat*self.contactPointA + self.A.pos)
	
	-- Completely remove bouncing for very slow closing velocity
	local restitution = 0 --self.restitution
	if -separatingVel.x < SLOW_CLOSING_VELOCITY_LIMIT then restitution = 0 end

	-- Baumgarte Stabilization
	-- This bias, proportional to penetration depth, is added to total impulse
	-- Allows penetrating bodies to push themselves apart
	local impulseNormalBias = 0
	if useBias then
		local bias = math.max(0, (penetration - 0.01) * (0.2/dt))
		impulseNormalBias = bias / contact.totalInertia[1][1]
	end

	-- This is our target change in separating velocity after collision
	-- //TODO readd restitution
	local targetVelChange = vec(
		-separatingVel.x*(1 --[[+ restitution]]),
		-separatingVel.y,
		-separatingVel.z
	)

	-- Distribute this targetVelChange to the 2 bodies
	-- Bouncing with static friction (planar velocity fully removed)
	--local totalImpulse = contact.totalInertia:inverted() * targetVelChange
	local totalImpulse = targetVelChange.x__ / contact.totalInertia[1][1]
	totalImpulse.x = totalImpulse.x + impulseNormalBias

	-- Bouncing with dynamic friction (I barely understand friction calculation for this :skull:)
	-- Temporarily always use friction = 1 until I actually implement friction correctly
	-- //TODO readd proper friction calculation
	local planarImpulse = totalImpulse.yz:length()
	if false and planarImpulse > totalImpulse.x * contact.friction then
		totalImpulse.y = totalImpulse.y / planarImpulse
		totalImpulse.z = totalImpulse.z / planarImpulse

		local totalImpulseX =
			targetVelChange.x / (contact.totalInertia[1][1]
			+ contact.totalInertia[1][2]*contact.friction*totalImpulse.y
			+ contact.totalInertia[1][3]*contact.friction*totalImpulse.z)
		totalImpulse = totalImpulse*totalImpulseX*contact.friction
		totalImpulse.x = totalImpulseX
	end

	-- Clamp accumulated impulse along normal so it's non-negative at the end
	local oldAccumImpulse = contact.accumulatedNormalImpulse
	contact.accumulatedNormalImpulse = math.max(contact.accumulatedNormalImpulse + totalImpulse.x, 0)
	totalImpulse.x = contact.accumulatedNormalImpulse - oldAccumImpulse

	-- Convert impulse to world space then applying it to both bodies
	local totalImpulseWorld = contact.contactMatrix * totalImpulse
	contact.A:addWorldImpulse(totalImpulseWorld, contactPointA)
	if contact.B then contact.B:addWorldImpulse(-totalImpulseWorld, contactPointB) end

end

return function(world)
	local dt = world.stepDuration/world.worldSubsteps

	for _, contact in ipairs(world.constraints) do
		common.prepareContact(contact)
	end

	for _ = 1, world.velocityIterations do

		local h = dt/world.velocityIterations

		world:integrateBodyVelocities(h)

		for i, contact in ipairs(world.constraints) do
			solveContact(contact, h, true)
		end

		world:integrateBodyPositions(h)

	end

	--[[
		for _ = 1, world.positionIterations do

			local h = dt/world.velocityIterations

			world:integrateBodyVelocities(h)

			for i, contact in ipairs(world.constraints) do
				solveContact(contact, h, false)
			end

		end
	--]]

	for i = 1, #world.constraints do world.constraints[i] = nil end

end