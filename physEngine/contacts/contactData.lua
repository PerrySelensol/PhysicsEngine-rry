local quatMath = require("physEngine/libs/quaternions")
require("physEngine/libs/vectors")

--[=============================================================================]--

local ContactData = {}
--[[
ContactDataObject = {
	-- Two bodies, B may be absent (such as when B is immovable)
	A, B,

	-- Local contact points; both points must exist, so we can find penetration during solving
	contactPointA, contactPointB,

	-- Contact normal; the contact normal always points into A
	contactNormal,

	friction,
	restitution
}
--]]

function ContactData:new(o)
	o = o or {}
	setmetatable(o, self)
	self.__index = self
	o.accumulatedNormalImpulse = 0
	return o
end

-- Generate an arbitary basis with a given fixed axis
local generateOrthoBasis; do
	local Y1, Y2 = vec(1,0,0), vec(0,1,0)
	local abs, mat3 = math.abs, matrices.mat3
	function generateOrthoBasis(fixedX)
		local genZ = fixedX^((abs(fixedX..Y1) < 0.75) and Y1 or Y2)

		genZ:normalize()
		local genY = genZ^fixedX

		return mat3(fixedX, genY, genZ)
	end
end

local function crossMat(v) -- Cross product matrix so that crossMat(v) * u = v ^ u
	return matrices.mat3(
		vec(0,		v.z,	-v.y),
		vec(-v.z,	0,		v.x ),
		vec(v.y,	-v.x,	0   )
	)
end

local I3 = matrices.mat3()

function ContactData:calculateInertiaAtContact()
	self.contactMatrix = generateOrthoBasis(self.contactNormal)

	local totalInertia

	local linearInertiaA = I3 * self.A.inverseMass
	local angularInertiaA

	local linearInertiaB
	local angularInertiaB

	do
		local cross_relativeContactPointA = crossMat(self.A.oriMat * self.contactPointA)

		local angularImpulsePerLinearImpulse = cross_relativeContactPointA * self.contactMatrix
		local rotPerUnit = self.A.inverseInertiaTensorWorld * angularImpulsePerLinearImpulse
		local velPerUnit = cross_relativeContactPointA * rotPerUnit * -1

		angularInertiaA = self.contactMatrix:transposed() * velPerUnit

		totalInertia = angularInertiaA + linearInertiaA
	end

	if self.B then
		linearInertiaB = I3 * self.B.inverseMass

		local cross_relativeContactPointB = crossMat(self.B.oriMat * self.contactPointB)

		local angularImpulsePerLinearImpulse = cross_relativeContactPointB * self.contactMatrix
		local rotPerUnit = self.B.inverseInertiaTensorWorld * angularImpulsePerLinearImpulse
		local velPerUnit = cross_relativeContactPointB * rotPerUnit * -1

		angularInertiaB = self.contactMatrix:transposed() * velPerUnit

		totalInertia = totalInertia + angularInertiaB + linearInertiaB
	end

	self.totalInertia = totalInertia

	self.linearInertiaA = linearInertiaA
	self.angularInertiaA = angularInertiaA

	self.linearInertiaB = linearInertiaB
	self.angularInertiaB = angularInertiaB

end

local function getSeparatingVel(A, B, contactPointA, contactPointB, contactMatrix)
	local totalSepVel = A.vel + (A.rot ^ contactPointA)
	if B then
		totalSepVel = totalSepVel - (B.vel + (B.rot ^ contactPointB))
	end
	return contactMatrix:transposed() * totalSepVel
end

local SLOW_CLOSING_VELOCITY_LIMIT = 0.1
function ContactData:solveVelocity(dt)
	-- Convert contact point to world orientation, but local position
	local contactPointA = self.A.oriMat*self.contactPointA
	local contactPointB = self.B and self.B.oriMat*self.contactPointB or self.B_oriMat*self.contactPointB

	local penetration = self.contactNormal ..
		((contactPointB + (self.B and self.B.pos or self.B_pos)) - (contactPointA + self.A.pos))

	local separatingVel = getSeparatingVel(
		self.A, self.B,
		contactPointA, contactPointB,
		self.contactMatrix
	)

	-- Skip contact pairs that aren't penetrating (with small tolerance)
	if penetration < -0.005 then return end
	--point(self.A.oriMat*self.contactPointA + self.A.pos)
	
	-- Completely remove bouncing for very slow closing velocity
	local restitution = 0 --self.restitution
	if -separatingVel.x < SLOW_CLOSING_VELOCITY_LIMIT then restitution = 0 end

	-- Baumgarte Stabilization
	-- This bias, proportional to penetration depth, is added to target velocity change
	-- Allows penetrating bodies to push themselves apart
	local bias = math.max(0, (penetration - 0.01) * (0.2/dt))

	-- This is our target change in separating velocity after collision
	-- //TODO readd restitution
	local targetVelChange = vec(
		-separatingVel.x*(1 --[[+ restitution]]) + bias,
		-separatingVel.y,
		-separatingVel.z
	)

	-- Distribute this targetVelChange to the 2 bodies
	-- Bouncing with static friction (planar velocity fully removed)
	local totalImpulse = self.totalInertia:inverted() * targetVelChange --print(self.totalInertia)

	-- Bouncing with dynamic friction (I barely understand friction calculation for this :skull:)
	-- Temporarily always use friction = 1 until I actually implement friction correctly
	-- //TODO readd proper friction calculation
	local planarImpulse = totalImpulse.yz:length()
	if false and planarImpulse > totalImpulse.x * self.friction then
		totalImpulse.y = totalImpulse.y / planarImpulse
		totalImpulse.z = totalImpulse.z / planarImpulse

		local totalImpulseX =
			targetVelChange.x / (self.totalInertia[1][1]
			+ self.totalInertia[1][2]*self.friction*totalImpulse.y
			+ self.totalInertia[1][3]*self.friction*totalImpulse.z)
		totalImpulse = totalImpulse*totalImpulseX*self.friction
		totalImpulse.x = totalImpulseX
	end

	-- Clamp accumulated impulse along normal so it's non-negative at the end
	local oldAccumImpulse = self.accumulatedNormalImpulse
	self.accumulatedNormalImpulse = math.max(self.accumulatedNormalImpulse + totalImpulse.x, 0)
	totalImpulse.x = self.accumulatedNormalImpulse - oldAccumImpulse

	-- Convert impulse to world space then applying it to both bodies
	local totalImpulseWorld = self.contactMatrix * totalImpulse
	self.A:addWorldImpulse(totalImpulseWorld, contactPointA)
	if self.B then self.B:addWorldImpulse(-totalImpulseWorld, contactPointB) end
end

--[[
local ANGULAR_LIMIT = 0.1
local function limitAngularMove(linearMove, angularMove, bias)
	local limit = ANGULAR_LIMIT*bias
	local totalMove = linearMove + angularMove
	if angularMove > limit then
		angularMove = limit
	elseif angularMove < -limit then
		angularMove = -limit
	end
	return totalMove-angularMove, angularMove
end

local RELAX_FACTOR = 0.2
function ContactData:solvePenetration()
	local contactA = self.A.oriMat*self.contactPointA
	local contactB = self.B and self.B.oriMat*self.contactPointB or self.B_oriMat*self.contactPointB
	local penetration = self.contactNormal ..
		((contactB + (self.B and self.B.pos or self.B_pos)) - (contactA + self.A.pos))

	if penetration <= 0 then return end
	local inverseInertia = 1 / self.totalInertia[1][1]
	penetration = penetration * RELAX_FACTOR

	local linearMoveA = penetration * self.linearInertiaA[1][1] * inverseInertia
	local angularMoveA = penetration * self.angularInertiaA[1][1] * inverseInertia

	linearMoveA, angularMoveA = limitAngularMove(linearMoveA, angularMoveA, contactA:length())

	self.A:nudge(
		linearMoveA * self.contactNormal,
		(
			self.A.inverseInertiaTensorWorld *
			(contactA ^ self.contactNormal) *
			(1/self.angularInertiaA[1][1]) * angularMoveA
		)
	)

	if self.B then
		local linearMoveB = -penetration * self.linearInertiaB[1][1] * inverseInertia
		local angularMoveB = -penetration * self.angularInertiaB[1][1] * inverseInertia

		linearMoveB, angularMoveB = limitAngularMove(linearMoveB, angularMoveB, contactB:length())

		self.B:nudge(
			linearMoveB * self.contactNormal,
			(
				self.B.inverseInertiaTensorWorld *
				(contactB ^ self.contactNormal) *
				(1/self.angularInertiaB[1][1]) * angularMoveB
			)
		)
	end
end
--]]

return ContactData
