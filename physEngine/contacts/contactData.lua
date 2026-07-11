local quatMath = require("physEngine/libs/quaternions")
require("physEngine/libs/vectors")

--[=============================================================================]--

local ContactData = {}
--[[
ContactData: {
	-- Two bodies, B may be absent (such as when B is immovable)
	A,
	B,

	contactPoint,

	-- Contact normal convention here: A's points into A itself
	-- B's contact normal is always a negation of A's
	contactNormalA,

	penetration,

	friction,
	restitution
}
--]]


function ContactData:new(o)
	o = o or {}
	setmetatable(o, self)
	self.__index = self
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
	self.contactMatrix = generateOrthoBasis(self.contactNormalA)

	local totalInertia

	local relativeContactPointA = self.contactPoint - self.A.pos
	local linearInertiaA = I3 * self.A.inverseMass
	local angularInertiaA

	local relativeContactPointB
	local linearInertiaB
	local angularInertiaB

	do
		local cross_relativeContactPointA = crossMat(relativeContactPointA)

		local angularImpulsePerLinearImpulse = cross_relativeContactPointA * self.contactMatrix
		local rotPerUnit = self.A.inverseInertiaTensorWorld * angularImpulsePerLinearImpulse
		local velPerUnit = cross_relativeContactPointA * rotPerUnit * -1

		angularInertiaA = self.contactMatrix:transposed() * velPerUnit

		totalInertia = angularInertiaA + linearInertiaA
	end

	if self.B then
		relativeContactPointB = self.contactPoint - self.B.pos
		linearInertiaB = I3 * self.B.inverseMass

		local cross_relativeContactPointB = crossMat(relativeContactPointB)

		local angularImpulsePerLinearImpulse = cross_relativeContactPointB * self.contactMatrix
		local rotPerUnit = self.B.inverseInertiaTensorWorld * angularImpulsePerLinearImpulse
		local velPerUnit = cross_relativeContactPointB * rotPerUnit * -1

		angularInertiaB = self.contactMatrix:transposed() * velPerUnit

		totalInertia = totalInertia + angularInertiaB + linearInertiaB
	end

	self.totalInertia = totalInertia

	self.relativeContactPointA = relativeContactPointA
	self.linearInertiaA = linearInertiaA
	self.angularInertiaA = angularInertiaA

	self.relativeContactPointB = relativeContactPointB
	self.linearInertiaB = linearInertiaB
	self.angularInertiaB = angularInertiaB

end

local function getSeparatingVel(A, B, contactPoint, contactMatrix)
	local totalSepVel = A.vel + (A.rot ^ (contactPoint - A.pos))
	if B then
		totalSepVel = totalSepVel - (B.vel + (B.rot ^ (contactPoint - B.pos)))
	end
	return contactMatrix:transposed() * totalSepVel
end

local SLOW_CLOSING_VELOCITY_LIMIT = 0.1
function ContactData:solveVelocity()
	local separatingVel = getSeparatingVel(self.A, self.B, self.contactPoint, self.contactMatrix)

	-- Pairs of contact points moving away need no solving
	if separatingVel.x > 0 then return end
	-- Completely remove bouncing for very slow closing velocity
	if -separatingVel.x < SLOW_CLOSING_VELOCITY_LIMIT then self.restitution = 0 end

	-- This is our target change in separating velocity after collision
	local targetVelChange = vec(
		-separatingVel.x*(1 + self.restitution),
		-separatingVel.y,
		-separatingVel.z
	)

	-- Distribute this targetVelChange to the 2 bodies
	-- Bouncing with static friction (planar velocity fully removed)
	local totalImpulse = self.totalInertia:inverted() * targetVelChange

	-- Bouncing with dynamic friction (I barely understand friction calculation for this :skull:)
	local planarImpulse = totalImpulse.yz:length()
	if planarImpulse > totalImpulse.x * self.friction then
		totalImpulse.y = totalImpulse.y / planarImpulse
		totalImpulse.z = totalImpulse.z / planarImpulse

		local totalImpulseX =
			targetVelChange.x / (self.totalInertia[1][1]
			+ self.totalInertia[2][1]*self.friction*totalImpulse.y
			+ self.totalInertia[3][1]*self.friction*totalImpulse.z)
		totalImpulse = totalImpulse*totalImpulseX*self.friction
		totalImpulse.x = totalImpulseX
	end

	-- Convert impulse to world space then applying it to both bodies
	local totalImpulseWorld = self.contactMatrix * totalImpulse
	self.A:addWorldImpulse(totalImpulseWorld, self.contactPoint-self.A.pos)
	if self.B then self.B:addWorldImpulse(-totalImpulseWorld, self.contactPoint-self.B.pos) end
end

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

function ContactData:solvePenetration()
	if self.penetration <= 0 then return end
	local inverseInertia = 1 / self.totalInertia[1][1]

	local linearMoveA = self.penetration * self.linearInertiaA[1][1] * inverseInertia
	local angularMoveA = self.penetration * self.angularInertiaA[1][1] * inverseInertia

	linearMoveA, angularMoveA = limitAngularMove(linearMoveA, angularMoveA, self.relativeContactPointA:length())

	self.A:nudge(
		linearMoveA * self.contactNormalA,
		(
			self.A.inverseInertiaTensorWorld *
			(self.relativeContactPointA ^ self.contactNormalA) *
			(1/self.angularInertiaA[1][1]) * angularMoveA
		)
	)

	if self.B then
		local linearMoveB = -self.penetration * self.linearInertiaB[1][1] * inverseInertia
		local angularMoveB = -self.penetration * self.angularInertiaB[1][1] * inverseInertia

		self.B:nudge(
			linearMoveB * self.contactNormalA,
			(
				self.B.inverseInertiaTensorWorld *
				(self.relativeContactPointB ^ self.contactNormalA) *
				(1/self.angularInertiaB[1][1]) * angularMoveB
			)
		)
	end
end

return ContactData
