local quatMath = require("physEngine/libs/quaternions")
require("physEngine/libs/vectors")

--[=============================================================================]--

local CollisionSolver = {}

CollisionSolver.addContactData = table.insert
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

local function calculateInertiaAtContact(A, B, contactPoint, contactMatrix)
	local totalInertia

	local relativeContactPointA = contactPoint - A.pos
	local linearInertiaA = I3 * A.inverseMass
	local angularInertiaA

	local relativeContactPointB
	local linearInertiaB
	local angularInertiaB

	do
		local cross_relativeContactPointA = crossMat(relativeContactPointA)

		local angularImpulsePerLinearImpulse = cross_relativeContactPointA * contactMatrix
		local rotPerUnit = A.inverseInertiaTensorWorld * angularImpulsePerLinearImpulse
		local velPerUnit = cross_relativeContactPointA * rotPerUnit * -1

		angularInertiaA = contactMatrix:transposed() * velPerUnit
		--print(contactMatrix:transposed()*cross_relativeContactPointA*A.inverseInertiaTensorWorld*cross_relativeContactPointA*contactMatrix*-1)
		--print(angularInertiaA)

		totalInertia = angularInertiaA + linearInertiaA
	end

	if B then
		relativeContactPointB = contactPoint - B.pos
		linearInertiaB = I3 * B.inverseMass

		local cross_relativeContactPointB = crossMat(relativeContactPointB)

		local angularImpulsePerLinearImpulse = cross_relativeContactPointB * contactMatrix
		local rotPerUnit = B.inverseInertiaTensorWorld * angularImpulsePerLinearImpulse
		local velPerUnit = cross_relativeContactPointB * rotPerUnit * -1

		angularInertiaB = contactMatrix:transposed() * velPerUnit

		totalInertia = totalInertia + angularInertiaB + linearInertiaB
	end

	return
		totalInertia,

		relativeContactPointA,
		linearInertiaA,
		angularInertiaA,

		relativeContactPointB,
		linearInertiaB,
		angularInertiaB
end

local function getSeparatingVel(A, B, contactPoint, contactMatrix)
	local totalSepVel = A.vel + (A.rot ^ (contactPoint - A.pos))
	if B then
		totalSepVel = totalSepVel + (B.vel + (B.rot ^ (contactPoint - B.pos)))
	end
	return contactMatrix:transposed() * totalSepVel
end

local VELOCITY_LIMIT = 0.1
local function solveVelocity(
	A,
	B,

	contactPoint,
	contactMatrix,

	totalInertia,

	friction,
	restitution
)
	local separatingVel = getSeparatingVel(A, B, contactPoint, contactMatrix)

	-- Pairs of contact points moving away need no solving
	if separatingVel.x > 0 then return end
	if -separatingVel.x < VELOCITY_LIMIT then restitution = 0 end

	-- This is our target separating velocity after collision
	local targetVelChange = vec(
		-separatingVel.x*(1 + restitution),
		-separatingVel.y,
		-separatingVel.z
	)

	-- Distribute this targetSepVel to the 2 masses
	local totalImpulse = totalInertia:inverted() * targetVelChange

	totalImpulse.y, totalImpulse.z = totalImpulse.y*friction, totalImpulse.z*friction
	local totalImpulseWorld = contactMatrix * totalImpulse

	A:addWorldImpulse(totalImpulseWorld, contactPoint-A.pos)
	if B then B:addWorldImpulse(-totalImpulseWorld, contactPoint-B.pos) end
end

local ANGULAR_LIMIT = 0.2
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

local function solvePenetration(
	A,
	B,
	penetration,
	contactNormalA,

	totalInertia,

	relativeContactPointA,
	linearInertiaA,
	angularInertiaA,
	
	relativeContactPointB,
	linearInertiaB,
	angularInertiaB
)
	local inverseInertia = 1 / totalInertia

	local linearMoveA = penetration * linearInertiaA * inverseInertia
	local angularMoveA = penetration * angularInertiaA * inverseInertia

	linearMoveA, angularMoveA = limitAngularMove(linearMoveA, angularMoveA, relativeContactPointA:length())

	A:nudge(
		linearMoveA * contactNormalA,
		A.inverseInertiaTensorWorld * (relativeContactPointA ^ contactNormalA) * (1/angularInertiaA) * angularMoveA
	)

	if B then
		local linearMoveB = -penetration * linearInertiaB * inverseInertia
		local angularMoveB = -penetration * angularInertiaB * inverseInertia

		B:nudge(
			linearMoveB * contactNormalA,
			B.inverseInertiaTensorWorld * (relativeContactPointB ^ contactNormalA) * (1/angularInertiaB) * angularMoveB
		)
	end
end

function CollisionSolver:solve(duration)
	for i, data in ipairs(self) do
		local A = data.A
		local B = data.B
		local contactPoint = data.contactPoint
		local contactMatrix = generateOrthoBasis(data.contactNormalA)
		local penetration = data.penetration

		local
			totalInertia, -- Change of vel per unit impulse

			relativeContactPointA,
			linearInertiaA,
			angularInertiaA,

			relativeContactPointB,
			linearInertiaB,
			angularInertiaB
		= calculateInertiaAtContact(A, B, contactPoint, contactMatrix)

		solvePenetration(
			A,
			B,
			penetration,
			data.contactNormalA,

			totalInertia[1][1],

			relativeContactPointA,
			linearInertiaA[1][1],
			angularInertiaA[1][1],
			
			relativeContactPointB,
			B and linearInertiaB[1][1],
			B and angularInertiaB[1][1]
		)

		solveVelocity(A, B, contactPoint, contactMatrix, totalInertia, data.friction, data.restitution)

		self[i] = nil
	end
end

return CollisionSolver