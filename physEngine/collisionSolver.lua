local quatMath = require("physEngine/quaternions")
require("physEngine/vectors")

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

local function generateOrthoBasis(fixedX, suggestY)
	local genZ = fixedX^suggestY

	-- Bad generation will fail
	if genZ:lengthSquared() == 0 then return end

	genZ:normalize()
	local genY = genZ^fixedX

	return fixedX, genY, genZ
end

local function calculateInertiaAtContact(A, B, contactPoint, contactNormalA)
	local totalInertia

	local relativeContactPointA = contactPoint - A.pos
	local linearInertiaA = A.inverseMass
	local angularInertiaA

	local relativeContactPointB
	local linearInertiaB
	local angularInertiaB

	do
		local angularImpulsePerLinearImpulse = relativeContactPointA ^ contactNormalA
		local rotPerUnit = A.inverseInertiaTensorWorld * angularImpulsePerLinearImpulse
		local velPerUnit = rotPerUnit ^ relativeContactPointA

		angularInertiaA = velPerUnit .. contactNormalA

		totalInertia = angularInertiaA + linearInertiaA
	end

	if B then
		relativeContactPointB = contactPoint - B.pos
		linearInertiaB = B.inverseMass

		local angularImpulsePerLinearImpulse = relativeContactPointB ^ contactNormalA
		local rotPerUnit = B.inverseInertiaTensorWorld * angularImpulsePerLinearImpulse
		local velPerUnit = rotPerUnit ^ relativeContactPointB

		angularInertiaB = velPerUnit .. contactNormalA

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

local function getSeparatingVel(A, B, contactPoint, contactNormalA)
	local totalSepVel = (A.vel + (A.rot ^ (contactPoint - A.pos))) .. contactNormalA
	if B then
		totalSepVel = totalSepVel + (B.vel + (B.rot ^ (contactPoint - B.pos))) .. contactNormalA
	end
	return totalSepVel
end

local function solveVelocity(
	A,
	B,

	contactPoint,
	contactNormalA,

	totalInertia,

	friction,
	restitution
)
	local separatingVel = getSeparatingVel(A, B, contactPoint, contactNormalA)

	-- Pairs of contact points moving away need no solving
	if separatingVel > 0 then return end

	-- This is our target separating velocity after collision
	local targetSepVel = -separatingVel*(1 + restitution)

	-- Distribute this targetSepVel to the 2 masses
	local totalImpulse = targetSepVel / totalInertia

	local totalImpulseWorld = totalImpulse * contactNormalA

	A:addWorldImpulse(totalImpulseWorld, contactPoint-A.pos)
	if B then B:addWorldImpulse(-totalImpulseWorld, contactPoint-B.pos) end
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
		local contactNormalA = data.contactNormalA
		local penetration = data.penetration

		local
			totalInertia, -- Change of vel per unit impulse

			relativeContactPointA,
			linearInertiaA,
			angularInertiaA,

			relativeContactPointB,
			linearInertiaB,
			angularInertiaB
		= calculateInertiaAtContact(A, B, contactPoint, contactNormalA)

		solvePenetration(
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

		solveVelocity(A, B, contactPoint, contactNormalA, totalInertia, data.friction, data.restitution)

		self[i] = nil
	end
end

return CollisionSolver