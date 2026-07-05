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

local function getRateOfVelChangePerImpulse(A, B, contactPoint, contactNormalA)
	local deltaVelAlongNormal

	local relativeContactPointA = contactPoint - A.pos
	local linearInertiaA = A.inverseMass
	local angularInertiaA

	local relativeContactPointB
	local linearInertiaB
	local angularInertiaB

	do
		local angularImpulsePerLinearImpulse = relativeContactPointA ^ contactNormalA
		local rotPerUnit = A.inverseInertiaTensor * angularImpulsePerLinearImpulse
		local velPerUnit = rotPerUnit ^ relativeContactPointA

		angularInertiaA = velPerUnit .. contactNormalA

		deltaVelAlongNormal = angularInertiaA + linearInertiaA
	end

	if B then
		relativeContactPointB = contactPoint - B.pos
		linearInertiaB = B.inverseMass

		local angularImpulsePerLinearImpulse = relativeContactPointB ^ contactNormalA
		local rotPerUnit = B.inverseInertiaTensor * angularImpulsePerLinearImpulse
		local velPerUnit = rotPerUnit ^ relativeContactPointB

		angularInertiaB = velPerUnit .. contactNormalA

		deltaVelAlongNormal = deltaVelAlongNormal + angularInertiaB + linearInertiaB
	end

	return
		deltaVelAlongNormal,

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

local function solveVelocity(data, duration)
	local separatingVel = getSeparatingVel(data.A, data.B, data.contactPoint, data.contactNormalA)

	-- Pairs of contact points moving away need no solving
	if separatingVel > 0 then return end

	-- This is our target separating velocity after collision
	local targetSepVel = -separatingVel*(1 + data.restitution)

	-- Distribute this targetSepVel to the 2 masses

	local
		deltaVelChangePerImpulse,

		relativeContactPointA,
		linearInertiaA,
		angularInertiaA,
		
		relativeContactPointB,
		linearInertiaB,
		angularInertiaB
	= getRateOfVelChangePerImpulse(data.A, data.B, data.contactPoint, data.contactNormalA)

	local totalImpulse = targetSepVel / deltaVelChangePerImpulse

	local totalImpulseWorld = totalImpulse * data.contactNormalA

	data.A:addWorldImpulse(totalImpulseWorld, relativeContactPointA)
	if data.B then data.B:addWorldImpulse(-totalImpulseWorld, relativeContactPointB) end

	--[[ 	
	local totalInverseMass = data.A.inverseMass + (data.B and data.B.inverseMass or 0)
	local impulse = targetSepVel / totalInverseMass

	data.A.vel = data.A.vel + (data.contactNormalA*impulse*data.A.inverseMass)
	if data.B then
		data.B.vel = data.B.vel - (data.contactNormalA*impulse*data.B.inverseMass)
	end
	--]]
end

function CollisionSolver:solve(duration)
	for i, data in ipairs(self) do
		solveVelocity(data, duration)
		self[i] = nil
	end
end

return CollisionSolver