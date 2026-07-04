local quatMath = require("physEngine/quaternions")
require("physEngine/vectors")

--[=============================================================================]--

local CollisionSolver = {}

CollisionSolver.addContactData = table.insert
--[[
ContactData: {
	bodyA,
	bodyB,
	contactPoint,
	contactNormalA,
	penetration
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

local function findImpulseAtContactPoint(bodyA, bodyB, contactPoint, contactNormalA)
	local relativeContactPoint = contactPoint - bodyA.pos

	local impulseTorquePerUnit = relativeContactPoint^contactNormalA
	local rotPerUnit = bodyA.inverseInertiaTensor* impulseTorquePerUnit
	local velPerUnit = rotPerUnit ^ relativeContactPoint

	local deltaVelAlongNormal = velPerUnit .. contactNormalA

	deltaVelAlongNormal = deltaVelAlongNormal + bodyA.inverseMass

	if bodyB then
		relativeContactPoint = contactPoint - bodyB.pos

		impulseTorquePerUnit = relativeContactPoint^contactNormalA
		rotPerUnit = bodyB.inverseInertiaTensor* impulseTorquePerUnit
		velPerUnit = rotPerUnit ^ relativeContactPoint

		deltaVelAlongNormal = velPerUnit .. contactNormalA

		deltaVelAlongNormal = deltaVelAlongNormal + bodyB.inverseMass
	end

	return deltaVelAlongNormal
end

function CollisionSolver:solve()
	for _, data in ipairs(self) do
		if not data.bodyB then -- Single body case (such as body-half space)


		else
		end
	end
end