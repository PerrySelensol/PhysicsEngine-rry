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

function CollisionSolver:solve()
	for _, data in ipairs(self) do
		if not data.bodyB then -- Single body case (such as body-half space)
			local A = self.bodyA
			local relativeContactPoint = self.contactPoint - A.pos
			local impulseTorquePerUnit = relativeContactPoint^self.contactNormalA
			local rotPerUnit = A.inverseInertiaTensor* impulseTorquePerUnit
		else
		end
	end
end