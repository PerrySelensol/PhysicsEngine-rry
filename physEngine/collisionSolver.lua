local ContactData = require("physEngine/contacts/contactData")

--[=============================================================================]--

local CollisionSolver = {}

function CollisionSolver:addContactData(o)
	table.insert(self, ContactData:new(o))
end

function CollisionSolver:solve(duration)
	for _, contact in ipairs(self) do
		contact:calculateInertiaAtContact()
		contact:calculateTargetSepVel()
		contact:solvePenetration()
		contact.penetration = contact.penetration*0.2
	end
	for _ = 1, 4 do
		for i, contact in ipairs(self) do
			contact:solveVelocity()
		end
	end
	for i = 1, #self do
		self[i] = nil
	end
end

return CollisionSolver