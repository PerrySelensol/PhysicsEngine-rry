local quatMath = require("physEngine/libs/quaternions")
require("physEngine/libs/vectors")
local CollisionSolver = require("physEngine/collisionSolver")

--[=============================================================================]--

local ContactGenerators = {}

-- Project box onto an axis, by convexity of it,
-- the legnth of the projected box is always one of its vertex as its endpoint
-- Use box.oriMat[i] as those are already box's axes direction in world space
local abs = math.abs
local function halfBoxLengthAlongAxis(box, axis)
	return
		box.halfSizeX * abs(axis .. box.oriMat[1]) +
		box.halfSizeY * abs(axis .. box.oriMat[2]) +
		box.halfSizeZ * abs(axis .. box.oriMat[3])
end

function ContactGenerators.boxBoxContacts(A, B)
	local worldAxisX_A, worldAxisY_A, worldAxisZ_A = A.oriMat[1], A.oriMat[2], A.oriMat[3]
	local worldAxisX_B, worldAxisY_B, worldAxisZ_B = B.oriMat[1], B.oriMat[2], B.oriMat[3]

	-- A's vertices inside B

	-- B's vertices inside A

	-- Edge-edge contacts
end

local function sortByPenetration(contact1, contact2)
	return contact1.penetration > contact2.penetration
end

function ContactGenerators.boxToHalfSpaceContacts(box, plane)
	local worldAxisX, worldAxisY, worldAxisZ = box.oriMat[1], box.oriMat[2], box.oriMat[3]
	
	-- Check all vertices
	local contacts = {}

	for i = -1, 1, 2 do for j = -1, 1, 2 do for k = -1, 1, 2 do
		local vert = vec(i*box.halfSizeX, j*box.halfSizeY, k*box.halfSizeZ)
		local vertInWorldSpace = box.oriMat*vert + box.pos
		local vertInPlaneSpace = plane.dir .. (vertInWorldSpace - plane.pos)
		if vertInPlaneSpace < 0 then
			--point(vertInWorldSpace)
			--particles["electric_spark"]:lifetime(20):pos(vertInWorldSpace):color(i, j, k):spawn()
			table.insert(contacts, {
				A = box,

				contactPoint = vertInWorldSpace,
				contactNormalA = plane.dir,

				penetration = -vertInPlaneSpace,

				restitution = box.restitution*plane.restitution,
				friction = box.friction*plane.friction
			})
		end
	end end end

	table.sort(contacts, sortByPenetration)
	--trint(2, contacts)
	for i = 1, #contacts do CollisionSolver:addContactData(contacts[i]) end
end

return ContactGenerators
