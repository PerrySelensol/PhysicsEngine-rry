local ContactGenerators = require("./contacts")

--[=============================================================================]--

local sortByPenetration = ContactGenerators.sortByPenetration
function ContactGenerators.boxhalfSpace(solver, box, plane)
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
	for i = 1, #contacts do solver:addContactData(contacts[i]) end
end
