local ContactGenerators = require("./contacts")

--[=============================================================================]--

function ContactGenerators.boxhalfSpace(solver, box, plane)
	-- Check all vertices
	for i = -1, 1, 2 do for j = -1, 1, 2 do for k = -1, 1, 2 do

		local vert = vec(i, j, k) * box.halfSizes
		local vertInWorldSpace = box.oriMat*vert + box.pos
		local vertInPlaneSpace = plane.inverseOriMat*(vertInWorldSpace - plane.pos)

		if vertInPlaneSpace.y < 0 then
			solver:addContactData{
				A = box,
				B_oriMat = plane.oriMat,

				contactPointA = vert,
				contactPointB = vertInPlaneSpace*vec(1,0,1),

				contactNormal = plane.oriMat[2],

				penetration = -vertInPlaneSpace.y,

				restitution = box.restitution*plane.restitution,
				friction = box.friction*plane.friction
			}
		end
		
	end end end
end
