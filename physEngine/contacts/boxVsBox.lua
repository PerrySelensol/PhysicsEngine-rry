local ContactGenerators = require("./contacts")

--[=============================================================================]--
local abs = math.abs

--[======================================================================[--
	>> Edge-Edge Contact Derivation <<

	Let A, B be a point on first and second line respectively (here we choose the midpoints)
	We first project 2 edges along the shortest line segment connecting between them.
	The problem reduces to finding an intersection between 2 lines, call that point T.
	This also forms a triangle ABT as shown.

		A _____________________ T
		 |alpha          theta /
		 |                  /
		 |                /
		 |              /
		 |            /
		 |          /
		 |        /
		 |      /
		 |beta/
		 |  /
		 |/
		B

	Let theta be an angle between 2 lines, i.e. angle(ATB).
	Also let alpha, beta be angles of angle(TAB) and angle(TBA) respectively.

	The formulae are based on law of sines, with some rewriting to
	have only in terms of cos(theta), cos(alpha), cos(beta).
	All the cos are then written as dot products of 2 triangle sides
	since we know the vectors pointing in the direction of them

	The formulae for length of AT and BT are
		AT = AB/sin(theta) * sin(beta) <<= law of sines
		= AB/sin(theta) * sin(beta)*sin(theta)/sin(theta)
		= AB/sin(theta)^2 * [cos(beta)*cos(theta) - cos(beta)*cos(theta) + sin(beta)*sin(theta)]
		= AB/sin(theta)^2 * [cos(beta)*cos(theta) - cos(beta+theta)] <<= angle sum for cosine
		= AB/sin(theta)^2 * [cos(beta)*cos(theta) + cos(180-beta-theta)]
		= AB/sin(theta)^2 * [cos(beta)*cos(theta) + cos(alpha)]
		= AB/(1-cos(theta)^2) * [cos(beta)*cos(theta) + cos(alpha)]
		= 1/(1-cos(theta)^2) * [AB*cos(beta)*cos(theta) + AB*cos(alpha)]
		= 1/(1-(dirTA .. dirTB)^2) * [(vecBA .. dirBT)*(dirTA .. dirTB) + (vecAB .. dirAT)]
	Similarly,
		BT = AB/(1-cos(theta)^2) * [cos(alpha)*cos(theta) + cos(beta)]
		= 1/(1-cos(theta)^2) * [AB*cos(alpha)*cos(theta) + AB*cos(beta)]
		= 1/(1-(dirTA .. dirTB)^2) * [(vecAB .. dirAT)*(dirTA .. dirTB) + (vecBA .. dirBT)]

--]======================================================================]--

local function edge_VS_edge_contact(edgeMidPointA, edgeDirA, edgeMidPointB, edgeDirB, halfSizeA, halfSizeB, selectA)
	-- Assume both edgeDirA and edgeDirB are already normalized
	local cosTheta = edgeDirB .. edgeDirA

	local vecBA = edgeMidPointA - edgeMidPointB
	local lengthAB_times_cosAlpha = -(edgeDirA .. vecBA)
	local lengthAB_times_cosBeta = edgeDirB .. vecBA

	local sinThetaSquared = 1 - cosTheta*cosTheta

	-- Parallel line case: just use edge midpoint
	if sinThetaSquared < 0.0001 and sinThetaSquared > -0.0001  then
		return selectA and edgeMidPointA or edgeMidPointB
	end

	local lengthAT = (lengthAB_times_cosBeta*cosTheta + lengthAB_times_cosAlpha) / sinThetaSquared
	local lengthBT = (lengthAB_times_cosAlpha*cosTheta + lengthAB_times_cosBeta) / sinThetaSquared

	-- The solution points lying outside an edge means the edge segments don't intersect
	-- We can just choose the midpoints as a contact point
	if false and (lengthAT > halfSizeA or lengthAT < -halfSizeA or
		lengthBT > halfSizeB or lengthBT < -halfSizeB)
	then
		return selectA and edgeMidPointA or edgeMidPointB
	end

	local T_on_A = edgeMidPointA + edgeDirA * lengthAT
	local T_on_B = edgeMidPointB + edgeDirB * lengthBT

	--for i = 1, 10 do point(edgeMidPointA + (2*math.random()-1)*halfSizeA*edgeDirA) end

	return (T_on_A + T_on_B) * 0.5
end

local function boxA_VS_pointB_contact(A, B, A_to_B, normalAxisA, penetration, friction, restitution)
	-- Flip so the A's face normal points into A
	if normalAxisA .. A_to_B > 0 then normalAxisA = -normalAxisA end

	-- Find the colliding corner
	local cornerB = vec(B.halfSizeX, B.halfSizeY, B.halfSizeZ)
	if (B.oriMat[1] .. normalAxisA) < 0 then cornerB.x = -cornerB.x end
	if (B.oriMat[2] .. normalAxisA) < 0 then cornerB.y = -cornerB.y end
	if (B.oriMat[3] .. normalAxisA) < 0 then cornerB.z = -cornerB.z end

	return {
		A = A,
		B = B,

		contactPoint = (B.oriMat * cornerB) + B.pos,
		contactNormalA = normalAxisA,

		penetration = penetration,

		friction = friction,
		restitution = restitution
	}
end

--[[
	local A, dirA, lenA = vec(1, 2, 3), vec(1, 1, -1), 2
	local B, dirB, lenB = vec(2, 3, 4), vec(0.5, -1, 0), 3
	local col = vec(1,0,0)

	local rand = function() return math.random()*2-1 end

	function events.render()
		for i = 1, 10 do
			point(A+dirA:normalized()*lenA*rand(), col)
		end
		for i = 1, 10 do
			point(B+dirB:normalized()*lenB*rand(), col)
		end

		point(A, vec(0,1,1))
		point(B, vec(0,1,1))
		point(nearestPointToTwoEdges(A, dirA:normalized(), B, dirB:normalized(), lenA, lenB, math.random()<0.5), vec(1,1,1))
	end
--]]

-- Project box onto an axis; by convexity of it,
-- an endpoint of the projected box is always one of its vertex.
-- Use box.oriMat[i] as those are already box's axes direction in world space
local function halfBoxLengthAlongAxis(box, axis)
	return
		box.halfSizeX * abs(axis .. box.oriMat[1]) +
		box.halfSizeY * abs(axis .. box.oriMat[2]) +
		box.halfSizeZ * abs(axis .. box.oriMat[3])
end

local function testSAT(A, B, A_to_B, axis)
	local projA = halfBoxLengthAlongAxis(A, axis)
	local projB = halfBoxLengthAlongAxis(B, axis)
	local projCenters = math.abs(A_to_B .. axis)

	local penetration = projA + projB - projCenters
	if penetration < 0 then return end
	return penetration
end

--[[ Scrapped cursed method for contact generation
local function AVerticesInBoxB(solver, A, B)
	-- Check all vertices
	for i = -1, 1, 2 do for j = -1, 1, 2 do for k = -1, 1, 2 do
		local vert = vec(i*A.halfSizeX, j*A.halfSizeY, k*A.halfSizeZ)
		local vertInWorldSpace = A.oriMat*vert + A.pos
		local vertInBSpace = B.oriMat:transposed() * (vertInWorldSpace - B.pos)
		if vertInBSpace < 0 then
			--point(vertInWorldSpace)
			--particles["electric_spark"]:lifetime(20):pos(vertInWorldSpace):color(i, j, k):spawn()
			solver:addContactData{
				A = A,
				B = B,

				contactPoint = vertInWorldSpace,
				contactNormalA = B.dir,

				penetration = -vertInBSpace,

				restitution = A.restitution*B.restitution,
				friction = A.friction*B.friction
			}
		end
	end end end
end

local COHERENT_PENETRATION_LIMIT = 0.02
function ContactGenerators.boxbox(solver, A, B)
	local X_A, Y_A, Z_A = A.oriMat[1], A.oriMat[2], A.oriMat[3]
	local X_B, Y_B, Z_B = B.oriMat[1], B.oriMat[2], B.oriMat[3]

	local A_to_B = B.pos - A.pos

	local testAxes = {}
	for i = 1, 3 do
		testAxes[i]		= A.oriMat[i]
		testAxes[i+3]	= B.oriMat[i]
	end
	for i = 1, 3 do for j = 1, 3 do
		local cross = testAxes[i]^testAxes[j+3]
		-- Ignore the parallel edges
		testAxes[(i+3*(j-1))+6] = (cross:lengthSquared() > 0.001) and cross:normalized()
	end end

	local penetrationData = {}
	local minPenetration
	for i = 1, 15 do
		if not testAxes[i] then goto endOfLoop end -- Skip parallel edges

		-- Return immediately if SAT test does separate the boxes
		local penetration = testSAT(A, B, A_to_B, testAxes[i])
		if not penetration then return end

		-- Otherwise add the penetration data for this axis
		-- while also add additional penetration data of similar depth
		if
			minPenetration and (abs(penetration - minPenetration) < COHERENT_PENETRATION_LIMIT)
		then
			penetrationData[i] = penetration
		elseif penetration < minPenetration then
			-- Replace the whole data if a much smaller penetration is found
			penetrationData = {[i] = penetration}
		end

		::endOfLoop::
	end

	-- If this chunk is run, SAT has failed in all axes; there is a collision
	-- We generate all the contact points from the obtained penetrationData
	for i = 1, 15 do
		if not penetrationData[i] then goto endOfLoop end
		if i <= 3 then
			-- B's vertices inside A
			AVerticesInBoxB(solver, B, A)
		elseif i <= 6 then
			-- A's vertices inside B
			AVerticesInBoxB(solver, A, B)
		else
			-- Edge-edge contacts
		end
		::endOfLoop::
	end
end
--]]

function ContactGenerators.boxbox(solver, A, B)
	local A_to_B = B.pos - A.pos

	-- Generate testing axes
	local testAxes = {}
	for i = 1, 3 do
		testAxes[i]		= A.oriMat[i]
		testAxes[i+3]	= B.oriMat[i]
	end
	for i = 1, 3 do for j = 1, 3 do
		local cross = testAxes[i]^testAxes[j+3]
		-- Ignore the parallel edges
		testAxes[(3*(i-1)+j)+6] = (cross:lengthSquared() > 0.001) and cross:normalized()
	end end

	local minPointFaceDepth = math.huge
	local minPointFaceIndex
	local minEdgeEdgeDepth = math.huge
	local minEdgeEdgeIndex
	for i = 1, 15 do
		if not testAxes[i] then goto endOfLoop end -- Skip parallel edges

		-- Return immediately if SAT test does separate the boxes
		local penetration = testSAT(A, B, A_to_B, testAxes[i])
		if not penetration then return end

		if i <= 6 then
			-- Find shallowest of point-face penetrations
			if penetration < minPointFaceDepth then
				minPointFaceDepth = penetration
				minPointFaceIndex = i
			end
		else
			-- Find shallowest of edge-edge penetrations
			if penetration < minEdgeEdgeDepth then
				minEdgeEdgeDepth = penetration
				minEdgeEdgeIndex = i
			end
		end
		::endOfLoop::
	end

	-- Get the shallower of the two and add the contact point to the solver
	local friction = A.friction*B.friction
	local restitution = A.restitution*B.restitution

	if minPointFaceIndex and minPointFaceDepth < minEdgeEdgeDepth then
		-- Vertex-Face contact
		local testAxis = testAxes[minPointFaceIndex]

		-- Flip so the A's face normal points into A
		if testAxis .. A_to_B > 0 then testAxis = -testAxis end

		if minPointFaceIndex <= 3 then
			solver:addContactData(
				boxA_VS_pointB_contact(
					A, B, A_to_B,
					testAxis, minPointFaceDepth,
					friction, restitution
				)
			)
		else
			solver:addContactData(
				boxA_VS_pointB_contact(
					B, A, -A_to_B,
					testAxis, minPointFaceDepth,
					friction, restitution
				)
			)
		end
	elseif minEdgeEdgeIndex and minEdgeEdgeDepth < minPointFaceDepth then
		-- Edge-Edge contact
		local testAxis = testAxes[minEdgeEdgeIndex]

		-- Flip so the A's face normal points into A
		if testAxis .. A_to_B > 0 then testAxis = -testAxis end

		minEdgeEdgeIndex = minEdgeEdgeIndex - 7
		local axisIndexA, axisIndexB = math.floor(minEdgeEdgeIndex/3)+1, (minEdgeEdgeIndex%3)+1
		local halfSizeA = vec(A.halfSizeX, A.halfSizeY, A.halfSizeZ)
		local halfSizeB = vec(B.halfSizeX, B.halfSizeY, B.halfSizeZ)
		
		-- Get correct edge midpoints
		local edgeMidPointA = halfSizeA:copy()
		local edgeMidPointB = halfSizeB:copy()
		for i = 1, 3 do
			if i == axisIndexA then
				edgeMidPointA[i] = 0
			elseif A.oriMat[i] .. testAxis > 0 then
				edgeMidPointA[i] = -edgeMidPointA[i]
			end

			if i == axisIndexB then
				edgeMidPointB[i] = 0
			elseif B.oriMat[i] .. testAxis < 0 then
				edgeMidPointB[i] = -edgeMidPointB[i]
			end
		end

		--for _ = 1, 20 do
		--	point(B.oriMat[1]*math.random() + B.pos)
		--	point(B.oriMat[2]*math.random() + B.pos)
		--	point(B.oriMat[3]*math.random() + B.pos)
		--
		--	point(B.pos + testAxis*math.random())
		--end

		edgeMidPointA = (A.oriMat * edgeMidPointA) + A.pos
		edgeMidPointB = (B.oriMat * edgeMidPointB) + B.pos

		solver:addContactData{
			A = A,
			B = B,

			contactPoint = edge_VS_edge_contact(
				edgeMidPointA, A.oriMat[axisIndexA],
				edgeMidPointB, B.oriMat[axisIndexB],
				halfSizeA[axisIndexA], halfSizeB[axisIndexB],
				minPointFaceIndex <= 3
			),

			contactNormalA = testAxis,

			penetration = minEdgeEdgeDepth,

			friction = friction,
			restitution = restitution
		}
	end

end
