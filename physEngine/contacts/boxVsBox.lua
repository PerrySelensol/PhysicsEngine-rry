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

local function edge_VS_edge_contact(
	A, B,
	edgeMidPointA, edgeDirA,
	edgeMidPointB, edgeDirB,
	halfLengthA, halfLengthB, reuse
)
	-- Assume both edgeDirA and edgeDirB are already normalized
	local cosTheta = edgeDirB .. edgeDirA

	local vecBA = edgeMidPointA - edgeMidPointB
	local lengthAB_times_cosAlpha = -(edgeDirA .. vecBA)
	local lengthAB_times_cosBeta = edgeDirB .. vecBA

	local sinThetaSquared = 1 - cosTheta*cosTheta

	-- Ignore parallel line case
	if sinThetaSquared < 0.0001 and sinThetaSquared > -0.0001  then
		return
	end

	local lengthAT = (lengthAB_times_cosBeta*cosTheta + lengthAB_times_cosAlpha) / sinThetaSquared
	local lengthBT = (lengthAB_times_cosAlpha*cosTheta + lengthAB_times_cosBeta) / sinThetaSquared

	-- The solution points lying outside an edge means the edge segments don't intersect
	if (lengthAT > halfLengthA or lengthAT < -halfLengthA or
		lengthBT > halfLengthB or lengthBT < -halfLengthB)
	then
		return
	end

	local T_on_A = edgeMidPointA + edgeDirA * lengthAT
	local T_on_B = edgeMidPointB + edgeDirB * lengthBT

	--if reuse then
	--	for i = 1, 100 do
	--		point(edgeMidPointA + (2*math.random()-1)*halfSizeA*edgeDirA, vec(0.2,0.2,0.2))
	--		point(edgeMidPointB + (2*math.random()-1)*halfSizeB*edgeDirB, vec(0,0,0))
	--	end
	--	point(edgeMidPointA, vec(1,0,1))
	--	point(edgeMidPointB, vec(1,0,1))
	--else
	--	--for i = 1, 100 do
	--	--	point(edgeMidPointA + (2*math.random()-1)*halfSizeA*edgeDirA, vec(0,0.3,0))
	--	--	point(edgeMidPointB + (2*math.random()-1)*halfSizeB*edgeDirB, vec(0,0.5,0))
	--	--end
	--end

	local vecTbTa = T_on_A - T_on_B

	return
		A.inverseOriMat*(T_on_A - A.pos),
		B.inverseOriMat*(T_on_B - B.pos),
		-vecTbTa:length()*math.sign(vecTbTa..vecBA)
end

local function vertex_VS_box_contact(A, B, vertA)
	local vertAInBoxB = B.inverseOriMat*((A.oriMat*vertA + A.pos) - B.pos)

	local clampedVertInB = vertAInBoxB:copy()
	for i = 1, 3 do
		clampedVertInB[i] = math.clamp(clampedVertInB[i], -B.halfSizes[i], B.halfSizes[i])
	end
	local separation = (vertAInBoxB-clampedVertInB):length()
	if separation > 0 then
		return vertA, clampedVertInB, -separation
	end

	local penetration, axisIndex = math.huge, nil
	for i = 1, 3 do
		local thisAxisPenetration = B.halfSizes[i] - math.abs(vertAInBoxB[i])
		if thisAxisPenetration < penetration then
			penetration = thisAxisPenetration
			axisIndex = i
		end
	end

	if vertAInBoxB[axisIndex] > 0 then
		vertAInBoxB[axisIndex] = B.halfSizes[axisIndex]
	else
		vertAInBoxB[axisIndex] = -B.halfSizes[axisIndex]
	end

	return vertA, vertAInBoxB, penetration
end

local function getCornerID(A, faceNormalB, A_to_B)
	-- Flip so the B's face normal points away from B
	if faceNormalB .. A_to_B < 0 then faceNormalB = -faceNormalB end

	-- Find the colliding corner
	local cornerA = A.halfSizes:copy()
	local cornerID = {"+","+","+"}
	if (A.oriMat[1] .. faceNormalB) < 0 then cornerA.x = -cornerA.x; cornerID[1] = "-" end
	if (A.oriMat[2] .. faceNormalB) < 0 then cornerA.y = -cornerA.y; cornerID[2] = "-" end
	if (A.oriMat[3] .. faceNormalB) < 0 then cornerA.z = -cornerA.z; cornerID[3] = "-" end

	return table.concat(cornerID)
end

-- Project box onto an axis; by convexity of it,
-- an endpoint of the projected box is always one of its vertex.
-- Use box.oriMat[i] as those are already box's axes direction in world space
local function halfBoxLengthAlongAxis(box, axis)
	return
		box.halfSizes[1] * abs(axis .. box.oriMat[1]) +
		box.halfSizes[2] * abs(axis .. box.oriMat[2]) +
		box.halfSizes[3] * abs(axis .. box.oriMat[3])
end

local function testSAT(A, B, A_to_B, axis)
	local projA = halfBoxLengthAlongAxis(A, axis)
	local projB = halfBoxLengthAlongAxis(B, axis)
	local projCenters = math.abs(A_to_B .. axis)

	local penetration = projA + projB - projCenters
	if penetration < 0 then return end
	return penetration
end

local BOX_POINT = {}
local EDGE_DIR = {}
local BOX_AXIS = {}
do
	local BOX_DIR = {[-1] = "-", [0] = "0", [1] = "+"}

	for i = -1, 1 do for j = -1, 1 do for k = -1, 1 do
		local id = BOX_DIR[i]..BOX_DIR[j]..BOX_DIR[k]
		BOX_POINT[id] = vec(i,j,k)
	end end end

	for i = -1, 1, 2 do for j = -1, 1, 2 do
		EDGE_DIR["0"..BOX_DIR[i]..BOX_DIR[j]] = vec(1, 0, 0)
		EDGE_DIR[BOX_DIR[i].."0"..BOX_DIR[j]] = vec(0, 1, 0)
		EDGE_DIR[BOX_DIR[i]..BOX_DIR[j].."0"] = vec(0, 0, 1)

		BOX_AXIS["0"..BOX_DIR[i]..BOX_DIR[j]] = 1
		BOX_AXIS[BOX_DIR[i].."0"..BOX_DIR[j]] = 2
		BOX_AXIS[BOX_DIR[i]..BOX_DIR[j].."0"] = 3
	end end
end

local contactPairCaches = {
	--[====[
	["1~2"] = {
		["vertA+++"] = {...},
		["edges++0--0"] = {...}
	}
	--]====]
}
function events.tick()
	--trint(2, contactPairCaches)
	for _, cache in next, contactPairCaches do
		for id, c in next, cache do
			local pA = c.A.oriMat*c.contactPointA + c.A.pos
			local pB = c.B.oriMat*c.contactPointB + c.B.pos
			if id:sub(1,4) == "vert" then
				for t=0, 1, 0.125 do point(math.lerp(pA, pB, t), vec(0,1,0)) end
			else
				for t=0, 1, 0.125 do point(math.lerp(pA, pB, t), vec(1,0,1)) end
			end
		end
	end
end

local function getContactInfoFromID(A, B, featureID)
	local idType = featureID:sub(1, 5)
	if idType == "vertA" then
		local vertA = BOX_POINT[featureID:sub(6)] * A.halfSizes
		return vertex_VS_box_contact(A, B, vertA)
	elseif idType == "vertB" then
		local vertB = BOX_POINT[featureID:sub(6)] * B.halfSizes
		local contactA, contactB, penetration = vertex_VS_box_contact(B, A, vertB)
		return contactB, contactA, penetration
	else
		local edgeA_ID, edgeB_ID = featureID:sub(6, 8), featureID:sub(9, 11)
		edgeA, edgeB = BOX_POINT[edgeA_ID] * A.halfSizes, BOX_POINT[edgeB_ID] * B.halfSizes
		edgeA, edgeB = (A.oriMat * edgeA) + A.pos, (B.oriMat * edgeB) + B.pos
		return edge_VS_edge_contact(
			A, B,
			edgeA, A.oriMat * EDGE_DIR[edgeA_ID],
			edgeB, B.oriMat * EDGE_DIR[edgeB_ID],
			A.halfSizes[BOX_AXIS[edgeA_ID]],
			B.halfSizes[BOX_AXIS[edgeB_ID]],
			true
		)
	end
end

local COHERENCE_LIMIT = -0.01
local function generatePartialContactManifoldFromCache(A, B, partialContactPairCache)
	for id, contact in next, partialContactPairCache do
		local contactPointA, contactPointB, penetration = getContactInfoFromID(A, B, id)

		-- Contact feature that drifts too far is removed
		if (not contactPointA) or (penetration < COHERENCE_LIMIT) then
			partialContactPairCache[id] = nil
		else
			contact.contactPointA = contactPointA
			contact.contactPointB = contactPointB
			contact.penetration = penetration
		end
	end
	return partialContactPairCache
end

local function generateFullContactManifold(partialContactPairCache, A_to_B)
	for _, contact in next, partialContactPairCache do
		local A, B, axisIndexA, axisIndexB
			= contact.A, contact.B, contact.axisIndexA, contact.axisIndexB
		if axisIndexA and axisIndexB then
			local normal = A.oriMat[axisIndexA] ^ B.oriMat[axisIndexB]
			if normal .. A_to_B > 0 then normal = -normal end
			contact.contactNormal = normal:normalized()
		elseif axisIndexA then
			local normal = A.oriMat[axisIndexA]
			if normal .. A_to_B > 0 then normal = -normal end
			contact.contactNormal = normal
		elseif axisIndexB then
			local normal = B.oriMat[axisIndexB]
			if normal .. A_to_B > 0 then normal = -normal end
			contact.contactNormal = normal
		end
	end
	return partialContactPairCache
end

local function performBoxBoxSAT(A, B, A_to_B, testAxes)
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

	return minPointFaceDepth, minPointFaceIndex, minEdgeEdgeDepth, minEdgeEdgeIndex
end

local function newPartialContactCache(
	A, B, A_to_B, testAxes,
	minPointFaceDepth, faceNormalIndex,
	minEdgeEdgeDepth, edgeEdgeIndex
)
	-- Get the shallower of the two and add the contact point to the solver
	local friction = A.friction*B.friction
	local restitution = A.restitution*B.restitution

	if faceNormalIndex and minPointFaceDepth <= minEdgeEdgeDepth then
		-- Vertex-Face contact
		local testAxis = testAxes[faceNormalIndex]

		-- Flip so the A's face normal points into A
		if testAxis .. A_to_B > 0 then testAxis = -testAxis end

		if faceNormalIndex <= 3 then
			-- B's vertex is inside A
			local cornerID = "vertB"..getCornerID(B, testAxis, -A_to_B)
			local contactPointA, contactPointB, penetration = getContactInfoFromID(A, B, cornerID)

			return cornerID, {
				axisIndexA = faceNormalIndex,

				A = A, B = B,

				contactPointA = contactPointA,
				contactPointB = contactPointB,
				penetration = penetration,

				friction = friction,
				restitution = restitution
			}
		else
			-- A's vertex is inside B
			local cornerID = "vertA"..getCornerID(A, testAxis, A_to_B)
			local contactPointA, contactPointB, penetration = getContactInfoFromID(A, B, cornerID)

			return cornerID, {
				axisIndexB = faceNormalIndex-3,

				A = A, B = B,

				contactPointA = contactPointA,
				contactPointB = contactPointB,
				penetration = penetration,

				friction = friction,
				restitution = restitution
			}
		end
	elseif edgeEdgeIndex and minEdgeEdgeDepth <= minPointFaceDepth then
		-- Edge-Edge contact
		local testAxis = testAxes[edgeEdgeIndex]

		-- Flip so the A's face normal points into A
		if testAxis .. A_to_B > 0 then testAxis = -testAxis end

		edgeEdgeIndex = edgeEdgeIndex - 7
		local axisIndexA, axisIndexB = math.floor(edgeEdgeIndex/3)+1, (edgeEdgeIndex%3)+1
		
		-- Get correct edge midpoints
		local edgePairID = {"+","+","+","+","+","+"}
		local edgeMidPointA = A.halfSizes:copy()
		local edgeMidPointB = B.halfSizes:copy()
		for i = 1, 3 do
			if i == axisIndexA then
				edgeMidPointA[i] = 0; edgePairID[i] = "0"
			elseif A.oriMat[i] .. testAxis > 0 then
				edgeMidPointA[i] = -edgeMidPointA[i]; edgePairID[i] = "-"
			end

			if i == axisIndexB then
				edgeMidPointB[i] = 0; edgePairID[i+3] = "0"
			elseif B.oriMat[i] .. testAxis < 0 then
				edgeMidPointB[i] = -edgeMidPointB[i]; edgePairID[i+3] = "-"
			end
		end

		edgeMidPointA = (A.oriMat * edgeMidPointA) + A.pos
		edgeMidPointB = (B.oriMat * edgeMidPointB) + B.pos

		local contactPointA, contactPointB, penetration = edge_VS_edge_contact(
			A, B,
			edgeMidPointA, A.oriMat[axisIndexA],
			edgeMidPointB, B.oriMat[axisIndexB],
			A.halfSizes[axisIndexA], B.halfSizes[axisIndexB]
		)

		if contactPointA then
			return "edges"..table.concat(edgePairID), {
				axisIndexA = axisIndexA,
				axisIndexB = axisIndexB,

				A = A,
				B = B,

				contactPointA = contactPointA,
				contactPointB = contactPointB,
				penetration = penetration,

				friction = friction,
				restitution = restitution
			}
		end
	end
end

function ContactGenerators.boxbox(solver, A, B)
	if A.id > B.id then A, B = B, A end
	local contactPairID = A.id.."~"..B.id
	local contactPairCache = contactPairCaches[contactPairID]
	if not contactPairCache then
		contactPairCache = {}
		contactPairCaches[contactPairID] = contactPairCache
	end
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

	local
		minPointFaceDepth,
		minPointFaceIndex,
		minEdgeEdgeDepth,
		minEdgeEdgeIndex
	= performBoxBoxSAT(A, B, A_to_B, testAxes)
	-- Skip everything if SAT does separate the boxes
	if not minPointFaceDepth then
		--contactPairCaches[contactPairID] = {}
		--print(contactPairID.." skipped")
		return
	end

	-- Otherwise get a single partial contact data from SAT
	local newContactID, newPartialContact = newPartialContactCache(
		A, B, A_to_B, testAxes,
		minPointFaceDepth, minPointFaceIndex,
		minEdgeEdgeDepth, minEdgeEdgeIndex
	)-- print(newContactID, newPartialContact)

	-- Create partial contact manifold from old contact cache, then add the new one from SAT in
	local partialContactPairCache = generatePartialContactManifoldFromCache(A, B, contactPairCache)
	if newContactID then partialContactPairCache[newContactID] = newPartialContact end

	-- Turn the whole thing into full contact manifold data which can be given to solver
	for _, contact in next, generateFullContactManifold(partialContactPairCache, A_to_B) do
		solver:addContactData(contact)
	end
end

function events.tick()
	for _, v in next, contactPairCaches do
		for _, c in next, v do
			--point(c.contactPoint)
		end
	end
end
