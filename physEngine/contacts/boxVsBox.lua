local ContactGenerators = require("./contacts")

--[=============================================================================]--

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

local function edgeEgdeContact(edgeMidPointA, edgeDirA, edgeMidPointB, edgeDirB, halfSizeA, halfSizeB, selectA)
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

	return (T_on_A + T_on_B) * 0.5
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
local abs = math.abs
local function halfBoxLengthAlongAxis(box, axis)
	return
		box.halfSizeX * abs(axis .. box.oriMat[1]) +
		box.halfSizeY * abs(axis .. box.oriMat[2]) +
		box.halfSizeZ * abs(axis .. box.oriMat[3])
end

local function tryAxis(contactData, A, B, A_to_B, axis)
	local projA = halfBoxLengthAlongAxis(A, axis)
	local projB = halfBoxLengthAlongAxis(B, axis)
	local projCenters = math.abs(A_to_B .. axis)

	--contactData.penetration = 0
end

function ContactGenerators.boxBoxContacts(A, B)
	local X_A, Y_A, Z_A = A.oriMat[1], A.oriMat[2], A.oriMat[3]
	local X_B, Y_B, Z_B = B.oriMat[1], B.oriMat[2], B.oriMat[3]

	local contactData = {} --minPenetrationAxis, penetration, type
	-- A's vertices inside B: project both onto B's axes

	-- B's vertices inside A: project both onto A's axes

	-- Edge-edge contacts: project both onto cross products of A's edge and B's edge
end

