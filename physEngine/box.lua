---@type any, ModelPart
local simWorld, simWorldPart = require("physEngine/simWorld")
local RigidBody = require("physEngine/rigidBody")
local quatMath = require("physEngine/quaternions")

--[=============================================================================]--

local Box = RigidBody:newSubclass()

do
	local cube_id = 0
	local super_new = Box.new
	function Box:new(blockState, sizeX, sizeY, sizeZ, mass)
		local x2, y2, z2 = sizeX*sizeX, sizeY*sizeY, sizeZ*sizeZ
		local m_12 = mass/12
		
		local o = super_new(self,
			{
				halfSizeX = sizeX/2,
				halfSizeY = sizeY/2,
				halfSizeZ = sizeZ/2,
			
				inverseMass = 1/mass,
				inverseInertiaTensor = matrices.mat3(
					vec(m_12*(y2+z2),	0,				0			),
					vec(0,				m_12*(x2+z2),	0			),
					vec(0,				0,				m_12*(y2+x2))
				):inverted(),
				renderTask = simWorldPart:newBlock("physCube_"..cube_id):block(blockState)
			}
		)
		setmetatable(o, self)
		self.__index = self

		cube_id = cube_id+1
		table.insert(simWorld, o)

		return o
	end
end

local lerp = math.lerp
local slerp = quatMath.slerp
local quatToRotMat = quatMath.quatToRotMat
function Box:render(delta)
	local pos_l, ori_l = lerp(self.pos_, self.pos, delta), slerp(self.ori_, self.ori, delta)
	self.renderTask:setMatrix(
		matrices.scale4(16)*
		matrices.translate4(pos_l)*
		quatToRotMat(ori_l):augmented()*
		matrices.scale4(self.halfSizeX*2, self.halfSizeY*2, self.halfSizeZ*2)*
		matrices.translate4(-0.5,-0.5,-0.5)*
		matrices.scale4(1/16)
	)
end

function Box:collideWithHalfSpace()
end

--[[ 
local newCube
do
	Cube_Geometry = {
		vertices = {
			vec( 1, 1, 1)/2,
			vec( 1, 1,-1)/2,
			vec( 1,-1, 1)/2,
			vec( 1,-1,-1)/2,
			vec(-1, 1, 1)/2,
			vec(-1, 1,-1)/2,
			vec(-1,-1, 1)/2,
			vec(-1,-1,-1)/2,
		},
		axes = Basis,
		inertiaTensor = matrices.mat3(
			vec(1/6,0,0),
			vec(0,1/6,0),
			vec(0,0,1/6)
		),
		faces = {
			{vec( 1, 0, 0), vec(-1, 0, 0)},
			{vec( 0, 1, 0), vec( 0,-1, 0)},
			{vec( 0, 0, 1), vec( 0, 0,-1)},
		},
		-- Endpoints of edges plus edge normal
		edges = {
			{
				{vec( 1, 1, 1)/2, vec(-1, 1, 1)/2, vec( 0, 1, 1):normalized()},
				{vec( 1, 1,-1)/2, vec(-1, 1,-1)/2, vec( 0, 1,-1):normalized()},
				{vec( 1,-1, 1)/2, vec(-1,-1, 1)/2, vec( 0,-1, 1):normalized()},
				{vec( 1,-1,-1)/2, vec(-1,-1,-1)/2, vec( 0,-1,-1):normalized()},
			},
			{
				{vec( 1, 1, 1)/2, vec( 1,-1, 1)/2, vec( 1, 0, 1):normalized()},
				{vec( 1, 1,-1)/2, vec( 1,-1,-1)/2, vec( 1, 0,-1):normalized()},
				{vec(-1, 1, 1)/2, vec(-1,-1, 1)/2, vec(-1, 0, 1):normalized()},
				{vec(-1, 1,-1)/2, vec(-1,-1,-1)/2, vec(-1, 0,-1):normalized()},
			},
			{
				{vec( 1, 1, 1)/2, vec( 1, 1,-1)/2, vec( 1, 1, 0):normalized()},
				{vec( 1,-1, 1)/2, vec( 1,-1,-1)/2, vec( 1,-1, 0):normalized()},
				{vec(-1, 1, 1)/2, vec(-1, 1,-1)/2, vec(-1, 1, 0):normalized()},
				{vec(-1,-1, 1)/2, vec(-1,-1,-1)/2, vec(-1,-1, 0):normalized()},
			},
		}
	}

	newCube = function(blockState, mass, init_pos, init_rot, init_vel, init_wel)

		local Cube = {
			init_pos = init_pos:copy(),
			init_rot = init_rot:copy() or quat(1,0,0,0),
			init_vel = init_vel:copy() or vec(0,0,0),
			init_wel = init_wel:copy() or vec(0,0,0),

			type = "cube", mass = mass,
			bounciness = 0.95,
			axes = Cube_Geometry.axes,
			inertiaTensor = Cube_Geometry.inertiaTensor*mass,
			vertices = Cube_Geometry.vertices,

			render_pos = init_pos, render_rot = init_rot or quat(1,0,0,0),

			pos_ = init_pos, pos = init_pos,
			rot_ = init_rot or quat(1,0,0,0), rot = init_rot or quat(1,0,0,0),

			momentumL = init_vel or vec(0,0,0),
			momentumA = init_wel or vec(0,0,0),

			nudgeQueue = {},

			task = PhysWorld:newBlock("Cube"..#Rigid_Bodies):block(blockState):pos(init_pos)
		}
		table.insert(Rigid_Bodies, Cube)

		return Cube

	end
end
--]]

--[======================================================================[--
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

local function nearestPointToTwoEdges(edgeMidPointA, edgeDirA, edgeMidPointB, edgeDirB, halfSizeA, halfSizeB, selectA)
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
	-- We can just choose the midpoints
	if false and (lengthAT > halfSizeA or lengthAT < -halfSizeA or
		lengthBT > halfSizeB or lengthBT < -halfSizeB)
	then
		return selectA and edgeMidPointA or edgeMidPointB

	else
		local T_on_A = edgeMidPointA + edgeDirA * lengthAT
		local T_on_B = edgeMidPointB + edgeDirB * lengthBT

		return selectA and T_on_A or T_on_B--(T_on_A + T_on_B) * 0.5
	end
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

return Box
