--===========================================--
-- Rigid Body Physics (Old Version)
--===========================================--

local quatMath = require("physEngine/libs/quaternions")
local quatToRotMat = quatMath.quatToRotMat

	local sin, cos, tan, atan2, pi, deg, rad	= math.sin, math.cos, math.tan, math.atan2, math.pi, math.deg, math.rad
	local min, max, abs, random				= math.min, math.max, math.abs, math.random
	local clamp, lerp, lerpAngle				= math.clamp, math.lerp, math.lerpAngle
	local sign								= math.sign
	local vRotAxis							= vectors.rotateAroundAxis
	local rgbToHex							= vectors.rgbToHex
	local floor, ceil							= math.floor, math.ceil
	local toRad, toDeg						= pi/180, 180/pi

local PhysWorld = models:newPart("PhysWorld", "WORLD")
local OFFSET = vec(0,0,0)--vec(-547, 74+2/16, 386)

local Basis = {
	vec(1,0,0),
	vec(0,1,0),
	vec(0,0,1),
}

local Rigid_Bodies = {}


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
			bounciness = 0,
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

function reset()
	for _, v in pairs(Rigid_Bodies) do
		v.pos, v.pos_ = v.init_pos:copy(), v.init_pos:copy()
		v.rot, v.rot_ = v.init_rot:copy(), v.init_rot:copy()
		v.momentumL = v.init_vel:copy()
		v.momentumA = v.init_wel:copy()
		v.nudgeQueue = {}
	end
end

pings.reset = reset

local isInsideBox = function (point)
	for i = 1, 3 do
		if -0.5 > point[i] or 0.5 < point[i] then
			return false
		end
	end
	return true
end

local function getPointsInsideBox(points)
	local t = {}
	for i=1, #points do
		if isInsideBox(points[i]) then table.insert(t, points[i]) end
	end
	return t
end

local function getMaxFacing(Vec_List, ref)
	local copy = {}; for i=1, #Vec_List do copy[i] = Vec_List[i]:copy() end
	table.sort(copy, function(v,w) return w:dot(ref) < v:dot(ref) end)
	return copy[1]
end

local function getMotion(body)
	local rotMat = quatToRotMat(body.rot)
	local inertia = rotMat* body.inertiaTensor* rotMat:transposed()
	return body.momentumL/body.mass, inertia:inverted()*body.momentumA
end

local function storeLastTick(body)
	body.render_pos = body.pos
	body.render_rot = body.rot
end

local function step(body, t)
	local vel, wel_ = getMotion(body)
	local wel = quat(0, wel_.x, wel_.y, wel_.z)

	body.pos_, body.rot_ = body.pos, body.rot

	body.pos = body.pos + vel*t
	body.rot = (body.rot + wel:scaled(0.5*t)*body.rot):normalized()
end

local function recalculateMotion(Steps, body)

	body.momentumL = 20*Steps*(body.pos - body.pos_)*body.mass
	local dq = body.rot*body.rot_:inverted()

	local wel = 2*vec(dq[2], dq[3], dq[4])*20*Steps

	local rotMat = quatToRotMat(body.rot)
	local inertia = rotMat* body.inertiaTensor* rotMat:transposed()

	body.momentumA = inertia*wel

	if body.momentumL:lengthSquared() < 0.005 then body.momentumL:set(0,0,0) end
	if body.momentumA:lengthSquared() < 0.005 then body.momentumA:set(0,0,0) end

end

local function getTransformationMatrix(body)
	local M = matrices.translate4(body.pos)*quatToRotMat(body.rot):augmented()
	return M, M:inverted()
end

function dampVelocities(body, scale)
	if not body.wasNudged then return end
	body.momentumL = body.momentumL*scale
	body.momentumA = body.momentumA*0.99
	body.wasNudged = nil
end

function applyForce(body, pos, dir, scale)
	local T, T_ = getTransformationMatrix(body)
	local loc_pos, loc_dir = T_:apply(pos), T_:applyDir(dir)
	body.momentumL = body.momentumL + dir*scale*body.mass
	--body.momentumA = body.momentumA + T:applyDir(loc_pos:crossed(loc_dir))*scale
end


local function applyForceLoc(body, pos, dir, scale)
	local T, T_ = getTransformationMatrix(body)
	--body.momentumL = body.momentumL + T:applyDir(dir)*scale/body.mass
	body.momentumA = body.momentumA + T:applyDir(pos:crossed(dir))*scale*body.mass
end

local function worldNudge(body, vel, wel, t)
	body.pos = body.pos + vel*t

	local rotMat = quatToRotMat(body.rot)
	local inertia = rotMat* body.inertiaTensor* rotMat:transposed()
	
	local dq_ = 0.5*(inertia:inverted()*wel*t)
	local dq = quat(0,dq_.x,dq_.y,dq_.z)
	body.rot = (body.rot + dq*body.rot):normalized()
end

function pings.push(pos, dir)
	applyForce(C1, pos, dir, 5)
end

local function renderBody(body, delta)
	--drint(body.rot_, body.rot, delta)
	local pos_l, rot_l = lerp(body.render_pos, body.pos, delta), quatMath.slerp(body.render_rot, body.rot, delta)
	body.task:setMatrix(
		matrices.scale4(16)*
		matrices.translate4(OFFSET)*
		matrices.translate4(pos_l)*
		quatToRotMat(rot_l):augmented()*
		matrices.translate4(-0.5,-0.5,-0.5)*
		matrices.scale4(1/16)
	)
end

C1 = newCube("light_gray_stained_glass", 1,
	vec(3,2.54,0),
	quat(1,0,0,0):normalized(),
	vec(0,0,0),
	vec(0,0,0)
)
C2 = newCube("glass", 1,
	vec(0,0.5,0),
	quat(1,0,0.5,0):normalized(),
	vec(0,0,0),
	vec(0,0,0)
)

C3 = newCube("diamond_block", 1,
	vec(0,1.6,0),
	quat(1,0,0.001,0):normalized(),
	vec(0,0,0),
	vec(0,0,0)
)

C4 = newCube("gold_block", 1,
	vec(0,2.6,0),
	quat(1,0,0.21,0):normalized(),
	vec(0,0,0),
	vec(0,0,0)
)

local pickup = false
local LClick = keybinds:fromVanilla("key.attack")
function pings.setPickUp(b) pickup = b end
LClick.press = function()
	pickup = pings.setPickUp(not pickup)
	--applyForce(C1, player:getPos()+vec(0,player:getEyeHeight(),0), player:getLookDir(), 0)
	if not pickup then
		--pings.push(player:getPos()+vec(0,player:getEyeHeight(),0), player:getLookDir())
	end
end

local function mapPoints(points, matrix)
	local points_ = {}
	for i = 1, #points do points_[i] = (matrix*points[i]:augmented()).xyz end
	return points_
end

local function projectToAxis(axis, vertices)
	local S = {}
	for i = 1, #vertices do
		S[i] = axis:dot(vertices[i])
	end
	return min(table.unpack(S)), max(table.unpack(S))
end

local Cube_Contact_Types = {
	{1,0},{2,0},{3,0}, --Face 1 (xyz)
	{0,1},{0,2},{0,3}, --Face 2 (xyz)
	{1,1},{2,1},{3,1}, --Edges (xyz)~x
	{1,2},{2,2},{3,2}, --Edges (xyz)~y
	{1,3},{2,3},{3,3}, --Edges (xyz)~z
}

local function setPos(body, pos)
	body.pos_, body.pos = pos, pos
end

local CUBE = {{vec(1,1,1)/2, vec(-1,-1,-1)/2}}
local function raycastToCenteredCube(start, end_)
	return raycast:aabb(start, end_, CUBE)
end

local function cubecubeContact(body1, body2)

	local M1, M1_ = getTransformationMatrix(body1)
	local M2, M2_ = getTransformationMatrix(body2)

	local D = M1_*M2; local D_ = (M1_*M2):inverted()
	local C, C_ = D:deaugmented(), D_:deaugmented()

	local Test_Axes = {}
	local Overlaps = {}

	-- === Candidates for Separation Planes (Applies to any convex meshes) ===
	--
	-- 1. Planes parallel to one of the faces of each mesh
	--    (normal is thus just a normal of that face)
	-- 2. Planes parallel to a pair of edges, one from each mesh
	--    (normal is thus a cross product of 2 edges)
	--
	
	for i = 1, 3 do
		Test_Axes[i]	= Basis[i]
		Test_Axes[i+3]	= C[i]
	end

	for i = 1, 3 do for j = 1, 3 do
		local cross = Test_Axes[i]:crossed(Test_Axes[j+3])
		Test_Axes[(i+3*(j-1))+6] = cross:lengthSquared() > 0.001 and cross:normalized() or nil
	end end

	local body2_verts_rel = mapPoints(body2.vertices, D)

	local Penetrations = {}

	for i = 1, 15 do

		local axis = Test_Axes[i]

		if axis then
			local v1_min, v1_max = projectToAxis(axis, body1.vertices)
			local v2_min, v2_max = projectToAxis(axis, body2_verts_rel)
			local penetration_i = min(v1_max-v2_min, v2_max-v1_min)

			if penetration_i < 0 then return end

			if (Penetrations[1] == nil) or (abs(penetration_i - Penetrations[1][1]) < 0.02) then
				table.insert(Penetrations, {penetration_i, Cube_Contact_Types[i], axis})
			elseif penetration_i < Penetrations[1][1] then
				Penetrations = {{penetration_i, Cube_Contact_Types[i], axis}}
			end
		end

	end

	local Verts_In_1 = getPointsInsideBox(body2_verts_rel)
	local Verts_In_2 = getPointsInsideBox(mapPoints(body1.vertices, D_))
	local N = #Penetrations

	for i = 1, N do

		local axis1, axis2 = Penetrations[i][2][1], Penetrations[i][2][2]

		if axis2 == 0 then --Face1
			local dir = getMaxFacing(Cube_Geometry.faces[axis1], M1_:apply(body2.pos))
			for j = 1, #Verts_In_1 do
				table.insert(body2.nudgeQueue, {D_:apply(Verts_In_1[j]), C_*dir, Penetrations[i][1]/2})
				table.insert(body1.nudgeQueue, {Verts_In_1[j], -dir, Penetrations[i][1]/2})
			end
		elseif axis1 == 0 then --Face2
			local dir = getMaxFacing(Cube_Geometry.faces[axis2], M2_:apply(body1.pos))
			for j = 1, #Verts_In_2 do
				table.insert(body1.nudgeQueue, {D:apply(Verts_In_2[j]), C*dir, Penetrations[i][1]/2})
				table.insert(body2.nudgeQueue, {Verts_In_2[j], -dir, Penetrations[i][1]/2})
			end
		elseif axis1 ~= nil and axis2 ~= nil then --Edges
			local edges1 = Cube_Geometry.edges[axis1]
			local edges2 = Cube_Geometry.edges[axis2]
			--trint(1, {M1_:apply(body2.pos), M2_:apply(body1.pos)}, edge1, edge2)
			--error()

			for j = 1, 4 do
				local _, hit1a = raycastToCenteredCube(D :apply(edges2[j][1]), D :apply(edges2[j][2]))
				local _, hit2a = raycastToCenteredCube(D_:apply(edges1[j][1]), D_:apply(edges1[j][2]))
				local _, hit1b = raycastToCenteredCube(D :apply(edges2[j][2]), D :apply(edges2[j][1]))
				local _, hit2b = raycastToCenteredCube(D_:apply(edges1[j][2]), D_:apply(edges1[j][1]))

				local normalEdge1 = getMaxFacing({Penetrations[i][3], -Penetrations[i][3]}, M1_:apply(body2.pos))

				--point(M1:apply(hit1)) point(M2:apply(hit2))

				if hit1a then table.insert(body1.nudgeQueue, {hit1a, -normalEdge1, Penetrations[i][1]/2}) end
				if hit1b then table.insert(body1.nudgeQueue, {hit1b, -normalEdge1, Penetrations[i][1]/2}) end
				if hit2a then table.insert(body2.nudgeQueue, {hit2a, C_*normalEdge1, Penetrations[i][1]/2}) end
				if hit2b then table.insert(body2.nudgeQueue, {hit2b, C_*normalEdge1, Penetrations[i][1]/2}) end
			end
		end

	end

end

local function groundContact(steps, body)
	if body.pos.y > 1 then return end

	local points = body.vertices
	local M, M_ = getTransformationMatrix(body)
	local t = {}

	for i = 1, #points do
		local altitude = (M:apply(points[i])).y
		if altitude < 0 then table.insert(t, {points[i], -altitude}) end
	end
	for i = 1, #t do
		table.insert(body.nudgeQueue, {t[i][1], M_:applyDir(vec(0,1,0)), t[i][2]})
	end

end

--local function cubecubeSolve(steps, body1, body2, CP1, CP2, depth)
--	for i = 1, #CP1 do
--		localNudge(body1, CP1[i][1], CP1[i][2], steps*depth/(20*#CP1))
--	end
--	for i = 1, #CP2 do
--		localNudge(body2, CP2[i][1], CP2[i][2], steps*depth/(20*#CP2))
--	end
--end

local function showAllContacts(body)
	local T, T_ = getTransformationMatrix(body)
	for i=1, #body.nudgeQueue do
		point(T:apply(body.nudgeQueue[i][1]))
	end
end

local function solveAllNudges(body, h)
	local sumVel, sumWel = vec(0,0,0), vec(0,0,0)
	local T, T_ = getTransformationMatrix(body)
	for i = 1, #body.nudgeQueue do
		local nudge = body.nudgeQueue[i]
		sumVel:add(nudge[2]*nudge[3])
		sumWel:add(nudge[1]:crossed(nudge[2])*nudge[3])
	end
	if #body.nudgeQueue == 0 then return end
	worldNudge(body, T:applyDir(sumVel), T:applyDir(sumWel), h/(body.mass*#body.nudgeQueue))
	body.nudgeQueue = {}
	body.wasNudged = true
end

local Steps = 4
local SolveIter = 2

function events.tick()
	--drint(player:getLookDir())
	--drint(getTransformationMatrix(C1))
	for _, v in pairs(Rigid_Bodies) do
		storeLastTick(v)
	end
	--trint(1, Cube_Geometry.vertices)
	for i = 1, Steps do
		for _, body in pairs(Rigid_Bodies) do
			step(body, 0.05/Steps)
		end
		if pickup then
			setPos(C1, player:getPos()+vec(0,player:getEyeHeight(),0)+3*player:getLookDir()-OFFSET)
		end
		for j = 1, SolveIter do
			cubecubeContact(C1, C2)
			cubecubeContact(C1, C3)
			cubecubeContact(C1, C4)
			cubecubeContact(C2, C3)
			cubecubeContact(C2, C4)
			cubecubeContact(C3, C4)
			showAllContacts(C2)
			--trint(2, C2.nudgeQueue)
			for _, body in pairs(Rigid_Bodies) do
				groundContact(Steps, body)
				solveAllNudges(body, 1/SolveIter)
			end
		end
		for _, body in pairs(Rigid_Bodies) do
			recalculateMotion(Steps, body)
			dampVelocities(body, body.bounciness)
			applyForce(body, body.pos, vec(0,-1,0), 0.5/Steps)
		end
	end
end

do
	local function render(delta)
		for _, v in pairs(Rigid_Bodies) do
			renderBody(v, delta)
		end
	end
	if host:isHost() then
		events.world_render:register(render)
	else
		events.render:register(render)
	end
end