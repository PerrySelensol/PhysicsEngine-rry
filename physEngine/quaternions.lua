--===========================================--
--             Quaterions API
--===========================================--

-- Currently uses Figura's matrix library. Can be swapped for another.
--local matrices = matrices

local quaterions = {}
quaterions.__index = quaterions

function quat(w, x, y, z)
	return setmetatable({w, x, y, z}, quaterions)
end

quaterions.unpack = table.unpack

do
	local template = "§e<%s, %s, %s, %s>§r"
	function quaterions.__tostring(q)
		return template:format(q[1], q[2], q[3], q[4])
	end
end

function quaterions.__add(p, q)
	return quat(
		p[1] + q[1],
		p[2] + q[2],
		p[3] + q[3],
		p[4] + q[4]
	)
end

function quaterions.__sub(p, q)
	return quat(
		p[1] - q[1],
		p[2] - q[2],
		p[3] - q[3],
		p[4] - q[4]
	)
end

function quaterions.__unm(q)
	return quat(-q[1], -q[2], -q[3], -q[4])
end

--[=============================================================[--
	-- Relation to 3D vectors --
	let _s and _v be the scalar and vector part of a quaterion

		pq = (
			p_s * q_s
			- dot(p_v, q_v)
		)
		+ (
			scale(p_s, q_v)
			+ scale(q_s, p_v)
			+ cross(p_v, q_v)
		)
--]=============================================================]--

function quaterions.__mul(p, q)
	local p1, p2, p3, p4 = p[1], p[2], p[3], p[4]
	local q1, q2, q3, q4 = q[1], q[2], q[3], q[4]
	return quat(
		p1*q1 - p2*q2 - p3*q3 - p4*q4,
		p1*q2 + p2*q1 + p3*q4 - p4*q3,
		p1*q3 - p2*q4 + p3*q1 + p4*q2,
		p1*q4 + p2*q3 - p3*q2 + p4*q1
	)
end

local function scaled(self, x)
	return quat(self[1]*x, self[2]*x, self[3]*x, self[4]*x)
end
quaterions.scaled = scaled

local function normSquared(self)
	return self[1]^2 + self[2]^2 + self[3]^2 + self[4]^2
end
quaterions.normSquared = normSquared

local function norm(self)
	return (self[1]^2 + self[2]^2 + self[3]^2 + self[4]^2)^0.5
end
quaterions.norm = norm

local function normalized(self)
	return scaled(self, 1/norm(self))
end
quaterions.normalized = normalized

local function inverted(self)
	local k = normSquared(self)
	return quat(self[1]/k, -self[2]/k, -self[3]/k, -self[4]/k)
end
quaterions.inverted = inverted
quaterions.__len = inverted

local function copy(self)
	return quat(self:unpack())
end
quaterions.copy = copy

-- === Extra Matrix Functions === --

function trace3(m)
	return m[1][1] + m[2][2] + m[3][3]
end

function almostEq3(m1, m2, threshold)
	local t = (threshold or 0.001)^2
	local eq = true
	for i = 1, 3 do
		for j = 1, 3 do
			if (m1[i][j]-m2[i][j])^2 > t then eq = false break end
		end
	end
	return eq
end

-- === Other Quaternion Functions === --

local quatMath = {}

local sin, cos, acos = math.sin, math.cos, math.acos

local function unitQuatPower(q, t)
	-- Unit quaternion in versor form: cos(theta) + v*sin(theta) where v is unit vector
	-- Thus q^t = cos(theta*t) + v*sin(theta*t)
	local theta = acos(q[1])

	local q2, q3, q4 = q[2], q[3], q[4]
	local X = sin(theta*t)/((q2*q2 + q3*q3 + q4*q4)^0.5)
	return normalized(quat(cos(theta*t), q2*X, q3*X, q4*X))
end
quatMath.unitQuatPower = unitQuatPower

local function rotMatToQuat(m)
	local m11, m22, m33 = m[1][1], m[2][2], m[3][3]
	local t = m11 + m22 + m33
	local r, s, w, x, y, z = 0, 0, 0, 0, 0, 0

	-- Positive trace is numerically stable
	if t > 0 then
		r = (1 + t)^0.5
		s = 2*r
		w = r/2
		x = (m[2][3] - m[3][2]) / s
		y = (m[3][1] - m[1][3]) / s
		z = (m[1][2] - m[2][1]) / s
	-- Otherwise, pick the largest diagonal entry to be subtracted by the rest
	elseif (m11 > m22) and (m11 > m33) then
		r = (1 + m11 - m22 - m33)^0.5
		s = 2*r
		w = (m[2][3] - m[3][2]) / s
		x = r/2
		y = (m[1][2] + m[2][1]) / s
		z = (m[3][1] + m[1][3]) / s
	elseif m22 > m33 then
		r = (1 + m22 - m11 - m33)^0.5
		s = 2*r
		w = (m[3][1] - m[1][3]) / s
		x = (m[1][2] + m[2][1]) / s
		y = r/2
		z = (m[2][3] + m[3][2]) / s
	else
		r = (1 + m33 - m11 - m22)^0.5
		s = 2*r
		w = (m[1][2] - m[2][1]) / s
		x = (m[3][1] + m[1][3]) / s
		y = (m[2][3] + m[3][2]) / s
		z = r/2
	end

	return normalized(quat(w, x, y, z))
end
quatMath.rotMatToQuat = rotMatToQuat

local function quatToRotMat(q)
	local w, x, y, z = q[1], q[2], q[3], q[4]
	return matrices.mat3(
		vec( 1 -2*y*y -2*z*z,	   2*x*y +2*z*w,	   2*x*z -2*y*w ),
		vec(    2*x*y -2*z*w,	1 -2*x*x -2*z*z,	   2*y*z +2*x*w ),
		vec(    2*x*z +2*y*w,	   2*y*z -2*x*w,	1 -2*x*x -2*y*y )
	)
end
quatMath.quatToRotMat = quatToRotMat

local function slerp(q1, q2, t)
	local dot = q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3] + q1[4]*q2[4]
	-- To ensure the slerp goes the short path, negate one end if the dot product is negative
	if dot < 0 then q2 = -q2; dot = -dot end
	-- A case with very close ends is basically cheated with regular lerp (and normalization)
	if dot > 0.995 then
		local r = q1:scaled(1-t) + q2:scaled(t)
		return normalized(r)
	end
	return q1*unitQuatPower(#q1*q2, t)
end
quatMath.slerp = slerp

local function lerpRotMat(mat1, mat2, t)
	return quatToRotMat(slerp(rotMatToQuat(mat1), rotMatToQuat(mat2), t))
end
quatMath.lerpRotMat = lerpRotMat

local function slerpPoint(p1, p2, t) -- Non-quaternion slerp
	local q1, q2 = p1:normalized(), p2:normalized()
	-- Angle subtended by the two vectors both start at origin
	local O = acos(p1:normalized():dot(p2:normalized()))
	-- The lerped vector has length of p1
	return (sin((1-t)*O)*q1 + sin(t*O)*q2):scale(p1:length())/sin(O)
end
quatMath.slerpPoint = slerpPoint

-- === Exponentially Eased Sudden Change === --
--[[
do
	local tickPresents, tickPasts = {}, {}
	local speedMemory = {}

	function rotEased(register, x, Duration, delta)

		local speed = speedMemory[register]

		if not speed then
			speedMemory[register] = 1000^(-0.05/(Duration or 0.5))
		end

		local x_ = tickPasts[register]

		-- Integrity check: sometimes the entires ends up as NaN
		if x_ == nil or x_[1][1] ~= x_[1][1] then tickPasts[register] = matrices.mat3(); x_ = tickPasts[register] end

		-- Put the easing to sleep so it doesn't call rotLerp all the time
		if almostEq3(x_, x) then return x end

		local z = rotLerp(x, x_, speedMemory[register])
		tickPresents[register] = z

		return rotLerp(x_, z, delta or 0)

	end

	function forceFinishEase(register, x)
		tickPasts[register], tickPresents[register] = x:copy(), x:copy()
	end

	function finishTick()
		for k, v in pairs(tickPresents) do
			tickPasts[k] = v; tickPresents[k] = nil
		end
	end

end
--]]

return quatMath