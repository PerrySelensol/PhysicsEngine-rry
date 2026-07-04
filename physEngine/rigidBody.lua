local quatMath = require("physEngine/quaternions")
require("physEngine/vectors")

--[=============================================================================]--

local RigidBody = {
	inverseMass = nil,
	inverseInertiaTensor = nil, -- The tensor is in local coords
	---@type Matrix3
	inverseInertiaTensorWorld = nil,

	pos_ = nil,
	ori_ = nil, 

	pos = nil,
	ori = nil, 

	vel = nil,
	rot = nil,

	totalForce = nil,
	totalTorque = nil,
}

function RigidBody:newSubclass(o)
	o = o or {}
	setmetatable(o, self)
	self.__index = self
	return o
end

function RigidBody:new(o)
	o = o or {}

	o.pos_ = vec(0,0,0)
	o.ori_ = quat(1,0,0,0)

	o.pos = vec(0,0,0)
	o.ori = quat(1,0,0,0)

	o.vel = vec(0,0,0)
	o.rot = quat(0,0,0,0)

	o.totalForce = vec(0,0,0)
	o.totalTorque = vec(0,0,0)

	setmetatable(o, self)
	self.__index = self
	return o
end

function RigidBody:setPos(pos)
	self.pos_, self.pos = pos, pos
	return self
end

function RigidBody:setVel(vel)
	self.vel = vel
	return self
end

function RigidBody:setOrientation(ori)
	ori = ori:normalized()
	self.ori_, self.ori = ori, ori
	return self
end

function RigidBody:setAngularVelocity(x, y, z)
	self.rot = quat(0, x, y, z)
	return self
end

function RigidBody:calculateDerivedData()
	local ori = quatMath.quatToRotMat(self.ori)
	self.oriMat = ori
	self.inverseInertiaTensorWorld = ori* self.inverseInertiaTensor* ori:transposed()
end

function RigidBody:addForceAtCenter(force)
	self.totalForce = self.totalForce + force
end

-- This default addForce uses force and point in world space
function RigidBody:addForce(force, point)
	self.totalForce = self.totalForce + force
	self.totalTorque = self.totalTorque + (point-self.pos)^force
end

-- World space direction, local application point
function RigidBody:addForceAtBodyPoint(force, point)
	self:addForce(force, self.oriMat*point + self.pos)
end

function RigidBody:integrate(dt)
	self.pos_, self.ori_ = self.pos, self.ori

	self.pos = self.pos + dt*self.vel
	self.vel = self.vel + dt*self.inverseMass*self.totalForce
	self.totalForce = vec(0,0,0)

	
	local angularMomentum = self.inverseInertiaTensorWorld:inverted()*vec(self.rot:unpack()).yzw
	self.ori = (self.ori + self.rot:scaled(0.5*dt)*self.ori):normalized()
	self:calculateDerivedData()
	
	--self.rot = self.rot + (self.inverseInertiaTensorWorld*(dt*self.totalTorque))._xyz
	self.rot = quat(0, (self.inverseInertiaTensorWorld*angularMomentum):unpack())
	drint(angularMomentum, self.rot)

	self.totalTorque = vec(0,0,0)
end

function RigidBody:recalculateMotion(dt)

	-- pos = pos_ + dt*vel
	self.vel = (self.pos - self.pos_)/dt

	-- ori = ori_ + (dt/2)*(rot*ori_)
	-- Therefore rot = (2/dt)*(ori*#ori_ - quat(1,0,0,0)), but we will ignore the real component anyways
	local dq = self.ori * #self.ori_
	self.rot = dq:scaled(2/dt)
	self.rot[1] = 0

end


return RigidBody