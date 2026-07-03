local quatMath = require("physEngine/quaternions")
require("physEngine/vectors")

--[=============================================================================]--

local RigidBody = {
	inverseMass = nil,
	inverseInertiaTensor = nil, -- The tensor is in local coords

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
	self.ori_, self.ori = ori, ori
	return self
end

function RigidBody:setAngularVelocity(x, y, z)
	self.rot = quat(0, x, y, z)
	return self
end

function RigidBody:calculateWorldInertiaTensor()
	local ori = self.ori
	self.inverseInertiaTensor = ori* self.inverseInertiaTensor* ori:transposed()
end

function RigidBody:addForceAtCenter(force)
	self.totalForce = self.totalForce + force
end

function RigidBody:addLocalCoordForce(force, point)
	self.totalForce = self.totalForce + force
	self.totalTorque = self.totalTorque + point^force
end

function RigidBody:integrate(dt)
	self.pos_, self.ori_ = self.pos, self.ori

	self.pos = self.pos + self.vel
	self.ori = self.ori + self.rot:scaled(0.5)*self.ori
end

return RigidBody