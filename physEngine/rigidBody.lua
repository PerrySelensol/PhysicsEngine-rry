--[=============================================================================]--

local RigidBody = {}

function RigidBody:new(o)
	o = o or {}
	setmetatable(o, self)
	self.__index = self
	return o
end

RigidBody.newSubclass = RigidBody.new

return RigidBody