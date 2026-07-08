---@type any, ModelPart
local simWorld, simWorldPart = require("physEngine/simWorld")
local quatMath = require("physEngine/libs/quaternions")
require("physEngine/libs/vectors")

--[=============================================================================]--

local HalfSpace = {
	colliderOnly = true,
	noRender = true
}

function HalfSpace:new(pos, dir)
	local o = {
		pos = pos,
		dir = dir:normalized(),
		restitution = 1,
		friction = 1
	}

	setmetatable(o, self)
	self.__index = self

	table.insert(simWorld, o)
	return o
end

return HalfSpace
