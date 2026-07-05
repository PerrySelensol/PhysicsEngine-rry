---@type any, ModelPart
local simWorld, simWorldPart = require("physEngine/simWorld")
local quatMath = require("physEngine/quaternions")
require("physEngine/vectors")

--[=============================================================================]--

local HalfSpace = {
	colliderOnly = true,
	noRender = true
}

function HalfSpace:new(pos, dir)
	local o = {
		pos = pos,
		dir = dir:normalized()
	}

	setmetatable(o, self)
	self.__index = self

	table.insert(simWorld, o)
	return o
end

return HalfSpace
