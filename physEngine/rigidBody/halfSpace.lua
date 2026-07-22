require("physEngine/libs/vectors")

--[=============================================================================]--

local HalfSpace = {
	type = "halfSpace",
	colliderOnly = true,
	noRender = true
}

-- Generate an arbitary basis with a given fixed axis
local generateOrthoBasis; do
	local Z1, Z2 = vec(1,0,0), vec(0,1,0)
	local abs, mat3 = math.abs, matrices.mat3
	function generateOrthoBasis(fixedY)
		local genX = fixedY^((abs(fixedY..Z1) < 0.75) and Z1 or Z2)

		genX:normalize()
		local genZ = genX^fixedY

		return mat3(genX, fixedY, genZ)
	end
end

function HalfSpace:new(pos, dir)
	local o = {
		pos = pos,
		oriMat = generateOrthoBasis(dir:normalized()),
		restitution = 1,
		friction = 1
	}
	o.inverseOriMat = o.oriMat:transposed()

	setmetatable(o, self)
	self.__index = self

	return o
end

return HalfSpace
