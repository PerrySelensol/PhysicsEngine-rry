---@type any, ModelPart
local simWorld, simWorldPart = require("physEngine/simWorld")
local RigidBody = require("physEngine/rigidBody/rigidBody")
local quatMath = require("physEngine/libs/quaternions")

--[=============================================================================]--

local Box = RigidBody:newSubclass{
	type = "box"
}

do
	local boxID = 1
	local super_new = Box.new
	function Box:new(blockState, sizeX, sizeY, sizeZ, mass)
		local x2, y2, z2 = sizeX*sizeX, sizeY*sizeY, sizeZ*sizeZ
		local m_12 = mass/12
		
		local o = super_new(self,
			{
				id = boxID,

				halfSizeX = sizeX/2,
				halfSizeY = sizeY/2,
				halfSizeZ = sizeZ/2,

				halfSizes = vec(sizeX/2, sizeY/2, sizeZ/2),
			
				inverseMass = 1/mass,
				inverseInertiaTensor = matrices.mat3(
					vec(m_12*(y2+z2),	0,				0			),
					vec(0,				m_12*(x2+z2),	0			),
					vec(0,				0,				m_12*(y2+x2))
				):inverted(),

				restitution = 1,
				friction = 0,
				
				renderTask = simWorldPart:newBlock("physBox_"..boxID):block(blockState)
			}
		)
		setmetatable(o, self)
		self.__index = self

		boxID = boxID+1
		table.insert(simWorld, o)
		o:calculateDerivedData()

		return o
	end
end

local lerp = math.lerp
local slerp = quatMath.slerp
local quatToRotMat = quatMath.quatToRotMat
function Box:render(delta)
	local pos_l, ori_l = lerp(self.render_pos, self.pos, delta), slerp(self.render_ori, self.ori, delta)
	self.renderTask:setMatrix(
		matrices.scale4(16)*
		matrices.translate4(pos_l)*
		quatToRotMat(ori_l):augmented()*
		matrices.scale4(self.halfSizeX*2, self.halfSizeY*2, self.halfSizeZ*2)*
		matrices.translate4(-0.5,-0.5,-0.5)*
		matrices.scale4(1/16)
	)
end

return Box
