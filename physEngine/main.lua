local quatMath = require("physEngine/libs/quaternions")
local Box = require("physEngine/rigidBody/box")
local HalfSpace = require("physEngine/rigidBody/halfSpace")
local ForceGenerators = require("physEngine/forceGenerators/forceGens")

local CollisionSolver = require("physEngine/collisionSolver")

--[=============================================================================]--

local ground = HalfSpace:new(vec(0,0,0), vec(0,1,0))
--local ceiling = HalfSpace:new(vec(0,10,0), vec(0,-1,0))
--
--local SIZE = 5
--local wall1 = HalfSpace:new(vec(-SIZE,0,0), vec(1,0,0))
--local wall2 = HalfSpace:new(vec(SIZE,0,0), vec(-1,0,0))
--local wall3 = HalfSpace:new(vec(0,0,-SIZE), vec(0,0,1))
--local wall4 = HalfSpace:new(vec(0,0,SIZE), vec(0,0,-1))
--
--local simWorldPart = models.simWorldPart
--renderTask = simWorldPart:newBlock("jail"):block("glass"):scale(SIZE*2, 10, -SIZE*2):pos(-SIZE*16,0,SIZE*16)

local ROTS = {}; do
	local generateOrthoBasis; do
		local Z1, Z2 = vec(0,0,1), vec(0,1,0)
		local abs, mat3 = math.abs, matrices.mat3
		function generateOrthoBasis(fixedY)
			local genX = fixedY^((abs(fixedY..Z1) < 0.75) and Z1 or Z2)

			genX:normalize()
			local genZ = genX^fixedY

			return mat3(genX, fixedY, genZ)
		end
	end

	local SIGN = {[-1] = "-", [0] = "0", [1] = "+"}
	local function sign(i,j,k) return SIGN[i]..SIGN[j]..SIGN[k] end
	for i = -1, 1 do for j = -1, 1 do for k = -1, 1 do
		--local len = math.abs(i)+math.abs(j)+math.abs(k)
		local axis = vec(i,-j,k):normalized()
		ROTS[sign(i,j,k)] = quatMath.rotMatToQuat(generateOrthoBasis(axis):transposed())
	end end end
end

local q0 = quat(1,0,0,0)
--local q1 = quat(0.888073833977, 0.32505758, 0, 0.32505758):normalized()
local q1 = quat(0, 0.624784, 0.455179, 0.624784):normalized()

local q2 = quat(0.9238795325112868,0.3826834323650898,0,0):normalized()
local q3 = quat(0.9238795325112868,0,0,0.3826834323650898):normalized()
local q4 = quat(0.9238795325112868,0,0.3826834323650898,0):normalized()

--[[
	local box1 = Box:new("carved_pumpkin", 1, 1, 1, 10000):setRestitution(1):setFriction(0)
	:setPos(vec(0,1,0)):setOrientation(ROTS["+00"] + quat(0,0,0.1,-0.1))
	:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
	--ForceGenerators.register(box2, ForceGenerators.gravityForceGen(vec(0,-2,0)))

	local box2 = Box:new("dropper", 1, 1, 1, 1):setRestitution(1):setFriction(0)
	:setPos(vec(0.2,3.4,0)):setOrientation(ROTS["---"])
	:setVel(vec(-0,-1,0)):setAngularVelocity(0,0,0)
	--ForceGenerators.register(box1, ForceGenerators.gravityForceGen(vec(0,-2,0))); --print(box1)
--]]

--[[
	local box2 = Box:new("glass", 1, 1, 1, 10000):setRestitution(0.4):setFriction(0.5)
	:setPos(vec(0.4,1.7,0)):setOrientation(q4)
	:setVel(vec(0,0,0)):setAngularVelocity(0,0,0.4)
	--ForceGenerators.register(box2, ForceGenerators.gravityForceGen(vec(0,-2,0)))

	local box3 = Box:new("spawner", 1, 1, 1, 1):setRestitution(0.4):setFriction(0.5)
	:setPos(vec(0,2.65,0)):setOrientation(q0+quat(0,0.05,0,0))
	:setVel(vec(0,0,0)):setAngularVelocity(-0.2,0,0)
	--ForceGenerators.register(box3, ForceGenerators.gravityForceGen(vec(0,-2,0)))
--]]

---[[
	local box1 = Box:new("carved_pumpkin", 1, 1, 1, 1):setRestitution(0.4):setFriction(0.5)
	:setPos(vec(0,0.6,0)):setOrientation(quat(1,0,0.2,0))
	:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
	ForceGenerators.register(box1, ForceGenerators.gravityForceGen(vec(0,-2,0)))

	local box2 = Box:new("dropper", 1, 1, 1, 1):setRestitution(0.4):setFriction(0.5)
	:setPos(vec(0.4,1.7,0)):setOrientation(q4)
	:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
	ForceGenerators.register(box2, ForceGenerators.gravityForceGen(vec(0,-2,0)))

	local box3 = Box:new("crafter", 1, 1, 1, 1):setRestitution(0.4):setFriction(0.5)
	:setPos(vec(0,2.8,0)):setOrientation(q0)
	:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
	ForceGenerators.register(box3, ForceGenerators.gravityForceGen(vec(0,-2,0)))
	
	local box4 = Box:new("observer", 1, 1, 1, 1):setRestitution(0.4):setFriction(0.5)
	:setPos(vec(0,3.9,0)):setOrientation(quat(1,0,-0.3,0))
	:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
	ForceGenerators.register(box4, ForceGenerators.gravityForceGen(vec(0,-2,0)))
--]]

--[[
	local box1 = Box:new("slime_block", 1, 1, 1, 1):setRestitution(0.9):setFriction(0)
	:setPos(vec(0,5,0)):setOrientation(ROTS["0-0"])
	:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
	ForceGenerators.register(box1, ForceGenerators.gravityForceGen(vec(0,-10,0)))
--]]

function events.render()
	--drint(box2.vel, box2.rot)
end
