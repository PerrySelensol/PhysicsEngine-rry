require("physEngine/libs/quaternions")
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

local q0 = quat(1,0,0,0)
local q1 = quat(0.888073833977, 0.32505758, 0, 0.32505758):normalized()

local q2 = quat(0.9238795325112868,0.3826834323650898,0,0):normalized()
local q3 = quat(0.9238795325112868,0,0,0.3826834323650898):normalized()
local q4 = quat(0.9238795325112868,0,0.3826834323650898,0):normalized()


local box1 = Box:new("glass", 1, 1, 1, 1):setRestitution(0.4):setFriction(0.5)
:setPos(vec(0,2.2,0)):setOrientation(q4)
:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
ForceGenerators.register(box1, ForceGenerators.gravityForceGen(vec(0,-10,0)))

local box2 = Box:new("spawner", 1, 1, 1, 1):setRestitution(0.4):setFriction(0.5)
:setPos(vec(0,1,0)):setOrientation(q0)
:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
ForceGenerators.register(box2, ForceGenerators.gravityForceGen(vec(0,-10,0)))

--local box3 = Box:new("honey_block", 1, 1, 1, 1):setRestitution(0.4):setFriction(0.5)
--:setPos(vec(0,4,0)):setOrientation(q0)
--:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
--ForceGenerators.register(box3, ForceGenerators.gravityForceGen(vec(0,-10,0)))

--for i = 1, 10 do
--	local B = Box:new("slime_block", i/3, 2, 1, 1):setRestitution(1):setFriction(0.4)
--	:setPos(vec(0,7,0)):setOrientation(q1)
--	:setVel(vec(0,0,0)):setAngularVelocity(0,10,0)
--	ForceGenerators.register(B, ForceGenerators.gravityForceGen(vec(0,-1,0)))
--end


function events.render()
	--drint(box2.vel, box2.rot)
end
