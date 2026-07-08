require("physEngine/libs/quaternions")
local Box = require("physEngine/rigidBody/box")
local HalfSpace = require("physEngine/rigidBody/halfSpace")
local ForceGenerators = require("physEngine/forceGenerators/forceGens")

local CollisionSolver = require("physEngine/collisionSolver")

--[=============================================================================]--

--local box1 = Box:new("glass", 1, 1, 1, 1):setPos(vec(0,2,0)):setOrientation(quat(1,0,0,0))
--:setVel(vec(0,0,0)):setAngularVelocity(1,0,0)

local ground = HalfSpace:new(vec(0,0,0), vec(0,1,0))

local q1 = quat(0.888073833977, 0.32505758, 0, 0.32505758):normalized()
local q2 = quat(1,0,0,0):normalized()
local q3 = quat(0.9238795325112868,0.3826834323650898,0,0):normalized()

--local box1 = Box:new("slime_block", 2, 0, 1, 1):setRestitution(1)
--:setPos(vec(0,1,0)):setOrientation(quat(1,0,0,0):normalized())
--:setVel(vec(0,-1,0)):setAngularVelocity(0,0,0*math.pi/2)

--local box2 = Box:new("slime_block", 2, 0, 1, 1):setRestitution(1)
--:setPos(vec(0,0,0)):setOrientation(quat(1,0,-0.1,-1):normalized())
--:setVel(vec(0,0,0)):setAngularVelocity(0,0,1)

local box2 = Box:new("slime_block", 1, 1, 1, 1):setRestitution(0.4):setFriction(0.6)
:setPos(vec(0,5,0)):setOrientation(q1)
:setVel(vec(0,0,0)):setAngularVelocity(0,2,0)

ForceGenerators.register(box2, ForceGenerators.gravityForceGen(vec(0,-10,0)))

--local box3 = Box:new("glass", 1, 1, 1, 1):setRestitution(1)
--:setPos(vec(-1.5,0.5+1*math.pi/2,0)):setOrientation(quat(1,0,0,0))
--:setVel(vec(0,-1,0)):setAngularVelocity(0,0,0)
--
--local box4 = Box:new("tinted_glass", 1, 1, 1, 1):setRestitution(1)
--:setPos(vec(-1.5,-0.5-1*math.pi/2,0)):setOrientation(quat(1,0,0,0))
--:setVel(vec(0,1,0)):setAngularVelocity(0,0,0)

--box2:addWorldImpulse(vec(1,0,0), vec(0,1,0))

function events.render()
	drint(box2.vel:length(), box2.rot)
end

--[[ 
local inv = getmetatable(quat(1,2,3,4)).__len

local pow = getmetatable(quat(1,2,3,4)).__concat
function events.tick()
	local q = quat(1, 2, 3, 4)
	markBench"cursed"
	for i = 1, 1000 do
		local p1 = q..-4
	end
	markBench"global"
	for i = 1, 1000 do
		local p3 = unitQuatPower(q, -4)
	end
	markBench"local"
	for i = 1, 1000 do
		local p2 = pow(q, -4)
	end
	brint()
end
--]]

--[[
function events.tick()
	local q = quat(0,1,0,0)
	drint(q..-1, q:inverted())
end
--]]