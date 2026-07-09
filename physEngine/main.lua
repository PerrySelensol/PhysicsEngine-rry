require("physEngine/libs/quaternions")
local Box = require("physEngine/rigidBody/box")
local HalfSpace = require("physEngine/rigidBody/halfSpace")
local ForceGenerators = require("physEngine/forceGenerators/forceGens")

local CollisionSolver = require("physEngine/collisionSolver")

--[=============================================================================]--

local ground = HalfSpace:new(vec(0,0,0), vec(0,1,0))

local q0 = quat(1,0,0,0)
local q1 = quat(0.888073833977, 0.32505758, 0, 0.32505758):normalized()

local q2 = quat(0.9238795325112868,0.3826834323650898,0,0):normalized()
local q3 = quat(0.9238795325112868,0,0,0.3826834323650898):normalized()
local q4 = quat(0.9238795325112868,0,0.3826834323650898,0):normalized()


local box1 = Box:new("glass", 1, 1, 1, 1):setRestitution(0.4):setFriction(0.6)
:setPos(vec(0,0.6,0)):setOrientation(q0)
:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)

local box2 = Box:new("spawner", 1, 1, 1, 1):setRestitution(0.4):setFriction(0.6)
:setPos(vec(0,2,0)):setOrientation(q4)
:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)

ForceGenerators.register(box1, ForceGenerators.gravityForceGen(vec(0,-0.1,0)))
ForceGenerators.register(box2, ForceGenerators.gravityForceGen(vec(0,-0.1,0)))

function events.render()
	--drint(box2.vel, box2.rot)
end
