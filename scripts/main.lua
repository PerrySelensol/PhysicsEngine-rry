local quatMath = require("scripts.quaternions")


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