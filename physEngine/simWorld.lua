
--[=============================================================================]--

local simWorld = {}

local simWorldPart = models:newPart("simWorldPart", "World"):pos(16*vec(-86, 163, -207))

local inext = ipairs({})
local renderName = host:isHost() and "world_render" or "render"
events[renderName] = function(delta)
	--drint(simWorld[1].ori_, simWorld[1].ori)
	for i, body in inext, simWorld, 0 do
		body:render(delta)
	end
end

function events.tick()
	for i, body in inext, simWorld, 0 do
		body:integrate(1/20)
	end
end

return simWorld, simWorldPart