local ContactGenerators = require("./contacts")

--[=============================================================================]--

for _, path in ipairs(listFiles(".")) do
	if not path:match("init") then require(path) end
end

return ContactGenerators