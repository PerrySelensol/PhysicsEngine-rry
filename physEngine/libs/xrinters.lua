--[=============================================================================[--
|    ______                                | 
|   |  __  \______ ______ ______ _    _    | The Xrinters (Debug library)
|   |  ____/  __  \  ____\  ____\ \  / /   | 
|   | |    |  ____| |    | |     \ \/ /    | https://github.com/PerrySelensol
|   |_|    \______/_|    |_|      \  /     | 
|                                 /_/      | 
--]=============================================================================]--


--==================== Settings ====================--

local HUD_TEXT_POS = vec(120,10)
local HUD_TEXT_SIZE = 0.5

local MODEL_TEXT_POS = vec(-10,48)
local MODEL_TEXT_SIZE = 0.25

--==================================================--

local hudText = models:newPart("dummy_part_hud", "HUD"):newText("debug_hud")
	:pos(-HUD_TEXT_POS.xy_):scale(HUD_TEXT_SIZE):light(15,15)
	:shadow(true):background(true):backgroundColor(0,0,0,0.8):wrap(true):width(450)
local modelText = models:newPart("dummy_part_model"):newText("debug_model")
	:pos(MODEL_TEXT_POS.xy_):scale(MODEL_TEXT_SIZE):light(15,15)
	:shadow(true):background(true):backgroundColor(0,0,0,0.8):wrap(true):width(450)

--==================== Data Printer ====================--

function drint(...)
	local t = table.pack(...)
	for i=1, t.n do
		local color = (tostring(t[i]):len() >= 255) and "§6" or "§f"
		t[i] = color..((t[i] == nil) and "§cnil" or tostring(t[i]))
		t[i] = string.gsub(t[i], "\t", [[   ]])
	end
	table.insert(t, 1, "§7  ====== §fDrinter §7======  ")
	local fullText = table.concat(t, "\n§8[> ")
	hudText:text(fullText); modelText:text(fullText)
end

--==================== Color Printer ====================--

function crint(...)
	local Display = {
		{text = "§7  ====== §fCrinter §7======  "}
	}
	for _, v in pairs{...} do
		local Hex = vectors.rgbToHex(v)
		table.insert(Display, {text = "\n█", color = "#"..Hex})
		table.insert(Display, {text = "§r #"..Hex})
	end
	local fullText = toJson(Display)
	hudText:text(fullText); modelText:text(fullText)
end

--==================== Table Printer ====================--

function trint(depth, ...)
	local t = table.pack(...)
	for i=1, t.n do
		local color = (tostring(t[i]):len() >= 255) and "§6" or "§f"
		t[i] = color..((t[i] == nil) and "§cnil" or logTable(t[i], depth, true))
		t[i] = string.gsub(t[i], "\t", [[   ]])
	end
	table.insert(t, 1, "§7  ====== §fTrinter §7======  ")
	local fullText = table.concat(t, "\n§8[> ")
	hudText:text(fullText); modelText:text(fullText)
end

--==================== Benchmark Printer ====================--

local Benchmarks, Benchmark_Labels = {}, {}

if not silly then -- Default
	function markBench(label)
		table.insert(Benchmark_Labels, label)
		table.insert(Benchmarks,
			{avatar:getCurrentInstructions()-27*(#Benchmarks-1)-8, client:getSystemTime()}
		)
	end

	function brint()
		markBench""
		local t = Benchmarks
		t[#t][1] = t[#t][1]-2
		local total = "§8[§fTotal§8]: §6"..(t[#t][1] - t[1][1]).." §7("..(t[#t][2] - t[1][2]).."ms)"
		for i = #t, 2, -1 do t[i][1], t[i][2] = t[i][1]-t[i-1][1], t[i][2]-t[i-1][2] end
		table.remove(t, 1)

		for i=1, #t do
			t[i] = "§8[§7"..Benchmark_Labels[i].."§8]: §f"..t[i][1].." §7("..(t[i][2]).."ms)"
		end
		table.insert(Benchmarks, 1, "§7  ====== §fBrinter §7======  ")
		table.insert(Benchmarks, total)
		local fullText = table.concat(t, "\n")
		hudText:text(fullText); modelText:text(fullText)
		Benchmarks, Benchmark_Labels = {}, {}
	end
else -- With sillyAPI harnessing java nanosecond clock
	function markBench(label)
		table.insert(Benchmark_Labels, label)
		table.insert(Benchmarks,
			{avatar:getCurrentInstructions()-31*(#Benchmarks-1)-8, ("%.0f"):format(silly:getNanoTime()/1000)}
		)
	end

	function brint()
		markBench""
		local t = Benchmarks
		t[#t][1] = t[#t][1]-2
		local total = "§8[§fTotal§8]: §6"..(t[#t][1] - t[1][1]).." §7("..(t[#t][2] - t[1][2]).."µs)"
		for i = #t, 2, -1 do t[i][1], t[i][2] = t[i][1]-t[i-1][1], t[i][2]-t[i-1][2] end
		table.remove(t, 1)

		for i=1, #t do
			t[i] = "§8[§7"..Benchmark_Labels[i].."§8]: §f"..t[i][1].." §7("..(t[i][2]).."µs)"
		end
		table.insert(Benchmarks, 1, "§7  ====== §fBrinter §7======  ")
		table.insert(Benchmarks, total)
		local fullText = table.concat(t, "\n")
		hudText:text(fullText); modelText:text(fullText)
		Benchmarks, Benchmark_Labels = {}, {}
	end
end

--==================== Quick Pos Particle ====================--

function point(v, color)
	particles.explosion:size(1):color(color):pos(v):lifetime(200000):spawn()
end

--==================== Reveal Axis After Mapping ====================--

do
	local Current_Points = {}
	local I = matrices.mat4()

	local function renderParticles()
		for _, PointAndMat in pairs(Current_Points) do
			local Point, Mat = PointAndMat[1], PointAndMat[2] or I
			local Particle = "block minecraft:white_concrete"
			particles[Particle]:scale(1/4):color(1,0.5,0):pos(Mat:apply(Point)):spawn():remove()

			for j = 1, 10 do local i = j
				particles[Particle]:scale(1/16):color(1,0,0):pos(Mat:apply(Point +vec(i,0,0))):spawn():remove()
				particles[Particle]:scale(1/16):color(0,1,0):pos(Mat:apply(Point +vec(0,i,0))):spawn():remove()
				particles[Particle]:scale(1/16):color(0,0,1):pos(Mat:apply(Point +vec(0,0,i))):spawn():remove()

				particles[Particle]:scale(1/16):color(0.5,0,0):pos(Mat:apply(Point +vec(-i,0,0))):spawn():remove()
				particles[Particle]:scale(1/16):color(0,0.5,0):pos(Mat:apply(Point +vec(0,-i,0))):spawn():remove()
				particles[Particle]:scale(1/16):color(0,0,0.5):pos(Mat:apply(Point +vec(0,0,-i))):spawn():remove()

			end
		end
	end
	
	function showAxis(Name, Point, Mat)
		Current_Points[Name] = {Point, Mat}
		events.RENDER:register(renderParticles)
	end
end

return hudText, modelText