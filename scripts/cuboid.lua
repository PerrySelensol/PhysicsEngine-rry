local newCube
do
	Cube_Geometry = {
		vertices = {
			vec( 1, 1, 1)/2,
			vec( 1, 1,-1)/2,
			vec( 1,-1, 1)/2,
			vec( 1,-1,-1)/2,
			vec(-1, 1, 1)/2,
			vec(-1, 1,-1)/2,
			vec(-1,-1, 1)/2,
			vec(-1,-1,-1)/2,
		},
		axes = Basis,
		inertiaTensor = matrices.mat3(
			vec(1/6,0,0),
			vec(0,1/6,0),
			vec(0,0,1/6)
		),
		faces = {
			{vec( 1, 0, 0), vec(-1, 0, 0)},
			{vec( 0, 1, 0), vec( 0,-1, 0)},
			{vec( 0, 0, 1), vec( 0, 0,-1)},
		},
		-- Endpoints of edges plus edge normal
		edges = {
			{
				{vec( 1, 1, 1)/2, vec(-1, 1, 1)/2, vec( 0, 1, 1):normalized()},
				{vec( 1, 1,-1)/2, vec(-1, 1,-1)/2, vec( 0, 1,-1):normalized()},
				{vec( 1,-1, 1)/2, vec(-1,-1, 1)/2, vec( 0,-1, 1):normalized()},
				{vec( 1,-1,-1)/2, vec(-1,-1,-1)/2, vec( 0,-1,-1):normalized()},
			},
			{
				{vec( 1, 1, 1)/2, vec( 1,-1, 1)/2, vec( 1, 0, 1):normalized()},
				{vec( 1, 1,-1)/2, vec( 1,-1,-1)/2, vec( 1, 0,-1):normalized()},
				{vec(-1, 1, 1)/2, vec(-1,-1, 1)/2, vec(-1, 0, 1):normalized()},
				{vec(-1, 1,-1)/2, vec(-1,-1,-1)/2, vec(-1, 0,-1):normalized()},
			},
			{
				{vec( 1, 1, 1)/2, vec( 1, 1,-1)/2, vec( 1, 1, 0):normalized()},
				{vec( 1,-1, 1)/2, vec( 1,-1,-1)/2, vec( 1,-1, 0):normalized()},
				{vec(-1, 1, 1)/2, vec(-1, 1,-1)/2, vec(-1, 1, 0):normalized()},
				{vec(-1,-1, 1)/2, vec(-1,-1,-1)/2, vec(-1,-1, 0):normalized()},
			},
		}
	}

	newCube = function(blockState, mass, init_pos, init_rot, init_vel, init_wel)

		local Cube = {
			init_pos = init_pos:copy(),
			init_rot = init_rot:copy() or quat(1,0,0,0),
			init_vel = init_vel:copy() or vec(0,0,0),
			init_wel = init_wel:copy() or vec(0,0,0),

			type = "cube", mass = mass,
			bounciness = 0.95,
			axes = Cube_Geometry.axes,
			inertiaTensor = Cube_Geometry.inertiaTensor*mass,
			vertices = Cube_Geometry.vertices,

			render_pos = init_pos, render_rot = init_rot or quat(1,0,0,0),

			pos_ = init_pos, pos = init_pos,
			rot_ = init_rot or quat(1,0,0,0), rot = init_rot or quat(1,0,0,0),

			momentumL = init_vel or vec(0,0,0),
			momentumA = init_wel or vec(0,0,0),

			nudgeQueue = {},
			
			task = PhysWorld:newBlock("Cube"..#Rigid_Bodies):block(blockState):pos(init_pos)
		}
		table.insert(Rigid_Bodies, Cube)

		return Cube

	end
end
