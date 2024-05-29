1. Here we place all objects at the position with Y = 0.

2. We don't create any mesh with the tool. There are standard Unity meshes of the Capsule, Cube, Cylinder, Sphere.

3. Check how the objects are placed on the Terrain and how they are bending.

This is because detail meshes should have the vertex color alpha painted to determine how much wind affects them. 
	vertex color alpha: 0 value - wind will not affect, 1 value - wind affects fully. 
	If we are talking about Color, so the range is 0 - 1. 
	If is a Color32 so 0 - 255.