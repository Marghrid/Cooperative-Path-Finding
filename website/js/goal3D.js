class Goal3D extends THREE.Object3D {
	constructor(a_id, radius, pos_x, pos_y, a_color) {
		super();
		this.a_id = a_id; //Agent ID
		this.geometry = new THREE.CylinderGeometry(radius, radius, GOAL_HEIGHT, 16);
		this.material = new THREE.MeshPhongMaterial(goal_material);
		this.material.color.set(a_color);
		this.goal_mesh = new THREE.Mesh(this.geometry, this.material);
		this.goal_mesh.rotateX(Math.PI/2);
		this.add(this.goal_mesh);
		this.position.x = pos_x;
		this.position.y = pos_y;
		this.position.z = GOAL_HEIGHT/2;
	}
}