class Agent3D extends THREE.Object3D {
	constructor(a_id, radius, pos_x, pos_y, a_color) {
		super();
		this.a_id = a_id; //Agent ID
		this.geometry = new THREE.SphereGeometry( radius, 32, 32 );
		this.material = new THREE.MeshPhongMaterial(agent_material);
		this.material.color.set(a_color);
		this.agent_mesh = new THREE.Mesh(this.geometry, this.material);
		this.add(this.agent_mesh);
		this.position.x = pos_x;
		this.position.y = pos_y;
		this.position.z = radius;
	}

}