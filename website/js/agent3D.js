class Agent3D extends THREE.Object3D {
	constructor(radius, a_id) {
		super();
			this.a_id = a_id;
			this.geometry = new THREE.SphereGeometry( radius, 32, 32 );
			this.material = agent_material;
			this.agent_mesh = new THREE.Mesh(this.geometry, this.material);
			this.add(this.agent_mesh);
	}

}