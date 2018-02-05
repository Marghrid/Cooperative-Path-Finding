// Goal3D is the 3D representation of the agents' goals on the Board.
class Goal3D extends THREE.Object3D {
	constructor(a_id, radius, pos_x, pos_y, a_color) {
		super();
		this.a_id = a_id; //Agent ID
		this.geometry = new THREE.TorusGeometry(radius, radius/4, 8, 16 );
		this.material = new THREE.MeshPhongMaterial(goal_material);
		this.material.color.set(a_color.multiplyScalar(0.6));
		this.material.emissive.set("hsl(0, 0%, 15%)");
		this.goal_mesh = new THREE.Mesh(this.geometry, this.material);
		this.add(this.goal_mesh);
		this.position.x = pos_x;
		this.position.y = pos_y;
		this.position.z = radius/4;
	}
}