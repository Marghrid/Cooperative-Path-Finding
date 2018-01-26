// The 3D representation of the board's squares, i. e., the instance's vertices.

class Square3D extends THREE.Object3D {
	constructor(length, pos_x, pos_y, odd) {
		super();
		this.geometry = new THREE.BoxGeometry(length, length, BOARD_HEIGHT);
		this.material = board_material_e;
		if(odd) {
			this.material = board_material_o;
		}

		this.square_mesh = new THREE.Mesh(this.geometry, this.material);
		this.add(this.square_mesh);
		this.position.x = pos_x;
		this.position.y = pos_y;
		this.position.z = - BOARD_HEIGHT/2;
	}
}