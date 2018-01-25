class Square3D extends THREE.Object3D {
	constructor(length, pos_x, pos_y) {
		super();
		let geometry = new THREE.BoxGeometry(length, length, BOARD_HEIGHT);
		let material = board_material;
		let square_mesh = new THREE.Mesh(geometry, material);
		this.add(square_mesh);
		this.position.x = pos_x;
		this.position.y = pos_y;
		this.position.z = - BOARD_HEIGHT/2;
	}
}