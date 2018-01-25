class Board extends THREE.Object3D {
	constructor(instance) {
		super();
		this.instance = instance;
		this.material = board_material;
		let edges0 = this.instance.get_edges(0);
		let adj = edges0[0];
		if(adj == 1) adj = edges0[1];

		this.width_squares = adj;
		this.height_squares = this.instance.n_vertices()/this.width_squares;

		if(this.width_squares > this.height_squares) {
			this.width = 80;
			this.height = this.height_squares * 80 / this.width_squares;
			this.square_length = 80/this.width_squares;
		}
		else {
			this.height = 80;
			this.width = this.width_squares * 80 / this.height_squares;
			this.square_length = 80/this.height_squares;
		}

		this.geometry = new THREE.BoxGeometry(this.width, this.height, 4);
		this.board_mesh = new THREE.Mesh(this.geometry, this.material);
		this.board_mesh.position.z -= 2;
		this.add(this.board_mesh);

		this.agents3d = new Array();
		for(let a in this.instance.agents) {
			let a3d = new Agent3D(this.square_length/4, a);
			this.agents3d[a] = a3d;
			a3d.position.z = this.square_length/4;
			a3d.position.x = ((this.instance.agents[a].start_pos)%this.width_squares)*this.square_length - 0.5*this.width + 0.5*this.square_length;
			a3d.position.y = Math.floor((this.instance.agents[a].start_pos)/this.width_squares)*this.square_length - 0.5*this.height + 0.5*this.square_length;
			//console.log(a3d.position.x);
			//console.log(a3d.position.y);
			this.add(a3d);
		}
	}

	animate() {
		this.rotateZ(0.005);
	}
}