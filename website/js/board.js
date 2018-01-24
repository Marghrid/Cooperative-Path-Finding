class Square {
	constructor() {
		this.x = -1;
		this.y = -1;

		this.left = null;
		this.right = null;
	}
}

class Board extends THREE.Object3D {
	constructor(instance) {
		super();

		console.log("Board constructor");
		this.instance = instance;
		this.material = board_material;

		var current_vertex = 0;
		var current_depth = 0;
		var leftmost_column = new Array();
		var final_leftmost = false;

		while(!final_leftmost) /* cycle to find the leftmost column */ {
			leftmost_column = new Array();
			while(true) /* cycle to fill te sequence */ {
				leftmost_column.push(current_vertex);
				var neighbours = this.instance.get_edges(current_vertex);
				if(neighbours[neighbours.length-1] <= current_vertex + 1) {
					// If highest neighbour is lower or the next one
					// Then this is the last of the sequence
					// There is a problem: I assume that the next vertex (n+1) is
					// not below the current (n). If it were, it would be part of the
					// leftmost column as well.
					final_leftmost = true;
					break;
				}
				current_vertex = neighbours[neighbours.length-1];
				// new current_vertex is the highest neighbour of old current_vertex

				++current_depth;
				if(this.instance.get_edges(current_vertex).indexOf(current_vertex-1) != -1) {
					// If the vertex we were adding is connected to its immediate precedent
					// Then this is not the leftmost column.
					--current_vertex;
					// This is the lowest vertex on the new supposedly leftmost column
					break;
				}
			}
			console.log("Here goes!");
			console.log(leftmost_column);
		}


		this.geometry = new THREE.BoxGeometry(this.width, this.height, 4);
		this.board_mesh = new THREE.Mesh(this.geometry, this.material);
		this.board_mesh.position.z -= 2;
		this.add(this.board_mesh);

		this.agents3d = new Array();
		for(var a in this.instance.agents) {
			var a3d = new Agent3D(this.square_length/4, a);
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