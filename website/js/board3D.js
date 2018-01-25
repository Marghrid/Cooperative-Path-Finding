class BoardSquare {
	constructor() {
		this.x = -1;
		this.y = -1;
	}
}

function increment_column(column, instance, squares) {
	new_column = new Array();
	for(let i in column) {
		let v = column[i];
		for(let j in instance.get_neighbours(v)) {
			let n = instance.get_neighbours(v)[j];
			if(n == v+1) {
				new_column.push(n);
				squares[n].x = squares[v].x + 1;
				squares[n].y = squares[v].y;
			}
		}
	}
	//console.log("Incrementin column " + column + ", result: " + new_column);
	return new_column;
}

function expand_column_vertically(column, instance, squares) {
	let new_in_column = new Array();
	let to_expand = column.slice();

	while(to_expand.length > 0) {
		let v = to_expand[0];
		for(let j in instance.get_neighbours(v)) {
			let n = instance.get_neighbours(v)[j];
			if(n > v+1 && column.indexOf(n) == -1 && new_in_column.indexOf(n) == -1) {
				new_in_column.push(n);
				to_expand.push(n);
				squares[n].x = squares[v].x;
				squares[n].y = squares[v].y + 1;
			}
			else if(n < v-1 && column.indexOf(n) == -1 && new_in_column.indexOf(n) == -1) {
				new_in_column.push(n);
				to_expand.push(n);
				squares[n].x = squares[v].x;
				squares[n].y = squares[v].y - 1;
			}
		}
		to_expand.splice(0, 1);
	}
	//console.log("Exapnding column" + column + " vertically. Adding " + new_in_column);
	column = column.concat(new_in_column);
	return new_in_column;
}

function expand_new(new_vertices, instance, squares) {
	while(new_vertices.length > 0) {
		let v = new_vertices[0];
		//console.log("Expanding " + v);
		for(let i in instance.get_neighbours(v)) {
			n = instance.get_neighbours(v)[i];
			if(squares[n].x == -1 || squares[n].y == -1) {
				if(n == v-1) {
					squares[n].x = squares[v].x - 1;
					squares[n].y = squares[v].y;
					//console.log("visiting " + n );
					new_vertices.push(n);
				} else if(n < v-1) {
					squares[n].x = squares[v].x;
					squares[n].y = squares[v].y - 1;
					//console.log("visiting " + n );
					new_vertices.push(n);
				} else if(n == v+1) {
					squares[n].x = squares[v].x + 1;
					squares[n].y = squares[v].y;
					//console.log("visiting " + n );
					new_vertices.push(n);
				} else if(n > v+1) {
					squares[n].x = squares[v].x;
					squares[n].y = squares[v].y + 1;
					//console.log("visiting " + n );
					new_vertices.push(n);
				}
			}
		}
		new_vertices.splice(0, 1);
	}
}

function find_leftmost_column(instance, squares) {

	let current_vertex = 0;
	let initial_depth = 0;
	let current_depth = 0;
	let final_leftmost = false;
	
	let leftmost_column = new Array();
	while(!final_leftmost) /* cycle to find the leftmost column */ {
		leftmost_column = new Array();
		initial_depth = current_depth;

		while(true) /* cycle to fill te sequence */ {
			leftmost_column.push(current_vertex);
			let neighbours = instance.get_neighbours(current_vertex);
			//console.log("neighbours of " + current_vertex + ": " + neighbours);
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
			if(instance.get_neighbours(current_vertex).indexOf(current_vertex-1) != -1) {
				// If the vertex we were adding is connected to its immediate precedent
				// Then this is not the leftmost column.
				--current_vertex;
				// This is the lowest vertex on the new supposedly leftmost column
				break;
			}
		}
	}

	for(let i = 0; i < leftmost_column.length; ++i) {
		squares[leftmost_column[i]].x = 0;
		squares[leftmost_column[i]].y = initial_depth + i;
	}

	return leftmost_column;
}

function get_height_squares(squares) {
	let ys = new Array();

	for(let i in squares) {
		ys.push(squares[i].y);
	}

	return ys.sort(function (a, b) { return a - b; })[ys.length-1] + 1;
}

function get_width_squares(squares) {
	let xs = new Array();

	for(let i in squares) {
		xs.push(squares[i].x);
	}

	return xs.sort(function (a, b) { return a - b; })[xs.length-1] + 1;
}

function check_squares(squares) {
	//console.log(squares);
	let ok = true;
	for(let i in squares) {
		if(squares[i].x == -1 || squares[i].y == -1) {
			console.log("Isolated square: " + i);
			okay = false;
		}
	}
	return ok;
}

class Board3D extends THREE.Object3D {
	constructor(instance) {
		super();

		this.instance = instance;

		let squares = new Array();

		for(let i = 0; i < this.instance.n_vertices(); ++i) {
			squares.push(new BoardSquare());
		}


		// Interpret the instance and come up with coordinates for each square.
		let current_column = find_leftmost_column(this.instance, squares);

		while(current_column.length > 0) {
			let new_column    = increment_column(current_column, this.instance, squares);
			let new_in_column = expand_column_vertically(new_column, this.instance, squares);
			expand_new(new_in_column, this.instance, squares);

			current_column = new_column;
		}

		check_squares(squares);

		this.height_squares = get_height_squares(squares);
		this.width_squares  = get_width_squares(squares);

		this.square_length = this.width_squares > this.height_squares ?
			BOARD_LENGTH / this.width_squares :
			BOARD_LENGTH / this.height_squares;

		this.height = this.height_squares * this.square_length;
		this.width  = this.width_squares  * this.square_length;

		this.squares3D = new Array();

		for(let i in squares) {
			if(squares[i].x == -1 || squares[i].y == -1) continue;
			let square_pos_x =  (squares[i].x + .5) * this.square_length - 0.5 * this.width;
			let square_pos_y = -(squares[i].y + .5) * this.square_length + 0.5 * this.height;
			let s3D = new Square3D(this.square_length, square_pos_x, square_pos_y);
			this.squares3D.push(s3D);
			this.add(s3D);
		}

		this.agents3d = new Array();
		for(let a in this.instance.agents) {
			if(squares[this.instance.agents[a].start_pos].x == -1 ||
				squares[this.instance.agents[a].start_pos].y == -1 ) {
				console.log("agent " + a + " positioned in isolated vertex " + this.instance.agents[a].start_pos);
				throw "agent " + a + " positioned in isolated vertex " + this.instance.agents[a].start_pos;
				break;
			}
			
			if(squares[this.instance.agents[a].goal_pos].x == -1 ||
				squares[this.instance.agents[a].goal_pos].y == -1 ) {
				console.log("agent's " + a + " goal positioned in isolated vertex " + this.instance.agents[a].start_pos);
				throw "agent " + a + " goal positioned in isolated vertex " + this.instance.agents[a].start_pos;
				break;
			}

			let color_s = a * 360 / this.instance.n_agents();
			let agent_color = new THREE.Color("hsl(" + color_s + ", 100%, 50%)");
			//console.log("Agent " + a + " starts at " + this.instance.agents[a].start_pos);
			//console.log(this.squares3D[this.instance.agents[a].start_pos].position);
			let agent_pos_x = this.squares3D[this.instance.agents[a].start_pos].position.x;
			let agent_pos_y = this.squares3D[this.instance.agents[a].start_pos].position.y;

			let goal_pos_x = this.squares3D[this.instance.agents[a].goal_pos].position.x;
			let goal_pos_y = this.squares3D[this.instance.agents[a].goal_pos].position.y;

			let a3d = new Agent3D(a, this.square_length/4, agent_pos_x, agent_pos_y, agent_color);
			let g3d = new  Goal3D(a, this.square_length/4, goal_pos_x,  goal_pos_y,  agent_color);

			this.agents3d[a] = a3d;
			this.add(a3d);
			this.add(g3d);
		}
	}

	animate(dt) {
		this.rotateZ(0.03*dt);
	}
}