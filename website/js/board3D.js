// Board3D is the 3D graphic representation of an instance.

class Board3D extends THREE.Object3D {
	constructor(instance) {
		super();

		this.instance = instance;

		// SquarePos is a auxiliary class to represent the position
		//  of a square in square coordinates.
		// Square coordinates: counted in number of squares, with
		//  origin on the top left corner of the board.
		let squares_pos = new Array();
		for(let i = 0; i < this.instance.n_vertices(); ++i) {
			squares_pos.push(new SquarePos());
		}


		// Interpret the instance and come up with coordinates for each square.
		// Done as follows:
		//  1. Find the board's leftmost column, and deduce its squares' coordinates;
		//  
		//  2. Repeat until there are no more columns:
		//    1. Find the squares immediately to the right of the ones on the last column,
		//         and deduce their coordinates.
		//    2. Find new squares directly above or below the current column's (they belong 
		//         to the column as well) and deduce their coordinates.
		//    3. For those newly added squares (in step 2), deduce coordinates for every
		//         neighbour that hasn't been visited.

		let current_column = find_leftmost_column(this.instance, squares_pos);

		while(current_column.length > 0) {
			let new_column    = increment_column(current_column, this.instance, squares_pos);
			let new_in_column = expand_column_vertically(new_column, this.instance, squares_pos);
			expand_new(new_in_column, this.instance, squares_pos);

			current_column = new_column;
		}

		check_for_isolated_squares(squares_pos);

		// Height and width in square coordinates.
		this.height_squares = get_height_squares(squares_pos);
		this.width_squares  = get_width_squares(squares_pos);

		// The length of a single square.
		this.square_length = this.width_squares > this.height_squares ?
			BOARD_LENGTH / this.width_squares :
			BOARD_LENGTH / this.height_squares;

		// Height and width in world coordinates, the largest of which will be BOARD_LENGTH
		this.height = this.height_squares * this.square_length;
		this.width  = this.width_squares  * this.square_length;

		this.squares3D = new Array();
		for(let i in squares_pos) {
			if(squares_pos[i].x == -1 || squares_pos[i].y == -1) continue; // isolated vertex.

			// Position in square coordinates => position in world coordinates
			let square_pos_x =  (squares_pos[i].x + .5) * this.square_length - 0.5 * this.width;
			let square_pos_y = -(squares_pos[i].y + .5) * this.square_length + 0.5 * this.height;

			let odd = false;
			if( (squares_pos[i].x + squares_pos[i].y) % 2 == 1)
				odd = true;

			let s3D = new Square3D(this.square_length, square_pos_x, square_pos_y, odd);
			this.squares3D.push(s3D);
			this.add(s3D);
		}

		this.agents3d = new Array();
		this.goals3d  = new Array();
		for(let a in this.instance.agents) {
			if(squares_pos[this.instance.agents[a].start_pos].x == -1 ||
				squares_pos[this.instance.agents[a].start_pos].y == -1 ) {
				let warning = "agent " + a + " positioned in isolated vertex "
					+ this.instance.agents[a].start_pos;
				console.log(warning);
				throw warning;
				break;
			}
			
			if(squares_pos[this.instance.agents[a].goal_pos].x == -1 ||
				squares_pos[this.instance.agents[a].goal_pos].y == -1 ) {
				let warning = "agent's " + a + " goal positioned in isolated vertex "
					+ this.instance.agents[a].start_pos;
				console.log(warning);
				throw warning;
				break;
			}

			// Distributes agents' colors evenly;
			let color_s = a * 360 / this.instance.n_agents();
			let agent_color = new THREE.Color("hsl(" + color_s + ", 100%, 50%)");

			//console.log("Agent " + a + " starts at " + this.instance.agents[a].start_pos);
			let agent_pos_x = this.squares3D[this.instance.agents[a].start_pos].position.x;
			let agent_pos_y = this.squares3D[this.instance.agents[a].start_pos].position.y;
			
			//console.log("Agent's " + a + " goal is " + this.instance.agents[a].start_pos);
			let goal_pos_x = this.squares3D[this.instance.agents[a].goal_pos].position.x;
			let goal_pos_y = this.squares3D[this.instance.agents[a].goal_pos].position.y;

			let a3d = new Agent3D(a, this.square_length/4, agent_pos_x, agent_pos_y, agent_color);
			let g3d = new  Goal3D(a, this.square_length/4, goal_pos_x,  goal_pos_y,  agent_color);

			this.agents3d[a] = a3d;
			this.goals3d[a]  = g3d;
			this.add(a3d);
			this.add(g3d);
		}
	}

	animate(dt) {
		this.rotateZ(0.03*dt);
	}
}



// Auxiliary class:
class SquarePos {
	constructor() {
		this.x = -1;
		this.y = -1;
	}
}

// Auxiliary functions:

// find_leftmost_column: given an instance, will find the leftmost column
//  of adjacent vertices. Problem: it assumes vertices with following numbers
//  don't belong in the same column (they're in the same row).
function find_leftmost_column(instance, squares_pos) {
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

			if(neighbours.length == 0) {
				// This can only happen if the FIRST vertex evaluated (usually 0)
				//  is an isolated vertex. In that case we move onto the next one.
				++current_vertex;
				break;
			}

			if(neighbours[neighbours.length-1] <= current_vertex + 1) {
				// If highest neighbour is lower or the next one
				// Then this is the last of the sequence
				// There is a problem: It's assumed that the next vertex (n+1) is
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
		squares_pos[leftmost_column[i]].x = 0;
		squares_pos[leftmost_column[i]].y = initial_depth + i;
	}

	return leftmost_column;
}

// increment_column: given a column, will find and give coordinates to all
//  vertices immediately to the right of those in the column.
function increment_column(column, instance, squares_pos) {
	new_column = new Array();
	for(let i in column) {
		let v = column[i];
		for(let j in instance.get_neighbours(v)) {
			let n = instance.get_neighbours(v)[j];
			if(n == v+1) {
				new_column.push(n);
				squares_pos[n].x = squares_pos[v].x + 1;
				squares_pos[n].y = squares_pos[v].y;
			}
		}
	}
	//console.log("Incrementin column " + column + ", result: " + new_column);
	return new_column;
}

// expand_column_vertically: Given a column of vertices, will find and give
//  coordinates to all vertices directly above or below the one in the column
//  (i. e., all vertices belonging to the same column.)
function expand_column_vertically(column, instance, squares_pos) {
	let new_in_column = new Array();
	let to_expand = column.slice();

	while(to_expand.length > 0) {
		let v = to_expand[0];
		for(let j in instance.get_neighbours(v)) {
			let n = instance.get_neighbours(v)[j];
			if(n > v+1 && column.indexOf(n) == -1
				&& new_in_column.indexOf(n) == -1) {

				new_in_column.push(n);
				to_expand.push(n);
				squares_pos[n].x = squares_pos[v].x;
				squares_pos[n].y = squares_pos[v].y + 1;
			}
			else if(n < v-1 && column.indexOf(n) == -1
				&& new_in_column.indexOf(n) == -1) {

				new_in_column.push(n);
				to_expand.push(n);
				squares_pos[n].x = squares_pos[v].x;
				squares_pos[n].y = squares_pos[v].y - 1;
			}
		}
		to_expand.splice(0, 1);
	}
	//console.log("Expanding column" + column + " vertically. Adding " + new_in_column);
	column = column.concat(new_in_column);
	return new_in_column;
}

// expand_new: Given a set of vertices (new_vertices), will find and give
//  coordinates to their previously unvisited neighbours, and neighbours' neighbours.
function expand_new(new_vertices, instance, squares_pos) {
	while(new_vertices.length > 0) {
		let v = new_vertices[0];
		//console.log("Expanding " + v);
		for(let i in instance.get_neighbours(v)) {
			n = instance.get_neighbours(v)[i];
			if(squares_pos[n].x == -1 || squares_pos[n].y == -1) {
				if(n == v-1) {
					squares_pos[n].x = squares_pos[v].x - 1;
					squares_pos[n].y = squares_pos[v].y;
					//console.log("visiting " + n );
					new_vertices.push(n);
				} else if(n < v-1) {
					squares_pos[n].x = squares_pos[v].x;
					squares_pos[n].y = squares_pos[v].y - 1;
					//console.log("visiting " + n );
					new_vertices.push(n);
				} else if(n == v+1) {
					squares_pos[n].x = squares_pos[v].x + 1;
					squares_pos[n].y = squares_pos[v].y;
					//console.log("visiting " + n );
					new_vertices.push(n);
				} else if(n > v+1) {
					squares_pos[n].x = squares_pos[v].x;
					squares_pos[n].y = squares_pos[v].y + 1;
					//console.log("visiting " + n );
					new_vertices.push(n);
				}
			}
		}
		new_vertices.splice(0, 1);
	}
}

// Given all the squares positions in squares coordinates, retrieves the height
//  of the board in square coordinates.
// Basically, finds the maximum value of all Ys.
function get_height_squares(squares_pos) {
	let ys = new Array();

	for(let i in squares_pos) {
		ys.push(squares_pos[i].y);
	}

	// ...: spread operator:
	// https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Operators/Spread_operator
	return Math.max(...ys) + 1;
}

// Given all the squares positions in squares coordinates, retrieves the height
//  of the board in square coordinates.
// Basically, finds the maximum value of all Ys.
function get_width_squares(squares_pos) {
	let xs = new Array();

	for(let i in squares_pos) {
		xs.push(squares_pos[i].x);
	}

	// ...: spread operator:
	// https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Operators/Spread_operator
	return Math.max(...xs) + 1;
}

// check_for_isolated_squares: Checks the squares for existence of isolated
//  vertices. Not that they're a problem. They're not, unless they have an
//  agent/goal in them. In that case they render the instance unsolvable.
// Isolated squares are not represented on the board, because their position
//  cannot easily be deduced.
function check_for_isolated_squares(squares_pos) {
	//console.log(squares_pos);
	let isolated = false;
	for(let i in squares_pos) {
		if(squares_pos[i].x == -1 || squares_pos[i].y == -1) {
			console.log("Isolated square: " + i);
			isolated = true;
		}
	}
	return isolated;
}
