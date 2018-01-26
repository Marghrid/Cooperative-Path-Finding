// Represents a solution to an instance of CPF.
//  terminology:
//  solution: a set of moves.
//  move: A set of actions for some agents in the instance.
//        There's a move for each timestep.
//  action: an single agent's "move", from one vertex to another.

class Action {
	constructor(a_id, dest) {
		this.agent_id = a_id;
		// this.orig = 0; // I don't think this is necessary. we'll see.
		this.dest = dest;
		// If this.dest == -1, that means the agent shouldn't move.
	}

	check(instance) {
		if(this.agent_id > instance.n_agents() - 1) {
			console.log("Invalid agent in solution: " + this.agent_id);
			return false;
		}
		if(this.dest > instance.n_vertices() - 1) {
			console.log("Invalid move in solution to vertex " + this.dest);
			return false;
		}
		return true;
	}
}

class Move {
	constructor() {
		// constains all the actions for each agent in the move.
		// this.actions[k] is the action of agent k
		this.actions = new Array();
	}

	add_action(action) {
		this.actions[action.agent_id] = action;
	}

	check(instance) {
		for(let i = 0; i < instance.n_agents(); ++i) {
			let action = this.actions[i];
			if(action == null) {
				action = new Action(i, -1);
			}
			if(!action.check(instance)) {
				return false;
			}
		}
		return true;
	}
}

class Solution {
	constructor() {
		// contains the instance's solution's moves. The move this.moves[i]
		//  is executed at timestep i.
		this.moves = new Array();
	}

	set_move(timestep, move) {
		this.moves[timestep] = move;
	}

	add_action_to_move(timestep, action) {
		if(this.moves[timestep] == null)
			this.moves[timestep] = new Move();
		this.moves[timestep].add_action(action);
	}

	add_action_to_move(timestep, a_id, dest) {
		if(this.moves[timestep] == null)
			this.moves[timestep] = new Move();
		this.moves[timestep].add_action(new Action(a_id, dest));
	}

	check(instance) {
		for(let i = 0; i < this.moves.length; ++i) {
			let move = this.moves[i];
			if(move == null)
				move = new Move();
			else if(!move.check(instance))
				return false;
		}
		return true;
	}
}