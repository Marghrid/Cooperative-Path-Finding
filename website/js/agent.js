// Represents an agent in an instance.
class Agent {
	constructor(id) {
		this.id = id;

		// start_pos and goal_pos are the vertex IDs of
		// the agent's start and goal positions.
		this.start_pos = -1;
		this.goal_pos  = -1;
	}

	set_start_pos(pos) {
		this.start_pos = pos;
	}

	set_goal_pos(pos) {
		this.goal_pos = pos;
	}
}