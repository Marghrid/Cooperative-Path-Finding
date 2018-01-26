// Represents a CPF instance.
class Instance {
	constructor() {
		this.agents = new Array();

		// The graph used to describe the environment:
		this.adjacencies = new Array();
	}

	agent(id) { return this.agents[id]; }

	get_neighbours(vertex_id) { return this.adjacencies[vertex_id]; }

	n_agents() { return this.agents.length; }
	n_vertices() { return this.adjacencies.length; }

	add_agent(id) {
		let a = new Agent(id);
		this.agents[a.id] = a;
		//console.log("added agent " + a.id);
	}

	add_edge(edge) {
		if(edge.length != 2) {
			console.log("error: " + edge + " not a valid edge.");
			return;
		}
		//console.log(edge);
		if(this.adjacencies[edge[0]] == null)
			this.adjacencies[edge[0]] = new Array();

		if(this.adjacencies[edge[1]] == null)
			this.adjacencies[edge[1]] = new Array();

		this.adjacencies[edge[0]].push(edge[1]);
		this.adjacencies[edge[1]].push(edge[0]);
	}

	set_agent_start_pos(id, pos) { this.agents[id].set_start_pos(pos); }

	set_agent_goal_pos(id, pos) { this.agents[id].set_goal_pos(pos); }

	print_agents() {
		for(let a in this.agents) {
			console.log("agent " + this.agents[a].id +
				" : (" + this.agents[a].start_pos +
				", " + this.agents[a].goal_pos + ")");
		}
	}

	print_edges() {
		for(let e in this.adjacencies) {
			console.log(e + ": " + this.adjacencies[e]);
		}
	}

	// Checks for inconsistencies, and prepares the instance to be drawn.
	check() {

		for(let i = 0; i < this.agents.length; ++i) {
			if(this.agents[i] == null) {
				console.log("error: unexistent agent: " + i);
				return false;
			}
			else if(this.agents[i].id != i) {
				console.log("error: incorrectly stored agent: id = " +
					this.agents[i].id + "; index = " + i);
				return false;
			}
		}

		for(let i = 0; i < this.adjacencies.length; ++i) {
			if(this.adjacencies[i] == null)
				// It's a vertex with no edges, an isolated vertex.
				// It's adjacency list is initialized, just to prevent
				//  it from being undefined or null.
				this.adjacencies[i] = new Array();
			
			else
				this.adjacencies[i] = this.adjacencies[i].sort(function (x, y) { return x - y; });
				// Orders the adjacencies so  it's easy to find the largest.
		}
		return true;
	}
}