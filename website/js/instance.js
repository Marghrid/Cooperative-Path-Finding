class Instance {
	constructor() {
		this.agents = new Array();
		this.adjacencies = new Array();
	}

	agent(id) { return this.agents[id]; }

	get_edges(vertex_id) { return this.adjacencies[vertex_id]; }

	n_agents() { return this.agents.length; }
	n_vertices() { return this.adjacencies.length; }

	add_agent(id) {
		var a = new Agent(id);
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
		for(var a in this.agents) {
			console.log("agent " + this.agents[a].id +
				" : (" + this.agents[a].start_pos +
				", " + this.agents[a].goal_pos + ")");
		}
	}

	print_edges() {
		for(var e in this.adjacencies) {
			console.log(e + ": " + this.adjacencies[e]);
		}
	}

	check() {
		for(var e in this.adjacencies) {
			if(this.adjacencies == null) {
				console.log("error: Vertex with no edges: " + e);
				return false;
			}
		}

		for(var a in this.agents) {
			if(this.agents == null) {
				console.log("error: unexistent agent: " + a);
				return false;
			}
			else if(this.agents[a].id != a) {
				console.log("error: incorrectly stored agent: id = " +
					this.agents[a].id + "; index = " + a);
				return false;
			}
		}

		for(a in this.adjacencies) {
			this.adjacencies[a].sort();
		}
		return true;
	}
}