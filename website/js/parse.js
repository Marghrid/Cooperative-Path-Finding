

function parse(cpf) {
	let instance = new Instance();

    // parse CPF:
    let lines = cpf.split("\n");
    lines.splice(0, 1);

    for(l in lines) {
    	if(lines[l] == "E =") {
    		lines.splice(0, l);
    		break;
    	}
    	let line = lines[l];
    	line = line.replace("[", " ");
    	line = line.replace("]", " ");
    	line = line.replace("(", " ");
    	line = line.replace(")", " ");
    	line = line.replace(":", " ");
    	line = line.replace(":", " ");
    	line = line.replace(":", " ");
    	line = line.split(" ");
    	//console.log(line);
    	// 0: empty
    	// 1: vertex_id
    	// 2: -1
    	// 3: empty
    	// 4: agent that starts here
    	// 5: something
    	// 6: agent that finishes here

    	let v_id       = parseInt(line[1]);
    	let a_start_id = parseInt(line[4]);
    	let a_goal_id  = parseInt(line[6]);

    	if(a_start_id > 0) {
    		--a_start_id;

    	    if(instance.agent(a_start_id) == null) {
    	    	instance.add_agent(a_start_id);
    	    }
    	    instance.set_agent_start_pos(a_start_id, v_id);
    	}
    	
    	if(a_goal_id > 0) {
    		--a_goal_id;

     	    if(instance.agent(a_goal_id) == null) {
    	    	instance.add_agent(a_goal_id);
    	    }
    	    instance.set_agent_goal_pos(a_goal_id, v_id);
    	}

    }

    lines.splice(0, 1);
    lines.splice(-1, 1);

    for(l in lines) {
    	let line = lines[l];
    	line = line.slice(0,-6);
    	line = line.slice(1);
    	line = line.split(",");

    	for(i in line) {
    		line[i] = parseInt(line[i]);
    	}

    	instance.add_edge(line);
    }

    //instance.print_agents();
    //instance.print_edges();

    if(!instance.check()) {
    	console.log("error: there is an error in the instance generated from parsing");
    	return null;
    }
    return instance;

}