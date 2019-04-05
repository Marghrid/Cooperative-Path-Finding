// parse: given a string with and instance read from a file, returns an Instance Object.
function parse_cpf(cpf) {
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
        // (0:-1)[0:0:0]
    	line = line.slice(1);
    	line = line.replace(":-1)[", " ");
        line = line.replace(":", " ");
    	line = line.replace(":", " ");
        line = line.slice(0, -1);
        // 0 0 0 0

    	line = line.split(" ");
    	// 0: vertex_id
    	// 1: agent that starts here
    	// 2: something
    	// 3: agent that finishes here

    	let v_id       = parseInt(line[0]);
    	let a_start_id = parseInt(line[1]);
    	let a_goal_id  = parseInt(line[3]);

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

function parse_Surynek_solution(sol_lines) {
    let solution_length = parseInt(sol_lines[0].split(":")[1]); //pretty sure this is useless.
    console.log("Parsing Sury solution");
    console.log(sol_lines);
    let solution = new Solution();

    for(l in sol_lines) {
        // 1 # 2 ---> 1 (0)
        line = sol_lines[l];
        line = line.replace(" @ ", " ");
        line = line.replace(" ---> ", " ");
        line = line.replace(" (", " ");
        line = line.slice(0, -1);
        // 1 2 1 0

        let words = line.split(" ");

        let a_id        = parseInt(words[0]) - 1;
        // ignore oringin vertex.
        let dest_vertex = parseInt(words[2]);
        let timestep    = parseInt(words[3]);
        if(a_id >= 0) {
            solution.add_action_to_move(timestep, a_id, dest_vertex);
            //TEST ELSE
        }
    }

    return solution;
}

function parse_M_solution(sol_lines) {
    let solution = new Solution();

    console.log("Parsing M solution");
    console.log(sol_lines);

    let timestep = -1;

    for(l in sol_lines) {
    /*  Timestep 0:
            0 @ 5
            1 @ 8
            2 @ 1   */

        line = sol_lines[l];

        if(line.slice(0, 8) == "Timestep") {
            line = line.replace("Timestep ", "");
            timestep = parseInt(line);
            continue;
        }

        line = line.replace(" @ ", " ");
        /* 0 5
           1 8
           2 1  */

        let words = line.split(" ");

        let a_id        = parseInt(words[0]);
        // ignore origin vertex.
        let dest_vertex = parseInt(words[1]);
        if(a_id >= 0) {
            solution.add_action_to_move(timestep, a_id, dest_vertex);
            //TEST ELSE
        }
    }

    return solution;
}

function parse_solution(sol_str) {
    let lines = sol_str.split("\n");

    console.log(lines[0]);
    if(lines[0] == "Fine solution") {
        lines.splice(0, 1);
        return parse_Surynek_solution(lines);
    }
    
    lines[0] = lines[0].slice(0, 17);
    if(lines[0] == "Solution makespan") {
        lines.splice(0, 1);
        return parse_M_solution(lines);
    }

    console.log("Unknown solution format.");
}
