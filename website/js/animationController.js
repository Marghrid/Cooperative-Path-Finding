//controls all animation in the scene


class AnimationController{
	constructor(timescale){
		this.timescale = timescale; //frames per second
		this.targets = [];
		this.current_frame = 0;
		this.playing = true;
	}

	update(dt){
		console.log(this.current_frame);
		for (let i = 0; i<this.targets.length; i++){
			let pos = this.targets[i].animation_data.get_frame(this.current_frame);
			this.targets[i].position.x = pos.x;
			this.targets[i].position.y = pos.y;
			this.targets[i].position.z = pos.z;
		}

		if(this.playing == true){
			this.current_frame +=dt*this.timescale;
		}
	}

	set_targets_at_frame(frame){
		this.current_frame = frame;
		this.update();
	}

	set_targets_at_time(time){
		set_target_at_frame(time*this.timescale);
	}


	load_from_solution(solution){
		for(let i = 0; i<this.targets.length; i++){
			//isto e feio. gostava de ter as posicoes originais noutro sitio
			let start_vert = scene.board.instance.agents[i].start_pos;
			let start_pos = scene.board.squares3D[start_vert].position;
			let pos_array = [new THREE.Vector2(start_pos.x, start_pos.y)];

			//isto e feio tambem. Devia usar funcoes e nao quadrados?
			for(let j = 0; j<solution.moves.length; j++){
				if (typeof solution.moves[j].actions[i] != 'undefined'){

					let vert = solution.moves[j].actions[i].dest;
					console.log("timestep= " + j + ", agent = " + i);
					console.log(vert);
					if (vert != -1){
						let pos = scene.board.squares3D[vert].position;
						pos_array.push(new THREE.Vector2(pos.x, pos.y));
					}
					else {
						pos_array.push(new THREE.Vector2(pos_array[j].x, pos_array[j].y));
					}
				} else {
					pos_array.push(new THREE.Vector2(pos_array[j].x, pos_array[j].y));
				}
			}

			this.targets[i].animation_data = new AnimationData(pos_array, scene.board.agents3d[i].base_pos_z);
		}
	}
}