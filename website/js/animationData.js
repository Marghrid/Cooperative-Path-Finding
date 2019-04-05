//Contains animation data of an Object3D (only 2D position)


function get_jump_pos(t){
	return -4*t*(t-1);
}

class AnimationData {
	
	//position_array must be array of THREE.Vector2.
	// contains one position per frame, starting at 0.
	constructor(pos_array, base_pos_z){
		this.position_array = pos_array;
		this.base_pos_z = base_pos_z;


		this.interpolation_type = 2; //0 means linear, 1 means smoothstep (first order derivatives = 0 at edges), 2 means smootheststep
									 // (second order derivatives = 0 at edges);
				
		this.jumping = false;
		this.jump_height = 2; //in radiuses
		this.n_jumps = 1; //number of jumps per timestep
	}

	find_weight_by_interpolation(t){
		switch(this.interpolation_type) {
		case 0:
				return t;
		case 1:
			return t * t * (t*(-2) + 3);
		case 2:
		default:
			return t * t * t * (t * (t * 6 - 15) + 10); 

		}	
	}

	interpolate_linear(v1, v2, alpha){
		let x = v1.x*(1-alpha) + v2.x*(alpha);
		let y = v1.y*(1-alpha) + v2.y*(alpha);

		return new THREE.Vector2(x,y);
	}

	//returns Vector3, becausewe procedurally add movement in z axis

	get_frame(frame){
		let pos2D = this.get_frame_2D(frame);
		let z = this.base_pos_z;

		if(this.jumping == true)
			z += this.jump_height*this.base_pos_z*get_jump_pos(frame*this.n_jumps - Math.floor(frame*this.n_jumps));

		return new THREE.Vector3(pos2D.x, pos2D.y, z);
	}

	get_frame_2D(frame){
		if (this.position_array.length == 0){
			console.log("WARNING: AnimationData not set!");
			return new Vector2(0,0);
		}
		let base = Math.floor(frame);
		let alpha = frame - base;
		alpha = this.find_weight_by_interpolation(alpha);

		if(base < 0){
			return this.position_array[0];
		}
		if(base >= this.position_array.length-1){
			return this.position_array[this.position_array.length-1];
		}

		

		//let pos = this.position_array[base].lerp(this.position_array[base+1], alpha);
		let pos = this.interpolate_linear(this.position_array[base], this.position_array[base+1], alpha);
		return pos;
	}



}
