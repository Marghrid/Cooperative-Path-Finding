// Materials for board's Squares3D. Even and odd squares, respectively;
let board_material_e = new THREE.MeshPhongMaterial({color:0x353535});
let board_material_o = new THREE.MeshPhongMaterial({color:0x252525});

// Materials for Agents3D and their Goals3D.
let agent_material = new THREE.MeshPhongMaterial({color:0xff0000});
let goal_material  = new THREE.MeshPhongMaterial({color:0xff0000, emissive:0xffffff, specular: 0x000000});