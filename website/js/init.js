let renderer, scene, camera, clock;

function animate() {
    requestAnimationFrame( animate );
    let dt = clock.getDelta();
    if(scene.run_solution) {
        scene.board.animate_with_solution(dt);
    } else {
        scene.board.animate(dt);
    }
    renderer.render(scene, camera);
}

function solve(solution) {
    if(scene.instance == null)
        console.log("No instance!");

    scene.solution = parse_solution(solution);
    return scene.solution.check(scene.instance);
}

function run_solution() {
    scene.run_solution = true;
}

// show: given a string with an instance, it will parse the string,
//  build the instance, and show it.
function show(cpf) {
    if(renderer != null)
    	document.body.removeChild(renderer.domElement);
    // So the boards won't stack up next to each other.

    scene = new THREE.Scene();

    scene.run_solution = false;
    scene.instance = parse_cpf(cpf);

    camera = new THREE.PerspectiveCamera( 60, 1, 0.1, 1000 );
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize( window.innerHeight*.8, window.innerHeight*.8 );
    renderer.setClearColor( 0xffffff, 1 ); //So the backgroun´d is white
    document.body.appendChild( renderer.domElement );

    camera.position.z = CAMERA_Z;
    camera.position.x = 40;
    camera.position.y = 40;
    camera.up.set(0, 0, 1);
    camera.lookAt(new THREE.Vector3(0, 0, -BOARD_LENGTH/16));

    scene.board = new Board3D(scene.instance);
    scene.add( scene.board );

    let sun = new THREE.DirectionalLight();
    sun.position.z = 130;
    sun.position.x = 10;
    sun.position.y = 10;
    scene.add( sun );

    clock = new THREE.Clock();
    clock.start();
    animate();
}