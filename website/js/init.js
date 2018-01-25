let renderer, scene, camera, clock;

function animate() {
    requestAnimationFrame( animate );
    let dt = clock.getDelta();
    scene.board.animate(dt);
    renderer.render(scene, camera);
}

function init(cpf) {
    if(renderer != null)
    	document.body.removeChild(renderer.domElement);
    // So the boards won't stack up next to each other.

    let instance = parse(cpf);

    scene = new THREE.Scene();
    camera = new THREE.PerspectiveCamera( 60, 1, 0.1, 1000 );
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize( window.innerHeight*.8, window.innerHeight*.8 );
    renderer.setClearColor( 0xffffff, 1 ); //So the backgroun´d is white
    document.body.appendChild( renderer.domElement );

    camera.position.z = CAMERA_Z;
    camera.lookAt(new THREE.Vector3(0, 0, 0));

    scene.board = new Board3D(instance);
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