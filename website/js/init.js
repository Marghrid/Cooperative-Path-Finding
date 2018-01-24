var renderer, scene, camera, board;
var instance;

function animate() {
    requestAnimationFrame( animate );
    board.animate();
    renderer.render(scene, camera);
}

function init() {
    scene = new THREE.Scene();
    camera = new THREE.PerspectiveCamera( 60, 1, 0.1, 1000 );
    renderer = new THREE.WebGLRenderer();
    renderer.setSize( window.innerWidth*.5, window.innerWidth*.5 );
    renderer.setClearColor( 0xffffff, 1 );
    document.body.appendChild( renderer.domElement );

    camera.position.z = 120;
    camera.lookAt(new THREE.Vector3(0, 0, 0));

    board = new Board(instance);
    scene.add( board );

    var sun = new THREE.DirectionalLight();
    sun.position.z = 130;
    sun.position.x = 10;
    sun.position.y = 10;
    scene.add( sun );

    animate();
}

function refresh_button() {
    // Read files:
    var cpf_file      = document.getElementById('cpf_file').files[0];
    var solution_file = document.getElementById('solution_file').files[0];

    var cpf, solution;
    var cpf_reader = new FileReader();
    var sol_reader = new FileReader();

    cpf_reader.onload = function(e) {
        cpf = cpf_reader.result;
        //console.log("CPF:");
        //console.log(cpf);
        sol_reader.onload = function(e) {
            solution = sol_reader.result;
            //console.log("Solution:");
            //console.log(solution);
            instance = parse(cpf, solution);
            init();
        }
        sol_reader.readAsText(solution_file);
    }
    cpf_reader.readAsText(cpf_file);
}