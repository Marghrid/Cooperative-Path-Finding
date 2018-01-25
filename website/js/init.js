var renderer, scene, camera, board;
var instance;

function animate() {
    requestAnimationFrame( animate );
    board.animate();
    renderer.render(scene, camera);
}

function init() {
    if(renderer != null) document.body.removeChild(renderer.domElement);
    scene = new THREE.Scene();
    camera = new THREE.PerspectiveCamera( 60, 1, 0.1, 1000 );
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize( window.innerHeight*.8, window.innerHeight*.8 );
    renderer.setClearColor( 0xffffff, 1 );
    document.body.appendChild( renderer.domElement );

    camera.position.z = CAMERA_Z;
    camera.lookAt(new THREE.Vector3(0, 0, 0));

    board = new Board3D(instance);
    scene.add( board );

    var sun = new THREE.DirectionalLight();
    sun.position.z = 130;
    sun.position.x = 10;
    sun.position.y = 10;
    scene.add( sun );

    animate();
}

function start(cpf_file, solution_file) {
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

function refresh_button() {
    // Read files:
    var cpf_file      = document.getElementById('cpf_file').files[0];
    var solution_file = document.getElementById('solution_file').files[0];

    start(cpf_file, solution_file);
}

function example1_button() {
    get_example_file("grid_4x4_r6_5");
}
function example2_button() {
    get_example_file("grid_8x8_r6_5");
}

function example3_button() {
    get_example_file("grid_8x8_r6_895");
}

function example4_button() {
    get_example_file("grid_16x16_r25_5");
}

function get_example_file(ex_name) {
    var cpf_blob = null;
    var sol_blob = null;
    var cpf_request = new XMLHttpRequest();
    var sol_request = new XMLHttpRequest();

    cpf_request.open("GET", "examples/" + ex_name + ".cpf");
    sol_request.open("GET", "examples/" + ex_name + ".out");

    cpf_request.responseType = "blob";//force the HTTP response, response-type header to be blob
    sol_request.responseType = "blob";//force the HTTP response, response-type header to be blob

    cpf_request.onload = function() {
        cpf_blob = cpf_request.response;//cpf_request.response is now a blob object

        sol_request.onload = function() {
            sol_blob = sol_request.response;//sol_request.response is now a blob object

            start(cpf_blob, sol_blob);

        }
        sol_request.send();

    }
    cpf_request.send();
}