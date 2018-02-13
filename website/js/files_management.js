let cpf_file_change = false;
let sol_file_change = false;

let showing_cpf = false;

function update_cpf_file_flag() {
    cpf_file_change = true;
    sol_file_change = false;
}

function update_sol_file_flag() {
    sol_file_change = true;
}

function start_reading_cpf(cpf_file) {
	let cpf;
    let cpf_reader = new FileReader();
    cpf_reader.onload = function(e) {
        // cpf is a string with the file's contents
        cpf = cpf_reader.result;
        show(cpf);
        showing_cpf = true;
        let show_solution_button = document.getElementById('show_solution_button');
        show_solution_button.style.visibility = "visible";
        show_solution_button.disabled = true;
    }
    cpf_reader.readAsText(cpf_file);
}

function start_reading_sol(sol_file) {
    let solution;
    let sol_reader = new FileReader();
    sol_reader.onload = function(e) {
        // solution is a string with the file's contents
        solution = sol_reader.result;

        if(solve(solution)) {
            scene.board.animation_controller.load_from_solution(scene.solution);
            let show_solution_button = document.getElementById('show_solution_button');
            show_solution_button.disabled = false;
        }
    }
    sol_reader.readAsText(sol_file);
}

function refresh_button() {
    if(cpf_file_change) {
        cpf_file_change = false;
        let cpf_file = document.getElementById('cpf_file').files[0];
        start_reading_cpf(cpf_file);
    }

    if(sol_file_change) {
        sol_file_change = false;
        let solution_file = document.getElementById('sol_file').files[0];
        start_reading_sol(solution_file);
    }

}

function solve_button_onclick() {
    console.log("Start solving!");
    run_solution();
}

function example1_button() {
    get_example_file("grid_4x4_a4_o0.3_s31");
}

function example2_button() {
    get_example_file("grid_4x4_a6_o0.1_s616");
}

function example3_button() {
    get_example_file("grid_8x8_a16_o0.1_s31");
}

function example4_button() {
    get_example_file("subway");
}

function example5_button() {
    get_example_file("untitled");
}

function example6_button() {
    get_example_file("grid_16x16_a25_o0.1_s5");
}

function get_example_file(ex_name) {
    let cpf_blob = null;
    let sol_blob = null;
    let cpf_request = new XMLHttpRequest();
    let sol_request = new XMLHttpRequest();

    cpf_request.open("GET", "examples/" + ex_name + ".cpf");
    sol_request.open("GET", "examples/" + ex_name + ".out");

    cpf_request.responseType = "blob";//force the HTTP response, response-type header to be blob
    sol_request.responseType = "blob";//force the HTTP response, response-type header to be blob

    cpf_request.onload = function() {
        cpf_blob = cpf_request.response;//cpf_request.response is now a blob object
        start_reading_cpf(cpf_blob);

        sol_request.onload = function() {
            sol_blob = sol_request.response;//sol_request.response is now a blob object
            start_reading_sol(sol_blob);

        }
        sol_request.send();

    }
    cpf_request.send();
}