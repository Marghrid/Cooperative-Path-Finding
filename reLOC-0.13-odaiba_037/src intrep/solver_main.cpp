/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                             reLOC 0.13-odaiba                              */
/*                                                                            */
/*                      (C) Copyright 2016 Pavel Surynek                      */
/*                http://www.surynek.com | <pavel@surynek.com>                */
/*                                                                            */
/*                                                                            */
/*============================================================================*/
/* solver_main.cpp / 0.13-odaiba_037                                          */
/*----------------------------------------------------------------------------*/
//
// Solution generator - main program.
//
// Solves a given multirobot instance by the SAT solving technique from scratch.
// SATPlan iterative solution length increasing is adopted.
//
//
/*----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <sys/times.h>
#include <unistd.h>

#include "config.h"
#include "compile.h"
#include "defs.h"
#include "reloc.h"
#include "multirobot.h"
#include "compress.h"
#include "statistics.h"
#include "version.h"
#include "search.h"

#include "solver_main.h"


using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{


/*----------------------------------------------------------------------------*/

	sCommandParameters::sCommandParameters()
	: m_base_strategy(sCommandParameters::STRATEGY_LINEAR_UP)
	, m_completion(sCommandParameters::COMPLETION_SIMULTANEOUS)
	, m_cnf_encoding(sMultirobotSolutionCompressor::ENCODING_UNDEFINED)
	, m_makespan_upper_bound(8)
	, m_layer_upper_bound(4)
	, m_total_cost_bound(64)
	, m_minisat_timeout(-1)
	, m_total_timeout(600)
	, m_independence_detection(false)
	, m_avoidance_detection(false)	
	{
      // nothing
	}


/*----------------------------------------------------------------------------*/

	void print_IntroductoryMessage(void)
	{
		printf("================================================================\n");
		printf("%s : Multirobot Solver\n", sPRODUCT);
		printf("%s\n", sCOPYRIGHT);
		printf("----------------------------------------------------------------\n");
	}


	void print_ConcludingMessage(void)
	{
		printf("----------------------------------------------------------------\n");
	}


	void print_Help(void)
	{
		printf("Usage:\n");
		printf("solver_reLOC  --input-file=<string>\n");
		printf("             [--bgu-input=<string>]\n");
		printf("             [--output-file=<string>]\n");
		printf("             [--graphrec-file=<string>]\n");	
		printf("             [--total-timeout=<int>]\n");
		printf("             [--minisat-timeout=<int>]\n");
		printf("             [--pddl-domain-file=<string>]\n");
		printf("             [--pddl-problem-file=<string>]\n");
		printf("             [--encoding={inverse|advanced|differential|bijection|Hadvanced|Hdifferential|Hbijection|\n");
		printf("                          bitwise|flow|matching|Hmatching|direct|Hdirect|simplicial|Hsimplicial|\n");
		printf("                          singular|plural|plural2|heighted|mddnomdd|decomposed|independent|\n");
		printf("                          mdd|mdd+|mdd++|mmdd|mmdd+|mmdd++|rmdd|rmmdd\n");
		printf("                          idmdd|idmdd+|idmdd++|idmmdd|idmmdd+|idmmdd++|\n");
		printf("                          admdd|admdd+|admdd++|admmdd|admmdd+|admmdd++}]\n");
		printf("             [--strategy={linear-down|linear-up|binary}]\n");
		printf("             [--completion={simultaneous|unirobot|whca}]\n");
		printf("             [--makespan-limit=<int>]\n");
		printf("             [--layer-limit=<int>]\n");
		printf("\n");
		printf("Examples:\n");
		printf("solver_reLOC --input-file=grid_02.txt\n");
		printf("             --output-file=grid_02.out\n");
		printf("\n");
		printf("Defaults: --minisat-timeout=-1 (unlimited)\n");
		printf("          --strategy=linear-up\n");
		printf("          --encoding=inverse\n");	
		printf("          --total-timeout=600\n");
		printf("          --makespan-limit=8\n");
		printf("          --layer-limit=8\n");
		printf("          --completion=simultaneous\n");
	}


	sResult solve_MultirobotInstance_SAT(const sCommandParameters &command_parameters)
	{
		sResult result;
		sUndirectedGraph environment;

		sRobotArrangement initial_arrangement;
		sRobotArrangement goal_arrangement;
		sRobotGoal robot_goal;

		if (!command_parameters.m_input_filename.empty())
		{
			printf("Reading graph...\n");

			result  = environment.from_File_multirobot(command_parameters.m_input_filename);
			if (sFAILED(result))
			{
				printf("Error: Reading graph from file %s failed (code = %d).\n", command_parameters.m_input_filename.c_str(), result);
				return result;
			}

			printf("Reading graph... FINISHED\n");
		}

		if (!command_parameters.m_input_filename.empty())
		{
			printf("Reading initial arrangement...\n");
			result = initial_arrangement.from_File_multirobot(command_parameters.m_input_filename);
			if (sFAILED(result))
			{
				printf("Error: Reading arrangement from file %s failed (code = %d).\n", command_parameters.m_input_filename.c_str(), result);
				return result;
			}
			printf("Reading initial arrangement... FINISHED.\n");
		}

		if (!command_parameters.m_input_filename.empty())
		{
			printf("Reading goal arrangement...\n");

			result = goal_arrangement.from_File_multirobot(command_parameters.m_input_filename, 2);
			if (sFAILED(result))
			{
				printf("Error: Reading arrangement from file %s failed (code = %d).\n", command_parameters.m_input_filename.c_str(), result);
				return result;
			}

			result = robot_goal.from_File_multirobot(command_parameters.m_input_filename, 2);
			if (sFAILED(result))
			{
				printf("Error: Reading arrangement from file %s failed (code = %d).\n", command_parameters.m_input_filename.c_str(), result);
				return result;
			}
			printf("Reading goal arrangement... FINISHED.\n");
		}
	/*
	original_solution.execute_Solution(initial_arrangement, goal_arrangement);
	*/

	//	sMultirobotInstance instance(environment, initial_arrangement, goal_arrangement);

//	printf("Goal\n");
//	robot_goal.to_Screen();
//	printf("Arrangement\n");
//	goal_arrangement.to_Screen();

		printf("Building instance...\n");
		sMultirobotInstance instance(environment, initial_arrangement, robot_goal);
		sMultirobotInstance instance_whca(environment, initial_arrangement, goal_arrangement);
		printf("Building instance... FINISHED.\n");

		if (!command_parameters.m_bgu_input.empty())
		{
			sResult result = instance.from_File_bgu(command_parameters.m_bgu_input);
			if (sFAILED(result))
			{
				printf("Error: Reading instance from BGU file %s failed (code = %d).\n", command_parameters.m_input_filename.c_str(), result);
				return result;
			}
			environment = instance.m_environment;
			initial_arrangement = instance.m_initial_arrangement;
			goal_arrangement = instance.m_goal_arrangement;
			robot_goal = instance.m_goal_specification;
		}

		if (!command_parameters.m_pddl_problem_filename.empty())
		{
			result = instance.to_File_problemPDDL(command_parameters.m_pddl_problem_filename);

			if (sFAILED(result))
			{
				printf("Error: Failed to write PDDL problem file %s (code = %d).\n", command_parameters.m_pddl_problem_filename.c_str(), result);
				return result;
			}
		}
		if (!command_parameters.m_pddl_domain_filename.empty())
		{
			result = instance.to_File_domainPDDL(command_parameters.m_pddl_domain_filename);

			if (sFAILED(result))
			{
				printf("Error: Failed to write PDDL domain file %s (code = %d).\n", command_parameters.m_pddl_domain_filename.c_str(), result);
				return result;
			}
		}

	#ifdef sVERBOSE
		{
/*
	    environment.to_Screen();
	    initial_arrangement.to_Screen();
	    goal_arrangement.to_Screen();
	    robot_goal.to_Screen();
*/
		}
	#endif


		if (!command_parameters.m_output_filename.empty())
		{
			int optimal_makespan, optimal_cost;
			sMultirobotSolution optimal_solution;
			
			if (command_parameters.m_completion == sCommandParameters::COMPLETION_UNIROBOT)
			{
				sASSERT(command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_PLURAL2 || command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_PLURAL)

				sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
					command_parameters.m_minisat_timeout,
					command_parameters.m_total_timeout,
					command_parameters.m_makespan_upper_bound,
					sDEFAULT_N_PARALLEL_THREADS,
					command_parameters.m_cnf_encoding);	    
				
				result = compressor.compute_UnirobotsSolution(initial_arrangement,
					robot_goal,
					environment,
					instance.m_sparse_environment,
					command_parameters.m_layer_upper_bound,
					optimal_makespan,
					optimal_solution);

				printf("Computed UNIROBOT makespan:%d\n", optimal_makespan);
				printf("Computed UNIROBOT solution:\n");
				optimal_solution.to_Screen();
				optimal_solution.to_File_multirobot(command_parameters.m_output_filename.c_str());

				if (!command_parameters.m_graphrec_filename.empty())
				{
					optimal_solution.to_File_graphrec(command_parameters.m_graphrec_filename.c_str(), instance);
				}		
			}
			else if (command_parameters.m_completion == sCommandParameters::COMPLETION_WHCA)
			{
				sMultirobotSolver_WHCAstar solver_WHCAstar;
				sAstarHeuristic_Distance distance_heuristic;

				solver_WHCAstar.setup_Solver(&distance_heuristic, 16);
				solver_WHCAstar.setup_Instance(instance_whca);

				bool answer = solver_WHCAstar.solve_Instance(optimal_solution);

				if (!answer)
				{
					printf("Unable to provide a solution.\n");
				}
				else
				{
					printf("Computed solution:\n");
					optimal_solution.to_Screen();
					optimal_solution.to_File_multirobot(command_parameters.m_output_filename.c_str());
				}

				if (!command_parameters.m_graphrec_filename.empty())
				{
					optimal_solution.to_File_graphrec(command_parameters.m_graphrec_filename.c_str(), instance);
				}		
			}
			else if (command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_SINGULAR)
			{
				sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
					command_parameters.m_minisat_timeout,
					command_parameters.m_total_timeout,
					command_parameters.m_makespan_upper_bound,
					sDEFAULT_N_PARALLEL_THREADS,
					command_parameters.m_cnf_encoding);		    
				
				result = compressor.compute_OrtoOptimalSolution(initial_arrangement,
					robot_goal,
					environment,
					instance.m_sparse_environment,
					command_parameters.m_layer_upper_bound,
					optimal_makespan,
					optimal_solution);

				printf("Computed makespan:%d\n", optimal_makespan);
				printf("Computed solution:\n");
				optimal_solution.to_Screen();
				optimal_solution.to_File_multirobot(command_parameters.m_output_filename.c_str());

				if (!command_parameters.m_graphrec_filename.empty())
				{
					optimal_solution.to_File_graphrec(command_parameters.m_graphrec_filename.c_str(), instance);
				}		
			}
			else if (command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_PLURAL)
			{
				sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
					command_parameters.m_minisat_timeout,
					command_parameters.m_total_timeout,
					command_parameters.m_makespan_upper_bound,
					sDEFAULT_N_PARALLEL_THREADS,
					command_parameters.m_cnf_encoding);		    
				
				result = compressor.compute_OrtoOptimalSolution(initial_arrangement,
					robot_goal,
					environment,
					instance.m_sparse_environment,
					command_parameters.m_layer_upper_bound,
					optimal_makespan,
					optimal_solution);

				printf("Computed makespan:%d\n", optimal_makespan);
				printf("Computed solution:\n");
				optimal_solution.to_Screen();
				optimal_solution.to_File_multirobot(command_parameters.m_output_filename.c_str());

				if (!command_parameters.m_graphrec_filename.empty())
				{
					optimal_solution.to_File_graphrec(command_parameters.m_graphrec_filename.c_str(), instance);
				}		
			}
			else if (command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_PLURAL2)
			{
				sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
					command_parameters.m_minisat_timeout,
					command_parameters.m_total_timeout,
					command_parameters.m_makespan_upper_bound,
					sDEFAULT_N_PARALLEL_THREADS,
					command_parameters.m_cnf_encoding);		    
				
				result = compressor.compute_OrtoOptimalSolution(initial_arrangement,
					robot_goal,
					environment,
					instance.m_sparse_environment,
					command_parameters.m_layer_upper_bound,
					optimal_makespan,
					optimal_solution);

				printf("Computed makespan:%d\n", optimal_makespan);
				printf("Computed solution:\n");
				optimal_solution.to_Screen();
				optimal_solution.to_File_multirobot(command_parameters.m_output_filename.c_str());
				
				if (!command_parameters.m_graphrec_filename.empty())
				{
					optimal_solution.to_File_graphrec(command_parameters.m_graphrec_filename.c_str(), instance);
				}		
			}
			else if (command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_HEIGHTED)
			{
				sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
					command_parameters.m_minisat_timeout,
					command_parameters.m_total_timeout,
					command_parameters.m_makespan_upper_bound,
					sDEFAULT_N_PARALLEL_THREADS,
					command_parameters.m_cnf_encoding);		    
				
				result = compressor.compute_CostOptimalSolution(initial_arrangement,
					robot_goal,
					environment,
					instance.m_sparse_environment,
					instance.m_heighted_Environments,
					command_parameters.m_total_cost_bound,
					optimal_cost,
					optimal_solution);

				printf("Computed makespan:%d\n", optimal_makespan);
				printf("Computed solution:\n");
				optimal_solution.to_Screen();
				optimal_solution.to_File_multirobot(command_parameters.m_output_filename.c_str());
				
				if (!command_parameters.m_graphrec_filename.empty())
				{
					optimal_solution.to_File_graphrec(command_parameters.m_graphrec_filename.c_str(), instance);
				}
			}
			else if (command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_MDD)
			{
				sMultirobotInstance::MDD_vector MDD;

				sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
					command_parameters.m_minisat_timeout,
					command_parameters.m_total_timeout,
					command_parameters.m_makespan_upper_bound,
					sDEFAULT_N_PARALLEL_THREADS,
					command_parameters.m_cnf_encoding);		    
				
				result = compressor.compute_CostOptimalSolution(initial_arrangement,
					robot_goal,
					environment,
					instance.m_sparse_environment,
					MDD,
					command_parameters.m_total_cost_bound,
					optimal_cost,
					optimal_solution);

				printf("Computed sum of costs:%d\n", optimal_cost);
				printf("Computed solution:\n");
				optimal_solution.to_Screen();
				optimal_solution.to_File_multirobot(command_parameters.m_output_filename.c_str());

				if (!command_parameters.m_graphrec_filename.empty())
				{
					optimal_solution.to_File_graphrec(command_parameters.m_graphrec_filename.c_str(), instance);
				}
			}
			else if (command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_RELAXED_MDD)
			{
				sMultirobotInstance::MDD_vector MDD;

				sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
					command_parameters.m_minisat_timeout,
					command_parameters.m_total_timeout,
					command_parameters.m_makespan_upper_bound,
					sDEFAULT_N_PARALLEL_THREADS,
					command_parameters.m_cnf_encoding);		    
				
				result = compressor.compute_CostOptimalSolution(initial_arrangement,
					robot_goal,
					environment,
					instance.m_sparse_environment,
					MDD,
					command_parameters.m_total_cost_bound,
					optimal_cost,
					optimal_solution);

				printf("Computed sum of costs:%d\n", optimal_cost);
				printf("Computed solution:\n");
				optimal_solution.to_Screen();
				optimal_solution.to_File_multirobot(command_parameters.m_output_filename.c_str());

				if (!command_parameters.m_graphrec_filename.empty())
				{
					optimal_solution.to_File_graphrec(command_parameters.m_graphrec_filename.c_str(), instance);
				}
			}	    
			else if (command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_MDD_plus)
			{	
				sMultirobotInstance::MDD_vector MDD;

				sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
					command_parameters.m_minisat_timeout,
					command_parameters.m_total_timeout,
					command_parameters.m_makespan_upper_bound,
					sDEFAULT_N_PARALLEL_THREADS,
					command_parameters.m_cnf_encoding);  
				
				result = compressor.compute_CostOptimalSolution(initial_arrangement,
					robot_goal,
					environment,
					instance.m_sparse_environment,
					MDD,
					command_parameters.m_total_cost_bound,
					optimal_cost,
					optimal_solution);

				printf("Computed sum of costs:%d\n", optimal_cost);
				printf("Computed solution:\n");
				optimal_solution.to_Screen();
				optimal_solution.to_File_multirobot(command_parameters.m_output_filename.c_str());

				if (!command_parameters.m_graphrec_filename.empty())
				{
					optimal_solution.to_File_graphrec(command_parameters.m_graphrec_filename.c_str(), instance);
				}
			}
			else if (command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_MDD_plus_plus)
			{	
				sMultirobotInstance::MDD_vector MDD;

				sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
					command_parameters.m_minisat_timeout,
					command_parameters.m_total_timeout,
					command_parameters.m_makespan_upper_bound,
					sDEFAULT_N_PARALLEL_THREADS,
					command_parameters.m_cnf_encoding);  
				
				result = compressor.compute_CostOptimalSolution(initial_arrangement,
					robot_goal,
					environment,
					instance.m_sparse_environment,
					MDD,
					command_parameters.m_total_cost_bound,
					optimal_cost,
					optimal_solution);

				printf("Computed sum of costs:%d\n", optimal_cost);
				printf("Computed solution:\n");
				optimal_solution.to_Screen();
				optimal_solution.to_File_multirobot(command_parameters.m_output_filename.c_str());

				if (!command_parameters.m_graphrec_filename.empty())
				{
					optimal_solution.to_File_graphrec(command_parameters.m_graphrec_filename.c_str(), instance);
				}
			}	    
			else if (command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_IDMDD)
			{
				sMultirobotInstance::MDD_vector MDD;

				sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
					command_parameters.m_minisat_timeout,
					command_parameters.m_total_timeout,
					command_parameters.m_makespan_upper_bound,
					sDEFAULT_N_PARALLEL_THREADS,
					command_parameters.m_cnf_encoding);		    
				
				result = compressor.compute_CostOptimalSolutionID(initial_arrangement,
					robot_goal,
					environment,
					instance.m_sparse_environment,
					MDD,
					command_parameters.m_total_cost_bound,
					optimal_cost,
					optimal_solution);

				printf("Computed sum of costs:%d\n", optimal_cost);
				printf("Computed solution:\n");
				optimal_solution.to_Screen();
				optimal_solution.to_File_multirobot(command_parameters.m_output_filename.c_str());

				if (!command_parameters.m_graphrec_filename.empty())
				{
					optimal_solution.to_File_graphrec(command_parameters.m_graphrec_filename.c_str(), instance);
				}		
			}
			else if (command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_ADMDD)
			{
				sMultirobotInstance::MDD_vector MDD;

				sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
					command_parameters.m_minisat_timeout,
					command_parameters.m_total_timeout,
					command_parameters.m_makespan_upper_bound,
					sDEFAULT_N_PARALLEL_THREADS,
					command_parameters.m_cnf_encoding);		    
				
				result = compressor.compute_CostOptimalSolutionAD(initial_arrangement,
					robot_goal,
					environment,
					instance.m_sparse_environment,
					MDD,
					command_parameters.m_total_cost_bound,
					optimal_cost,
					optimal_solution);

				printf("Computed sum of costs:%d\n", optimal_cost);
				printf("Computed solution:\n");
				optimal_solution.to_Screen();
				optimal_solution.to_File_multirobot(command_parameters.m_output_filename.c_str());

				if (!command_parameters.m_graphrec_filename.empty())
				{
					optimal_solution.to_File_graphrec(command_parameters.m_graphrec_filename.c_str(), instance);
				}		
			}	    
			else if (   command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_IDMDD_plus
				|| command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_IDMDD_plus_plus)
			{
				sMultirobotInstance::MDD_vector MDD;

				sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
					command_parameters.m_minisat_timeout,
					command_parameters.m_total_timeout,
					command_parameters.m_makespan_upper_bound,
					sDEFAULT_N_PARALLEL_THREADS,
					command_parameters.m_cnf_encoding);		    
				
				result = compressor.compute_CostOptimalSolutionID(initial_arrangement,
					robot_goal,
					environment,
					instance.m_sparse_environment,
					MDD,
					command_parameters.m_total_cost_bound,
					optimal_cost,
					optimal_solution);

				printf("Computed sum of costs:%d\n", optimal_cost);
				printf("Computed solution:\n");
				optimal_solution.to_Screen();
				optimal_solution.to_File_multirobot(command_parameters.m_output_filename.c_str());

				if (!command_parameters.m_graphrec_filename.empty())
				{
					optimal_solution.to_File_graphrec(command_parameters.m_graphrec_filename.c_str(), instance);
				}		
			}
			else if (   command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_ADMDD_plus
				|| command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_ADMDD_plus_plus)
			{
				sMultirobotInstance::MDD_vector MDD;

				sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
					command_parameters.m_minisat_timeout,
					command_parameters.m_total_timeout,
					command_parameters.m_makespan_upper_bound,
					sDEFAULT_N_PARALLEL_THREADS,
					command_parameters.m_cnf_encoding);		    
				
				result = compressor.compute_CostOptimalSolutionAD(initial_arrangement,
					robot_goal,
					environment,
					instance.m_sparse_environment,
					MDD,
					command_parameters.m_total_cost_bound,
					optimal_cost,
					optimal_solution);

				printf("Computed sum of costs:%d\n", optimal_cost);
				printf("Computed solution:\n");
				optimal_solution.to_Screen();
				optimal_solution.to_File_multirobot(command_parameters.m_output_filename.c_str());

				if (!command_parameters.m_graphrec_filename.empty())
				{
					optimal_solution.to_File_graphrec(command_parameters.m_graphrec_filename.c_str(), instance);
				}		
			}
			else if (command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_RXMDD_BINARY)
			{
				sMultirobotInstance::MDD_vector MDD;

				sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
					command_parameters.m_minisat_timeout,
					command_parameters.m_total_timeout,
					command_parameters.m_makespan_upper_bound,
					sDEFAULT_N_PARALLEL_THREADS,
					command_parameters.m_cnf_encoding);		    
				
				result = compressor.compute_CostOptimalSolution_binary(initial_arrangement,
					robot_goal,
					environment,
					instance.m_sparse_environment,
					MDD,
					command_parameters.m_total_cost_bound,
					optimal_cost,
					optimal_solution);
				
				printf("Computed sum of costs:%d\n", optimal_cost);
				printf("Computed solution:\n");
				optimal_solution.to_Screen();
				optimal_solution.to_File_multirobot(command_parameters.m_output_filename.c_str());

				if (!command_parameters.m_graphrec_filename.empty())
				{
					optimal_solution.to_File_graphrec(command_parameters.m_graphrec_filename.c_str(), instance);
				}		
			}
			else if (command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_BMDD)
			{
				sMultirobotInstance::MDD_vector MDD;

				sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
					command_parameters.m_minisat_timeout,
					command_parameters.m_total_timeout,
					command_parameters.m_makespan_upper_bound,
					sDEFAULT_N_PARALLEL_THREADS,
					command_parameters.m_cnf_encoding);		    
				
				result = compressor.compute_BestCostSolution(initial_arrangement,
					robot_goal,
					environment,
					instance.m_sparse_environment,
					MDD,
					command_parameters.m_total_cost_bound,
					optimal_cost,
					optimal_solution);
				
				printf("Computed sum of costs:%d\n", optimal_cost);
				printf("Computed solution:\n");
				optimal_solution.to_Screen();
				optimal_solution.to_File_multirobot(command_parameters.m_output_filename.c_str());

				if (!command_parameters.m_graphrec_filename.empty())
				{
					optimal_solution.to_File_graphrec(command_parameters.m_graphrec_filename.c_str(), instance);
				}		
			}
			else if (command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_BCMDD)
			{
				sMultirobotInstance::MDD_vector MDD;

				sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
					command_parameters.m_minisat_timeout,
					command_parameters.m_total_timeout,
					command_parameters.m_makespan_upper_bound,
					sDEFAULT_N_PARALLEL_THREADS,
					command_parameters.m_cnf_encoding);		    
				
				result = compressor.compute_BestCostSolution(initial_arrangement,
					robot_goal,
					environment,
					instance.m_sparse_environment,
					MDD,
					command_parameters.m_total_cost_bound,
					optimal_cost,
					optimal_solution);
				
				printf("Computed sum of costs:%d\n", optimal_cost);
				printf("Computed solution:\n");
				optimal_solution.to_Screen();
				optimal_solution.to_File_multirobot(command_parameters.m_output_filename.c_str());

				if (!command_parameters.m_graphrec_filename.empty())
				{
					optimal_solution.to_File_graphrec(command_parameters.m_graphrec_filename.c_str(), instance);
				}		
			}
			else if (command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_BNOMDD)
			{
				sMultirobotInstance::MDD_vector MDD;

				sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
					command_parameters.m_minisat_timeout,
					command_parameters.m_total_timeout,
					command_parameters.m_makespan_upper_bound,
					sDEFAULT_N_PARALLEL_THREADS,
					command_parameters.m_cnf_encoding);		    
				
				result = compressor.compute_BestCostSolution(initial_arrangement,
					robot_goal,
					environment,
					instance.m_sparse_environment,
					MDD,
					command_parameters.m_total_cost_bound,
					optimal_cost,
					optimal_solution);
				
				printf("Computed sum of costs:%d\n", optimal_cost);
				printf("Computed solution:\n");
				optimal_solution.to_Screen();
				optimal_solution.to_File_multirobot(command_parameters.m_output_filename.c_str());

				if (!command_parameters.m_graphrec_filename.empty())
				{
					optimal_solution.to_File_graphrec(command_parameters.m_graphrec_filename.c_str(), instance);
				}		
			}
			else if (command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_BCNOMDD)
			{
				sMultirobotInstance::MDD_vector MDD;

				sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
					command_parameters.m_minisat_timeout,
					command_parameters.m_total_timeout,
					command_parameters.m_makespan_upper_bound,
					sDEFAULT_N_PARALLEL_THREADS,
					command_parameters.m_cnf_encoding);		    
				
				result = compressor.compute_BestCostSolution(initial_arrangement,
					robot_goal,
					environment,
					instance.m_sparse_environment,
					MDD,
					command_parameters.m_total_cost_bound,
					optimal_cost,
					optimal_solution);
				
				printf("Computed sum of costs:%d\n", optimal_cost);
				printf("Computed solution:\n");
				optimal_solution.to_Screen();
				optimal_solution.to_File_multirobot(command_parameters.m_output_filename.c_str());

				if (!command_parameters.m_graphrec_filename.empty())
				{
					optimal_solution.to_File_graphrec(command_parameters.m_graphrec_filename.c_str(), instance);
				}		
			}
			else if (command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_NOMDD)
			{
				sMultirobotInstance::MDD_vector MDD;

				sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
					command_parameters.m_minisat_timeout,
					command_parameters.m_total_timeout,
					command_parameters.m_makespan_upper_bound,
					sDEFAULT_N_PARALLEL_THREADS,
					command_parameters.m_cnf_encoding);		    
				
				result = compressor.compute_CostOptimalSolution(initial_arrangement,
					robot_goal,
					environment,
					instance.m_sparse_environment,
					MDD,
					command_parameters.m_total_cost_bound,
					optimal_cost,
					optimal_solution);

				printf("Computed sum of costs:%d\n", optimal_cost);
				printf("Computed solution:\n");
				optimal_solution.to_Screen();
				optimal_solution.to_File_multirobot(command_parameters.m_output_filename.c_str());

				if (!command_parameters.m_graphrec_filename.empty())
				{
					optimal_solution.to_File_graphrec(command_parameters.m_graphrec_filename.c_str(), instance);
				}		
			}
			else
			{
				if (command_parameters.m_cnf_encoding != sMultirobotSolutionCompressor::ENCODING_UNDEFINED)
				{
					if (command_parameters.m_cnf_encoding != sMultirobotSolutionCompressor::ENCODING_UNUSED)
					{
						sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
							command_parameters.m_minisat_timeout,
							command_parameters.m_total_timeout,
							command_parameters.m_makespan_upper_bound,
							sDEFAULT_N_PARALLEL_THREADS,
							command_parameters.m_cnf_encoding);		    
						switch (command_parameters.m_base_strategy)
						{
							case sCommandParameters::STRATEGY_LINEAR_DOWN:
							{
								result = compressor.compute_OptimalSolution(initial_arrangement,
									robot_goal,
									environment,
									instance.m_sparse_environment,
									command_parameters.m_makespan_upper_bound,
									optimal_makespan,
									optimal_solution);
								break;
							}
							case sCommandParameters::STRATEGY_LINEAR_UP:
							{
								if (command_parameters.m_independence_detection)
								{
									result = compressor.compute_MakespanOptimalSolutionID(initial_arrangement,
										robot_goal,
										environment,
										instance.m_sparse_environment,
										command_parameters.m_makespan_upper_bound,
										optimal_makespan,
										optimal_solution);
								}
								else if (command_parameters.m_avoidance_detection)
								{
									result = compressor.compute_MakespanOptimalSolutionAD(initial_arrangement,
										robot_goal,
										environment,
										instance.m_sparse_environment,
										command_parameters.m_makespan_upper_bound,
										optimal_makespan,
										optimal_solution);
								}
								else
								{
									result = compressor.compute_OptimalSolution_(initial_arrangement,
										robot_goal,
										environment,
										instance.m_sparse_environment,
										command_parameters.m_makespan_upper_bound,
										optimal_makespan,
										optimal_solution);
								}
								break;
							}
							case sCommandParameters::STRATEGY_BINARY:
							{
								result = compressor.compute_OptimalSolution(initial_arrangement,
									robot_goal,
									environment,
									instance.m_sparse_environment,
									2,
									command_parameters.m_makespan_upper_bound,
									optimal_makespan,
									optimal_solution);
								
								break;
							}
							default:
							{
								sASSERT(false);
							}
						}
						
						if (sFAILED(result))
						{
							printf("Error: Solution optimization failed (code = %d).\n", result);
							return result;
						}
						printf("Computed optimal makespan:%d\n", optimal_makespan);
						printf("Makespan optimal solution:\n");
						optimal_solution.to_Screen();
						optimal_solution.to_File_multirobot(command_parameters.m_output_filename.c_str());

						if (!command_parameters.m_graphrec_filename.empty())
						{
							optimal_solution.to_File_graphrec(command_parameters.m_graphrec_filename.c_str(), instance);
						}			
					}
					else
					{
						sMultirobotSolver_DecomposedAstar solver_Astar(command_parameters.m_total_timeout);
						sAstarHeuristic_Distance distance_heuristic;
						
						solver_Astar.setup_Solver(&distance_heuristic);
						solver_Astar.setup_Instance(sMultirobotInstance(environment, initial_arrangement, robot_goal));
						
						switch (solver_Astar.solve_Instance(optimal_solution))
						{
							case sMultirobotSolver_DecomposedAstar::RESULT_SOLVABLE:
							{
								optimal_makespan = optimal_solution.get_StepCount();
								
								printf("Computed optimal makespan:%d\n", optimal_makespan);
								printf("Makespan optimal solution:\n");
								optimal_solution.to_Screen();
								optimal_solution.to_File_multirobot(command_parameters.m_output_filename.c_str());

								if (!command_parameters.m_graphrec_filename.empty())
								{
									optimal_solution.to_File_graphrec(command_parameters.m_graphrec_filename.c_str(), instance);
								}			    
								break;
							}
							case sMultirobotSolver_DecomposedAstar::RESULT_UNSOLVABLE:
							{
								printf("Unable to find solution.\n");
								break;
							}
							case sMultirobotSolver_DecomposedAstar::RESULT_INDETERMINATE:
							{
								printf("Cannot decide existence of solution..\n");
								break;
							}
							default:
							{
								sASSERT(false);
							}
						}
					}
				}
				else
				{
					sMultirobotSolver_DecomposedAstar solver_Astar(command_parameters.m_total_timeout);
					sAstarHeuristic_Distance distance_heuristic;
					
					solver_Astar.setup_Solver(&distance_heuristic);
					solver_Astar.setup_Instance(sMultirobotInstance(environment, initial_arrangement, robot_goal));
					
					switch (solver_Astar.solve_DecomposedInstance(optimal_solution))
					{
						case sMultirobotSolver_DecomposedAstar::RESULT_SOLVABLE:
						{
							optimal_makespan = optimal_solution.get_StepCount();
							
							printf("Computed optimal makespan:%d\n", optimal_makespan);
							printf("Makespan optimal solution:\n");
							optimal_solution.to_Screen();
							optimal_solution.to_File_multirobot(command_parameters.m_output_filename.c_str());

							if (!command_parameters.m_graphrec_filename.empty())
							{
								optimal_solution.to_File_graphrec(command_parameters.m_graphrec_filename.c_str(), instance);
							}			
							break;
						}
						case sMultirobotSolver_DecomposedAstar::RESULT_UNSOLVABLE:
						{
							printf("Unable to find solution.\n");
							break;
						}
						case sMultirobotSolver_DecomposedAstar::RESULT_INDETERMINATE:
						{
							printf("Cannot decide existence of solution..\n");
							break;
						}
						default:
						{
							sASSERT(false);
						}
					}
				}
			}
			sMultirobotSolutionAnalyzer solution_analyzer;
			solution_analyzer.analyze_Solution(optimal_solution, initial_arrangement, environment);
			solution_analyzer.to_Screen();
		}
		s_GlobalPhaseStatistics.to_Screen();

		return sRESULT_SUCCESS;
	}


	sResult parse_CommandLineParameter(const sString &parameter, sCommandParameters &command_parameters)
	{
		if (parameter.find("--input-file=") == 0)
		{
			command_parameters.m_input_filename = parameter.substr(13, parameter.size());
		}
		else if (parameter.find("--bgu-input=") == 0)
		{
			command_parameters.m_bgu_input = parameter.substr(12, parameter.size());
		}
		else if (parameter.find("--output-file=") == 0)
		{
			command_parameters.m_output_filename = parameter.substr(14, parameter.size());
		}
		else if (parameter.find("--graphrec-file=") == 0)
		{
			command_parameters.m_graphrec_filename = parameter.substr(16, parameter.size());
		}	
		else if (parameter.find("--pddl-domain-file=") == 0)
		{
			command_parameters.m_pddl_domain_filename = parameter.substr(19, parameter.size());
		}
		else if (parameter.find("--pddl-problem-file=") == 0)
		{
			command_parameters.m_pddl_problem_filename = parameter.substr(20, parameter.size());
		}
		else if (parameter.find("--makespan-limit=") == 0)
		{
			command_parameters.m_makespan_upper_bound = sInt_32_from_String(parameter.substr(17, parameter.size()));
		}
		else if (parameter.find("--layer-limit=") == 0)
		{
			command_parameters.m_layer_upper_bound = sInt_32_from_String(parameter.substr(14, parameter.size()));
		}
		else if (parameter.find("--cost-limit=") == 0)
		{
			command_parameters.m_total_cost_bound = sInt_32_from_String(parameter.substr(13, parameter.size()));
		}
		else if (parameter.find("--encoding=") == 0)
		{
			sString encoding_str = parameter.substr(11, parameter.size());

			if (encoding_str == "inverse")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_INVERSE;
			}
			else if (encoding_str == "idinverse")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_INVERSE;
				command_parameters.m_independence_detection = true;
			}
			else if (encoding_str == "advanced")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_ADVANCED;
			}
			else if (encoding_str == "idadvanced")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_ADVANCED;
				command_parameters.m_independence_detection = true;
			}
			else if (encoding_str == "differential")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_DIFFERENTIAL;
			}
			else if (encoding_str == "iddifferential")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_DIFFERENTIAL;
				command_parameters.m_independence_detection = true;
			}
			else if (encoding_str == "bijection")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_BIJECTION;
			}
			else if (encoding_str == "idbijection")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_BIJECTION;
				command_parameters.m_independence_detection = true;
			}
			else if (encoding_str == "Hadvanced")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_HEURISTIC_ADVANCED;
			}
			else if (encoding_str == "idHadvanced")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_HEURISTIC_ADVANCED;
				command_parameters.m_independence_detection = true;
			}
			else if (encoding_str == "Hdifferential")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_HEURISTIC_DIFFERENTIAL;
			}
			else if (encoding_str == "idHdifferential")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_HEURISTIC_DIFFERENTIAL;
				command_parameters.m_independence_detection = true;
			}
			else if (encoding_str == "Hbijection")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_HEURISTIC_BIJECTION;
			}
			else if (encoding_str == "idHbijection")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_HEURISTIC_BIJECTION;
				command_parameters.m_independence_detection = true;
			}
			else if (encoding_str == "bitwise")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_BITWISE;
			}
			else if (encoding_str == "flow")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_FLOW;
			}
			else if (encoding_str == "matching")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_MATCHING;
			}
			else if (encoding_str == "idmatching")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_MATCHING;
				command_parameters.m_independence_detection = true;
			}
			else if (encoding_str == "Hmatching")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_HEURISTIC_MATCHING;
			}
			else if (encoding_str == "idHmatching")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_HEURISTIC_MATCHING;
				command_parameters.m_independence_detection = true;
			}
			else if (encoding_str == "direct")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_DIRECT;
			}
			else if (encoding_str == "iddirect")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_DIRECT;
				command_parameters.m_independence_detection = true;
			}
			else if (encoding_str == "Hdirect")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_HEURISTIC_DIRECT;
			}
			else if (encoding_str == "idHdirect")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_HEURISTIC_DIRECT;
				command_parameters.m_independence_detection = true;
			}
			else if (encoding_str == "simplicial")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_SIMPLICIAL;
			}
			else if (encoding_str == "idsimplicial")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_SIMPLICIAL;
				command_parameters.m_independence_detection = true;
			}
			else if (encoding_str == "Hsimplicial")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_HEURISTIC_SIMPLICIAL;
			}
			else if (encoding_str == "idHsimplicial")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_HEURISTIC_SIMPLICIAL;
				command_parameters.m_independence_detection = true;
			}
			else if (encoding_str == "mmdd")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_MMDD;
			}
			else if (encoding_str == "rmmdd")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_RELAXED_MMDD;
			}	    
			else if (encoding_str == "mmdd+")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_MMDD_plus;
			}
			else if (encoding_str == "mmdd++")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_MMDD_plus_plus;
			}	    	    
			else if (encoding_str == "idmmdd")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_MMDD;
				command_parameters.m_independence_detection = true;
			}
			else if (encoding_str == "idmmdd+")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_MMDD_plus;
				command_parameters.m_independence_detection = true;
			}
			else if (encoding_str == "idmmdd++")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_MMDD_plus_plus;
				command_parameters.m_independence_detection = true;
			}
			else if (encoding_str == "admmdd")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_MMDD;
				command_parameters.m_avoidance_detection = true;
			}
			else if (encoding_str == "admmdd+")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_MMDD_plus;
				command_parameters.m_avoidance_detection = true;
			}
			else if (encoding_str == "admmdd++")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_MMDD_plus_plus;
				command_parameters.m_avoidance_detection = true;
			}	    	    	    
			else if (encoding_str == "singular")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_SINGULAR;
			}
			else if (encoding_str == "plural")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_PLURAL;
			}
			else if (encoding_str == "plural2")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_PLURAL2;
			}
			else if (encoding_str == "heighted")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_HEIGHTED;
			}
			else if (encoding_str == "mdd")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_MDD;
			}
			else if (encoding_str == "rmdd")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_RELAXED_MDD;
			}	    
			else if (encoding_str == "idmdd")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_IDMDD;
				command_parameters.m_independence_detection = true;
			}
			else if (encoding_str == "admdd")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_ADMDD;
				command_parameters.m_avoidance_detection = true;
			}	    
			else if (encoding_str == "mdd+")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_MDD_plus;
			}
			else if (encoding_str == "mdd++")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_MDD_plus_plus;
			}	    
			else if (encoding_str == "idmdd+")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_IDMDD_plus;
				command_parameters.m_independence_detection = true;
			}
			else if (encoding_str == "idmdd++")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_IDMDD_plus_plus;
				command_parameters.m_independence_detection = true;
			}
			else if (encoding_str == "admdd+")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_ADMDD_plus;
				command_parameters.m_avoidance_detection = true;
			}
			else if (encoding_str == "admdd++")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_ADMDD_plus_plus;
				command_parameters.m_avoidance_detection = true;
			}	    	    	    	    
			else if (encoding_str == "rxmdd")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_RXMDD;
			}
			else if (encoding_str == "bmdd")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_BMDD;
			}
			else if (encoding_str == "bcmdd")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_BCMDD;
			}
			else if (encoding_str == "bnomdd")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_BNOMDD;
			}
			else if (encoding_str == "bcnomdd")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_BCNOMDD;
			}
			else if (encoding_str == "rxbmdd")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_RXMDD_BINARY;
			}
			else if (encoding_str == "nomdd")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_NOMDD;
			}
			else if (encoding_str == "decomposed")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_UNUSED;
			}
			else if (encoding_str == "independent")
			{
				command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_UNDEFINED;
			}
			else
			{
				return sOPTIMIZER_PROGRAM_UNRECOGNIZED_PARAMETER_ERROR;
			}
		}
		else if (parameter.find("--strategy=") == 0)
		{
			sString encoding_str = parameter.substr(11, parameter.size());

			if (encoding_str == "linear-down")
			{
				command_parameters.m_base_strategy = sCommandParameters::STRATEGY_LINEAR_DOWN;
			}
			else if (encoding_str == "linear-up")
			{
				command_parameters.m_base_strategy = sCommandParameters::STRATEGY_LINEAR_UP;
			}
			else if (encoding_str == "binary")
			{
				command_parameters.m_base_strategy = sCommandParameters::STRATEGY_BINARY;
			}
			else
			{
				return sOPTIMIZER_PROGRAM_UNRECOGNIZED_PARAMETER_ERROR;
			}
		}
		else if (parameter.find("--completion=") == 0)
		{
			sString encoding_str = parameter.substr(13, parameter.size());

			if (encoding_str == "simultaneous")
			{
				command_parameters.m_completion = sCommandParameters::COMPLETION_SIMULTANEOUS;
			}
			else if (encoding_str == "unirobot")
			{
				command_parameters.m_completion = sCommandParameters::COMPLETION_UNIROBOT;
			}
			else if (encoding_str == "whca")
			{
				command_parameters.m_completion = sCommandParameters::COMPLETION_WHCA;
			}
			else
			{
				return sOPTIMIZER_PROGRAM_UNRECOGNIZED_PARAMETER_ERROR;
			}
		}
		else if (parameter.find("--minisat-timeout=") == 0)
		{
			command_parameters.m_minisat_timeout = sInt_32_from_String(parameter.substr(18, parameter.size()));
		}
		else if (parameter.find("--total-timeout=") == 0)
		{
			command_parameters.m_total_timeout = sInt_32_from_String(parameter.substr(16, parameter.size()));
		}
		else
		{
			return sSOLVER_PROGRAM_UNRECOGNIZED_PARAMETER_ERROR;
		}
		return sRESULT_SUCCESS;
	}


/*----------------------------------------------------------------------------*/

} // namespace sReloc


/*----------------------------------------------------------------------------*/
// main program

int main(int argc, char **argv)
{
	sResult result;
	sCommandParameters command_parameters;

	print_IntroductoryMessage();

	if (argc >= 2 && argc <= 9)
	{
		for (int i = 1; i < argc; ++i)
		{
			result = parse_CommandLineParameter(argv[i], command_parameters);
			if (sFAILED(result))
			{
				printf("Error: Cannot parse command line parameters (code = %d).\n", result);
				print_Help();
				return result;
			}
		}
		result = solve_MultirobotInstance_SAT(command_parameters);

		if (sFAILED(result))
		{
			return result;
		}
	}
	else
	{
		print_Help();
	}
	print_ConcludingMessage();

	return sRESULT_SUCCESS;
}

