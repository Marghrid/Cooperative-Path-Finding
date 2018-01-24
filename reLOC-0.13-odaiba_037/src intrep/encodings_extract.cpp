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
/* encodings_extract.cpp / 0.13-odaiba_037                                    */
/*----------------------------------------------------------------------------*/
//
// Multi-robot path-finding encodings related
// procedures for solution extraction.
//
/*----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <unistd.h>

#include <map>

#include "config.h"
#include "compile.h"
#include "version.h"
#include "defs.h"
#include "types.h"
#include "result.h"
#include "cnf.h"
#include "multirobot.h"
#include "compress.h"
#include "statistics.h"


using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{


/*----------------------------------------------------------------------------*/
// sMultirobotSolutionCompressor encodings related

	sResult sMultirobotSolutionCompressor::extract_ComputedInverseSolution(const sRobotArrangement                 &start_arrangement,
		const sUndirectedGraph                  &environment,
		int                                      computed_makespan,
		const sMultirobotEncodingContext_CNFsat &final_encoding_context,
		sMultirobotSolution                     &computed_solution,
		int                                      thread_id)
	{
		sString output_filename;
		
		if (thread_id != THREAD_ID_UNDEFINED)
		{
			output_filename = OUTPUT_INVERSE_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
			output_filename = OUTPUT_INVERSE_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		FILE *fr;
		
		if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
		{
			return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
		}
		
		char answer[32];
		answer[0] = '\0';

		fscanf(fr, "%s\n", answer);
		
		sMultirobotSolution pre_solution;
		
		int transition_neighbor = 0;
		int transition_vertex_id = -1;
		int transition_step = -1;
		
		while (!feof(fr))
		{
			int literal;
			fscanf(fr, "%d", &literal); 
			
			if (literal != 0)
			{
				sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));
				
				if (!specified_identifier.is_Null())
				{
					const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();
					
					if (base_name.find("transition_action") == 0)
					{   
						int transition_state_bit = *specified_identifier.get_ScopeIndexes()[1] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[1]->get_Begin());
						
						if (transition_state_bit == 0)
						{
							if (transition_vertex_id >= 0)
							{
								int N_Neighbors = environment.get_Vertex(transition_vertex_id)->calc_NeighborCount();
								if (transition_neighbor < N_Neighbors && transition_step < computed_makespan)
								{
									sMultirobotSolution::Move move(-1, transition_vertex_id, environment.get_Vertex(transition_vertex_id)->calc_NeighborID(transition_neighbor));
									pre_solution.add_Move(transition_step, move);
								}
							}
							transition_neighbor = 0;
						}
						transition_vertex_id = sInt_32_from_String(base_name.substr(base_name.find("-") + 1).c_str());
						transition_step = *specified_identifier.get_ScopeIndexes()[0] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[0]->get_Begin());
						
						transition_neighbor += (literal > 0) ? sIndexableStateIdentifier::calc_Exp2(transition_state_bit) : 0;
					}
				}
			}
			else
			{
				break;
			}
		}
		
		fclose(fr);

	#ifndef sDEBUG
		{
			if (unlink(output_filename.c_str()) < 0)
			{
				return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
			}	
		}
	#endif
		sRobotArrangement current_arrangement = start_arrangement;
		
		int N_Steps = pre_solution.m_Steps.size();
		for (int step = 0; step < N_Steps; ++step)
		{
			const sMultirobotSolution::Moves_list &step_Moves = pre_solution.m_Steps[step].m_Moves;
			for (sMultirobotSolution::Moves_list::const_iterator move = step_Moves.begin(); move != step_Moves.end(); ++move)
			{
				int robot_id = current_arrangement.get_VertexOccupancy(move->m_src_vrtx_id);
				
				if (robot_id >= 1)
				{
					computed_solution.add_Move(step, sMultirobotSolution::Move(robot_id, move->m_src_vrtx_id, move->m_dest_vrtx_id));
					sASSERT(current_arrangement.verify_Move(robot_id, move->m_dest_vrtx_id, environment));
					current_arrangement.move_Robot(robot_id, move->m_dest_vrtx_id);
				}
			}
		}
		
		return sRESULT_SUCCESS;
	}


	sResult sMultirobotSolutionCompressor::extract_ComputedDifferentialSolution(const sRobotArrangement                 &start_arrangement,
		const sUndirectedGraph                  &sUNUSED(environment),
		int                                      computed_makespan,
		const sMultirobotEncodingContext_CNFsat &final_encoding_context,
		sMultirobotSolution                     &computed_solution,
		int                                      thread_id)
	{
		sString output_filename;
		
		if (thread_id != THREAD_ID_UNDEFINED)
		{
			output_filename = OUTPUT_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
			output_filename = OUTPUT_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		FILE *fr;
		
		if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
		{
			return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
		}
		
		char answer[32];
		answer[0] = '\0';

		fscanf(fr, "%s\n", answer);
		
		int robot_location_vertex_id = 0;
		std::vector<int> robot_Locations;
		robot_Locations.resize(start_arrangement.get_RobotCount() + 1, (const int)sRobotArrangement::UNDEFINED_LOCATION);
		
		while (!feof(fr))
		{
			int literal;
			fscanf(fr, "%d", &literal); 
			
			if (literal != 0)
			{
				sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));
				
				if (!specified_identifier.is_Null())
				{
					const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();
					
					if (base_name.find("robot_location") == 0)
					{   
						int robot_id = *specified_identifier.get_ScopeIndexes()[0] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[0]->get_Begin()) + 1;
						int layer = *specified_identifier.get_ScopeIndexes()[1] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[1]->get_Begin());
						int robot_location_bit = *specified_identifier.get_ScopeIndexes()[2] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[2]->get_Begin());
						int N_Bits = *specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[2]->get_End() - *specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[2]->get_Begin();
						robot_location_vertex_id += (literal > 0) ? sIndexableStateIdentifier::calc_Exp2(robot_location_bit) : 0;
						
						if (robot_location_bit >= N_Bits - 1)
						{
							if (robot_Locations[robot_id] != sRobotArrangement::UNDEFINED_LOCATION && robot_Locations[robot_id] != robot_location_vertex_id)
							{
								sMultirobotSolution::Move move(robot_id, robot_Locations[robot_id], robot_location_vertex_id);
								computed_solution.add_Move(layer - 1, move);
							}
							robot_Locations[robot_id] = robot_location_vertex_id;
							robot_location_vertex_id = 0;
						}			
					}
				}
			}
			else
			{
				break;
			}
		}
		
		fclose(fr);

	#ifndef sDEBUG
		{
			if (unlink(output_filename.c_str()) < 0)
			{
				return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
			}	
		}
	#endif
		
		return sRESULT_SUCCESS;
	}


	sResult sMultirobotSolutionCompressor::extract_ComputedAdvancedSolution(const sRobotArrangement                 &start_arrangement,
		const sUndirectedGraph                  &environment,
		int                                      computed_makespan,
		const sMultirobotEncodingContext_CNFsat &final_encoding_context,
		sMultirobotSolution                     &computed_solution,
		int                                      thread_id)
	{
		sString output_filename;
		
		if (thread_id != THREAD_ID_UNDEFINED)
		{
			output_filename = OUTPUT_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
			output_filename = OUTPUT_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		FILE *fr;
		
		if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
		{
			return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
		}
		
		char answer[32];
		answer[0] = '\0';

		fscanf(fr, "%s\n", answer);
		
		sMultirobotSolution pre_solution;
		
		int transition_neighbor = 0;
		int transition_vertex_id = -1;
		int transition_step = -1;
		
		while (!feof(fr))
		{
			int literal;
			fscanf(fr, "%d", &literal); 
			
			if (literal != 0)
			{
				sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));
				
				if (!specified_identifier.is_Null())
				{
					const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();
					
					if (base_name.find("transition_action") == 0)
					{   
						int transition_state_bit = *specified_identifier.get_ScopeIndexes()[1] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[1]->get_Begin());

						if (transition_state_bit == 0)
						{
							if (transition_vertex_id >= 0)
							{
								int N_Neighbors = environment.get_Vertex(transition_vertex_id)->calc_NeighborCount();
								if (transition_neighbor < N_Neighbors && transition_step < computed_makespan)
								{
									sMultirobotSolution::Move move(-1, transition_vertex_id, environment.get_Vertex(transition_vertex_id)->calc_NeighborID(transition_neighbor));
									pre_solution.add_Move(transition_step, move);
								}
							}
							transition_neighbor = 0;
						}
						transition_vertex_id = sInt_32_from_String(base_name.substr(base_name.find("-") + 1).c_str());
						transition_step = *specified_identifier.get_ScopeIndexes()[0] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[0]->get_Begin());

						transition_neighbor += (literal > 0) ? sIndexableStateIdentifier::calc_Exp2(transition_state_bit) : 0;
					}
				}
			}
			else
			{
				break;
			}
		}
		fclose(fr);

	#ifndef sDEBUG
		{
			if (unlink(output_filename.c_str()) < 0)
			{
				return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
			}	
		}
	#endif
		
		sRobotArrangement current_arrangement = start_arrangement;
		
		int N_Steps = pre_solution.m_Steps.size();
		for (int step = 0; step < N_Steps; ++step)
		{
			const sMultirobotSolution::Moves_list &step_Moves = pre_solution.m_Steps[step].m_Moves;
			for (sMultirobotSolution::Moves_list::const_iterator move = step_Moves.begin(); move != step_Moves.end(); ++move)
			{
				int robot_id = current_arrangement.get_VertexOccupancy(move->m_src_vrtx_id);
				
				if (robot_id >= 1)
				{
					computed_solution.add_Move(step, sMultirobotSolution::Move(robot_id, move->m_src_vrtx_id, move->m_dest_vrtx_id));
					sASSERT(current_arrangement.verify_Move(robot_id, move->m_dest_vrtx_id, environment));
					current_arrangement.move_Robot(robot_id, move->m_dest_vrtx_id);
				}
			}
		}
		
		return sRESULT_SUCCESS;
	}


	sResult sMultirobotSolutionCompressor::extract_ComputedBijectionSolution(const sRobotArrangement                 &start_arrangement,
		const sUndirectedGraph                  &sUNUSED(environment),
		int                                      computed_makespan,
		const sMultirobotEncodingContext_CNFsat &final_encoding_context,
		sMultirobotSolution                     &computed_solution,
		int                                      thread_id)
	{
		sString output_filename;
		
		if (thread_id != THREAD_ID_UNDEFINED)
		{
			output_filename = OUTPUT_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
			output_filename = OUTPUT_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		FILE *fr;
		
		if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
		{
			return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
		}
		
		char answer[32];
		answer[0] = '\0';
		
		fscanf(fr, "%s\n", answer);
		
		int robot_location_vertex_id = 0;
		std::vector<int> robot_Locations;
		robot_Locations.resize(start_arrangement.get_RobotCount() + 1, (const int)sRobotArrangement::UNDEFINED_LOCATION);
		
		while (!feof(fr))
		{
			int literal;
			fscanf(fr, "%d", &literal); 
			
			if (literal != 0)
			{
				sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));		
				if (!specified_identifier.is_Null())
				{
					const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();
					
					if (base_name.find("robot_location") == 0)
					{
						int robot_id = *specified_identifier.get_ScopeIndexes()[0] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[0]->get_Begin()) + 1;
						int layer = *specified_identifier.get_ScopeIndexes()[1] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[1]->get_Begin());
						int robot_location_bit = *specified_identifier.get_ScopeIndexes()[2] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[2]->get_Begin());
						int N_Bits = *specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[2]->get_End() - *specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[2]->get_Begin();
						robot_location_vertex_id += (literal > 0) ? sIndexableStateIdentifier::calc_Exp2(robot_location_bit) : 0;
						
						if (robot_location_bit >= N_Bits - 1)
						{
							if (robot_Locations[robot_id] != sRobotArrangement::UNDEFINED_LOCATION && robot_Locations[robot_id] != robot_location_vertex_id)
							{
								sMultirobotSolution::Move move(robot_id, robot_Locations[robot_id], robot_location_vertex_id);
								computed_solution.add_Move(layer - 1, move);
							}
							robot_Locations[robot_id] = robot_location_vertex_id;
							robot_location_vertex_id = 0;
						}			
					}
				}
			}
			else
			{
				break;
			}
		}
		
		fclose(fr);

	#ifndef sDEBUG
		{
			if (unlink(output_filename.c_str()) < 0)
			{
				return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
			}	
		}
	#endif
		
		return sRESULT_SUCCESS;
	}


	sResult sMultirobotSolutionCompressor::extract_ComputedHeuristicAdvancedSolution(const sRobotArrangement                 &start_arrangement,
		const sUndirectedGraph                  &environment,
		int                                      computed_makespan,
		const sMultirobotEncodingContext_CNFsat &final_encoding_context,
		sMultirobotSolution                     &computed_solution,
		int                                      thread_id)
	{
		sString output_filename;
		
		if (thread_id != THREAD_ID_UNDEFINED)
		{
			output_filename = OUTPUT_HEURISTIC_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
			output_filename = OUTPUT_HEURISTIC_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		FILE *fr;
		
		if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
		{
			return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
		}
		
		char answer[32];
		answer[0] = '\0';

		fscanf(fr, "%s\n", answer);
		
		sMultirobotSolution pre_solution;
		
		int transition_neighbor = 0;
		int transition_vertex_id = -1;
		int transition_step = -1;
		
		while (!feof(fr))
		{
			int literal;
			fscanf(fr, "%d", &literal); 
			
			if (literal != 0)
			{
				sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));
				
				if (!specified_identifier.is_Null())
				{
					const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();
					
					if (base_name.find("transition_action") == 0)
					{   
						int transition_state_bit = *specified_identifier.get_ScopeIndexes()[1] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[1]->get_Begin());

						if (transition_state_bit == 0)
						{
							if (transition_vertex_id >= 0)
							{
								int N_Neighbors = environment.get_Vertex(transition_vertex_id)->calc_NeighborCount();
								if (transition_neighbor < N_Neighbors && transition_step < computed_makespan)
								{
									sMultirobotSolution::Move move(-1, transition_vertex_id, environment.get_Vertex(transition_vertex_id)->calc_NeighborID(transition_neighbor));
									pre_solution.add_Move(transition_step, move);
								}
							}
							transition_neighbor = 0;
						}
						transition_vertex_id = sInt_32_from_String(base_name.substr(base_name.find("-") + 1).c_str());
						transition_step = *specified_identifier.get_ScopeIndexes()[0] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[0]->get_Begin());

						transition_neighbor += (literal > 0) ? sIndexableStateIdentifier::calc_Exp2(transition_state_bit) : 0;
					}
				}
			}
			else
			{
				break;
			}
		}
		fclose(fr);

	#ifndef sDEBUG
		{
			if (unlink(output_filename.c_str()) < 0)
			{
				return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
			}	
		}
	#endif
		
		sRobotArrangement current_arrangement = start_arrangement;
		
		int N_Steps = pre_solution.m_Steps.size();
		for (int step = 0; step < N_Steps; ++step)
		{
			const sMultirobotSolution::Moves_list &step_Moves = pre_solution.m_Steps[step].m_Moves;
			for (sMultirobotSolution::Moves_list::const_iterator move = step_Moves.begin(); move != step_Moves.end(); ++move)
			{
				int robot_id = current_arrangement.get_VertexOccupancy(move->m_src_vrtx_id);
				
				if (robot_id >= 1)
				{
					computed_solution.add_Move(step, sMultirobotSolution::Move(robot_id, move->m_src_vrtx_id, move->m_dest_vrtx_id));
					sASSERT(current_arrangement.verify_Move(robot_id, move->m_dest_vrtx_id, environment));
					current_arrangement.move_Robot(robot_id, move->m_dest_vrtx_id);
				}
			}
		}
		
		return sRESULT_SUCCESS;
	}


	sResult sMultirobotSolutionCompressor::extract_ComputedHeuristicDifferentialSolution(const sRobotArrangement                 &start_arrangement,
		const sUndirectedGraph                  &sUNUSED(environment),
		int                                      computed_makespan,
		const sMultirobotEncodingContext_CNFsat &final_encoding_context,
		sMultirobotSolution                     &computed_solution,
		int                                      thread_id)
	{
		sString output_filename;
		
		if (thread_id != THREAD_ID_UNDEFINED)
		{
			output_filename = OUTPUT_HEURISTIC_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
			output_filename = OUTPUT_HEURISTIC_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		FILE *fr;
		
		if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
		{
			return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
		}
		
		char answer[32];
		answer[0] = '\0';

		fscanf(fr, "%s\n", answer);
		
		int robot_location_vertex_id = 0;
		std::vector<int> robot_Locations;
		robot_Locations.resize(start_arrangement.get_RobotCount() + 1, (const int)sRobotArrangement::UNDEFINED_LOCATION);
		
		while (!feof(fr))
		{
			int literal;
			fscanf(fr, "%d", &literal); 
			
			if (literal != 0)
			{
				sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));
				
				if (!specified_identifier.is_Null())
				{
					const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();
					
					if (base_name.find("robot_location") == 0)
					{   
						int robot_id = *specified_identifier.get_ScopeIndexes()[0] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[0]->get_Begin()) + 1;
						int layer = *specified_identifier.get_ScopeIndexes()[1] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[1]->get_Begin());
						int robot_location_bit = *specified_identifier.get_ScopeIndexes()[2] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[2]->get_Begin());
						int N_Bits = *specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[2]->get_End() - *specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[2]->get_Begin();
						robot_location_vertex_id += (literal > 0) ? sIndexableStateIdentifier::calc_Exp2(robot_location_bit) : 0;
						
						if (robot_location_bit >= N_Bits - 1)
						{
							if (robot_Locations[robot_id] != sRobotArrangement::UNDEFINED_LOCATION && robot_Locations[robot_id] != robot_location_vertex_id)
							{
								sMultirobotSolution::Move move(robot_id, robot_Locations[robot_id], robot_location_vertex_id);
								computed_solution.add_Move(layer - 1, move);
							}
							robot_Locations[robot_id] = robot_location_vertex_id;
							robot_location_vertex_id = 0;
						}			
					}
				}
			}
			else
			{
				break;
			}
		}
		
		fclose(fr);

	#ifndef sDEBUG
		{
			if (unlink(output_filename.c_str()) < 0)
			{
				return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
			}	
		}
	#endif
		
		return sRESULT_SUCCESS;
	}


	sResult sMultirobotSolutionCompressor::extract_ComputedHeuristicBijectionSolution(const sRobotArrangement                 &start_arrangement,
		const sUndirectedGraph                  &sUNUSED(environment),
		int                                      computed_makespan,
		const sMultirobotEncodingContext_CNFsat &final_encoding_context,
		sMultirobotSolution                     &computed_solution,
		int                                      thread_id)
	{
		sString output_filename;
		
		if (thread_id != THREAD_ID_UNDEFINED)
		{
			output_filename = OUTPUT_HEURISTIC_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
			output_filename = OUTPUT_HEURISTIC_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		FILE *fr;
		
		if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
		{
			return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
		}
		
		char answer[32];
		answer[0] = '\0';

		fscanf(fr, "%s\n", answer);
		
		int robot_location_vertex_id = 0;
		std::vector<int> robot_Locations;
		robot_Locations.resize(start_arrangement.get_RobotCount() + 1, (const int)sRobotArrangement::UNDEFINED_LOCATION);
		
		while (!feof(fr))
		{
			int literal;
			fscanf(fr, "%d", &literal); 
			
			if (literal != 0)
			{
				sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));
				
				if (!specified_identifier.is_Null())
				{
					const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();
					
					if (base_name.find("robot_location") == 0)
					{   
						int robot_id = *specified_identifier.get_ScopeIndexes()[0] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[0]->get_Begin()) + 1;
						int layer = *specified_identifier.get_ScopeIndexes()[1] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[1]->get_Begin());
						int robot_location_bit = *specified_identifier.get_ScopeIndexes()[2] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[2]->get_Begin());
						int N_Bits = *specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[2]->get_End() - *specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[2]->get_Begin();
						robot_location_vertex_id += (literal > 0) ? sIndexableStateIdentifier::calc_Exp2(robot_location_bit) : 0;
						
						if (robot_location_bit >= N_Bits - 1)
						{
							if (robot_Locations[robot_id] != sRobotArrangement::UNDEFINED_LOCATION && robot_Locations[robot_id] != robot_location_vertex_id)
							{
								sMultirobotSolution::Move move(robot_id, robot_Locations[robot_id], robot_location_vertex_id);
								computed_solution.add_Move(layer - 1, move);
							}
							robot_Locations[robot_id] = robot_location_vertex_id;
							robot_location_vertex_id = 0;
						}			
					}
				}
			}
			else
			{
				break;
			}
		}
		
		fclose(fr);

	#ifndef sDEBUG
		{
			if (unlink(output_filename.c_str()) < 0)
			{
				return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
			}	
		}
	#endif
		
		return sRESULT_SUCCESS;
	}


	sResult sMultirobotSolutionCompressor::extract_ComputedBitwiseSolution(const sRobotArrangement                 &start_arrangement,
		const sUndirectedGraph                  &environment,
		int                                      computed_makespan,
		const sMultirobotEncodingContext_CNFsat &final_encoding_context,
		sMultirobotSolution                     &computed_solution,
		int                                      thread_id)
	{
		sString output_filename;
		
		if (thread_id != THREAD_ID_UNDEFINED)
		{
			output_filename = OUTPUT_BITWISE_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
			output_filename = OUTPUT_BITWISE_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		FILE *fr;
		
		if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
		{
			return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
		}
		
		char answer[32];
		answer[0] = '\0';

		fscanf(fr, "%s\n", answer);
		
		sMultirobotSolution pre_solution;
		
		int transition_neighbor = 0;
		int transition_vertex_id = -1;
		int transition_step = -1;
		
		while (!feof(fr))
		{
			int literal;
			fscanf(fr, "%d", &literal); 
			
			if (literal != 0)
			{
				sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));
				
				if (!specified_identifier.is_Null())
				{
					const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();
					
					if (base_name.find("transition_action") == 0)
					{   
						int transition_state_bit = *specified_identifier.get_ScopeIndexes()[1] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[1]->get_Begin());

						if (transition_state_bit == 0)
						{
							if (transition_vertex_id >= 0)
							{
								int N_Neighbors = environment.get_Vertex(transition_vertex_id)->calc_NeighborCount();
								if (transition_neighbor < N_Neighbors && transition_step < computed_makespan)
								{
									sMultirobotSolution::Move move(-1, transition_vertex_id, environment.get_Vertex(transition_vertex_id)->calc_NeighborID(transition_neighbor));
									pre_solution.add_Move(transition_step, move);
								}
							}
							transition_neighbor = 0;
						}
						transition_vertex_id = sInt_32_from_String(base_name.substr(base_name.find("-") + 1).c_str());
						transition_step = *specified_identifier.get_ScopeIndexes()[0] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[0]->get_Begin());

						transition_neighbor += (literal > 0) ? sIndexableStateIdentifier::calc_Exp2(transition_state_bit) : 0;
					}
				}
			}
			else
			{
				break;
			}
		}
		fclose(fr);

	#ifndef sDEBUG
		{
			if (unlink(output_filename.c_str()) < 0)
			{
				return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
			}	
		}
	#endif
		
		sRobotArrangement current_arrangement = start_arrangement;
		
		int N_Steps = pre_solution.m_Steps.size();
		for (int step = 0; step < N_Steps; ++step)
		{
			const sMultirobotSolution::Moves_list &step_Moves = pre_solution.m_Steps[step].m_Moves;
			for (sMultirobotSolution::Moves_list::const_iterator move = step_Moves.begin(); move != step_Moves.end(); ++move)
			{
				int robot_id = current_arrangement.get_VertexOccupancy(move->m_src_vrtx_id);
				
				if (robot_id >= 1)
				{
					computed_solution.add_Move(step, sMultirobotSolution::Move(robot_id, move->m_src_vrtx_id, move->m_dest_vrtx_id));
					sASSERT(current_arrangement.verify_Move(robot_id, move->m_dest_vrtx_id, environment));
					current_arrangement.move_Robot(robot_id, move->m_dest_vrtx_id);
				}
			}
		}
		
		return sRESULT_SUCCESS;
	}


	sResult sMultirobotSolutionCompressor::extract_ComputedFlowSolution(const sRobotArrangement                 &start_arrangement,
#ifdef sDEBUG									
		const sUndirectedGraph                  &environment,
#else
		const sUndirectedGraph                  &,
#endif								      
		int                                      computed_makespan,
		const sMultirobotEncodingContext_CNFsat &final_encoding_context,
		sMultirobotSolution                     &computed_solution,
		int                                      thread_id)
	{
		sString output_filename;

		if (thread_id != THREAD_ID_UNDEFINED)
		{
			output_filename = OUTPUT_FLOW_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
			output_filename = OUTPUT_FLOW_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		FILE *fr;
		
		if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
		{
			return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
		}
		
		char answer[32];
		answer[0] = '\0';

		fscanf(fr, "%s\n", answer);

		typedef std::vector<int> Locations_vector;
		typedef std::vector<Locations_vector> Layers_vector;

		Layers_vector robot_Locations;

		robot_Locations.resize(computed_makespan + 1);
		for (int layer = 0; layer <= computed_makespan; ++layer)
		{
			robot_Locations[layer].resize(start_arrangement.get_RobotCount() + 1, (const int)sRobotArrangement::UNDEFINED_LOCATION);
		}
		
		while (!feof(fr))
		{
			int literal;
			fscanf(fr, "%d", &literal); 

			if (literal != 0)
			{		
				sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));
				
				if (!specified_identifier.is_Null())
				{
					const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();
					
					if (base_name.find("vertex_occupancy_by_robot") == 0)
					{
						int robot_id = *specified_identifier.get_ScopeIndexes()[0] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[0]->get_Begin());
						++robot_id;
						int vertex_id = *specified_identifier.get_ScopeIndexes()[1] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[1]->get_Begin());
						int layer = *specified_identifier.get_ScopeIndexes()[2] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[2]->get_Begin());

						if (literal > 0)
						{
							sASSERT(robot_Locations[layer][robot_id] == sRobotArrangement::UNDEFINED_LOCATION);
							robot_Locations[layer][robot_id] = vertex_id;
						}
					}
				}
			}
			else
			{
				break;
			}
		}
		fclose(fr);

	#ifndef sDEBUG
		{
			if (unlink(output_filename.c_str()) < 0)
			{
				return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
			}	
		}
	#endif
		
		sRobotArrangement current_arrangement = start_arrangement;	
		int N_Robots = start_arrangement.get_RobotCount();

		for (int layer = 0; layer < computed_makespan; ++layer)
		{
			for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
			{
				sASSERT(robot_Locations[layer][robot_id] == robot_Locations[layer + 1][robot_id] || current_arrangement.verify_Move(robot_id, robot_Locations[layer + 1][robot_id], environment));
				if (robot_Locations[layer][robot_id] != robot_Locations[layer + 1][robot_id])
				{
					computed_solution.add_Move(layer, sMultirobotSolution::Move(robot_id, robot_Locations[layer][robot_id], robot_Locations[layer + 1][robot_id]));
					current_arrangement.move_Robot(robot_id, robot_Locations[layer + 1][robot_id]);
				}
			}
		}

		return sRESULT_SUCCESS;
	}


	sResult sMultirobotSolutionCompressor::extract_ComputedMatchingSolution(const sRobotArrangement                 &start_arrangement,
#ifdef sDEBUG									
		const sUndirectedGraph                  &environment,
#else
		const sUndirectedGraph                  &,
#endif								      
		
		int                                      computed_makespan,
		const sMultirobotEncodingContext_CNFsat &final_encoding_context,
		sMultirobotSolution                     &computed_solution,
		int                                      thread_id)
	{
		sString output_filename;

		if (thread_id != THREAD_ID_UNDEFINED)
		{
			output_filename = OUTPUT_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
			output_filename = OUTPUT_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		FILE *fr;
		
		if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
		{
			return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
		}
		
		char answer[32];
		answer[0] = '\0';

		fscanf(fr, "%s\n", answer);

		typedef std::vector<int> Locations_vector;
		typedef std::vector<Locations_vector> Layers_vector;

		Layers_vector robot_Locations;

		robot_Locations.resize(computed_makespan + 1);
		for (int layer = 0; layer <= computed_makespan; ++layer)
		{
			robot_Locations[layer].resize(start_arrangement.get_RobotCount() + 1, (const int)sRobotArrangement::UNDEFINED_LOCATION);
		}
		

		int occupying_robot_id = -1;
		int occupy_vertex_id = -1;
		int occupy_layer = -1;

		while (!feof(fr))
		{
			int literal;
			fscanf(fr, "%d", &literal); 

			if (literal != 0)
			{
				sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));

				if (!specified_identifier.is_Null())
				{
					const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();
					
					if (base_name.find("vertex_occupancy") == 0)
					{   
						int occupying_robot_bit = *specified_identifier.get_ScopeIndexes()[2] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[2]->get_Begin());

						if (occupying_robot_bit == 0)
						{
							if (occupying_robot_id >= 1)
							{
								sASSERT(robot_Locations[occupy_layer][occupying_robot_id] == sRobotArrangement::UNDEFINED_LOCATION);
								robot_Locations[occupy_layer][occupying_robot_id] = occupy_vertex_id;
							}
							occupying_robot_id = 0;
						}
						occupy_layer = *specified_identifier.get_ScopeIndexes()[1] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[1]->get_Begin());
						occupy_vertex_id = *specified_identifier.get_ScopeIndexes()[0] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[0]->get_Begin());
						occupying_robot_id += (literal > 0) ? sIndexableStateIdentifier::calc_Exp2(occupying_robot_bit) : 0;
					}
				}
			}
			else
			{
				break;
			}
		}

		fclose(fr);

	#ifndef sDEBUG
		{
			if (unlink(output_filename.c_str()) < 0)
			{
				return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
			}	
		}
	#endif
		
		sRobotArrangement current_arrangement = start_arrangement;	
		int N_Robots = start_arrangement.get_RobotCount();

		for (int layer = 0; layer < computed_makespan; ++layer)
		{
			for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
			{
				sASSERT(robot_Locations[layer][robot_id] == robot_Locations[layer + 1][robot_id] || current_arrangement.verify_Move(robot_id, robot_Locations[layer + 1][robot_id], environment));
				if (robot_Locations[layer][robot_id] != robot_Locations[layer + 1][robot_id])
				{
					computed_solution.add_Move(layer, sMultirobotSolution::Move(robot_id, robot_Locations[layer][robot_id], robot_Locations[layer + 1][robot_id]));
					current_arrangement.move_Robot(robot_id, robot_Locations[layer + 1][robot_id]);
				}
			}
		}

		return sRESULT_SUCCESS;
	}


	sResult sMultirobotSolutionCompressor::extract_ComputedHeuristicMatchingSolution(const sRobotArrangement                 &start_arrangement,
#ifdef sDEBUG
		const sUndirectedGraph                  &environment,
#else
		const sUndirectedGraph                  &,
#endif
		int                                      computed_makespan,
		const sMultirobotEncodingContext_CNFsat &final_encoding_context,
		sMultirobotSolution                     &computed_solution,
		int                                      thread_id)
	{
		sString output_filename;

		if (thread_id != THREAD_ID_UNDEFINED)
		{
			output_filename = OUTPUT_HEURISTIC_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
			output_filename = OUTPUT_HEURISTIC_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		FILE *fr;
		
		if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
		{
			return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
		}
		
		char answer[32];
		answer[0] = '\0';

		fscanf(fr, "%s\n", answer);

		typedef std::vector<int> Locations_vector;
		typedef std::vector<Locations_vector> Layers_vector;

		Layers_vector robot_Locations;

		robot_Locations.resize(computed_makespan + 1);
		for (int layer = 0; layer <= computed_makespan; ++layer)
		{
			robot_Locations[layer].resize(start_arrangement.get_RobotCount() + 1, (const int)sRobotArrangement::UNDEFINED_LOCATION);
		}
		

		int occupying_robot_id = -1;
		int occupy_vertex_id = -1;
		int occupy_layer = -1;

		while (!feof(fr))
		{
			int literal;
			fscanf(fr, "%d", &literal); 
			
			if (literal != 0)
			{
				sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));
				
				if (!specified_identifier.is_Null())
				{
					const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();
					
					if (base_name.find("vertex_occupancy") == 0)
					{   
						int occupying_robot_bit = *specified_identifier.get_ScopeIndexes()[2] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[2]->get_Begin());

						if (occupying_robot_bit == 0)
						{
							if (occupying_robot_id >= 1)
							{
								sASSERT(robot_Locations[occupy_layer][occupying_robot_id] == (const int)sRobotArrangement::UNDEFINED_LOCATION);
								robot_Locations[occupy_layer][occupying_robot_id] = occupy_vertex_id;
							}
							occupying_robot_id = 0;
						}
						occupy_layer = *specified_identifier.get_ScopeIndexes()[1] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[1]->get_Begin());
						occupy_vertex_id = *specified_identifier.get_ScopeIndexes()[0] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[0]->get_Begin());
						occupying_robot_id += (literal > 0) ? sIndexableStateIdentifier::calc_Exp2(occupying_robot_bit) : 0;
					}
				}
			}
			else
			{
				break;
			}
		}

		fclose(fr);

	#ifndef sDEBUG
		{
			if (unlink(output_filename.c_str()) < 0)
			{
				return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
			}	
		}
	#endif
		
		sRobotArrangement current_arrangement = start_arrangement;	
		int N_Robots = start_arrangement.get_RobotCount();

		for (int layer = 0; layer < computed_makespan; ++layer)
		{
			for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
			{
				sASSERT(robot_Locations[layer][robot_id] == robot_Locations[layer + 1][robot_id] || current_arrangement.verify_Move(robot_id, robot_Locations[layer + 1][robot_id], environment));
				if (robot_Locations[layer][robot_id] != robot_Locations[layer + 1][robot_id])
				{
					computed_solution.add_Move(layer, sMultirobotSolution::Move(robot_id, robot_Locations[layer][robot_id], robot_Locations[layer + 1][robot_id]));
					current_arrangement.move_Robot(robot_id, robot_Locations[layer + 1][robot_id]);
				}
			}
		}

		return sRESULT_SUCCESS;
	}


	sResult sMultirobotSolutionCompressor::extract_ComputedDirectSolution(const sRobotArrangement                 &start_arrangement,
#ifdef sDEBUG
		const sUndirectedGraph                  &environment,
#else
		const sUndirectedGraph                  &,
#endif
		int                                      computed_makespan,
		const sMultirobotEncodingContext_CNFsat &final_encoding_context,
		sMultirobotSolution                     &computed_solution,
		int                                      thread_id)
	{
		sString output_filename;

		if (thread_id != THREAD_ID_UNDEFINED)
		{
			output_filename = OUTPUT_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
			output_filename = OUTPUT_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		FILE *fr;
		
		if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
		{
			return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
		}
		
		char answer[32];
		answer[0] = '\0';

		fscanf(fr, "%s\n", answer);

		typedef std::vector<int> Locations_vector;
		typedef std::vector<Locations_vector> Layers_vector;

		Layers_vector robot_Locations;

		robot_Locations.resize(computed_makespan + 1);
		for (int layer = 0; layer <= computed_makespan; ++layer)
		{
			robot_Locations[layer].resize(start_arrangement.get_RobotCount() + 1, (const int)sRobotArrangement::UNDEFINED_LOCATION);
		}
		

		while (!feof(fr))
		{
			int literal;
			fscanf(fr, "%d", &literal); 

			if (literal > 0)
			{
				sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));

				if (!specified_identifier.is_Null())
				{
					const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();
					
					if (base_name.find("robot_location_in_vertex") == 0)
					{   
						int robot_id = *specified_identifier.get_ScopeIndexes()[0] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[0]->get_Begin()) + 1;
						int vertex_id = *specified_identifier.get_ScopeIndexes()[1] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[1]->get_Begin());
						int layer_id = *specified_identifier.get_ScopeIndexes()[2] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[2]->get_Begin());

						sASSERT(robot_Locations[layer_id][robot_id] == (const int)sRobotArrangement::UNDEFINED_LOCATION);
						robot_Locations[layer_id][robot_id] = vertex_id;
					}
				}
			}
		}

		fclose(fr);

	#ifndef sDEBUG
		{
			if (unlink(output_filename.c_str()) < 0)
			{
				return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
			}	
		}
	#endif
		
		sRobotArrangement current_arrangement = start_arrangement;	
		int N_Robots = start_arrangement.get_RobotCount();

		for (int layer = 0; layer < computed_makespan; ++layer)
		{
			for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
			{
				sASSERT(robot_Locations[layer][robot_id] == robot_Locations[layer + 1][robot_id] || current_arrangement.verify_Move(robot_id, robot_Locations[layer + 1][robot_id], environment));
				if (robot_Locations[layer][robot_id] != robot_Locations[layer + 1][robot_id])
				{
					computed_solution.add_Move(layer, sMultirobotSolution::Move(robot_id, robot_Locations[layer][robot_id], robot_Locations[layer + 1][robot_id]));
					current_arrangement.move_Robot(robot_id, robot_Locations[layer + 1][robot_id]);
				}
			}
		}

		return sRESULT_SUCCESS;
	}


	sResult sMultirobotSolutionCompressor::extract_ComputedHeuristicDirectSolution(const sRobotArrangement                 &start_arrangement,
#ifdef sDEBUG
		const sUndirectedGraph                  &environment,
#else
		const sUndirectedGraph                  &,
#endif									  										   
		int                                      computed_makespan,
		const sMultirobotEncodingContext_CNFsat &final_encoding_context,
		sMultirobotSolution                     &computed_solution,
		int                                      thread_id)
	{
		sString output_filename;

		if (thread_id != THREAD_ID_UNDEFINED)
		{
			output_filename = OUTPUT_HEURISTIC_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
			output_filename = OUTPUT_HEURISTIC_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		FILE *fr;
		
		if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
		{
			return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
		}
		
		char answer[32];
		answer[0] = '\0';

		fscanf(fr, "%s\n", answer);

		typedef std::vector<int> Locations_vector;
		typedef std::vector<Locations_vector> Layers_vector;

		Layers_vector robot_Locations;

		robot_Locations.resize(computed_makespan + 1);
		for (int layer = 0; layer <= computed_makespan; ++layer)
		{
			robot_Locations[layer].resize(start_arrangement.get_RobotCount() + 1, (const int)sRobotArrangement::UNDEFINED_LOCATION);
		}
		

		while (!feof(fr))
		{
			int literal;
			fscanf(fr, "%d", &literal); 

			if (literal > 0)
			{
				sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));

				if (!specified_identifier.is_Null())
				{
					const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();
					
					if (base_name.find("robot_location_in_vertex") == 0)
					{   
						int robot_id = *specified_identifier.get_ScopeIndexes()[0] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[0]->get_Begin()) + 1;
						int vertex_id = *specified_identifier.get_ScopeIndexes()[1] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[1]->get_Begin());
						int layer_id = *specified_identifier.get_ScopeIndexes()[2] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[2]->get_Begin());

						sASSERT(robot_Locations[layer_id][robot_id] == (const int)sRobotArrangement::UNDEFINED_LOCATION);
						robot_Locations[layer_id][robot_id] = vertex_id;
					}
				}
			}
		}

		fclose(fr);

	#ifndef sDEBUG
		{
			if (unlink(output_filename.c_str()) < 0)
			{
				return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
			}	
		}
	#endif
		
		sRobotArrangement current_arrangement = start_arrangement;	
		int N_Robots = start_arrangement.get_RobotCount();

		for (int layer = 0; layer < computed_makespan; ++layer)
		{
			for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
			{
				sASSERT(robot_Locations[layer][robot_id] == robot_Locations[layer + 1][robot_id] || current_arrangement.verify_Move(robot_id, robot_Locations[layer + 1][robot_id], environment));
				if (robot_Locations[layer][robot_id] != robot_Locations[layer + 1][robot_id])
				{
					computed_solution.add_Move(layer, sMultirobotSolution::Move(robot_id, robot_Locations[layer][robot_id], robot_Locations[layer + 1][robot_id]));
					current_arrangement.move_Robot(robot_id, robot_Locations[layer + 1][robot_id]);
				}
			}
		}

		return sRESULT_SUCCESS;
	}


	sResult sMultirobotSolutionCompressor::extract_ComputedSimplicialSolution(const sRobotArrangement                 &start_arrangement,
#ifdef sDEBUG
		const sUndirectedGraph                  &environment,
#else
		const sUndirectedGraph                  &,
#endif
		int                                      computed_makespan,
		const sMultirobotEncodingContext_CNFsat &final_encoding_context,
		sMultirobotSolution                     &computed_solution,
		int                                      thread_id)
	{
		sString output_filename;

		if (thread_id != THREAD_ID_UNDEFINED)
		{
			output_filename = OUTPUT_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
			output_filename = OUTPUT_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		FILE *fr;
		
		if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
		{
			return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
		}
		
		char answer[32];
		answer[0] = '\0';

		fscanf(fr, "%s\n", answer);

		typedef std::vector<int> Locations_vector;
		typedef std::vector<Locations_vector> Layers_vector;

		Layers_vector robot_Locations;

		robot_Locations.resize(computed_makespan + 1);
		for (int layer = 0; layer <= computed_makespan; ++layer)
		{
			robot_Locations[layer].resize(start_arrangement.get_RobotCount() + 1, (const int)sRobotArrangement::UNDEFINED_LOCATION);
		}
		

		while (!feof(fr))
		{
			int literal;
			fscanf(fr, "%d", &literal); 

			if (literal > 0)
			{
				sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));

				if (!specified_identifier.is_Null())
				{
					const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();
					
					if (base_name.find("robot_location_in_vertex") == 0)
					{   
						int robot_id = *specified_identifier.get_ScopeIndexes()[0] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[0]->get_Begin()) + 1;
						int vertex_id = *specified_identifier.get_ScopeIndexes()[1] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[1]->get_Begin());
						int layer_id = *specified_identifier.get_ScopeIndexes()[2] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[2]->get_Begin());

						sASSERT(robot_Locations[layer_id][robot_id] == (const int)sRobotArrangement::UNDEFINED_LOCATION);
						robot_Locations[layer_id][robot_id] = vertex_id;
					}
				}
			}
		}

		fclose(fr);

	#ifndef sDEBUG
		{
			if (unlink(output_filename.c_str()) < 0)
			{
				return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
			}	
		}
	#endif
		
		sRobotArrangement current_arrangement = start_arrangement;	
		int N_Robots = start_arrangement.get_RobotCount();

		for (int layer = 0; layer < computed_makespan; ++layer)
		{
			for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
			{
				sASSERT(robot_Locations[layer][robot_id] == robot_Locations[layer + 1][robot_id] || current_arrangement.verify_Move(robot_id, robot_Locations[layer + 1][robot_id], environment));
				if (robot_Locations[layer][robot_id] != robot_Locations[layer + 1][robot_id])
				{
					computed_solution.add_Move(layer, sMultirobotSolution::Move(robot_id, robot_Locations[layer][robot_id], robot_Locations[layer + 1][robot_id]));
					current_arrangement.move_Robot(robot_id, robot_Locations[layer + 1][robot_id]);
				}
			}
		}

		return sRESULT_SUCCESS;
	}


	sResult sMultirobotSolutionCompressor::extract_ComputedHeuristicSimplicialSolution(const sRobotArrangement                 &start_arrangement,
#ifdef sDEBUG
		const sUndirectedGraph                  &environment,
#else
		const sUndirectedGraph                  &,
#endif										       
		int                                      computed_makespan,
		const sMultirobotEncodingContext_CNFsat &final_encoding_context,
		sMultirobotSolution                     &computed_solution,
		int                                      thread_id)
	{
		sString output_filename;

		if (thread_id != THREAD_ID_UNDEFINED)
		{
			output_filename = OUTPUT_HEURISTIC_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
			output_filename = OUTPUT_HEURISTIC_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		FILE *fr;
		
		if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
		{
			return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
		}
		
		char answer[32];
		answer[0] = '\0';

		fscanf(fr, "%s\n", answer);

		typedef std::vector<int> Locations_vector;
		typedef std::vector<Locations_vector> Layers_vector;

		Layers_vector robot_Locations;

		robot_Locations.resize(computed_makespan + 1);
		for (int layer = 0; layer <= computed_makespan; ++layer)
		{
			robot_Locations[layer].resize(start_arrangement.get_RobotCount() + 1, (const int)sRobotArrangement::UNDEFINED_LOCATION);
		}
		

		while (!feof(fr))
		{
			int literal;
			fscanf(fr, "%d", &literal); 

			if (literal > 0)
			{
				sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));

				if (!specified_identifier.is_Null())
				{
					const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();
					
					if (base_name.find("robot_location_in_vertex") == 0)
					{   
						int robot_id = *specified_identifier.get_ScopeIndexes()[0] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[0]->get_Begin()) + 1;
						int vertex_id = *specified_identifier.get_ScopeIndexes()[1] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[1]->get_Begin());
						int layer_id = *specified_identifier.get_ScopeIndexes()[2] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[2]->get_Begin());

			// sASSERT(robot_Locations[layer_id][robot_id] == (const int)sRobotArrangement::UNDEFINED_LOCATION);
						robot_Locations[layer_id][robot_id] = vertex_id;
					}
				}
			}
		}

		fclose(fr);

	#ifndef sDEBUG
		{
			if (unlink(output_filename.c_str()) < 0)
			{
				return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
			}	
		}
	#endif
		
		sRobotArrangement current_arrangement = start_arrangement;	
		int N_Robots = start_arrangement.get_RobotCount();

		for (int layer = 0; layer < computed_makespan; ++layer)
		{
			for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
			{
				sASSERT(robot_Locations[layer][robot_id] == robot_Locations[layer + 1][robot_id] || current_arrangement.verify_Move(robot_id, robot_Locations[layer + 1][robot_id], environment));
				if (robot_Locations[layer][robot_id] != robot_Locations[layer + 1][robot_id])
				{
					computed_solution.add_Move(layer, sMultirobotSolution::Move(robot_id, robot_Locations[layer][robot_id], robot_Locations[layer + 1][robot_id]));
					current_arrangement.move_Robot(robot_id, robot_Locations[layer + 1][robot_id]);
				}
			}
		}

		return sRESULT_SUCCESS;
	}


	struct Edge_
	{
		int m_source_id;
		int m_target_id;
	};

	sResult sMultirobotSolutionCompressor::extract_ComputedSingularSolution(const sRobotArrangement                 &start_arrangement,
		const sUndirectedGraph                  &environment,
		int                                      sUNUSED(computed_makespan),
		const sMultirobotEncodingContext_CNFsat &final_encoding_context,
		sMultirobotSolution                     &computed_solution,
		int                                      thread_id)
	{
		sString output_filename;

		printf("Extracting...\n");

		if (thread_id != THREAD_ID_UNDEFINED)
		{
			output_filename = OUTPUT_SINGULAR_FILENAME_PREFIX + "_-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
			output_filename = OUTPUT_SINGULAR_FILENAME_PREFIX + "_-" + sInt_32_to_String(getpid()) + ".txt";
		}

		FILE *fr;
		
		if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
		{
			return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
		}
		
		char answer[32];
		answer[0] = '\0';

		fscanf(fr, "%s\n", answer);

		typedef std::list<Edge_> Edges_list;
		Edges_list Edges;

		while (!feof(fr))
		{
			int literal;
			fscanf(fr, "%d", &literal); 

			if (literal > 0)
			{
				sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));

				if (!specified_identifier.is_Null())
				{
					const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();
					
					if (base_name.find("edge_occupancy_by_water-") == 0)
					{   
						int vertex_id = sUInt_32_from_String(base_name.substr(24, base_name.size()));
						int neighbor_id = *specified_identifier.get_ScopeIndexes()[0] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[0]->get_Begin());

						const sVertex::Neighbors_list &out_Neighbors = environment.get_Vertex(vertex_id)->m_Neighbors;
						sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin();

						while (--neighbor_id >= 0)
						{
							++out_neighbor;
						}

						Edge_ edge;

						edge.m_source_id = vertex_id;
						edge.m_target_id = (*out_neighbor)->m_target->m_id;
						Edges.push_back(edge);

		        #ifdef sDEBUG
						{
							printf("saturated edge: %d-->%d\n", edge.m_source_id, edge.m_target_id);
						}
			#endif
					}
				}
			}
		}

		fclose(fr);

	#ifndef sDEBUG
		{
			if (unlink(output_filename.c_str()) < 0)
			{
				return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
			}	
		}
	#endif
		
		sRobotArrangement current_arrangement = start_arrangement;	
		int N_Robots = start_arrangement.get_RobotCount();

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
			int step = 0;
			int vertex_id = start_arrangement.get_RobotLocation(robot_id);
            #ifdef sDEBUG
			{
				printf("v1:%d\n", vertex_id);
			}
	    #endif

			Edges_list::iterator edge = Edges.begin();
			while (edge != Edges.end())
			{
				if (vertex_id == edge->m_source_id)
				{
					computed_solution.add_Move(step, sMultirobotSolution::Move(robot_id, vertex_id, edge->m_target_id));
					vertex_id = edge->m_target_id;
                    #ifdef sDEBUG
					{
						printf("v2:%d\n", vertex_id);
					}
                    #endif
					Edges.erase(edge);
					edge = Edges.begin();
					++step;
				}
				else
				{
					++edge;
				}
			}
            #ifdef sDEBUG
			{
				printf("----\n");
			}
	    #endif
		}
        #ifdef sDEBUG
		{
			printf("Edges remain:%ld\n", Edges.size());
		}
        #endif

		return sRESULT_SUCCESS;
	}


	struct Edge__
	{
		int m_source_id;
		int m_target_id;

		int m_level;
		bool m_wait;
	};

	sResult sMultirobotSolutionCompressor::extract_ComputedPluralSolution(const sRobotArrangement                 &start_arrangement,
		const sUndirectedGraph                  &environment,
		int                                      computed_makespan,
		const sMultirobotEncodingContext_CNFsat &final_encoding_context,
		sMultirobotSolution                     &computed_solution,
		int                                      thread_id)
	{
		sString output_filename;

		printf("Extracting plural...\n");

		if (thread_id != THREAD_ID_UNDEFINED)
		{
			output_filename = OUTPUT_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
			output_filename = OUTPUT_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		FILE *fr;
		
		if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
		{
			return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
		}
		
		char answer[32];
		answer[0] = '\0';

		fscanf(fr, "%s\n", answer);
/*
	typedef std::vector<int> Locations_vector;
	typedef std::vector<Locations_vector> Layers_vector;
*/
		typedef std::list<Edge__> Edges_list;
		Edges_list Edges;

		while (!feof(fr))
		{
			int literal;
			fscanf(fr, "%d", &literal); 

			if (literal > 0)
			{
				sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));

				if (!specified_identifier.is_Null())
				{
					const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();
					
					if (base_name.find("edge_occupancy_by_water") == 0)
					{   
						int vertex_id = sUInt_32_from_String(base_name.substr(24, base_name.find('_', 24) - 24));
						int level = sUInt_32_from_String(base_name.substr(base_name.find('_', 24) + 1, base_name.size()));


						int neighbor_id = *specified_identifier.get_ScopeIndexes()[0] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[0]->get_Begin());

						if (level == computed_makespan - 1)
						{
							const sVertex::Neighbors_list &out_Neighbors = environment.get_Vertex(vertex_id)->m_Neighbors;
							sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin();

							while (--neighbor_id >= 0)
							{
								++out_neighbor;
							}

							Edge__ edge;
							
							edge.m_source_id = vertex_id;
							edge.m_target_id = (*out_neighbor)->m_target->m_id;
							edge.m_level = level;
							
							Edges.push_back(edge);
						}
						else
						{
							if (neighbor_id == 0)
							{
								Edge__ edge;

								edge.m_source_id = vertex_id;
								edge.m_target_id = vertex_id;
								edge.m_level = level;
								
								Edges.push_back(edge);
							}
							else
							{
								--neighbor_id;

								const sVertex::Neighbors_list &out_Neighbors = environment.get_Vertex(vertex_id)->m_Neighbors;
								sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin();
								
								while (--neighbor_id >= 0)
								{
									++out_neighbor;
								}
								
								Edge__ edge;
								
								edge.m_source_id = vertex_id;
								edge.m_target_id = (*out_neighbor)->m_target->m_id;
								edge.m_level = level;
								
								Edges.push_back(edge);
							}
						}
					}
				}
			}
		}

		fclose(fr);

	#ifndef sDEBUG
		{
			if (unlink(output_filename.c_str()) < 0)
			{
				return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
			}	
		}
	#endif
		
		sRobotArrangement current_arrangement = start_arrangement;	
		int N_Robots = start_arrangement.get_RobotCount();

		sRobotArrangement level_arrangement = start_arrangement;

		int level_step = 0;

		for (int level = 0; level < computed_makespan; ++level)
		{
			sRobotArrangement new_level_arrangement(level_arrangement.get_VertexCount(), level_arrangement.get_RobotCount());
			int new_level_step = level_step;
			for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
			{
				int step = level_step;
				int vertex_id = level_arrangement.get_RobotLocation(robot_id);

                #ifdef sDEBUG
				{
					printf("v1:%d\n", vertex_id);
				}
                #endif

				Edges_list::iterator edge = Edges.begin();
				while (edge != Edges.end())
				{
					if (edge->m_level == level && vertex_id == edge->m_source_id)
					{
						if (edge->m_target_id == vertex_id)
						{
							break;
						}
						else
						{
							computed_solution.add_Move(step, sMultirobotSolution::Move(robot_id, vertex_id, edge->m_target_id));
							vertex_id = edge->m_target_id;

                            #ifdef sDEBUG
							{
								printf("v2:%d\n", vertex_id);
							}
                            #endif

							Edges.erase(edge);
							edge = Edges.begin();
							++step;
						}
					}
					else
					{
						++edge;
					}
				}
                #ifdef sDEBUG
				{
					printf("placing: %d, %d\n", robot_id, vertex_id);
				}
		#endif
				new_level_arrangement.place_Robot(robot_id, vertex_id);

				if (step > new_level_step)
				{
					new_level_step = step;
				}
                #ifdef sDEBUG
				{
					printf("----: %d\n", level_step);
				}
                #endif
			}
			level_step = new_level_step;
			level_arrangement = new_level_arrangement;
		}
        #ifdef sDEBUG
		{
			printf("Edges remain:%ld\n", Edges.size());
		}
        #endif

		return sRESULT_SUCCESS;
	}


	sResult sMultirobotSolutionCompressor::extract_ComputedPlural2Solution(const sRobotArrangement                 &start_arrangement,
		const sUndirectedGraph                  &environment,
		const sUndirectedGraph                  &sparse_environment,
		int                                      computed_makespan,
		const sMultirobotEncodingContext_CNFsat &final_encoding_context,
		sMultirobotSolution                     &computed_solution,
		int                                      thread_id)
	{
		sString output_filename;

		printf("Extracting plural2...\n");

		if (thread_id != THREAD_ID_UNDEFINED)
		{
			output_filename = OUTPUT_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
			output_filename = OUTPUT_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		FILE *fr;
		
		if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
		{
			return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
		}
		
		char answer[32];
		answer[0] = '\0';

		fscanf(fr, "%s\n", answer);
/*
	typedef std::vector<int> Locations_vector;
	typedef std::vector<Locations_vector> Layers_vector;
*/
		typedef std::list<Edge__> Edges_list;
		Edges_list Edges;

		while (!feof(fr))
		{
			int literal;
			fscanf(fr, "%d", &literal); 

			if (literal > 0)
			{
				sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));

				if (!specified_identifier.is_Null())
				{
					const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();
					
					if (base_name.find("edge_occupancy_by_water") == 0)
					{   
						int vertex_id = sUInt_32_from_String(base_name.substr(24, base_name.find('_', 24) - 24));
						int level = sUInt_32_from_String(base_name.substr(base_name.find('_', 24) + 1, base_name.size()));

						int neighbor_id = *specified_identifier.get_ScopeIndexes()[0] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[0]->get_Begin());
						const sVertex::Neighbors_list &out_Neighbors = sparse_environment.get_Vertex(vertex_id)->m_Neighbors;
						const sVertex::Neighbors_list &full_out_Neighbors = environment.get_Vertex(vertex_id)->m_Neighbors;

						if (level == computed_makespan - 1)
						{
							sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin();
			    /*
			    sparse_environment.to_Screen();
			    environment.to_Screen();
			    */

							while (--neighbor_id >= 0)
							{
								++out_neighbor;
							}
							sASSERT(out_neighbor != out_Neighbors.end());

							Edge__ edge;
							
							edge.m_source_id = vertex_id;
							edge.m_target_id = (*out_neighbor)->m_target->m_id;
							edge.m_level = level;
							edge.m_wait = false;
							
							Edges.push_back(edge);
						}
						else
						{
							int out_neighbors_count = out_Neighbors.size();

							if (neighbor_id == 0)
							{
								Edge__ edge;

								edge.m_source_id = vertex_id;
								edge.m_target_id = vertex_id;
								edge.m_level = level + 1;
								edge.m_wait = true;
								
								Edges.push_back(edge);
							}
							else if (neighbor_id > out_neighbors_count)
							{
								neighbor_id -= out_neighbors_count;				
								--neighbor_id;
								sVertex::Neighbors_list::const_iterator out_neighbor = full_out_Neighbors.begin();
								
								while (--neighbor_id >= 0)
								{
									++out_neighbor;
								}
								sASSERT(out_neighbor != full_out_Neighbors.end());
								
								Edge__ edge;
								
								edge.m_source_id = vertex_id;
								edge.m_target_id = (*out_neighbor)->m_target->m_id;
								edge.m_level = level + 1;
								edge.m_wait = true;
								
								Edges.push_back(edge);
							}
							else				
							{
								--neighbor_id;
								sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin();
								
								while (--neighbor_id >= 0)
								{
									++out_neighbor;
								}
								sASSERT(out_neighbor != out_Neighbors.end());
								
								Edge__ edge;
								
								edge.m_source_id = vertex_id;
								edge.m_target_id = (*out_neighbor)->m_target->m_id;
								edge.m_level = level;
								edge.m_wait = false;
								
								Edges.push_back(edge);
							}
						}
					}
				}
			}
		}

		fclose(fr);

	#ifndef sDEBUG
		{
			if (unlink(output_filename.c_str()) < 0)
			{
				return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
			}	
		}
	#endif
		
		sRobotArrangement current_arrangement = start_arrangement;	
		int N_Robots = start_arrangement.get_RobotCount();

		sRobotArrangement level_arrangement = start_arrangement;

		int level_step = 0;

		for (int level = 0; level < computed_makespan; ++level)
		{
			sRobotArrangement new_level_arrangement(level_arrangement.get_VertexCount(), level_arrangement.get_RobotCount());
			int new_level_step = level_step;

			for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
			{
				int step = level_step;
				int vertex_id = level_arrangement.get_RobotLocation(robot_id);

				Edges_list::iterator edge = Edges.begin();
				while (edge != Edges.end())
				{
					if (edge->m_level == level && vertex_id == edge->m_source_id)
					{
						if (edge->m_target_id == vertex_id)
						{
							break;
						}
						else
						{
							computed_solution.add_Move(step, sMultirobotSolution::Move(robot_id, vertex_id, edge->m_target_id));
							vertex_id = edge->m_target_id;

							Edges.erase(edge);
							edge = Edges.begin();
							++step;
						}
					}
					else
					{
						++edge;
					}
				}
				new_level_arrangement.place_Robot(robot_id, vertex_id);

				if (step > new_level_step)
				{
					new_level_step = step;
				}
			}
			level_arrangement = new_level_arrangement;
			level_step = new_level_step;

			if (level == computed_makespan - 1)
			{
				break;
			}
			bool wait_only = true;

			sRobotArrangement newer_level_arrangement(level_arrangement.get_VertexCount(), level_arrangement.get_RobotCount());

			for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
			{
				int vertex_id = level_arrangement.get_RobotLocation(robot_id);

				Edges_list::iterator edge = Edges.begin();
				while (edge != Edges.end())
				{
					if (edge->m_level == level + 1 && vertex_id == edge->m_source_id && edge->m_wait)
					{
						if (edge->m_target_id != vertex_id)
						{
							computed_solution.add_Move(level_step, sMultirobotSolution::Move(robot_id, vertex_id, edge->m_target_id));
							vertex_id = edge->m_target_id;
							wait_only = false;
						}
						Edges.erase(edge);	
						break;
					}
					else
					{
						++edge;
					}
				}
				newer_level_arrangement.place_Robot(robot_id, vertex_id);
			}	    
			if (!wait_only)
			{
				++level_step;
			}
			level_arrangement = newer_level_arrangement;
		}
        #ifdef sDEBUG
		{
			printf("Edges remain:%ld\n", Edges.size());
		}
        #endif

		return sRESULT_SUCCESS;
	}


	sResult sMultirobotSolutionCompressor::extract_ComputedHeightedSolution(const sRobotArrangement                        &start_arrangement,
		const sUndirectedGraph                         &sUNUSED(environment),
		const sMultirobotInstance::Environments_vector &heighted_Environments,
		int                                             computed_cost,
		const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
		sMultirobotSolution                            &computed_solution,
		int                                             thread_id)
	{
		sString output_filename;

		printf("Extracting heighted ...\n");

		if (thread_id != THREAD_ID_UNDEFINED)
		{
			output_filename = OUTPUT_HEIGHTED_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
			output_filename = OUTPUT_HEIGHTED_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		FILE *fr;
		
		if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
		{
			return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
		}
		
		char answer[32];
		answer[0] = '\0';

		fscanf(fr, "%s\n", answer);

	/*
	typedef std::vector<int> Locations_vector;
	typedef std::vector<Locations_vector> Layers_vector;
	*/
		
		typedef std::list<Edge__> Edges_list;
		Edges_list Edges;

		while (!feof(fr))
		{
			int literal;
			fscanf(fr, "%d", &literal); 

			if (literal > 0)
			{
				sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));

				if (!specified_identifier.is_Null())
				{
					const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();
					
					if (base_name.find("edge_occupancy_by_water") == 0)
					{   
						int vertex_id = sUInt_32_from_String(base_name.substr(24, base_name.find('_', 24) - 24));
						int level = sUInt_32_from_String(base_name.substr(base_name.find('_', 24) + 1, base_name.size()));


						int neighbor_id = *specified_identifier.get_ScopeIndexes()[0] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[0]->get_Begin());
						printf("%s\n", base_name.c_str());
						printf("level:%d,%ld\n", level, heighted_Environments.size());

						if (level == heighted_Environments.size() - 1)
						{
							const sVertex::Neighbors_list &out_Neighbors = heighted_Environments[level].get_Vertex(vertex_id)->m_Neighbors;
							sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin();

							while (--neighbor_id >= 0)
							{
								++out_neighbor;
							}

							Edge__ edge;
							
							edge.m_source_id = vertex_id;
							edge.m_target_id = (*out_neighbor)->m_target->m_id;
							edge.m_level = level;
							
							Edges.push_back(edge);
						}
						else
						{
							if (neighbor_id == 0)
							{
								Edge__ edge;

								edge.m_source_id = vertex_id;
								edge.m_target_id = vertex_id;
								edge.m_level = level;
								
								Edges.push_back(edge);
							}
							else
							{
								--neighbor_id;

								const sVertex::Neighbors_list &out_Neighbors = heighted_Environments[level].get_Vertex(vertex_id)->m_Neighbors;
								sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin();
								
								while (--neighbor_id >= 0)
								{
									++out_neighbor;
								}
								
								Edge__ edge;
								
								edge.m_source_id = vertex_id;
								edge.m_target_id = (*out_neighbor)->m_target->m_id;
								edge.m_level = level;
								
								Edges.push_back(edge);
							}
						}
					}
				}
			}
		}

		fclose(fr);

	#ifndef sDEBUG
		{
			if (unlink(output_filename.c_str()) < 0)
			{
				return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
			}	
		}
	#endif
		
		sRobotArrangement current_arrangement = start_arrangement;	
		int N_Robots = start_arrangement.get_RobotCount();

		sRobotArrangement level_arrangement = start_arrangement;

		int level_step = 0;

		for (int level = 0; level < heighted_Environments.size() ; ++level)
		{
			sRobotArrangement new_level_arrangement(level_arrangement.get_VertexCount(), level_arrangement.get_RobotCount());
			int new_level_step = level_step;
			for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
			{
				int step = level_step;
				int vertex_id = level_arrangement.get_RobotLocation(robot_id);

                #ifdef sDEBUG
				{
					printf("v1:%d\n", vertex_id);
				}
                #endif

				Edges_list::iterator edge = Edges.begin();
				while (edge != Edges.end())
				{
					if (edge->m_level == level && vertex_id == edge->m_source_id)
					{
						if (edge->m_target_id == vertex_id)
						{
							break;
						}
						else
						{
							computed_solution.add_Move(step, sMultirobotSolution::Move(robot_id, vertex_id, edge->m_target_id));
							vertex_id = edge->m_target_id;

                            #ifdef sDEBUG
							{
								printf("v2:%d\n", vertex_id);
							}
                            #endif

							Edges.erase(edge);
							edge = Edges.begin();
							++step;
						}
					}
					else
					{
						++edge;
					}
				}
                #ifdef sDEBUG
				{
					printf("placing: %d, %d\n", robot_id, vertex_id);
				}
		#endif
				new_level_arrangement.place_Robot(robot_id, vertex_id);

				if (step > new_level_step)
				{
					new_level_step = step;
				}
                #ifdef sDEBUG
				{
					printf("----: %d\n", level_step);
				}
                #endif
			}
			level_step = new_level_step;
			level_arrangement = new_level_arrangement;
		}
        #ifdef sDEBUG
		{
			printf("Edges remain:%ld\n", Edges.size());
		}
        #endif

		return sRESULT_SUCCESS;
	}


	sResult sMultirobotSolutionCompressor::extract_ComputedMddSolution(const sRobotArrangement                 &start_arrangement,
#ifdef sDEBUG
		const sUndirectedGraph                  &environment,
#else
		const sUndirectedGraph                  &,
#endif										       								       
		const sMultirobotInstance::MDD_vector   &MDD,
		int                                      computed_cost,
		const sMultirobotEncodingContext_CNFsat &final_encoding_context,
		sMultirobotSolution                     &computed_solution,
		int                                      thread_id)
	{
		sString output_filename;

		printf("Extracting MDD ...\n");

		if (thread_id != THREAD_ID_UNDEFINED)
		{
			output_filename = OUTPUT_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
			output_filename = OUTPUT_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		FILE *fr;
		
		if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
		{
			return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
		}
		
		char answer[32];
		answer[0] = '\0';

		fscanf(fr, "%s\n", answer);

		Arrangements_vector mdd_Arrangements;
		int mdd_depth = MDD[1].size();

		for (int depth = 0; depth < mdd_depth; ++depth)
		{
			mdd_Arrangements.push_back(sRobotArrangement(start_arrangement.get_VertexCount(), start_arrangement.get_RobotCount()));
		}

		while (!feof(fr))
		{
			int literal;
			fscanf(fr, "%d", &literal); 

#ifdef sDEBUG
/*
	    sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));

	    if (!specified_identifier.is_Null())
	    {
		const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();
		
		if (base_name.find("vertex_water_cardinality") == 0)
		{
		    printf("%s\n", base_name.c_str());
		    printf("%d\n", literal);
		}
	    }
*/
#endif
			if (literal > 0)
			{
				sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));

				if (!specified_identifier.is_Null())
				{
					const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();

					if (base_name.find("vertex_occupancy_by_water") == 0)
					{   
						int robot_id = sUInt_32_from_String(base_name.substr(26, base_name.find('_', 26) - 26));
						int level = sUInt_32_from_String(base_name.substr(base_name.find('_', 26) + 1, base_name.size()));



						int mdd_index = *specified_identifier.get_ScopeIndexes()[0] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[0]->get_Begin());
//			printf("mdd idx:%d\n", mdd_index);

						int vertex_id = MDD[robot_id][level][mdd_index];

//			printf("%d: %d <- %d\n", level, vertex_id, robot_id);
						mdd_Arrangements[level].place_Robot(robot_id, vertex_id);
/*
			printf("%s\n", base_name.c_str());
			printf("level:%d,%d\n", level, mdd_Environments.size());

			if (level == heighted_Environments.size() - 1)
			{
			    const sVertex::Neighbors_list &out_Neighbors = heighted_Environments[level].get_Vertex(vertex_id)->m_Neighbors;
			    sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin();

			    while (--neighbor_id >= 0)
			    {
				++out_neighbor;
			    }

			    Edge__ edge;
			    
			    edge.m_source_id = vertex_id;
			    edge.m_target_id = (*out_neighbor)->m_target->m_id;
			    edge.m_level = level;
			    
			    Edges.push_back(edge);
			}
			else
			{
			    if (neighbor_id == 0)
			    {
				Edge__ edge;

				edge.m_source_id = vertex_id;
				edge.m_target_id = vertex_id;
				edge.m_level = level;
			    
				Edges.push_back(edge);
			    }
			    else
			    {
				--neighbor_id;

				const sVertex::Neighbors_list &out_Neighbors = heighted_Environments[level].get_Vertex(vertex_id)->m_Neighbors;
				sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin();
				
				while (--neighbor_id >= 0)
				{
				    ++out_neighbor;
				}
				
				Edge__ edge;
			    
				edge.m_source_id = vertex_id;
				edge.m_target_id = (*out_neighbor)->m_target->m_id;
				edge.m_level = level;
			    
				Edges.push_back(edge);
			    }
			}
*/
					}
				}
			}
		}

		fclose(fr);

		int N_Robots = start_arrangement.get_RobotCount();

		for (int depth = 0; depth < mdd_depth - 1; ++depth)
		{
			for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
			{
				if (mdd_Arrangements[depth].get_RobotLocation(robot_id) != mdd_Arrangements[depth + 1].get_RobotLocation(robot_id))
				{
					sASSERT(environment.is_Adjacent(mdd_Arrangements[depth].get_RobotLocation(robot_id), mdd_Arrangements[depth + 1].get_RobotLocation(robot_id)));
					computed_solution.add_Move(depth, sMultirobotSolution::Move(robot_id, mdd_Arrangements[depth].get_RobotLocation(robot_id), mdd_Arrangements[depth + 1].get_RobotLocation(robot_id)));
				}
			}
		}

	#ifdef sDEBUG
/*
	for (int depth = 0; depth < mdd_depth; ++depth)
	{
	    mdd_Arrangements[depth].to_Screen();
	}
*/
	#endif

	#ifndef sDEBUG
		{
			if (unlink(output_filename.c_str()) < 0)
			{
				return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
			}	
		}
	#endif

		return sRESULT_SUCCESS;
	}


	sResult sMultirobotSolutionCompressor::extract_ComputedRelaxedMddSolution(const sRobotArrangement                 &start_arrangement,
#ifdef sDEBUG
		const sUndirectedGraph                  &environment,
#else
		const sUndirectedGraph                  &,
#endif										       								       
		const sMultirobotInstance::MDD_vector   &MDD,
		int                                      computed_cost,
		const sMultirobotEncodingContext_CNFsat &final_encoding_context,
		sMultirobotSolution                     &computed_solution,
		int                                      thread_id)
	{
		sString output_filename;

		printf("Extracting Relaxed MDD ...\n");

		if (thread_id != THREAD_ID_UNDEFINED)
		{
			output_filename = OUTPUT_RELAXED_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
			output_filename = OUTPUT_RELAXED_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		FILE *fr;
		
		if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
		{
			return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
		}
		
		char answer[32];
		answer[0] = '\0';

		fscanf(fr, "%s\n", answer);

		Arrangements_vector mdd_Arrangements;
		int mdd_depth = MDD[1].size();

		for (int depth = 0; depth < mdd_depth; ++depth)
		{
			mdd_Arrangements.push_back(sRobotArrangement(start_arrangement.get_VertexCount(), start_arrangement.get_RobotCount()));
		}

		while (!feof(fr))
		{
			int literal;
			fscanf(fr, "%d", &literal); 

#ifdef sDEBUG
/*
	    sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));

	    if (!specified_identifier.is_Null())
	    {
		const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();
		
		if (base_name.find("vertex_water_cardinality") == 0)
		{
		    printf("%s\n", base_name.c_str());
		    printf("%d\n", literal);
		}
	    }
*/
#endif
			if (literal > 0)
			{
				sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));

				if (!specified_identifier.is_Null())
				{
					const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();

					if (base_name.find("vertex_occupancy_by_water") == 0)
					{   
						int robot_id = sUInt_32_from_String(base_name.substr(26, base_name.find('_', 26) - 26));
						int level = sUInt_32_from_String(base_name.substr(base_name.find('_', 26) + 1, base_name.size()));



						int mdd_index = *specified_identifier.get_ScopeIndexes()[0] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[0]->get_Begin());
//			printf("mdd idx:%d\n", mdd_index);

						int vertex_id = MDD[robot_id][level][mdd_index];

//			printf("%d: %d <- %d\n", level, vertex_id, robot_id);
						mdd_Arrangements[level].place_Robot(robot_id, vertex_id);
/*
			printf("%s\n", base_name.c_str());
			printf("level:%d,%d\n", level, mdd_Environments.size());

			if (level == heighted_Environments.size() - 1)
			{
			    const sVertex::Neighbors_list &out_Neighbors = heighted_Environments[level].get_Vertex(vertex_id)->m_Neighbors;
			    sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin();

			    while (--neighbor_id >= 0)
			    {
				++out_neighbor;
			    }

			    Edge__ edge;
			    
			    edge.m_source_id = vertex_id;
			    edge.m_target_id = (*out_neighbor)->m_target->m_id;
			    edge.m_level = level;
			    
			    Edges.push_back(edge);
			}
			else
			{
			    if (neighbor_id == 0)
			    {
				Edge__ edge;

				edge.m_source_id = vertex_id;
				edge.m_target_id = vertex_id;
				edge.m_level = level;
			    
				Edges.push_back(edge);
			    }
			    else
			    {
				--neighbor_id;

				const sVertex::Neighbors_list &out_Neighbors = heighted_Environments[level].get_Vertex(vertex_id)->m_Neighbors;
				sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin();
				
				while (--neighbor_id >= 0)
				{
				    ++out_neighbor;
				}
				
				Edge__ edge;
			    
				edge.m_source_id = vertex_id;
				edge.m_target_id = (*out_neighbor)->m_target->m_id;
				edge.m_level = level;
			    
				Edges.push_back(edge);
			    }
			}
*/
					}
				}
			}
		}

		fclose(fr);

		int N_Robots = start_arrangement.get_RobotCount();

		for (int depth = 0; depth < mdd_depth - 1; ++depth)
		{
			for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
			{
				if (mdd_Arrangements[depth].get_RobotLocation(robot_id) != mdd_Arrangements[depth + 1].get_RobotLocation(robot_id))
				{
					sASSERT(environment.is_Adjacent(mdd_Arrangements[depth].get_RobotLocation(robot_id), mdd_Arrangements[depth + 1].get_RobotLocation(robot_id)));
					computed_solution.add_Move(depth, sMultirobotSolution::Move(robot_id, mdd_Arrangements[depth].get_RobotLocation(robot_id), mdd_Arrangements[depth + 1].get_RobotLocation(robot_id)));
				}
			}
		}

	#ifdef sDEBUG
/*
	for (int depth = 0; depth < mdd_depth; ++depth)
	{
	    mdd_Arrangements[depth].to_Screen();
	}
*/
	#endif

	#ifndef sDEBUG
		{
			if (unlink(output_filename.c_str()) < 0)
			{
				return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
			}	
		}
	#endif

		return sRESULT_SUCCESS;
	}    


	sResult sMultirobotSolutionCompressor::extract_ComputedMddPlusSolution(const sRobotArrangement                 &start_arrangement,
#ifdef sDEBUG
		const sUndirectedGraph                  &environment,
#else
		const sUndirectedGraph                  &,
#endif
		const sMultirobotInstance::MDD_vector   &MDD,
		int                                      computed_cost,
		const sMultirobotEncodingContext_CNFsat &final_encoding_context,
		sMultirobotSolution                     &computed_solution,
		int                                      thread_id)
	{
		sString output_filename;

		printf("Extracting MDD+ ...\n");

		if (thread_id != THREAD_ID_UNDEFINED)
		{
			output_filename = OUTPUT_MDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
			output_filename = OUTPUT_MDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		FILE *fr;
		
		if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
		{
			return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
		}
		
		char answer[32];
		answer[0] = '\0';

		fscanf(fr, "%s\n", answer);

		Arrangements_vector mdd_Arrangements;
		int mdd_depth = MDD[1].size();

		for (int depth = 0; depth < mdd_depth; ++depth)
		{
			mdd_Arrangements.push_back(sRobotArrangement(start_arrangement.get_VertexCount(), start_arrangement.get_RobotCount()));
		}

		while (!feof(fr))
		{
			int literal;
			fscanf(fr, "%d", &literal); 

#ifdef sDEBUG
/*
	    sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));

	    if (!specified_identifier.is_Null())
	    {
		const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();
		
		if (base_name.find("vertex_water_cardinality") == 0)
		{
		    printf("%s\n", base_name.c_str());
		    printf("%d\n", literal);
		}
	    }
*/
#endif
			if (literal > 0)
			{
				sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));

				if (!specified_identifier.is_Null())
				{
					const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();

					if (base_name.find("vertex_occupancy_by_water") == 0)
					{   
						int robot_id = sUInt_32_from_String(base_name.substr(26, base_name.find('_', 26) - 26));
						int level = sUInt_32_from_String(base_name.substr(base_name.find('_', 26) + 1, base_name.size()));



						int mdd_index = *specified_identifier.get_ScopeIndexes()[0] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[0]->get_Begin());
//			printf("mdd idx:%d\n", mdd_index);

						int vertex_id = MDD[robot_id][level][mdd_index];

//			printf("%d: %d <- %d\n", level, vertex_id, robot_id);
						mdd_Arrangements[level].place_Robot(robot_id, vertex_id);
/*
			printf("%s\n", base_name.c_str());
			printf("level:%d,%d\n", level, mdd_Environments.size());

			if (level == heighted_Environments.size() - 1)
			{
			    const sVertex::Neighbors_list &out_Neighbors = heighted_Environments[level].get_Vertex(vertex_id)->m_Neighbors;
			    sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin();

			    while (--neighbor_id >= 0)
			    {
				++out_neighbor;
			    }

			    Edge__ edge;
			    
			    edge.m_source_id = vertex_id;
			    edge.m_target_id = (*out_neighbor)->m_target->m_id;
			    edge.m_level = level;
			    
			    Edges.push_back(edge);
			}
			else
			{
			    if (neighbor_id == 0)
			    {
				Edge__ edge;

				edge.m_source_id = vertex_id;
				edge.m_target_id = vertex_id;
				edge.m_level = level;
			    
				Edges.push_back(edge);
			    }
			    else
			    {
				--neighbor_id;

				const sVertex::Neighbors_list &out_Neighbors = heighted_Environments[level].get_Vertex(vertex_id)->m_Neighbors;
				sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin();
				
				while (--neighbor_id >= 0)
				{
				    ++out_neighbor;
				}
				
				Edge__ edge;
			    
				edge.m_source_id = vertex_id;
				edge.m_target_id = (*out_neighbor)->m_target->m_id;
				edge.m_level = level;
			    
				Edges.push_back(edge);
			    }
			}
*/
					}
				}
			}
		}

		fclose(fr);

		int N_Robots = start_arrangement.get_RobotCount();

		for (int depth = 0; depth < mdd_depth - 1; ++depth)
		{
			for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
			{
				if (mdd_Arrangements[depth].get_RobotLocation(robot_id) != mdd_Arrangements[depth + 1].get_RobotLocation(robot_id))
				{
					sASSERT(environment.is_Adjacent(mdd_Arrangements[depth].get_RobotLocation(robot_id), mdd_Arrangements[depth + 1].get_RobotLocation(robot_id)));
					computed_solution.add_Move(depth, sMultirobotSolution::Move(robot_id, mdd_Arrangements[depth].get_RobotLocation(robot_id), mdd_Arrangements[depth + 1].get_RobotLocation(robot_id)));
				}
			}
		}

	#ifdef sDEBUG
/*
	for (int depth = 0; depth < mdd_depth; ++depth)
	{
	    mdd_Arrangements[depth].to_Screen();
	}
*/
	#endif

	#ifndef sDEBUG
		{
			if (unlink(output_filename.c_str()) < 0)
			{
				return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
			}	
		}
	#endif

		return sRESULT_SUCCESS;
	}    


	sResult sMultirobotSolutionCompressor::extract_ComputedMddPlusPlusSolution(const sRobotArrangement                 &start_arrangement,
#ifdef sDEBUG
		const sUndirectedGraph                  &environment,
#else
		const sUndirectedGraph                  &,
#endif									       
		const sMultirobotInstance::MDD_vector   &MDD,
		int                                      computed_cost,
		const sMultirobotEncodingContext_CNFsat &final_encoding_context,
		sMultirobotSolution                     &computed_solution,
		int                                      thread_id)
	{
		sString output_filename;

		printf("Extracting MDD++ ...\n");

		if (thread_id != THREAD_ID_UNDEFINED)
		{
			output_filename = OUTPUT_MDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
			output_filename = OUTPUT_MDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		FILE *fr;
		
		if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
		{
			return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
		}
		
		char answer[32];
		answer[0] = '\0';

		fscanf(fr, "%s\n", answer);

		Arrangements_vector mdd_Arrangements;
		int mdd_depth = MDD[1].size();

		for (int depth = 0; depth < mdd_depth; ++depth)
		{
			mdd_Arrangements.push_back(sRobotArrangement(start_arrangement.get_VertexCount(), start_arrangement.get_RobotCount()));
		}

		while (!feof(fr))
		{
			int literal;
			fscanf(fr, "%d", &literal); 

#ifdef sDEBUG
/*
	    sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));

	    if (!specified_identifier.is_Null())
	    {
		const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();
		
		if (base_name.find("vertex_water_cardinality") == 0)
		{
		    printf("%s\n", base_name.c_str());
		    printf("%d\n", literal);
		}
	    }
*/
#endif
			if (literal > 0)
			{
				sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));

				if (!specified_identifier.is_Null())
				{
					const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();

					if (base_name.find("vertex_occupancy_by_water") == 0)
					{   
						int robot_id = sUInt_32_from_String(base_name.substr(26, base_name.find('_', 26) - 26));
						int level = sUInt_32_from_String(base_name.substr(base_name.find('_', 26) + 1, base_name.size()));



						int mdd_index = *specified_identifier.get_ScopeIndexes()[0] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[0]->get_Begin());
//			printf("mdd idx:%d\n", mdd_index);

						int vertex_id = MDD[robot_id][level][mdd_index];

//			printf("%d: %d <- %d\n", level, vertex_id, robot_id);
						mdd_Arrangements[level].place_Robot(robot_id, vertex_id);
/*
			printf("%s\n", base_name.c_str());
			printf("level:%d,%d\n", level, mdd_Environments.size());

			if (level == heighted_Environments.size() - 1)
			{
			    const sVertex::Neighbors_list &out_Neighbors = heighted_Environments[level].get_Vertex(vertex_id)->m_Neighbors;
			    sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin();

			    while (--neighbor_id >= 0)
			    {
				++out_neighbor;
			    }

			    Edge__ edge;
			    
			    edge.m_source_id = vertex_id;
			    edge.m_target_id = (*out_neighbor)->m_target->m_id;
			    edge.m_level = level;
			    
			    Edges.push_back(edge);
			}
			else
			{
			    if (neighbor_id == 0)
			    {
				Edge__ edge;

				edge.m_source_id = vertex_id;
				edge.m_target_id = vertex_id;
				edge.m_level = level;
			    
				Edges.push_back(edge);
			    }
			    else
			    {
				--neighbor_id;

				const sVertex::Neighbors_list &out_Neighbors = heighted_Environments[level].get_Vertex(vertex_id)->m_Neighbors;
				sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin();
				
				while (--neighbor_id >= 0)
				{
				    ++out_neighbor;
				}
				
				Edge__ edge;
			    
				edge.m_source_id = vertex_id;
				edge.m_target_id = (*out_neighbor)->m_target->m_id;
				edge.m_level = level;
			    
				Edges.push_back(edge);
			    }
			}
*/
					}
				}
			}
		}

		fclose(fr);

		int N_Robots = start_arrangement.get_RobotCount();

		for (int depth = 0; depth < mdd_depth - 1; ++depth)
		{
			for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
			{
				if (mdd_Arrangements[depth].get_RobotLocation(robot_id) != mdd_Arrangements[depth + 1].get_RobotLocation(robot_id))
				{
					sASSERT(environment.is_Adjacent(mdd_Arrangements[depth].get_RobotLocation(robot_id), mdd_Arrangements[depth + 1].get_RobotLocation(robot_id)));
					computed_solution.add_Move(depth, sMultirobotSolution::Move(robot_id, mdd_Arrangements[depth].get_RobotLocation(robot_id), mdd_Arrangements[depth + 1].get_RobotLocation(robot_id)));
				}
			}
		}

	#ifdef sDEBUG
/*
	for (int depth = 0; depth < mdd_depth; ++depth)
	{
	    mdd_Arrangements[depth].to_Screen();
	}
*/
	#endif

	#ifndef sDEBUG
		{
			if (unlink(output_filename.c_str()) < 0)
			{
				return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
			}	
		}
	#endif

		return sRESULT_SUCCESS;
	}    
	

	sResult sMultirobotSolutionCompressor::extract_ComputedMmddSolution(const sRobotArrangement                 &start_arrangement,
#ifdef sDEBUG
		const sUndirectedGraph                  &environment,
#else
		const sUndirectedGraph                  &,
#endif
		const sMultirobotInstance::MDD_vector   &MDD,
		int                                      computed_makespan,
		const sMultirobotEncodingContext_CNFsat &final_encoding_context,
		sMultirobotSolution                     &computed_solution,
		int                                      thread_id)
	{
		sString output_filename;

		printf("Extracting MMDD ...\n");

		if (thread_id != THREAD_ID_UNDEFINED)
		{
			output_filename = OUTPUT_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
			output_filename = OUTPUT_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		FILE *fr;
		
#ifdef sVERBOSE
		{
			printf("Filename:%s\n", output_filename.c_str());
		}
#endif
		
		if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
		{
			return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
		}
		
		char answer[32];
		answer[0] = '\0';

		fscanf(fr, "%s\n", answer);

		Arrangements_vector mdd_Arrangements;
		int mdd_depth = MDD[1].size();

		for (int depth = 0; depth < mdd_depth; ++depth)
		{
			mdd_Arrangements.push_back(sRobotArrangement(start_arrangement.get_VertexCount(), start_arrangement.get_RobotCount()));
		}

		while (!feof(fr))
		{
			int literal;
			fscanf(fr, "%d", &literal); 

#ifdef sDEBUG
/*
	    sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));

	    if (!specified_identifier.is_Null())
	    {
		const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();
		
		if (base_name.find("vertex_water_cardinality") == 0)
		{
		    printf("%s\n", base_name.c_str());
		    printf("%d\n", literal);
		}
	    }
*/
#endif
			if (literal > 0)
			{
				sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));

				if (!specified_identifier.is_Null())
				{
					const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();

					if (base_name.find("vertex_occupancy_by_water") == 0)
					{   
						int robot_id = sUInt_32_from_String(base_name.substr(26, base_name.find('_', 26) - 26));
						int level = sUInt_32_from_String(base_name.substr(base_name.find('_', 26) + 1, base_name.size()));

						int mdd_index = *specified_identifier.get_ScopeIndexes()[0] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[0]->get_Begin());
						int vertex_id = MDD[robot_id][level][mdd_index];

						mdd_Arrangements[level].place_Robot(robot_id, vertex_id);
					}
				}
			}
		}
		fclose(fr);

		int N_Robots = start_arrangement.get_RobotCount();

		for (int depth = 0; depth < mdd_depth - 1; ++depth)
		{
			for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
			{
				if (mdd_Arrangements[depth].get_RobotLocation(robot_id) != mdd_Arrangements[depth + 1].get_RobotLocation(robot_id))
				{
					sASSERT(environment.is_Adjacent(mdd_Arrangements[depth].get_RobotLocation(robot_id), mdd_Arrangements[depth + 1].get_RobotLocation(robot_id)));
					computed_solution.add_Move(depth, sMultirobotSolution::Move(robot_id, mdd_Arrangements[depth].get_RobotLocation(robot_id), mdd_Arrangements[depth + 1].get_RobotLocation(robot_id)));
				}
			}
		}

#ifdef sVERBOSE
		for (int depth = 0; depth < mdd_depth; ++depth)
		{
			mdd_Arrangements[depth].to_Screen();
		}
#endif

	#ifndef sDEBUG
		{
			if (unlink(output_filename.c_str()) < 0)
			{
				return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
			}	
		}
	#endif

		return sRESULT_SUCCESS;
	}


	sResult sMultirobotSolutionCompressor::extract_ComputedRelaxedMmddSolution(const sRobotArrangement                 &start_arrangement,
#ifdef sDEBUG
		const sUndirectedGraph                  &environment,
#else
		const sUndirectedGraph                  &,
#endif
		const sMultirobotInstance::MDD_vector   &MDD,
		int                                      computed_makespan,
		const sMultirobotEncodingContext_CNFsat &final_encoding_context,
		sMultirobotSolution                     &computed_solution,
		int                                      thread_id)
	{
		sString output_filename;

		printf("Extracting Relaxed MMDD ...\n");

		if (thread_id != THREAD_ID_UNDEFINED)
		{
			output_filename = OUTPUT_RELAXED_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
			output_filename = OUTPUT_RELAXED_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		FILE *fr;
		
#ifdef sVERBOSE
		{
			printf("Filename:%s\n", output_filename.c_str());
		}
#endif
		
		if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
		{
			return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
		}
		
		char answer[32];
		answer[0] = '\0';

		fscanf(fr, "%s\n", answer);

		Arrangements_vector mdd_Arrangements;
		int mdd_depth = MDD[1].size();

		for (int depth = 0; depth < mdd_depth; ++depth)
		{
			mdd_Arrangements.push_back(sRobotArrangement(start_arrangement.get_VertexCount(), start_arrangement.get_RobotCount()));
		}

		while (!feof(fr))
		{
			int literal;
			fscanf(fr, "%d", &literal); 

#ifdef sDEBUG
/*
	    sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));

	    if (!specified_identifier.is_Null())
	    {
		const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();
		
		if (base_name.find("vertex_water_cardinality") == 0)
		{
		    printf("%s\n", base_name.c_str());
		    printf("%d\n", literal);
		}
	    }
*/
#endif
			if (literal > 0)
			{
				sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));

				if (!specified_identifier.is_Null())
				{
					const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();

					if (base_name.find("vertex_occupancy_by_water") == 0)
					{   
						int robot_id = sUInt_32_from_String(base_name.substr(26, base_name.find('_', 26) - 26));
						int level = sUInt_32_from_String(base_name.substr(base_name.find('_', 26) + 1, base_name.size()));

						int mdd_index = *specified_identifier.get_ScopeIndexes()[0] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[0]->get_Begin());
						int vertex_id = MDD[robot_id][level][mdd_index];

						mdd_Arrangements[level].place_Robot(robot_id, vertex_id);
					}
				}
			}
		}
		fclose(fr);

		int N_Robots = start_arrangement.get_RobotCount();

		for (int depth = 0; depth < mdd_depth - 1; ++depth)
		{
			for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
			{
				if (mdd_Arrangements[depth].get_RobotLocation(robot_id) != mdd_Arrangements[depth + 1].get_RobotLocation(robot_id))
				{
					sASSERT(environment.is_Adjacent(mdd_Arrangements[depth].get_RobotLocation(robot_id), mdd_Arrangements[depth + 1].get_RobotLocation(robot_id)));
					computed_solution.add_Move(depth, sMultirobotSolution::Move(robot_id, mdd_Arrangements[depth].get_RobotLocation(robot_id), mdd_Arrangements[depth + 1].get_RobotLocation(robot_id)));
				}
			}
		}

#ifdef sVERBOSE
		for (int depth = 0; depth < mdd_depth; ++depth)
		{
			mdd_Arrangements[depth].to_Screen();
		}
#endif

	#ifndef sDEBUG
		{
			if (unlink(output_filename.c_str()) < 0)
			{
				return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
			}	
		}
	#endif

		return sRESULT_SUCCESS;
	}    


	sResult sMultirobotSolutionCompressor::extract_ComputedMmddPlusSolution(const sRobotArrangement                 &start_arrangement,
#ifdef sDEBUG
		const sUndirectedGraph                  &environment,
#else
		const sUndirectedGraph                  &,
#endif
		const sMultirobotInstance::MDD_vector   &MDD,
		int                                      computed_makespan,
		const sMultirobotEncodingContext_CNFsat &final_encoding_context,
		sMultirobotSolution                     &computed_solution,
		int                                      thread_id)
	{
		sString output_filename;

		printf("Extracting MMDD+ ...\n");

		if (thread_id != THREAD_ID_UNDEFINED)
		{
			output_filename = OUTPUT_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
			output_filename = OUTPUT_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		FILE *fr;
		
		if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
		{
			return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
		}
		
		char answer[32];
		answer[0] = '\0';

		fscanf(fr, "%s\n", answer);

		Arrangements_vector mdd_Arrangements;
		int mdd_depth = MDD[1].size();

		for (int depth = 0; depth < mdd_depth; ++depth)
		{
			mdd_Arrangements.push_back(sRobotArrangement(start_arrangement.get_VertexCount(), start_arrangement.get_RobotCount()));
		}

		while (!feof(fr))
		{
			int literal;
			fscanf(fr, "%d", &literal); 

#ifdef sDEBUG
/*
	    sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));

	    if (!specified_identifier.is_Null())
	    {
		const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();
		
		if (base_name.find("vertex_water_cardinality") == 0)
		{
		    printf("%s\n", base_name.c_str());
		    printf("%d\n", literal);
		}
	    }
*/
#endif
			if (literal > 0)
			{
				sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));

				if (!specified_identifier.is_Null())
				{
					const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();

					if (base_name.find("vertex_occupancy_by_water") == 0)
					{   
						int robot_id = sUInt_32_from_String(base_name.substr(26, base_name.find('_', 26) - 26));
						int level = sUInt_32_from_String(base_name.substr(base_name.find('_', 26) + 1, base_name.size()));

						int mdd_index = *specified_identifier.get_ScopeIndexes()[0] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[0]->get_Begin());

						int vertex_id = MDD[robot_id][level][mdd_index];

						mdd_Arrangements[level].place_Robot(robot_id, vertex_id);
					}
				}
			}
		}
		fclose(fr);

		int N_Robots = start_arrangement.get_RobotCount();

		for (int depth = 0; depth < mdd_depth - 1; ++depth)
		{
			for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
			{
				if (mdd_Arrangements[depth].get_RobotLocation(robot_id) != mdd_Arrangements[depth + 1].get_RobotLocation(robot_id))
				{
					sASSERT(environment.is_Adjacent(mdd_Arrangements[depth].get_RobotLocation(robot_id), mdd_Arrangements[depth + 1].get_RobotLocation(robot_id)));
					computed_solution.add_Move(depth, sMultirobotSolution::Move(robot_id, mdd_Arrangements[depth].get_RobotLocation(robot_id), mdd_Arrangements[depth + 1].get_RobotLocation(robot_id)));
				}
			}
		}

#ifdef sVERBOSE
		for (int depth = 0; depth < mdd_depth; ++depth)
		{
			mdd_Arrangements[depth].to_Screen();
		}
#endif

	#ifndef sDEBUG
		{
			if (unlink(output_filename.c_str()) < 0)
			{
				return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
			}	
		}
	#endif

		return sRESULT_SUCCESS;
	}


	sResult sMultirobotSolutionCompressor::extract_ComputedMmddPlusPlusSolution(const sRobotArrangement                 &start_arrangement,
#ifdef sDEBUG
		const sUndirectedGraph                  &environment,
#else
		const sUndirectedGraph                  &,
#endif
		const sMultirobotInstance::MDD_vector   &MDD,
		int                                      computed_makespan,
		const sMultirobotEncodingContext_CNFsat &final_encoding_context,
		sMultirobotSolution                     &computed_solution,
		int                                      thread_id)
	{
		sString output_filename;

		printf("Extracting MMDD++ ...\n");

		if (thread_id != THREAD_ID_UNDEFINED)
		{
			output_filename = OUTPUT_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
			output_filename = OUTPUT_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_makespan + 1) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		FILE *fr;
		
		if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
		{
			return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
		}
		
		char answer[32];
		answer[0] = '\0';

		fscanf(fr, "%s\n", answer);

		Arrangements_vector mdd_Arrangements;
		int mdd_depth = MDD[1].size();

		for (int depth = 0; depth < mdd_depth; ++depth)
		{
			mdd_Arrangements.push_back(sRobotArrangement(start_arrangement.get_VertexCount(), start_arrangement.get_RobotCount()));
		}

		while (!feof(fr))
		{
			int literal;
			fscanf(fr, "%d", &literal); 

#ifdef sDEBUG
/*
	    sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));

	    if (!specified_identifier.is_Null())
	    {
		const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();
		
		if (base_name.find("vertex_water_cardinality") == 0)
		{
		    printf("%s\n", base_name.c_str());
		    printf("%d\n", literal);
		}
	    }
*/
#endif
			if (literal > 0)
			{
				sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));

				if (!specified_identifier.is_Null())
				{
					const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();

					if (base_name.find("vertex_occupancy_by_water") == 0)
					{   
						int robot_id = sUInt_32_from_String(base_name.substr(26, base_name.find('_', 26) - 26));
						int level = sUInt_32_from_String(base_name.substr(base_name.find('_', 26) + 1, base_name.size()));

						int mdd_index = *specified_identifier.get_ScopeIndexes()[0] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[0]->get_Begin());

						int vertex_id = MDD[robot_id][level][mdd_index];

						mdd_Arrangements[level].place_Robot(robot_id, vertex_id);
					}
				}
			}
		}
		fclose(fr);

		int N_Robots = start_arrangement.get_RobotCount();

		for (int depth = 0; depth < mdd_depth - 1; ++depth)
		{
			for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
			{
				if (mdd_Arrangements[depth].get_RobotLocation(robot_id) != mdd_Arrangements[depth + 1].get_RobotLocation(robot_id))
				{
					sASSERT(environment.is_Adjacent(mdd_Arrangements[depth].get_RobotLocation(robot_id), mdd_Arrangements[depth + 1].get_RobotLocation(robot_id)));
					computed_solution.add_Move(depth, sMultirobotSolution::Move(robot_id, mdd_Arrangements[depth].get_RobotLocation(robot_id), mdd_Arrangements[depth + 1].get_RobotLocation(robot_id)));
				}
			}
		}

#ifdef sVERBOSE
		for (int depth = 0; depth < mdd_depth; ++depth)
		{
			mdd_Arrangements[depth].to_Screen();
		}
#endif

	#ifndef sDEBUG
		{
			if (unlink(output_filename.c_str()) < 0)
			{
				return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
			}	
		}
	#endif

		return sRESULT_SUCCESS;
	}


	sResult sMultirobotSolutionCompressor::extract_ComputedRXMddSolution(const sRobotArrangement                 &start_arrangement,
#ifdef sDEBUG
		const sUndirectedGraph                  &environment,
#else
		const sUndirectedGraph                  &,
#endif									 
		const sMultirobotInstance::MDD_vector   &MDD,
		int                                      computed_cost,
		const sMultirobotEncodingContext_CNFsat &final_encoding_context,
		sMultirobotSolution                     &computed_solution,
		int                                      thread_id)
	{
		sString output_filename;

		printf("Extracting RXMDD ...\n");

		if (thread_id != THREAD_ID_UNDEFINED)
		{
			output_filename = OUTPUT_RXMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
			output_filename = OUTPUT_RXMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		FILE *fr;
		
		if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
		{
			return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
		}
		
		char answer[32];
		answer[0] = '\0';

		fscanf(fr, "%s\n", answer);

		Arrangements_vector mdd_Arrangements;
		int mdd_depth = MDD[1].size();
		printf("Depths:%ld,%ld\n", MDD[1].size(), MDD[2].size());

		for (int depth = 0; depth < mdd_depth; ++depth)
		{
			mdd_Arrangements.push_back(sRobotArrangement(start_arrangement.get_VertexCount(), start_arrangement.get_RobotCount()));
		}

		while (!feof(fr))
		{
			int literal;
			fscanf(fr, "%d", &literal); 

#ifdef sDEBUG
/*
	    sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));

	    if (!specified_identifier.is_Null())
	    {
		const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();
		
		if (base_name.find("vertex_water_cardinality") == 0)
		{
		    printf("%s\n", base_name.c_str());
		    printf("%d\n", literal);
		}
	    }
*/
#endif
			if (literal > 0)
			{
				sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));

				if (!specified_identifier.is_Null())
				{
					const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();

					if (base_name.find("vertex_occupancy_by_water") == 0)
					{   
						int robot_id = sUInt_32_from_String(base_name.substr(26, base_name.find('_', 26) - 26));
						int level = sUInt_32_from_String(base_name.substr(base_name.find('_', 26) + 1, base_name.size()));

						int mdd_index = *specified_identifier.get_ScopeIndexes()[0] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[0]->get_Begin());

						int vertex_id = MDD[robot_id][level][mdd_index];
						mdd_Arrangements[level].place_Robot(robot_id, vertex_id);
//			printf("%d: %d <- %d\n", level, vertex_id, robot_id);
					}
				}
			}
		}
		printf("Extraction finished\n");

		fclose(fr);

		int N_Robots = start_arrangement.get_RobotCount();

		for (int depth = 0; depth < mdd_depth - 1; ++depth)
		{
			for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
			{
				if (mdd_Arrangements[depth].get_RobotLocation(robot_id) != mdd_Arrangements[depth + 1].get_RobotLocation(robot_id))
				{
					sASSERT(environment.is_Adjacent(mdd_Arrangements[depth].get_RobotLocation(robot_id), mdd_Arrangements[depth + 1].get_RobotLocation(robot_id)));
					computed_solution.add_Move(depth, sMultirobotSolution::Move(robot_id, mdd_Arrangements[depth].get_RobotLocation(robot_id), mdd_Arrangements[depth + 1].get_RobotLocation(robot_id)));
				}
			}
		}

	#ifdef sDEBUG
/*
	for (int depth = 0; depth < mdd_depth; ++depth)
	{
	    mdd_Arrangements[depth].to_Screen();
	}
*/
	#endif

	#ifndef sDEBUG
		{
			if (unlink(output_filename.c_str()) < 0)
			{
				return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
			}	
		}
	#endif

		return sRESULT_SUCCESS;
	}


	sResult sMultirobotSolutionCompressor::extract_ComputedNoMddSolution(const sRobotArrangement                 &start_arrangement,
#ifdef sDEBUG
		const sUndirectedGraph                  &environment,
#else
		const sUndirectedGraph                  &,
#endif
		const sMultirobotInstance::MDD_vector   &MDD,
		int                                      computed_cost,
		const sMultirobotEncodingContext_CNFsat &final_encoding_context,
		sMultirobotSolution                     &computed_solution,
		int                                      thread_id)
	{
		sString output_filename;

		printf("Extracting NoMDD ...\n");

		if (thread_id != THREAD_ID_UNDEFINED)
		{
			output_filename = OUTPUT_NOMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
			output_filename = OUTPUT_NOMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		FILE *fr;
		
		if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
		{
			return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
		}
		
		char answer[32];
		answer[0] = '\0';

		fscanf(fr, "%s\n", answer);

		Arrangements_vector mdd_Arrangements;
		int mdd_depth = MDD[1].size();

		for (int depth = 0; depth < mdd_depth; ++depth)
		{
			mdd_Arrangements.push_back(sRobotArrangement(start_arrangement.get_VertexCount(), start_arrangement.get_RobotCount()));
		}
/*
	typedef std::vector<int> Locations_vector;
	typedef std::vector<Locations_vector> Layers_vector;
*/
		typedef std::list<Edge__> Edges_list;
		Edges_list Edges;

		while (!feof(fr))
		{
			int literal;
			fscanf(fr, "%d", &literal); 

			if (literal > 0)
			{
				sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));

				if (!specified_identifier.is_Null())
				{
					const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();

					if (base_name.find("vertex_occupancy_by_water") == 0)
					{   
						int robot_id = sUInt_32_from_String(base_name.substr(26, base_name.find('_', 26) - 26));
						int level = sUInt_32_from_String(base_name.substr(base_name.find('_', 26) + 1, base_name.size()));

						int mdd_index = *specified_identifier.get_ScopeIndexes()[0] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[0]->get_Begin());
//			printf("mdd idx:%d\n", mdd_index);

						int vertex_id = MDD[robot_id][level][mdd_index];

//			printf("%d: %d <- %d\n", level, vertex_id, robot_id);
						mdd_Arrangements[level].place_Robot(robot_id, vertex_id);
					}
				}
			}
		}

		fclose(fr);

		int N_Robots = start_arrangement.get_RobotCount();

		for (int depth = 0; depth < mdd_depth - 1; ++depth)
		{
			for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
			{
				if (mdd_Arrangements[depth].get_RobotLocation(robot_id) != mdd_Arrangements[depth + 1].get_RobotLocation(robot_id))
				{
					sASSERT(environment.is_Adjacent(mdd_Arrangements[depth].get_RobotLocation(robot_id), mdd_Arrangements[depth + 1].get_RobotLocation(robot_id)));
					computed_solution.add_Move(depth, sMultirobotSolution::Move(robot_id, mdd_Arrangements[depth].get_RobotLocation(robot_id), mdd_Arrangements[depth + 1].get_RobotLocation(robot_id)));
				}
			}
		}

	#ifdef sDEBUG
/*
	for (int depth = 0; depth < mdd_depth; ++depth)
	{
	    mdd_Arrangements[depth].to_Screen();
	}
*/
	#endif

	#ifndef sDEBUG
		{
			if (unlink(output_filename.c_str()) < 0)
			{
				return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
			}	
		}
	#endif

		return sRESULT_SUCCESS;
	}


	sResult sMultirobotSolutionCompressor::extract_ComputedRXNoMddSolution(const sRobotArrangement                 &start_arrangement,
#ifdef sDEBUG
		const sUndirectedGraph                  &environment,
#else
		const sUndirectedGraph                  &,
#endif
		const sMultirobotInstance::MDD_vector   &MDD,
		int                                      computed_cost,
		const sMultirobotEncodingContext_CNFsat &final_encoding_context,
		sMultirobotSolution                     &computed_solution,
		int                                      thread_id)
	{
		sString output_filename;

		printf("Extracting RXNoMDD ...\n");

		if (thread_id != THREAD_ID_UNDEFINED)
		{
			output_filename = OUTPUT_RXNOMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
			output_filename = OUTPUT_RXNOMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(computed_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		FILE *fr;
		
		if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
		{
			return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
		}
		
		char answer[32];
		answer[0] = '\0';

		fscanf(fr, "%s\n", answer);

		Arrangements_vector mdd_Arrangements;
		int mdd_depth = MDD[1].size();

		for (int depth = 0; depth < mdd_depth; ++depth)
		{
			mdd_Arrangements.push_back(sRobotArrangement(start_arrangement.get_VertexCount(), start_arrangement.get_RobotCount()));
		}

		while (!feof(fr))
		{
			int literal;
			fscanf(fr, "%d", &literal); 

#ifdef sDEBUG
/*
	    sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));

	    if (!specified_identifier.is_Null())
	    {
		const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();
		
		if (base_name.find("vertex_water_cardinality") == 0)
		{
		    printf("%s\n", base_name.c_str());
		    printf("%d\n", literal);
		}
	    }
*/
#endif
			if (literal > 0)
			{
				sSpecifiedIdentifier specified_identifier = final_encoding_context.translate_CNF_Variable(sABS(literal));

				if (!specified_identifier.is_Null())
				{
					const sString &base_name = specified_identifier.get_IndexableIdentifier()->get_BaseName();

					if (base_name.find("vertex_occupancy_by_water") == 0)
					{   
						int robot_id = sUInt_32_from_String(base_name.substr(26, base_name.find('_', 26) - 26));
						int level = sUInt_32_from_String(base_name.substr(base_name.find('_', 26) + 1, base_name.size()));



						int mdd_index = *specified_identifier.get_ScopeIndexes()[0] - *(specified_identifier.get_IndexableIdentifier()->get_IndexScopes()[0]->get_Begin());
//			printf("mdd idx:%d\n", mdd_index);

						int vertex_id = MDD[robot_id][level][mdd_index];

//			printf("%d: %d <- %d\n", level, vertex_id, robot_id);
						mdd_Arrangements[level].place_Robot(robot_id, vertex_id);
					}
				}
			}
		}

		fclose(fr);

		int N_Robots = start_arrangement.get_RobotCount();

	#ifdef sDEBUG
/*
	for (int depth = 0; depth < mdd_depth; ++depth)
	{
	    mdd_Arrangements[depth].to_Screen();
	}
*/
	#endif

		for (int depth = 0; depth < mdd_depth - 1; ++depth)
		{
			for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
			{
				if (mdd_Arrangements[depth].get_RobotLocation(robot_id) != mdd_Arrangements[depth + 1].get_RobotLocation(robot_id))
				{
					sASSERT(environment.is_Adjacent(mdd_Arrangements[depth].get_RobotLocation(robot_id), mdd_Arrangements[depth + 1].get_RobotLocation(robot_id)));
					computed_solution.add_Move(depth, sMultirobotSolution::Move(robot_id, mdd_Arrangements[depth].get_RobotLocation(robot_id), mdd_Arrangements[depth + 1].get_RobotLocation(robot_id)));
				}
			}
//	    computed_solution.to_Screen();
		}

	#ifndef sDEBUG
		{
			if (unlink(output_filename.c_str()) < 0)
			{
				return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
			}	
		}
	#endif

		return sRESULT_SUCCESS;
	}

	
/*----------------------------------------------------------------------------*/

} // namespace sReloc
