/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                            reLOC 0.12-beersheva                            */
/*                                                                            */
/*                   (C) Copyright 2011-2015 Pavel Surynek                    */
/*            http://www.surynek.com | <pavel.surynek@mff.cuni.cz>            */
/*                                                                            */
/*                                                                            */
/*============================================================================*/
// multirobot.cpp / 0.12-beersheva_077
/*----------------------------------------------------------------------------*/
//
// Multirobot coordinated path-finding solving package.
//
/*----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

#include <map>
//#include <unordered_map>

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
// sRobotArrangement

    sRobotArrangement::sRobotArrangement()
    {
	// nothing
    }
    

    sRobotArrangement::sRobotArrangement(int N_Vertices, int N_Robots, bool random)
	: m_robot_Locs(N_Robots + 1),
	  m_vertex_Occups(N_Vertices)
    {
	for (int i = 0; i < N_Vertices; ++i)
	{
	    m_vertex_Occups[i] = VACANT_VERTEX;
	}
	if (random)
	{
	    Vertices_vector Vertices;
	    for (int i = 0; i < N_Vertices; ++i)
	    {
		Vertices.push_back(i);
	    }

	    int remain = N_Vertices;
	    for (int r = N_Robots; r >= 1;)
	    {
		if (remain <= 0)
		{
		    break;
		}
		int rnd = rand() % remain;
		place_Robot(r--, Vertices[rnd]);

		Vertices[rnd] = Vertices[--remain];
	    }
	}
	else
	{
	    for (int i = 0; i <= N_Robots; ++i)
	    {
		m_robot_Locs[i] = UNDEFINED_LOCATION;
	    }
	}
    }


    sRobotArrangement::sRobotArrangement(const sRobotArrangement &initial_arrangement, int N_Vertices, int N_Robots, bool random)
	: m_robot_Locs(N_Robots + 1),
	  m_vertex_Occups(N_Vertices)
    {
	for (int i = 0; i < N_Vertices; ++i)
	{
	    m_vertex_Occups[i] = VACANT_VERTEX;
	}
	if (random)
	{
	    Vertices_vector Vertices;
	    int remain = 0;
	    for (int i = 0; i < N_Vertices; ++i)
	    {
		if (initial_arrangement.get_VertexOccupancy(i) == 0)
		{
		    Vertices.push_back(i);
		    ++remain;
		}
	    }
	    for (int r = N_Robots; r >= 1;)
	    {
		if (remain <= 0)
		{
		    break;
		}
		int rnd = rand() % remain;
		place_Robot(r--, Vertices[rnd]);

		Vertices[rnd] = Vertices[--remain];
	    }
	}
	else
	{
	    for (int i = 0; i <= N_Robots; ++i)
	    {
		m_robot_Locs[i] = UNDEFINED_LOCATION;
	    }
	}
    }


    sRobotArrangement::sRobotArrangement(const sRobotArrangement &initial_arrangement, int N_Vertices, int N_Robots, int N_fixed, bool random)
	: m_robot_Locs(N_Robots + 1),
	  m_vertex_Occups(N_Vertices)
    {
	for (int i = 0; i < N_Vertices; ++i)
	{
	    m_vertex_Occups[i] = VACANT_VERTEX;
	}
	if (random)
	{
	    Vertices_vector Vertices;
	    int remain = 0;

	    for (int i = 0; i < N_Vertices; ++i)
	    {
		if (initial_arrangement.get_VertexOccupancy(i) == 0)
		{
		    Vertices.push_back(i);
		    ++remain;
		}
	    }

	    int r = N_Robots;
	    for (int f = 0; f < N_fixed; ++f)
	    {
		place_Robot(r, initial_arrangement.get_RobotLocation(r));
		--r;
	    }
	    while (r >= 1)
	    {
		if (remain <= 0)
		{
		    break;
		}
		int rnd = rand() % remain;
		place_Robot(r--, Vertices[rnd]);

		Vertices[rnd] = Vertices[--remain];
	    }
	}
	else
	{
	    for (int i = 0; i <= N_Robots; ++i)
	    {
		m_robot_Locs[i] = UNDEFINED_LOCATION;
	    }
	}
    }


    sRobotArrangement::sRobotArrangement(const sRobotArrangement &robot_arrangement)
	: m_robot_Locs(robot_arrangement.m_robot_Locs),
	  m_vertex_Occups(robot_arrangement.m_vertex_Occups)
    {
	// nothing
    }


    const sRobotArrangement& sRobotArrangement::operator=(const sRobotArrangement &robot_arrangement)
    {
	m_robot_Locs = robot_arrangement.m_robot_Locs;
	m_vertex_Occups = robot_arrangement.m_vertex_Occups;

	return *this;
    }


    bool sRobotArrangement::operator==(const sRobotArrangement &robot_arrangement) const
    {
	sASSERT(m_robot_Locs.size() == robot_arrangement.m_robot_Locs.size());

	Robots_vector::const_iterator robot_A, robot_B;
	
	for (robot_A = m_robot_Locs.begin(), robot_B = robot_arrangement.m_robot_Locs.begin();
	     robot_A != m_robot_Locs.end();
	     ++robot_A, ++robot_B)
	{
	    if (*robot_A != *robot_B)
	    {
		return false;
	    }
	}
	return true;
    }


    bool sRobotArrangement::operator<(const sRobotArrangement &robot_arrangement) const
    {
	sASSERT(m_robot_Locs.size() == robot_arrangement.m_robot_Locs.size());

	Robots_vector::const_iterator robot_A, robot_B;
	
	for (robot_A = m_robot_Locs.begin(), robot_B = robot_arrangement.m_robot_Locs.begin();
	     robot_A != m_robot_Locs.end();
	     ++robot_A, ++robot_B)
	{
	    if (*robot_A < *robot_B)
	    {
		return true;
	    }
	    else
	    {
		if (*robot_A > *robot_B)
		{
		    return false;
		}
	    }
	}

	return false;
    }


    int sRobotArrangement::get_RobotCount(void) const
    {
	return (m_robot_Locs.size() - 1);
    }

    
    int sRobotArrangement::get_VertexCount(void) const
    {
	return m_vertex_Occups.size();
    }
	

    int sRobotArrangement::get_RobotLocation(int robot_id) const
    {
	sASSERT(robot_id > 0 && robot_id < m_robot_Locs.size());

	return m_robot_Locs[robot_id];
    }


    int sRobotArrangement::get_VertexOccupancy(int vertex_id) const
    {
	sASSERT(vertex_id < m_vertex_Occups.size());

	return m_vertex_Occups[vertex_id];
    }


    void sRobotArrangement::place_Robot(int robot_id, int vertex_id)
    {
	sASSERT(robot_id > 0 && robot_id < m_robot_Locs.size());
	sASSERT(vertex_id < m_vertex_Occups.size());
	sASSERT(m_vertex_Occups[vertex_id] == VACANT_VERTEX);
	
	m_robot_Locs[robot_id] = vertex_id;
	m_vertex_Occups[vertex_id] = robot_id;
    }


    void sRobotArrangement::remove_Robot(int robot_id)
    {
	sASSERT(robot_id > 0 && robot_id < m_robot_Locs.size());

	m_vertex_Occups[m_robot_Locs[robot_id]] = VACANT_VERTEX;
	m_robot_Locs[robot_id] = UNDEFINED_LOCATION;
    }


    void sRobotArrangement::clean_Vertex(int vertex_id)
    {
	sASSERT(vertex_id < m_vertex_Occups.size());

	m_robot_Locs[m_vertex_Occups[vertex_id]] = UNDEFINED_LOCATION;
	m_vertex_Occups[vertex_id] = VACANT_VERTEX;
    }


    void sRobotArrangement::move_Robot(int robot_id, int dest_vertex_id)
    {
	sASSERT(robot_id > 0 && robot_id < m_robot_Locs.size());
	sASSERT(dest_vertex_id < m_vertex_Occups.size());

//	printf("%d # %d -> %d\n", robot_id, m_robot_Locs[robot_id], dest_vertex_id);

	#ifdef sDEBUG
	{
	    if (m_vertex_Occups[dest_vertex_id] != VACANT_VERTEX)
	    {
		printf("--> %d # %d -> %d\n", robot_id, m_robot_Locs[robot_id], dest_vertex_id);
	    }
	}
        #endif
	sASSERT(m_vertex_Occups[dest_vertex_id] == VACANT_VERTEX);

	m_vertex_Occups[m_robot_Locs[robot_id]] = VACANT_VERTEX;
	m_robot_Locs[robot_id] = dest_vertex_id;
	m_vertex_Occups[dest_vertex_id] = robot_id;

        #ifdef sSTATISTICS
	{
	    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_move_Executions;
	}
	#endif
    }


    bool sRobotArrangement::verify_Move(int robot_id, int dest_vertex_id, const sUndirectedGraph &graph) const
    {
	if (   (robot_id > 0 && robot_id < m_robot_Locs.size())
	    && (dest_vertex_id < m_vertex_Occups.size())
	    && (m_vertex_Occups[dest_vertex_id] == VACANT_VERTEX)
	    && (m_vertex_Occups[m_robot_Locs[robot_id]] == robot_id)
	    && graph.is_Adjacent(m_robot_Locs[robot_id], dest_vertex_id))
	{
	    return true;
	}

	#ifdef sDEBUG
	{
	    printf("%d # %d -> %d (adjacent:%d)\n", robot_id, m_robot_Locs[robot_id], dest_vertex_id, graph.is_Adjacent(m_robot_Locs[robot_id], dest_vertex_id) ? 1 : 0);
	}
	#endif

	return false;
    }


    bool sRobotArrangement::verify_Move(int robot_id, int dest_vertex_id) const
    {
	if (   (robot_id > 0 && robot_id < m_robot_Locs.size())
	    && (dest_vertex_id < m_vertex_Occups.size())
	    && (m_vertex_Occups[dest_vertex_id] == VACANT_VERTEX)
	    && (m_vertex_Occups[m_robot_Locs[robot_id]] == robot_id))
	{
	    return true;
	}

	#ifdef sDEBUG
	{
	    printf("%d # %d -> %d\n", robot_id, m_robot_Locs[robot_id], dest_vertex_id);
	}
	#endif

	return false;
    }


    bool sRobotArrangement::check_Move(int robot_id, int dest_vertex_id) const
    {
	if (   (dest_vertex_id < m_vertex_Occups.size())
	    && (m_vertex_Occups[dest_vertex_id] == VACANT_VERTEX))
	{
	    return true;
	}

	#ifdef sDEBUG
	{
	    printf("%d -> %d\n", robot_id, dest_vertex_id);
	}
	#endif

	return false;
    }


    void sRobotArrangement::generate_Walk(const sRobotArrangement &initial_arrangement, const sUndirectedGraph &environment)
    {
	for (int robot_id = 1; robot_id < m_robot_Locs.size(); ++robot_id)
	{
	    const sVertex *vertex = environment.get_Vertex(initial_arrangement.get_RobotLocation(robot_id));

	    for (int i = 0; i < RANDOM_WALK_LENGTH; ++i)
	    {
		if (!vertex->m_Neighbors.empty())
		{
		    int rn = rand() % vertex->m_Neighbors.size();

		    sVertex::Neighbors_list::const_iterator neighbor = vertex->m_Neighbors.begin();
		    for (int n = 0; n < rn; ++n)
		    {
			++neighbor;
		    }
		    vertex = (*neighbor)->m_target;
		}
		else
		{
		    break;
		}
	    }
	    for (int i = 0; i < RANDOM_WALK_LENGTH; ++i)
	    {
		if (get_VertexOccupancy(vertex->m_id) == VACANT_VERTEX)
		{
		    place_Robot(robot_id, vertex->m_id);
		    break;
		}
		if (!vertex->m_Neighbors.empty())
		{
		    int rn = rand() % vertex->m_Neighbors.size();

		    sVertex::Neighbors_list::const_iterator neighbor = vertex->m_Neighbors.begin();
		    for (int n = 0; n < rn; ++n)
		    {
			++neighbor;
		    }
		    vertex = (*neighbor)->m_target;
		}
		else
		{
		    sASSERT(false);
		    break;
		}
	    }
	}
    }


    void sRobotArrangement::generate_Walk(const sRobotArrangement &initial_arrangement, const sUndirectedGraph &environment, int N_fixed)
    {
	for (int robot_id = 1; robot_id < N_fixed; ++robot_id)
	{
	    place_Robot(robot_id, initial_arrangement.get_RobotLocation(robot_id));
	}
	for (int robot_id = N_fixed; robot_id < m_robot_Locs.size(); ++robot_id)
	{
	    const sVertex *vertex = environment.get_Vertex(initial_arrangement.get_RobotLocation(robot_id));

	    for (int i = 0; i < RANDOM_WALK_LENGTH; ++i)
	    {
		if (!vertex->m_Neighbors.empty())
		{
		    int rn = rand() % vertex->m_Neighbors.size();

		    sVertex::Neighbors_list::const_iterator neighbor = vertex->m_Neighbors.begin();
		    for (int n = 0; n < rn; ++n)
		    {
			++neighbor;
		    }
		    vertex = (*neighbor)->m_target;
		}
		else
		{
		    break;
		}
	    }

	    for (int i = 0; i < RANDOM_WALK_LENGTH; ++i)
	    {
		if (get_VertexOccupancy(vertex->m_id) == VACANT_VERTEX)
		{
		    place_Robot(robot_id, vertex->m_id);
		    break;
		}
		if (!vertex->m_Neighbors.empty())
		{
		    int rn = rand() % vertex->m_Neighbors.size();

		    sVertex::Neighbors_list::const_iterator neighbor = vertex->m_Neighbors.begin();
		    for (int n = 0; n < rn; ++n)
		    {
			++neighbor;
		    }
		    vertex = (*neighbor)->m_target;
		}
		else
		{
		    sASSERT(false);
		    break;
		}
	    }
	}
    }


  void sRobotArrangement::generate_Equidistant(const sRobotArrangement &initial_arrangement, sUndirectedGraph &environment, int distance)
  {
    int N_Robots = initial_arrangement.get_RobotCount();
    int N_Vertices = environment.get_VertexCount();

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	  int s_id = initial_arrangement.get_RobotLocation(robot_id);

	  VertexIDs_vector equidistant_IDs, free_equidistant_IDs;
	  environment.collect_EquidistantVertices(s_id, distance, equidistant_IDs);

	  for (VertexIDs_vector::const_iterator equidistant = equidistant_IDs.begin(); equidistant != equidistant_IDs.end(); ++equidistant)
	    {
	      if (get_VertexOccupancy(*equidistant) == VACANT_VERTEX)
		{
		  free_equidistant_IDs.push_back(*equidistant);
		}
	    }
	  if (free_equidistant_IDs.empty())
	    {
	      VertexIDs_vector free_vertex_IDs;

	      for (int i = 0; i < N_Vertices; ++i)
		{
		  if (get_VertexOccupancy(i) == VACANT_VERTEX)
		    {
		      free_vertex_IDs.push_back(i);
		    }
		}
	      int rnd = rand() % free_vertex_IDs.size();
	      place_Robot(robot_id, free_vertex_IDs[rnd]);
	    }
	  else
	    {
	      int rnd = rand() % free_equidistant_IDs.size();
	      place_Robot(robot_id, free_equidistant_IDs[rnd]);
	    }
	}
  }


/*----------------------------------------------------------------------------*/

    void sRobotArrangement::to_Screen(const sString &indent) const
    {
	to_Stream(stdout, indent);
    }


    void sRobotArrangement::to_Stream(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sRobot arrangement: (|R| = %ld, |V| = %ld) [\n", indent.c_str(), m_robot_Locs.size() - 1, m_vertex_Occups.size());
	fprintf(fw, "%s%s robot locations: {", indent.c_str(), sRELOC_INDENT.c_str());
	
	int N_Robots_1 = m_robot_Locs.size();
	for (int i = 1; i < N_Robots_1; ++i)
	{
	    fprintf(fw, "%d#%d ", i, m_robot_Locs[i]);
	}
	fprintf(fw, "}\n");

	fprintf(fw, "%s%s vertex occupancy: {", indent.c_str(), sRELOC_INDENT.c_str());
	
	int N_Vertices = m_vertex_Occups.size();
	for (int i = 0; i < N_Vertices; ++i)
	{
	    fprintf(fw, "%d#%d ", m_vertex_Occups[i], i);
	}
	fprintf(fw, "}\n");

	fprintf(fw, "%s]\n", indent.c_str());
    }


    sResult sRobotArrangement::to_File_multirobot(const sString &filename, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sROBOT_ARRANGEMENT_OPEN_ERROR;
	}
	
	to_Stream_multirobot(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    void sRobotArrangement::to_Stream_multirobot(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sV =\n", indent.c_str());
	
	int N_Vertices = m_vertex_Occups.size();
	for (int i = 0; i < N_Vertices; ++i)
	{
	    fprintf(fw, "(%d:-1)[%d:-1:-1]\n", i, m_vertex_Occups[i]);
	}
    }


    sResult sRobotArrangement::from_File_multirobot(const sString &filename, int component)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sROBOT_ARRANGEMENT_OPEN_ERROR;
	}
	
	result = from_Stream_multirobot(fr, component);
	if (sFAILED(result))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;
    }


    sResult sRobotArrangement::from_Stream_multirobot(FILE *fr, int component)
    {
	m_robot_Locs.clear();
	m_vertex_Occups.clear();

	int N_Robots = 0;
	int N_Vertices = 0;

	int c = fgetc(fr);

	while (c != 'V')
	{
	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
	}

	fscanf(fr, " =\n");

	long position = ftell(fr);
	c = fgetc(fr);

	while (c == '(')
	{
	    int vertex_id, cycle_id, robot_id;

	    switch (component)
	    {
	    case 0:
	    {
		fscanf(fr, "%d:%d)[%d", &vertex_id, &cycle_id, &robot_id);
		break;
	    }
	    case 1:
	    {
		int dummy_robot_1_id;
		fscanf(fr, "%d:%d)[%d:%d", &vertex_id, &cycle_id, &dummy_robot_1_id, &robot_id);
		break;
	    }
	    case 2:
	    {
		int dummy_robot_1_id, dummy_robot_2_id;
		fscanf(fr, "%d:%d)[%d:%d:%d", &vertex_id, &cycle_id, &dummy_robot_1_id, &dummy_robot_2_id, &robot_id);
		break;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }

	    if (robot_id > 0)
	    {
		++N_Robots;
	    }
	    ++N_Vertices;
	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
//	    printf("read: %d,%d,%d\n", vertex_id, cycle_id, robot_id);
	}

	if (fseek(fr, position, SEEK_SET) != 0)
	{
	    return sROBOT_ARRANGEMENT_SEEK_ERROR;
	}
	c = fgetc(fr);

	m_robot_Locs.resize(N_Robots + 1, UNDEFINED_LOCATION);
	m_vertex_Occups.resize(N_Vertices, VACANT_VERTEX);

	while (c == '(')
	{
	    int vertex_id, cycle_id, robot_id;

	    switch (component)
	    {
	    case 0:
	    {
		fscanf(fr, "%d:%d)[%d", &vertex_id, &cycle_id, &robot_id);
		break;
	    }
	    case 1:
	    {
		int dummy_robot_1_id;
		fscanf(fr, "%d:%d)[%d:%d", &vertex_id, &cycle_id, &dummy_robot_1_id, &robot_id);
		break;
	    }
	    case 2:
	    {
		int dummy_robot_1_id, dummy_robot_2_id;
		fscanf(fr, "%d:%d)[%d:%d:%d", &vertex_id, &cycle_id, &dummy_robot_1_id, &dummy_robot_2_id, &robot_id);
		break;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }

	    if (robot_id > 0)
	    {
		m_robot_Locs[robot_id] = vertex_id;
		m_vertex_Occups[vertex_id] = robot_id;
	    }
	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
	}

	return sRESULT_SUCCESS;
    }


/*----------------------------------------------------------------------------*/
// sRobotGoal

    sRobotGoal::sRobotGoal()
    {
	// nothing
    }
    

    sRobotGoal::sRobotGoal(int N_Vertices, int N_Robots)
	: m_robot_Goals(N_Robots + 1)
	, m_goal_Compats(N_Vertices)
    {
	// nothing
    }


    sRobotGoal::sRobotGoal(int N_Vertices, int N_Robots, int N_Goals)
	: m_robot_Goals(N_Robots + 1)
	, m_goal_Compats(N_Vertices)
    {
	for (int i = 1; i <= N_Robots; ++i)
	{
	    int rnd_N_goals = 1 + rand() % N_Goals;

	    sRobotArrangement::Vertices_vector vertex_IDs;

	    for (int j = 0; j < N_Vertices; ++j)
	    {
		vertex_IDs.push_back(j);
	    }
	    for (int j = 0; j < rnd_N_goals; ++j)
	    {
		int rnd_vertex_ID = rand() % vertex_IDs.size();
		charge_Robot(i, rnd_vertex_ID);
		vertex_IDs[rnd_vertex_ID] = *vertex_IDs.rbegin();
		vertex_IDs.pop_back();
	    }
	}
    }


    sRobotGoal::sRobotGoal(const sRobotArrangement &sUNUSED(initial_arrangement), int N_Vertices, int N_Robots, int sUNUSED(N_Goals))
	: m_robot_Goals(N_Robots + 1)
	, m_goal_Compats(N_Vertices)
    {
	sASSERT(false);
    }


    sRobotGoal::sRobotGoal(const sRobotArrangement &robot_arrangement)
	: m_robot_Goals(robot_arrangement.get_RobotCount() + 1)
	, m_goal_Compats(robot_arrangement.get_VertexCount())
    {
	int N_Vertices = robot_arrangement.get_VertexCount();

	for (int i = 0; i < N_Vertices; ++i)
	{
	    int robot_id = robot_arrangement.get_VertexOccupancy(i);
	    if (robot_id > 0)
	    {
		assign_Goal(i, robot_id);
	    }
	}
    }


    sRobotGoal::sRobotGoal(const sRobotGoal &robot_goal)
	: m_robot_Goals(robot_goal.m_robot_Goals)
	, m_goal_Compats(robot_goal.m_goal_Compats)
    {
	// nothing
    }


    const sRobotGoal& sRobotGoal::operator=(const sRobotGoal &robot_goal)
    {
	m_robot_Goals = robot_goal.m_robot_Goals;
	m_goal_Compats = robot_goal.m_goal_Compats;

	return *this;
    }


    int sRobotGoal::get_RobotCount(void) const
    {
	return (m_robot_Goals.size() - 1);
    }


    int sRobotGoal::get_VertexCount(void) const
    {
	return m_goal_Compats.size();
    }

    
    const sRobotGoal::Vertices_set& sRobotGoal::get_RobotGoal(int robot_id) const
    {
	sASSERT(robot_id > 0 && robot_id < m_robot_Goals.size());

	return m_robot_Goals[robot_id];
    }


    sRobotGoal::Vertices_set& sRobotGoal::provide_RobotGoal(int robot_id)
    {
	sASSERT(robot_id > 0 && robot_id < m_robot_Goals.size());

	return m_robot_Goals[robot_id];
    }


    const sRobotGoal::Robots_set& sRobotGoal::get_GoalCompatibility(int goal_id) const
    {
	sASSERT(goal_id >= 0 && goal_id < m_goal_Compats.size());

	return m_goal_Compats[goal_id];
    }


    sRobotGoal::Robots_set& sRobotGoal::provide_GoalCompatibility(int goal_id)
    {
	sASSERT(goal_id >= 0 && goal_id < m_goal_Compats.size());

	return m_goal_Compats[goal_id];
    }


    void sRobotGoal::charge_Robot(int robot_id, int goal_id)
    {
	sASSERT(robot_id > 0 && robot_id < m_robot_Goals.size());
	sASSERT(goal_id >= 0 && goal_id < m_goal_Compats.size());

	m_robot_Goals[robot_id].insert(goal_id);
	m_goal_Compats[goal_id].insert(robot_id);
    }


    void sRobotGoal::charge_Robot(int robot_id, const Vertices_set &goal_IDs)
    {
	sASSERT(robot_id > 0 && robot_id < m_robot_Goals.size());

	m_robot_Goals[robot_id].insert(goal_IDs.begin(), goal_IDs.end());

	for (Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
	{
	    sASSERT(*goal_id >= 0 && *goal_id < m_goal_Compats.size());
	    m_goal_Compats[*goal_id].insert(robot_id);
	}
    }


    void sRobotGoal::assign_Goal(int goal_id, int robot_id)
    {
	sASSERT(robot_id > 0 && robot_id < m_robot_Goals.size());
	sASSERT(goal_id >= 0 && goal_id < m_goal_Compats.size());

	m_goal_Compats[goal_id].insert(robot_id);
	m_robot_Goals[robot_id].insert(goal_id);
    }


    void sRobotGoal::assign_Goal(int goal_id, const Robots_set &robot_IDs)
    {
	sASSERT(goal_id >= 0 && goal_id < m_goal_Compats.size());

	m_goal_Compats[goal_id].insert(robot_IDs.begin(), robot_IDs.end());

	for (Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
	{
	    sASSERT(*robot_id > 0 && *robot_id < m_robot_Goals.size());
	    m_robot_Goals[*robot_id].insert(goal_id);
	}
    }


    void sRobotGoal::discharge_Robot(int robot_id)
    {
	sASSERT(robot_id > 0 && robot_id < m_robot_Goals.size());

	const Vertices_set &goal_IDs = m_robot_Goals[robot_id];
	for (Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
	{
	    m_goal_Compats[*goal_id].erase(robot_id);
	}
	m_robot_Goals[robot_id].clear();
    }


    void sRobotGoal::discharge_Robot(int robot_id, int goal_id)
    {
	sASSERT(robot_id > 0 && robot_id < m_robot_Goals.size());
	sASSERT(goal_id >= 0 && goal_id < m_goal_Compats.size());

	m_robot_Goals[robot_id].erase(goal_id);
	m_goal_Compats[goal_id].erase(robot_id);
    }


    void sRobotGoal::discharge_Robot(int robot_id, const Vertices_set &goal_IDs)
    {
	sASSERT(robot_id > 0 && robot_id < m_robot_Goals.size());

	for (Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
	{
	    m_goal_Compats[*goal_id].erase(robot_id);
	}
	m_robot_Goals[robot_id].erase(goal_IDs.begin(), goal_IDs.end());
    }


    bool sRobotGoal::is_Satisfied(const sRobotArrangement &robot_arrangement) const
    {
	int N_Robots = robot_arrangement.get_RobotCount();

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    const Vertices_set &robot_goal = get_RobotGoal(robot_id);
	    if (robot_goal.find(robot_arrangement.get_RobotLocation(robot_id)) == robot_goal.end())
	    {
		return false;
	    }
	}
	return true;
    }


/*----------------------------------------------------------------------------*/

    void sRobotGoal::to_Screen(const sString &indent) const
    {
	to_Stream(stdout, indent);
    }


    void sRobotGoal::to_Stream(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sRobot goal: (|R| = %ld, |V| = %ld) [\n", indent.c_str(), m_robot_Goals.size() - 1, m_goal_Compats.size());
	fprintf(fw, "%s%srobot goals: {\n", indent.c_str(), sRELOC_INDENT.c_str());
	
	int N_Robots_1 = m_robot_Goals.size();
	for (int i = 1; i < N_Robots_1; ++i)
	{
	    const Vertices_set &goal_IDs = m_robot_Goals[i];
	    fprintf(fw, "%s%s%s%d#{", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), i);
	    if (!goal_IDs.empty())
	    {
		Vertices_set::const_iterator goal_id = goal_IDs.begin();

		fprintf(fw, "%d", *goal_id);
		while (++goal_id != goal_IDs.end())
		{
		    fprintf(fw, ",%d", *goal_id);
		}
	    }
	    fprintf(fw, "}\n");
	}
	fprintf(fw, "%s%s}\n", indent.c_str(), sRELOC_INDENT.c_str());

	fprintf(fw, "%s%svertex compatibilities: {\n", indent.c_str(), sRELOC_INDENT.c_str());
	
	int N_Vertices = m_goal_Compats.size();
	for (int i = 0; i < N_Vertices; ++i)
	{
	    const Robots_set &robot_IDs = m_goal_Compats[i];
	    fprintf(fw, "%s%s%s%d@{", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), i);
	    if (!robot_IDs.empty())
	    {
		Robots_set::const_iterator robot_id = robot_IDs.begin();

		fprintf(fw, "%d", *robot_id);
		while (++robot_id != robot_IDs.end())
		{
		    fprintf(fw, ",%d", *robot_id);
		}
	    }
	    fprintf(fw, "}\n");
	}
	fprintf(fw, "%s%s}\n", indent.c_str(), sRELOC_INDENT.c_str());

	fprintf(fw, "%s]\n", indent.c_str());
    }


    sResult sRobotGoal::to_File_multirobot(const sString &filename, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sROBOT_GOAL_OPEN_ERROR;
	}
	
	to_Stream_multirobot(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    void sRobotGoal::to_Stream_multirobot(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sV =\n", indent.c_str());
	
	int N_Compats = m_goal_Compats.size();

	for (int i = 0; i < N_Compats; ++i)
	{
	    const Robots_set &robot_IDs = m_goal_Compats[i];
	    fprintf(fw, "(%d:-1)[", i);
	    to_Stream_multirobot(fw, robot_IDs, indent);
	    fprintf(fw, ":-1:-1]\n");
	}
    }


    void sRobotGoal::to_Stream_multirobot(FILE *fw, const Robots_set &robot_IDs, const sString &sUNUSED(indent)) const
    {
	fprintf(fw, "{");
	if (!robot_IDs.empty())
	{
	    Robots_set::const_iterator robot_id = robot_IDs.begin();
	    
	    fprintf(fw, "%d", *robot_id);
	    while (++robot_id != robot_IDs.end())
	    {
		fprintf(fw, ",%d", *robot_id);
	    }
	}
	fprintf(fw, "}");
    }


    sResult sRobotGoal::from_File_multirobot(const sString &filename, int component)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sROBOT_GOAL_OPEN_ERROR;
	}
	
	result = from_Stream_multirobot(fr, component);
	if (sFAILED(result))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;
    }


    sResult sRobotGoal::from_Stream_multirobot(FILE *fr, int component)
    {
	Robots_set all_robot_IDs;

	m_robot_Goals.clear();
	m_goal_Compats.clear();

	int N_Robots = 0;
	int N_Vertices = 0;

	int c = fgetc(fr);

	while (c != 'V')
	{
	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
	}

	fscanf(fr, " =\n");

	long position = ftell(fr);
	c = fgetc(fr);

	while (c == '(')
	{
	    int vertex_id, cycle_id, robot_id = 0;

	    switch (component)
	    {
	    case 0:
	    {
		fscanf(fr, "%d:%d)[", &vertex_id, &cycle_id);

		c = fgetc(fr);
		ungetc(c, fr);

		if (c == '{')
		{
		    Robots_set robot_IDs;
		    from_Stream_multirobot(fr, robot_IDs);

		    all_robot_IDs.insert(robot_IDs.begin(), robot_IDs.end());
		}
		else
		{
		    fscanf(fr, "%d", &robot_id);
		    if (robot_id > 0)
		    {
			all_robot_IDs.insert(robot_id);
		    }
		}
		break;
	    }
	    case 1:
	    {
		int dummy_robot_1_id;
		fscanf(fr, "%d:%d)[%d:", &vertex_id, &cycle_id, &dummy_robot_1_id);

		c = fgetc(fr);
		ungetc(c, fr);

		if (c == '{')
		{
		    Robots_set robot_IDs;
		    from_Stream_multirobot(fr, robot_IDs);

		    all_robot_IDs.insert(robot_IDs.begin(), robot_IDs.end());
		}
		else
		{
		    fscanf(fr, "%d", &robot_id);
		    if (robot_id > 0)
		    {
			all_robot_IDs.insert(robot_id);
		    }
		}
		break;
	    }
	    case 2:
	    {
		int dummy_robot_1_id, dummy_robot_2_id;
		fscanf(fr, "%d:%d)[%d:%d:", &vertex_id, &cycle_id, &dummy_robot_1_id, &dummy_robot_2_id);

		c = fgetc(fr);
		ungetc(c, fr);

		if (c == '{')
		{
		    Robots_set robot_IDs;
		    from_Stream_multirobot(fr, robot_IDs);

		    all_robot_IDs.insert(robot_IDs.begin(), robot_IDs.end());
		}
		else
		{
		    fscanf(fr, "%d", &robot_id);
		    if (robot_id > 0)
		    {
			all_robot_IDs.insert(robot_id);
		    }
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }
	    ++N_Vertices;
	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
	}
	N_Robots = all_robot_IDs.size();

	if (fseek(fr, position, SEEK_SET) != 0)
	{
	    return sROBOT_GOAL_SEEK_ERROR;
	}
	c = fgetc(fr);

	m_robot_Goals.resize(N_Robots + 1);
	m_goal_Compats.resize(N_Vertices);

	while (c == '(')
	{
	    int vertex_id, cycle_id, robot_id;

	    switch (component)
	    {
	    case 0:
	    {
		fscanf(fr, "%d:%d)[", &vertex_id, &cycle_id);

		c = fgetc(fr);
		ungetc(c, fr);

		if (c == '{')
		{
		    Robots_set robot_IDs;
		    from_Stream_multirobot(fr, robot_IDs);

		    if (!robot_IDs.empty())
		    {
			assign_Goal(vertex_id, robot_IDs);
		    }
		}
		else
		{
		    fscanf(fr, "%d", &robot_id);
		    if (robot_id > 0)
		    {
			assign_Goal(vertex_id, robot_id);
		    }
		}
		break;
	    }
	    case 1:
	    {
		int dummy_robot_1_id;
		fscanf(fr, "%d:%d)[%d:", &vertex_id, &cycle_id, &dummy_robot_1_id);

		c = fgetc(fr);
		ungetc(c, fr);

		if (c == '{')
		{
		    Robots_set robot_IDs;
		    from_Stream_multirobot(fr, robot_IDs);

		    if (!robot_IDs.empty())
		    {
			assign_Goal(vertex_id, robot_IDs);
		    }
		}
		else
		{
		    fscanf(fr, "%d", &robot_id);
		    if (robot_id > 0)
		    {
			assign_Goal(vertex_id, robot_id);
		    }
		}
		break;
	    }
	    case 2:
	    {
		int dummy_robot_1_id, dummy_robot_2_id;
		fscanf(fr, "%d:%d)[%d:%d:", &vertex_id, &cycle_id, &dummy_robot_1_id, &dummy_robot_2_id);

		c = fgetc(fr);
		ungetc(c, fr);

		if (c == '{')
		{
		    Robots_set robot_IDs;
		    from_Stream_multirobot(fr, robot_IDs);
		    if (!robot_IDs.empty())
		    {
			assign_Goal(vertex_id, robot_IDs);
		    }
		}
		else
		{
		    fscanf(fr, "%d", &robot_id);
		    if (robot_id > 0)
		    {
			assign_Goal(vertex_id, robot_id);
		    }
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }

	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
	}

	return sRESULT_SUCCESS;
    }


    sResult sRobotGoal::from_Stream_multirobot(FILE *fr, Robots_set &robot_IDs)
    {
	fscanf(fr, "{");

	int robot_ID;
	int c = fgetc(fr);

	while (c != '}')
	{
	    if (c != ',')
	    {
		ungetc(c, fr);
	    }
	    fscanf(fr, "%d", &robot_ID);
	    robot_IDs.insert(robot_ID);

	    c = fgetc(fr);
	}
	fscanf(fr, "}");

	return sRESULT_SUCCESS;
    }


/*----------------------------------------------------------------------------*/
// sMultirobotEncodingContext_CNFsat

    const int sMultirobotEncodingContext_CNFsat::UNDEFINED_LAYER_COUNT = -1;


/*----------------------------------------------------------------------------*/

    sMultirobotEncodingContext_CNFsat::sMultirobotEncodingContext_CNFsat()
	: m_N_Layers(UNDEFINED_LAYER_COUNT)
	, m_max_total_cost(0)
	, m_extra_cost(-1)
	, m_state_clause_generator(NULL)
	, m_advanced_clause_generator(NULL)
	, m_bitwise_clause_generator(NULL)
	, m_bit_clause_generator(NULL)
    {
	m_state_Identifiers[sINT_32_MAX] = NULL;
	m_bit_Identifiers[sINT_32_MAX] = NULL;
	switchTo_StandardGeneratingMode();
    }


    sMultirobotEncodingContext_CNFsat::sMultirobotEncodingContext_CNFsat(int N_Layers)
	: m_N_Layers(N_Layers)
	, m_max_total_cost(0)
	, m_extra_cost(-1)
	, m_state_clause_generator(&m_variable_store)
	, m_advanced_clause_generator(&m_variable_store)
	, m_bitwise_clause_generator(&m_variable_store)
	, m_bit_clause_generator(&m_variable_store)
    {
	m_state_Identifiers[sINT_32_MAX] = NULL;
	m_bit_Identifiers[sINT_32_MAX] = NULL;
	switchTo_StandardGeneratingMode();
    }


    sMultirobotEncodingContext_CNFsat::sMultirobotEncodingContext_CNFsat(const sMultirobotEncodingContext_CNFsat &encoding_context)
	: m_N_Layers(encoding_context.m_N_Layers)
	, m_max_total_cost(encoding_context.m_max_total_cost)
	, m_extra_cost(-1)
	, m_variable_store(encoding_context.m_variable_store)
	, m_state_clause_generator(encoding_context.m_state_clause_generator)
	, m_advanced_clause_generator(encoding_context.m_advanced_clause_generator)
	, m_bitwise_clause_generator(encoding_context.m_bitwise_clause_generator)
	, m_bit_clause_generator(encoding_context.m_bit_clause_generator)
	, m_vertex_occupancy(encoding_context.m_vertex_occupancy)
	, m_robot_location(encoding_context.m_robot_location)
	, m_transition_action(encoding_context.m_transition_action)
	, m_vertex_occupancy_by_water(encoding_context.m_vertex_occupancy_by_water)
	, m_vertex_occupancy_by_water_(encoding_context.m_vertex_occupancy_by_water_)
	, m_vertex_water_cardinality_(encoding_context.m_vertex_water_cardinality_)
	, m_vertex_occupancy_by_robot(encoding_context.m_vertex_occupancy_by_robot)
	, m_robot_location_in_vertex(encoding_context.m_robot_location_in_vertex)
	, m_edge_occupancy_by_water(encoding_context.m_edge_occupancy_by_water)
	, m_edge_occupancy_by_water_(encoding_context.m_edge_occupancy_by_water_)
	, m_edge_occupancy_by_water__(encoding_context.m_edge_occupancy_by_water__)
	, m_transition_Actions(encoding_context.m_transition_Actions)
    {
	register_InternalIdentifiers();
	switch_GeneratingMode(encoding_context.get_GeneratingMode());
    }


    const sMultirobotEncodingContext_CNFsat& sMultirobotEncodingContext_CNFsat::operator=(const sMultirobotEncodingContext_CNFsat &encoding_context)
    {
	m_N_Layers = encoding_context.m_N_Layers;
	m_max_total_cost = encoding_context.m_max_total_cost;
	m_extra_cost = encoding_context.m_extra_cost;
	m_variable_store = encoding_context.m_variable_store;
	m_clause_generator = encoding_context.m_clause_generator;
	m_bit_generator = encoding_context.m_bit_generator;
	m_state_clause_generator = encoding_context.m_state_clause_generator;
	m_advanced_clause_generator = encoding_context.m_advanced_clause_generator;
	m_bitwise_clause_generator = encoding_context.m_bitwise_clause_generator;
	m_bit_clause_generator = encoding_context.m_bit_clause_generator;
	m_vertex_occupancy = encoding_context.m_vertex_occupancy;
	m_robot_location = encoding_context.m_robot_location;
	m_robot_location_in_vertex = encoding_context.m_robot_location_in_vertex;
	m_edge_occupancy_by_water = encoding_context.m_edge_occupancy_by_water;
	m_edge_occupancy_by_water_ = encoding_context.m_edge_occupancy_by_water_;
	m_edge_occupancy_by_water__ = encoding_context.m_edge_occupancy_by_water__;
	m_transition_action = encoding_context.m_transition_action;
	m_vertex_occupancy_by_water = encoding_context.m_vertex_occupancy_by_water;
	m_vertex_occupancy_by_water_ = encoding_context.m_vertex_occupancy_by_water_;
	m_vertex_water_cardinality_ = encoding_context.m_vertex_water_cardinality_;
	m_vertex_occupancy_by_robot = encoding_context.m_vertex_occupancy_by_robot;
	m_transition_Actions = encoding_context.m_transition_Actions;
	switch_GeneratingMode(encoding_context.get_GeneratingMode());

	register_InternalIdentifiers();

	return *this;
    }


    sMultirobotEncodingContext_CNFsat::GeneratingMode sMultirobotEncodingContext_CNFsat::get_GeneratingMode(void) const
    {
	return m_generating_mode;
    }

	
    void sMultirobotEncodingContext_CNFsat::switch_GeneratingMode(GeneratingMode generating_mode)
    {
	m_generating_mode = generating_mode;

	switch (generating_mode)
	{
	case GENERATING_STANDARD:
	{
	    m_clause_generator = &m_state_clause_generator;
	    break;
	}
	case GENERATING_ADVANCED:
	{
	    m_clause_generator = &m_advanced_clause_generator;
	    break;
	}
	case GENERATING_BITWISE:
	{
	    m_clause_generator = &m_bitwise_clause_generator;
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
	m_bit_generator = &m_bit_clause_generator;
    }


    void sMultirobotEncodingContext_CNFsat::switchTo_StandardGeneratingMode(void)
    {
	m_clause_generator = &m_state_clause_generator;
	m_bit_generator = &m_bit_clause_generator;
	m_generating_mode = GENERATING_STANDARD;
    }


    void sMultirobotEncodingContext_CNFsat::switchTo_AdvancedGeneratingMode(void)
    {
	m_clause_generator = &m_advanced_clause_generator;
	m_bit_generator = &m_bit_clause_generator;
	m_generating_mode = GENERATING_ADVANCED;
    }


    void sMultirobotEncodingContext_CNFsat::switchTo_BitwiseGeneratingMode(void)
    {
	m_clause_generator = &m_bitwise_clause_generator;
	m_bit_generator = &m_bit_clause_generator;
	m_generating_mode = GENERATING_BITWISE;
    }


    void sMultirobotEncodingContext_CNFsat::register_InternalIdentifiers(void)
    {
	m_state_Identifiers.clear();
	m_state_Identifiers[sINT_32_MAX] = NULL;

	m_bit_Identifiers.clear();
	m_bit_Identifiers[sINT_32_MAX] = NULL;

	if (!m_vertex_occupancy.get_IndexableIdentifier().is_Anonymous())
	{
	    register_TranslateIdentifier(m_vertex_occupancy);
	}
	if (!m_robot_location.get_IndexableIdentifier().is_Anonymous())
	{
	    register_TranslateIdentifier(m_robot_location);
	}
	if (!m_robot_location_in_vertex.get_IndexableIdentifier().is_Anonymous())
	{
	    register_TranslateIdentifier(m_robot_location_in_vertex);
	}
	if (!m_vertex_occupancy_by_robot.get_IndexableIdentifier().is_Anonymous())
	{
	    register_TranslateIdentifier(m_vertex_occupancy_by_robot);
	}

	for (StateIdentifiers_vector::iterator transition_action = m_transition_Actions.begin(); transition_action != m_transition_Actions.end(); ++transition_action)
	{
	    register_TranslateIdentifier(*transition_action);
	}
	for (BitIdentifiers_vector::iterator edge_occupancy = m_edge_occupancy_by_water.begin(); edge_occupancy != m_edge_occupancy_by_water.end(); ++edge_occupancy)
	{
	    register_TranslateIdentifier(*edge_occupancy);
	}
	for (BitIdentifiers_2d_vector::iterator edge_occupancy_ = m_edge_occupancy_by_water_.begin(); edge_occupancy_ != m_edge_occupancy_by_water_.end(); ++edge_occupancy_)
	{
	    for (BitIdentifiers_vector::iterator edge_occupancy = edge_occupancy_->begin(); edge_occupancy != edge_occupancy_->end(); ++edge_occupancy)
	    {
		register_TranslateIdentifier(*edge_occupancy);
	    }
	}

	for (BitIdentifiers_2d_vector::iterator vertex_occupancy_ = m_vertex_occupancy_by_water_.begin(); vertex_occupancy_ != m_vertex_occupancy_by_water_.end(); ++vertex_occupancy_)
	{
	    for (BitIdentifiers_vector::iterator vertex_occupancy = vertex_occupancy_->begin(); vertex_occupancy != vertex_occupancy_->end(); ++vertex_occupancy)
	    {
		register_TranslateIdentifier(*vertex_occupancy);
	    }
	}

	for (BitIdentifiers_2d_vector::iterator vertex_cardinality_ = m_vertex_water_cardinality_.begin(); vertex_cardinality_ != m_vertex_water_cardinality_.end(); ++vertex_cardinality_)
	{
	    for (BitIdentifiers_vector::iterator vertex_cardinality = vertex_cardinality_->begin(); vertex_cardinality != vertex_cardinality_->end(); ++vertex_cardinality)
	    {
		register_TranslateIdentifier(*vertex_cardinality);
	    }
	}

	for (BitIdentifiers_3d_vector::iterator edge_occupancy__ = m_edge_occupancy_by_water__.begin(); edge_occupancy__ != m_edge_occupancy_by_water__.end(); ++edge_occupancy__)
	{
	    for (BitIdentifiers_2d_vector::iterator edge_occupancy_ = edge_occupancy__->begin(); edge_occupancy_ != edge_occupancy__->end(); ++edge_occupancy_)
	    {
		for (BitIdentifiers_vector::iterator edge_occupancy = edge_occupancy_->begin(); edge_occupancy != edge_occupancy_->end(); ++edge_occupancy)
		{
		    register_TranslateIdentifier(*edge_occupancy);
		}
	    }
	}
    }


    void sMultirobotEncodingContext_CNFsat::register_TranslateIdentifier(sIndexableStateIdentifier &state_identifier)
    {
	if (state_identifier.get_First_CNFVariable() >= 1)
	{
	    m_state_Identifiers[state_identifier.get_First_CNFVariable()] = &state_identifier;
	}
    }


    void sMultirobotEncodingContext_CNFsat::register_TranslateIdentifier(sIndexableBitIdentifier &bit_identifier)
    {
	if (bit_identifier.get_First_CNFVariable() >= 1)
	{
	    m_bit_Identifiers[bit_identifier.get_First_CNFVariable()] = &bit_identifier;
	}
    }


    sSpecifiedIdentifier sMultirobotEncodingContext_CNFsat::translate_CNF_Variable(int cnf_variable) const
    {
	sSpecifiedIdentifier state_translation;
	StateIdentifiers_map::const_iterator state_identifier = m_state_Identifiers.upper_bound(cnf_variable);

	if (state_identifier != m_state_Identifiers.end())
	{
	    if (state_identifier->first > cnf_variable)
	    {
		--state_identifier;
	    }
	    if (state_identifier != m_state_Identifiers.end() && state_identifier->second != NULL)
	    {
		state_translation = state_identifier->second->translate_CNFVariable(cnf_variable);
	    }
	}
	if (!state_translation.is_Null())
	{
	    return state_translation;
	}
	else
	{
	    sSpecifiedIdentifier bit_translation;
	    BitIdentifiers_map::const_iterator bit_identifier = m_bit_Identifiers.upper_bound(cnf_variable);
	    /*
	    printf("bit_var:%d,%ld\n", cnf_variable, m_bit_Identifiers.size());
	    for (BitIdentifiers_map::const_iterator identifier = m_bit_Identifiers.begin(); identifier != m_bit_Identifiers.end(); ++identifier)
	    {
		if (identifier->second != NULL)
		{
		    //		identifier->second->to_Screen();
		    printf("%d\n", identifier->first);
		}
	    }
	    */
	    if (bit_identifier != m_bit_Identifiers.end())
	    {
		if (bit_identifier->first > cnf_variable)
		{
		    --bit_identifier;
		}
		if (bit_identifier != m_bit_Identifiers.end() && bit_identifier->second != NULL)
		{
		    bit_translation = bit_identifier->second->translate_CNFVariable(cnf_variable);
		}
	    }
	    if (!bit_translation.is_Null())
	    {
		return bit_translation;
	    }
	    else
	    {
		return m_clause_generator->translate_AuxiliaryCNFVariable(cnf_variable);
	    }
	}
	return sSpecifiedIdentifier();
    }


/*----------------------------------------------------------------------------*/
// sMultirobotInstance

    sMultirobotInstance::sMultirobotInstance()
    {
	// nothing
    }


    sMultirobotInstance::sMultirobotInstance(const sUndirectedGraph &environment, const sRobotArrangement &initial_arrangement, const sRobotArrangement &goal_arrangement)
	: m_goal_type(GOAL_TYPE_ARRANGEMENT)
	, m_environment(environment)
	, m_initial_arrangement(initial_arrangement)
	, m_goal_arrangement(goal_arrangement)
    {
	sASSERT(environment.get_VertexCount() == initial_arrangement.get_VertexCount() && environment.get_VertexCount() == goal_arrangement.get_VertexCount());

	//m_environment.build_SpanningTree(initial_arrangement.get_RobotLocation(1), m_sparse_environment);
	/*
	sUndirectedGraph::VertexPairs_vector vertex_Pairs;

	int N_Robots = m_initial_arrangement.get_RobotCount();
	for (int robot = 1; robot <= N_Robots; ++robot)
	{
	    vertex_Pairs.push_back(sUndirectedGraph::Vertex_pair(initial_arrangement.get_RobotLocation(robot), goal_arrangement.get_RobotLocation(robot)));
	}
	m_environment.build_SparseGraph(vertex_Pairs, m_sparse_environment);
	*/
    }


    sMultirobotInstance::sMultirobotInstance(const sUndirectedGraph  &environment,
					     const sUndirectedGraph  &sparse_environment,
					     const sRobotArrangement &initial_arrangement,
					     const sRobotArrangement &goal_arrangement)
	: m_goal_type(GOAL_TYPE_ARRANGEMENT)
	, m_environment(environment)
	, m_sparse_environment(sparse_environment)
	, m_initial_arrangement(initial_arrangement)
	, m_goal_arrangement(goal_arrangement)
    {
	sASSERT(environment.get_VertexCount() == initial_arrangement.get_VertexCount() && environment.get_VertexCount() == goal_arrangement.get_VertexCount());
    }


    sMultirobotInstance::sMultirobotInstance(const sUndirectedGraph &environment, const sRobotArrangement &initial_arrangement, const sRobotGoal &goal_specification)
	: m_goal_type(GOAL_TYPE_SPECIFICATION)
	, m_environment(environment)
	, m_initial_arrangement(initial_arrangement)
	, m_goal_specification(goal_specification)
    {
	sASSERT(environment.get_VertexCount() == initial_arrangement.get_VertexCount() && environment.get_VertexCount() == goal_specification.get_VertexCount());

//	m_environment.build_SpanningTree(initial_arrangement.get_RobotLocation(1), m_sparse_environment);
	sUndirectedGraph::VertexPairs_vector vertex_Pairs;
 	int N_Robots = m_initial_arrangement.get_RobotCount();

	for (int robot = 1; robot <= N_Robots; ++robot)
	{
	    double p = (double)rand() / RAND_MAX;
	    //  printf("%f\n", p);

	    if (p <= 1.0)
	    {
		const sRobotGoal::Vertices_set &robot_goal = goal_specification.get_RobotGoal(robot);
		if (robot_goal.size() == 1)
		{
		    vertex_Pairs.push_back(sUndirectedGraph::Vertex_pair(initial_arrangement.get_RobotLocation(robot), *robot_goal.begin()));
		}
		else
		{
		    sASSERT(false);
		}
	    }
	}
//	m_environment.build_SparseGraph(vertex_Pairs, m_sparse_environment);	
//	m_environment.build_SpanningTree(0, m_sparse_environment);	
//	printf("%.3f\n", (double)m_sparse_environment.get_EdgeCount() / m_environment.get_EdgeCount());
    }


    sMultirobotInstance::sMultirobotInstance(const sUndirectedGraph  &environment,
					     const sUndirectedGraph  &sparse_environment,
					     const sRobotArrangement &initial_arrangement,
					     const sRobotGoal        &goal_specification)
	: m_goal_type(GOAL_TYPE_SPECIFICATION)
	, m_environment(environment)
	, m_sparse_environment(sparse_environment)
	, m_initial_arrangement(initial_arrangement)
	, m_goal_specification(goal_specification)
    {
	sASSERT(environment.get_VertexCount() == initial_arrangement.get_VertexCount() && environment.get_VertexCount() == goal_specification.get_VertexCount());
    }


    sMultirobotInstance::sMultirobotInstance(const sMultirobotInstance &multirobot_instance)
	: m_goal_type(multirobot_instance.m_goal_type)
	, m_environment(multirobot_instance.m_environment)
	, m_sparse_environment(multirobot_instance.m_sparse_environment)
	, m_initial_arrangement(multirobot_instance.m_initial_arrangement)
	, m_goal_arrangement(multirobot_instance.m_goal_arrangement)
	, m_goal_specification(multirobot_instance.m_goal_specification)
    {
	// nothing
    }


    const sMultirobotInstance& sMultirobotInstance::operator=(const sMultirobotInstance &multirobot_instance)
    {
	m_goal_type = multirobot_instance.m_goal_type;
	m_environment = multirobot_instance.m_environment;
	m_sparse_environment = multirobot_instance.m_sparse_environment;
	m_initial_arrangement = multirobot_instance.m_initial_arrangement;
	m_goal_arrangement = multirobot_instance.m_goal_arrangement;
	m_goal_specification = multirobot_instance.m_goal_specification;

	return *this;
    }


/*----------------------------------------------------------------------------*/


    void sMultirobotInstance::collect_Endpoints(VertexIDs_vector &source_IDs, VertexIDs_vector &goal_IDs)
    {
	int N_Robots = m_initial_arrangement.get_RobotCount();
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(robot_id);
	    int robot_sink_vertex_id;
	    
	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(robot_id);
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
		sASSERT(goal_IDs.size() == 1);
		robot_sink_vertex_id = *goal_IDs.begin();
		break;
	    }
	    default:
	    {
		break;
	    }
	    }
	    source_IDs.push_back(robot_source_vertex_id);
	    goal_IDs.push_back(robot_sink_vertex_id);
	}
    }


    int sMultirobotInstance::analyze_EdgeHeights(int max_total_cost)
    {
	int min_total_cost = INT_MAX;

	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;
	collect_Endpoints(source_IDs, goal_IDs);
	m_environment.calc_AllPairsShortestPaths(source_IDs, goal_IDs);

	const sUndirectedGraph::Distances_2d_vector &all_pairs_Distances = m_environment.get_AllPairsShortestPaths();

	for (sUndirectedGraph::Edges_list::const_iterator edge = m_environment.m_Edges.begin(); edge != m_environment.m_Edges.end(); ++edge)
	{
	    int min_edge_robot_id = -1;
	    int min_edge_cost = INT_MAX;
	    
	    int edge_vertex_u_id = edge->m_arc_uv.m_source->m_id;
	    int edge_vertex_v_id = edge->m_arc_uv.m_target->m_id;
	    
	    int N_Robots = m_initial_arrangement.get_RobotCount();
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(robot_id);
		int robot_sink_vertex_id;
		
		switch (m_goal_type)
		{
		case GOAL_TYPE_ARRANGEMENT:
		{
		    robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(robot_id);
		    break;
		}
		case GOAL_TYPE_SPECIFICATION:
		{
		    const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
		    sASSERT(goal_IDs.size() == 1);
		    robot_sink_vertex_id = *goal_IDs.begin();
		    break;
		}
		default:
		{
		    break;
		}
		}
		
		int edge_cost = sMIN(all_pairs_Distances[robot_source_vertex_id][edge_vertex_u_id] + all_pairs_Distances[edge_vertex_v_id][robot_sink_vertex_id],
				     all_pairs_Distances[robot_source_vertex_id][edge_vertex_v_id] + all_pairs_Distances[edge_vertex_u_id][robot_sink_vertex_id]);
		
		if (edge_cost < min_edge_cost)
		{
		    min_edge_cost = edge_cost;
		    min_edge_robot_id = robot_id;
		}
	    }
	    
	    int rest_cost = 0;
	    
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		if (robot_id != min_edge_robot_id)
		{
		    int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(robot_id);
		    int robot_sink_vertex_id;
		    
		    switch (m_goal_type)
		    {
		    case GOAL_TYPE_ARRANGEMENT:
		    {
			robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(robot_id);
			break;
		    }
		    case GOAL_TYPE_SPECIFICATION:
		    {
			const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
			sASSERT(goal_IDs.size() == 1);
			robot_sink_vertex_id = *goal_IDs.begin();
			break;
		    }
		    default:
		    {
			break;
		    }
		    }
		    rest_cost += all_pairs_Distances[robot_source_vertex_id][robot_sink_vertex_id];
		}
	    }
	    int total_cost = min_edge_cost + rest_cost;
	    int edge_height = max_total_cost - total_cost;
	    printf("Edge height: %d\n", edge_height);

	    if (min_total_cost > total_cost)
	    {
		min_total_cost = total_cost;
	    }
	}
	return min_total_cost;
    }


    int sMultirobotInstance::analyze_EdgeHeights_(int max_total_cost, int &max_vertex_height)
    {
	sUndirectedGraph::VertexIDs_set reachable_IDs;

	return analyze_EdgeHeights_(max_total_cost, max_vertex_height, reachable_IDs);
    }


    int sMultirobotInstance::analyze_EdgeHeights_(int max_total_cost, int &max_vertex_height, sUndirectedGraph::VertexIDs_set &reachable_IDs)
    {
	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;
	collect_Endpoints(source_IDs, goal_IDs);
	m_environment.calc_AllPairsShortestPaths(source_IDs, goal_IDs);

	const sUndirectedGraph::Distances_2d_vector &all_pairs_Distances = m_environment.get_AllPairsShortestPaths();

	max_vertex_height = 0;

	int min_total_cost = 0;

	int N_Robots = m_initial_arrangement.get_RobotCount();
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(robot_id);
	    int robot_sink_vertex_id;
	    
	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(robot_id);
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
		sASSERT(goal_IDs.size() == 1);
		robot_sink_vertex_id = *goal_IDs.begin();
		break;
	    }
	    default:
	    {
		break;
	    }
	    }
	    int robot_cost = all_pairs_Distances[robot_source_vertex_id][robot_sink_vertex_id];

	    min_total_cost += robot_cost;
	}

	for (sUndirectedGraph::Vertices_vector::const_iterator vertex = m_environment.m_Vertices.begin(); vertex != m_environment.m_Vertices.end(); ++vertex)
	{ 
	    int min_robot_cost = INT_MAX;
	    int min_robot_id = -1;

	    int N_Robots = m_initial_arrangement.get_RobotCount();
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(robot_id);
		int robot_sink_vertex_id;
		
		switch (m_goal_type)
		{
		case GOAL_TYPE_ARRANGEMENT:
		{
		    robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(robot_id);
		    break;
		}
		case GOAL_TYPE_SPECIFICATION:
		{
		    const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
		    sASSERT(goal_IDs.size() == 1);
		    robot_sink_vertex_id = *goal_IDs.begin();
		    break;
		}
		default:
		{
		    break;
		}
		}
		int cost = all_pairs_Distances[robot_source_vertex_id][vertex->m_id] + all_pairs_Distances[vertex->m_id][robot_sink_vertex_id];

		if (cost < min_robot_cost)
		{
		    min_robot_cost = cost;
		    min_robot_id = robot_id;
		}
	    }
	    printf("Min robot:%d (cost:%d)\n", min_robot_id, min_robot_cost);

	    int rest_cost = 0;
	    int overlaps = 0;

	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		if (robot_id != min_robot_id)
		{
		    int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(robot_id);
		    int robot_sink_vertex_id;
		
		    switch (m_goal_type)
		    {
		    case GOAL_TYPE_ARRANGEMENT:
		    {
			robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(robot_id);
			break;
		    }
		    case GOAL_TYPE_SPECIFICATION:
		    {
			const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
			sASSERT(goal_IDs.size() == 1);
			robot_sink_vertex_id = *goal_IDs.begin();
			break;
		    }
		    default:
		    {
			break;
		    }
		    }
		    int cost = all_pairs_Distances[robot_source_vertex_id][robot_sink_vertex_id];
		    int overlap_cost = all_pairs_Distances[robot_source_vertex_id][vertex->m_id] + all_pairs_Distances[vertex->m_id][robot_sink_vertex_id];

		    if (cost == overlap_cost)
		    {
			++overlaps;
		    }
		    rest_cost += cost;
		}
	    }
	    int total_cost = min_robot_cost + rest_cost;
	    int remaining_cost = max_total_cost - total_cost;

	    printf("Tc: %d\n", total_cost);
	    int vertex_height = -1;

	    if (remaining_cost >= 0)
	    {
		vertex_height = remaining_cost + overlaps;
		reachable_IDs.insert(vertex->m_id);
	    }
	    if (max_vertex_height < vertex_height)
	    {
		max_vertex_height = vertex_height;
	    }
	    printf("Height: %d\n", vertex_height);
	}
//	printf("Total: %d (%d) Size: %d (%.3f)\n", min_total_cost, max_vertex_height, reachable_IDs.size(), (double)reachable_IDs.size() / m_environment.get_VertexCount());

	return min_total_cost;
    }


    int sMultirobotInstance::build_HeightedEnvironments(int max_total_cost)
    {
	return build_HeightedEnvironments(max_total_cost, m_heighted_Environments);
    }


    int sMultirobotInstance::build_HeightedEnvironments(int max_total_cost, Environments_vector &heighted_Environments)
    {
	int min_total_cost = INT_MAX;
	heighted_Environments.clear();

	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;
	collect_Endpoints(source_IDs, goal_IDs);
	m_environment.calc_AllPairsShortestPaths(source_IDs, goal_IDs);

	const sUndirectedGraph::Distances_2d_vector &all_pairs_Distances = m_environment.get_AllPairsShortestPaths();

	for (sUndirectedGraph::Edges_list::const_iterator edge = m_environment.m_Edges.begin(); edge != m_environment.m_Edges.end(); ++edge)
	{
	    int min_edge_robot_id = -1;
	    int min_edge_cost = INT_MAX;
	    
	    int edge_vertex_u_id = edge->m_arc_uv.m_source->m_id;
	    int edge_vertex_v_id = edge->m_arc_uv.m_target->m_id;
	    
	    int N_Robots = m_initial_arrangement.get_RobotCount();
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(robot_id);
		int robot_sink_vertex_id;
		
		switch (m_goal_type)
		{
		case GOAL_TYPE_ARRANGEMENT:
		{
		    robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(robot_id);
		    break;
		}
		case GOAL_TYPE_SPECIFICATION:
		{
		    const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
		    sASSERT(goal_IDs.size() == 1);
		    robot_sink_vertex_id = *goal_IDs.begin();
		    break;
		}
		default:
		{
		    break;
		}
		}
		
		int edge_cost = sMIN(all_pairs_Distances[robot_source_vertex_id][edge_vertex_u_id] + all_pairs_Distances[edge_vertex_v_id][robot_sink_vertex_id],
				     all_pairs_Distances[robot_source_vertex_id][edge_vertex_v_id] + all_pairs_Distances[edge_vertex_u_id][robot_sink_vertex_id]);
		
		if (edge_cost < min_edge_cost)
		{
		    min_edge_cost = edge_cost;
		    min_edge_robot_id = robot_id;
		}
	    }
	    
	    int rest_cost = 0;
	    
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		if (robot_id != min_edge_robot_id)
		{
		    int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(robot_id);
		    int robot_sink_vertex_id;
		    
		    switch (m_goal_type)
		    {
		    case GOAL_TYPE_ARRANGEMENT:
		    {
			robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(robot_id);
			break;
		    }
		    case GOAL_TYPE_SPECIFICATION:
		    {
			const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
			sASSERT(goal_IDs.size() == 1);
			robot_sink_vertex_id = *goal_IDs.begin();
			break;
		    }
		    default:
		    {
			break;
		    }
		    }
		    rest_cost += all_pairs_Distances[robot_source_vertex_id][robot_sink_vertex_id];
		}
	    }
	    int total_cost = min_edge_cost + rest_cost;
	    int edge_height = max_total_cost - total_cost;

	    printf("eh: %d\n", edge_height);

	    if (edge_height >= 0)
	    {
		int environment_height = heighted_Environments.size();
		while (environment_height++ <= edge_height)
		{
		    heighted_Environments.push_back(sUndirectedGraph());
		    heighted_Environments.back().add_Vertices(m_environment.get_VertexCount());
		}
		for (int eh = 0; eh < edge_height; ++eh)
		{
		    if (!heighted_Environments[eh].is_Adjacent(edge_vertex_u_id, edge_vertex_v_id))
		    {
			heighted_Environments[eh].add_Edge(edge_vertex_u_id, edge_vertex_v_id);
		    }
		}
	    }
	    if (min_total_cost > total_cost)
	    {
		min_total_cost = total_cost;
	    }
	}
	printf("------>\n");
	if (heighted_Environments.empty())
	{
	    heighted_Environments.push_back(sUndirectedGraph());
	    heighted_Environments.back().add_Vertices(m_environment.get_VertexCount());
	}
	return min_total_cost;
    }


    int sMultirobotInstance::build_HeightedEnvironments_(int max_total_cost)
    {
	return build_HeightedEnvironments_(max_total_cost, m_heighted_Environments);
    }


    int sMultirobotInstance::build_HeightedEnvironments_(int max_total_cost, Environments_vector &heighted_Environments)
    {
	heighted_Environments.clear();

	int max_vertex_height = 0;
	sUndirectedGraph::VertexIDs_set reachable_IDs;

	int min_total_cost = analyze_EdgeHeights_(max_total_cost, max_vertex_height, reachable_IDs);

	sUndirectedGraph heighted_environment;
	heighted_environment.add_Vertices(m_environment.get_VertexCount());

	for (sUndirectedGraph::Edges_list::const_iterator edge = m_environment.m_Edges.begin(); edge != m_environment.m_Edges.end(); ++edge)
	{    
	    int edge_vertex_u_id = edge->m_arc_uv.m_source->m_id;
	    int edge_vertex_v_id = edge->m_arc_uv.m_target->m_id;
	    /*
	    sUndirectedGraph::VertexIDs_set::const_iterator reachable_u = reachable_IDs.find(edge_vertex_u_id);
	    sUndirectedGraph::VertexIDs_set::const_iterator reachable_v = reachable_IDs.find(edge_vertex_v_id);
	    */
//	    if (reachable_u != reachable_IDs.end() && reachable_v != reachable_IDs.end())
	    {
		heighted_environment.add_Edge(edge_vertex_u_id, edge_vertex_v_id);
	    }
	}
	for (int height = 0; height <= max_vertex_height; ++height)
	{
	    heighted_Environments.push_back(heighted_environment);
	}
	return min_total_cost;
    }


    int sMultirobotInstance::estimate_TotalCost(int &max_individual_cost)
    {
	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;

	collect_Endpoints(source_IDs, goal_IDs);
	m_environment.calc_SourceGoalShortestPaths(source_IDs, goal_IDs);

	const sUndirectedGraph::Distances_2d_vector &source_Distances = m_environment.get_SourceShortestPaths();
	const sUndirectedGraph::Distances_2d_vector &goal_Distances = m_environment.get_GoalShortestPaths();

	int min_total_cost = 0;
	max_individual_cost = 0;

	int N_Robots = m_initial_arrangement.get_RobotCount();
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(robot_id);
	    int robot_sink_vertex_id;
	    
	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(robot_id);
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
		sASSERT(goal_IDs.size() == 1);
		robot_sink_vertex_id = *goal_IDs.begin();
		break;
	    }
	    default:
	    {
		break;
	    }
	    }
	    int robot_cost = source_Distances[robot_source_vertex_id][robot_sink_vertex_id];
	    min_total_cost += robot_cost;

	    if (robot_cost > max_individual_cost)
	    {
		max_individual_cost = robot_cost;
	    }
	}
	return min_total_cost;
    }


    int sMultirobotInstance::construct_MDD(int max_total_cost, MDD_vector &MDD, int &extra_cost, MDD_vector &extra_MDD)
    {
	int max_individual_cost;
	int min_total_cost = estimate_TotalCost(max_individual_cost);

	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;
	collect_Endpoints(source_IDs, goal_IDs);
	m_environment.calc_SourceGoalShortestPaths(source_IDs, goal_IDs);

	const sUndirectedGraph::Distances_2d_vector &source_Distances = m_environment.get_SourceShortestPaths();
	const sUndirectedGraph::Distances_2d_vector &goal_Distances = m_environment.get_GoalShortestPaths();

	extra_cost = max_total_cost - min_total_cost;
	int mdd_depth = max_individual_cost + extra_cost;
/*
	printf("max_total:%d\n", max_total_cost);
	printf("mdd_depth:%d\n", mdd_depth);
	printf("extra_cost:%d\n", extra_cost);
*/
	int N_Robots = m_initial_arrangement.get_RobotCount();

	MDD.resize(N_Robots + 1);
	extra_MDD.resize(N_Robots + 1);

	for (int mdd_robot_id = 1; mdd_robot_id <= N_Robots; ++mdd_robot_id)
	{
//	    printf("mdd_robot:%d\n", mdd_robot_id);
	    MDD[mdd_robot_id].resize(mdd_depth + 1);
	    extra_MDD[mdd_robot_id].resize(mdd_depth + 1);

	    int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(mdd_robot_id);
	    int robot_sink_vertex_id;

	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(mdd_robot_id);
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(mdd_robot_id);
		sASSERT(goal_IDs.size() == 1);
		robot_sink_vertex_id = *goal_IDs.begin();
		break;
	    }
	    default:
	    {
		break;
	    }
	    }
//	    printf("  %d-->%d\n", robot_source_vertex_id, robot_sink_vertex_id);
	    int robot_cost = source_Distances[robot_source_vertex_id][robot_sink_vertex_id];
	    
	    for (int mdd_level = 0; mdd_level <= mdd_depth; ++mdd_level)
	    {
//		printf("level %d (%d):", mdd_level, mdd_depth);
		int N_Vertices = m_environment.get_VertexCount();
		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    if (   source_Distances[robot_source_vertex_id][vertex_id] <= mdd_level
			&& goal_Distances[robot_sink_vertex_id][vertex_id] <= robot_cost + extra_cost - mdd_level)
		    {
			MDD[mdd_robot_id][mdd_level].push_back(vertex_id);
//			printf("%d ", vertex_id);
		    }
		}
		if (MDD[mdd_robot_id][mdd_level].empty())
		{
		    MDD[mdd_robot_id][mdd_level].push_back(robot_sink_vertex_id);
//		    printf("%d ", robot_sink_vertex_id);
		}
		if (   mdd_level >= source_Distances[robot_source_vertex_id][robot_sink_vertex_id]
		    && mdd_level < source_Distances[robot_source_vertex_id][robot_sink_vertex_id] + extra_cost)
		{
		    extra_MDD[mdd_robot_id][mdd_level].push_back(robot_sink_vertex_id);
//		    printf(" * ");
		}
//		printf("\n");
	    }
	}
	
	std::vector<int> distribution;
	distribution.resize(N_Robots + 1);
	for (int robot_id = 0; robot_id <= N_Robots; ++robot_id)
	{
	    distribution[robot_id] = 0;
	}
	
	return mdd_depth;
    }


    int sMultirobotInstance::construct_NoMDD(int max_total_cost, MDD_vector &MDD, int &extra_cost, MDD_vector &extra_MDD)
    {
	int max_individual_cost;
	int min_total_cost = estimate_TotalCost(max_individual_cost);

	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;
	collect_Endpoints(source_IDs, goal_IDs);
	m_environment.calc_SourceGoalShortestPaths(source_IDs, goal_IDs);

	const sUndirectedGraph::Distances_2d_vector &source_Distances = m_environment.get_SourceShortestPaths();
	const sUndirectedGraph::Distances_2d_vector &goal_Distances = m_environment.get_GoalShortestPaths();

	extra_cost = max_total_cost - min_total_cost;
	int mdd_depth = max_individual_cost + extra_cost;
/*
	printf("mdd_depth:%d\n", mdd_depth);
	printf("extra_cost:%d\n", extra_cost);
*/
	int N_Robots = m_initial_arrangement.get_RobotCount();

	MDD.resize(N_Robots + 1);
	extra_MDD.resize(N_Robots + 1);

	for (int mdd_robot_id = 1; mdd_robot_id <= N_Robots; ++mdd_robot_id)
	{
//	    printf("mdd_robot:%d\n", mdd_robot_id);
	    MDD[mdd_robot_id].resize(mdd_depth + 1);
	    extra_MDD[mdd_robot_id].resize(mdd_depth + 1);

	    int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(mdd_robot_id);
	    int robot_sink_vertex_id;

	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(mdd_robot_id);
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(mdd_robot_id);
		sASSERT(goal_IDs.size() == 1);
		robot_sink_vertex_id = *goal_IDs.begin();
		break;
	    }
	    default:
	    {
		break;
	    }
	    }
  
	    for (int mdd_level = 0; mdd_level <= mdd_depth; ++mdd_level)
	    {
//		printf("level %d (%d):", mdd_level, mdd_depth);
		int N_Vertices = m_environment.get_VertexCount();
		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    MDD[mdd_robot_id][mdd_level].push_back(vertex_id);
//		    printf("%d ", vertex_id);
		}
		if (mdd_level >= source_Distances[robot_source_vertex_id][robot_sink_vertex_id])
		{
		    extra_MDD[mdd_robot_id][mdd_level].push_back(robot_sink_vertex_id);
//		    printf(" * ");
		}
//		extra_MDD[mdd_robot_id][mdd_level].push_back(robot_sink_vertex_id);
//		printf("\n");
	    }
	}
	std::vector<int> distribution;
	distribution.resize(N_Robots + 1);
	for (int robot_id = 0; robot_id <= N_Robots; ++robot_id)
	{
	    distribution[robot_id] = 0;
	}
	
	return mdd_depth;
    }


    void sMultirobotInstance::construct_MakespanMDD(int max_makespan, MDD_vector &MDD)
    {
	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;
	collect_Endpoints(source_IDs, goal_IDs);
	m_environment.calc_SourceGoalShortestPaths(source_IDs, goal_IDs);

	const sUndirectedGraph::Distances_2d_vector &source_Distances = m_environment.get_SourceShortestPaths();
	const sUndirectedGraph::Distances_2d_vector &goal_Distances = m_environment.get_GoalShortestPaths();

	int N_Robots = m_initial_arrangement.get_RobotCount();

	MDD.resize(N_Robots + 1);

	for (int mdd_robot_id = 1; mdd_robot_id <= N_Robots; ++mdd_robot_id)
	{
//	    printf("mdd_robot:%d\n", mdd_robot_id);
	    MDD[mdd_robot_id].resize(max_makespan + 1);

	    int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(mdd_robot_id);
	    int robot_sink_vertex_id;

	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(mdd_robot_id);
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(mdd_robot_id);
		sASSERT(goal_IDs.size() == 1);
		robot_sink_vertex_id = *goal_IDs.begin();
		break;
	    }
	    default:
	    {
		break;
	    }
	    }

#ifdef sVERBOSE
	    printf("Robot:%d\n", mdd_robot_id);
#endif
	    for (int mdd_level = 0; mdd_level <= max_makespan; ++mdd_level)
	    {
#ifdef sVERBOSE
		printf("mdd level %d: ", mdd_level, max_makespan);
#endif
		int N_Vertices = m_environment.get_VertexCount();
		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    if (   source_Distances[robot_source_vertex_id][vertex_id] <= mdd_level
			&& goal_Distances[robot_sink_vertex_id][vertex_id] <= max_makespan - mdd_level)
		    {
#ifdef sVERBOSE
			printf("%d ", vertex_id);
#endif
			MDD[mdd_robot_id][mdd_level].push_back(vertex_id);
		    }
		}
#ifdef sVERBOSE
		printf("\n");
#endif
	    }
#ifdef sVERBOSE
	    printf("\n");
#endif
	}
	
	std::vector<int> distribution;
	distribution.resize(N_Robots + 1);
	for (int robot_id = 0; robot_id <= N_Robots; ++robot_id)
	{
	    distribution[robot_id] = 0;
	}	
    }


/*----------------------------------------------------------------------------*/

    void sMultirobotInstance::to_Screen(const sString &indent) const
    {
	to_Stream(stdout, indent);
    }


    void sMultirobotInstance::to_Screen_multirobot(const sString &indent) const
    {
	to_Stream_multirobot(stdout, indent);
    }


    void sMultirobotInstance::to_Screen_domainPDDL(const sString &indent) const
    {
	to_Stream_domainPDDL(stdout, indent);
    }


    void sMultirobotInstance::to_Screen_problemPDDL(const sString &indent) const
    {
	to_Stream_problemPDDL(stdout, indent);
    }


    void sMultirobotInstance::to_Screen_bgu(const sString &indent, int instance_id) const
    {
	to_Stream_bgu(stdout, indent, instance_id);
    }


    void sMultirobotInstance::to_Screen_InverseCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	to_Stream_InverseCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_AdvancedCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	to_Stream_AdvancedCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_DifferentialCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	to_Stream_DifferentialCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_BijectionCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	to_Stream_BijectionCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_HeuristicDifferentialCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_HeuristicDifferentialCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_HeuristicBijectionCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_HeuristicBijectionCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_HeuristicAdvancedCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_HeuristicAdvancedCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_PuzzleCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	to_Stream_PuzzleCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_BitwiseCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_BitwiseCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_FlowCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	to_Stream_FlowCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_MatchingCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	to_Stream_MatchingCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_HeuristicMatchingCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_HeuristicMatchingCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_DirectCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	to_Stream_DirectCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_HeuristicDirectCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_HeuristicDirectCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_SimplicialCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	to_Stream_SimplicialCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_HeuristicSimplicialCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_HeuristicSimplicialCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_SingularCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	to_Stream_SingularCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_PluralCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	to_Stream_PluralCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_Plural2CNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_Plural2CNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_HeightedCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_HeightedCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_MddCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_MddCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_MmddCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_MmddCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_RXMddCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_RXMddCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_NoMddCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_NoMddCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_RXNoMddCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_RXNoMddCNFsat(stdout, encoding_context, indent, verbose);
    }


    sResult sMultirobotInstance::to_File(const sString &filename, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_OPEN_ERROR;
	}
	to_Stream(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_multirobot(const sString &filename, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_OPEN_ERROR;
	}
	to_Stream_multirobot(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_domainPDDL(const sString &filename, const sString &indent) const
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_PDDL_OPEN_ERROR;
	}
	to_Stream_domainPDDL(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_problemPDDL(const sString &filename, const sString &indent) const
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_PDDL_OPEN_ERROR;
	}
	to_Stream_problemPDDL(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_bgu(const sString &filename, const sString &indent, int instance_id) const
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_BGU_OPEN_ERROR;
	}
	to_Stream_bgu(fw, indent, instance_id);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_InverseCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_InverseCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_AdvancedCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_AdvancedCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_DifferentialCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_DifferentialCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_BijectionCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_BijectionCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_HeuristicDifferentialCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_HeuristicDifferentialCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_HeuristicBijectionCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_HeuristicBijectionCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_HeuristicAdvancedCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_HeuristicAdvancedCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_PuzzleCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_PuzzleCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_BitwiseCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_BitwiseCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_FlowCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_FlowCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_MatchingCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_MatchingCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_HeuristicMatchingCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_HeuristicMatchingCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_DirectCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_DirectCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_HeuristicDirectCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_HeuristicDirectCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_SimplicialCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_SimplicialCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_HeuristicSimplicialCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_HeuristicSimplicialCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_SingularCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_SingularCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_PluralCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_PluralCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_Plural2CNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_Plural2CNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_HeightedCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_HeightedCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_MddCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_MddCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_MmddCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_MmddCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_RXMddCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_RXMddCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_NoMddCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_NoMddCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_RXNoMddCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_RXNoMddCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    void sMultirobotInstance::to_Stream(FILE *fw, const sString &indent) const
    {       
	fprintf(fw, "%sMultirobot instance: [\n", indent.c_str());
	fprintf(fw, "%s%sEnvironment:\n", indent.c_str(), sRELOC_INDENT.c_str());
	m_environment.to_Stream_vertices(fw, indent + sRELOC_INDENT + sRELOC_INDENT);
	fprintf(fw, "%s%sInitial arrangement:\n", indent.c_str(), sRELOC_INDENT.c_str());
	m_initial_arrangement.to_Stream(fw, indent + sRELOC_INDENT + sRELOC_INDENT);

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    fprintf(fw, "%s%sGoal arrangement:\n", indent.c_str(), sRELOC_INDENT.c_str());
	    m_goal_arrangement.to_Stream(fw, indent + sRELOC_INDENT + sRELOC_INDENT);
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    fprintf(fw, "%s%sGoal specification:\n", indent.c_str(), sRELOC_INDENT.c_str());
	    m_goal_specification.to_Stream(fw, indent + sRELOC_INDENT + sRELOC_INDENT);
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
	fprintf(fw, "%s]\n", indent.c_str());
    }


    void sMultirobotInstance::to_Stream_multirobot(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sV =\n", indent.c_str());
	
	int N_Vertices;
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    N_Vertices = m_initial_arrangement.m_vertex_Occups.size();
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    N_Vertices = m_goal_specification.m_goal_Compats.size();
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
	for (int i = 0; i < N_Vertices; ++i)
	{
	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		fprintf(fw, "(%d:-1)[%d:%d:%d]\n", i,
		 	                           m_initial_arrangement.m_vertex_Occups[i],
		 	                           m_goal_arrangement.m_vertex_Occups[i],
			                           m_goal_arrangement.m_vertex_Occups[i]);
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		fprintf(fw, "(%d:-1)[%d:", i, m_initial_arrangement.m_vertex_Occups[i]);
		m_goal_specification.to_Stream_multirobot(fw, m_goal_specification.m_goal_Compats[i]);
		fprintf(fw, ":");
		m_goal_specification.to_Stream_multirobot(fw, m_goal_specification.m_goal_Compats[i]);
		fprintf(fw, "]\n");
		break;
	    }
	    default:
	    {
		break;
	    }
	    }
	}
	m_environment.to_Stream_multirobot(fw, indent);
    }


    void sMultirobotInstance::to_Stream_domainPDDL(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%s(define (domain multirobot)\n", indent.c_str());
	fprintf(fw, "%s%s(:predicates\n", indent.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s(adjacent ?u ?v)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s(robot_location ?r ?v)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s(no_robot ?v)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s)\n", indent.c_str(), sRELOC_INDENT.c_str());

	fprintf(fw, "%s%s(:action move\n", indent.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s:parameters (?r ?u ?v)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s:precondition (and\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s%s(robot_location ?r ?u) (no_robot ?v) (adjacent ?u ?v)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s:effect (and\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s%s(robot_location ?r ?v) (no_robot ?u) (not (no_robot ?v))\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s)\n", indent.c_str(), sRELOC_INDENT.c_str());

	fprintf(fw, "%s)\n", indent.c_str());
    }


    void sMultirobotInstance::to_Stream_problemPDDL(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%s(define (problem multirobot_instance)\n", indent.c_str());
	fprintf(fw, "%s%s(:domain multirobot)\n", indent.c_str(), sRELOC_INDENT.c_str());

	fprintf(fw, "%s%s(:objects\n", indent.c_str(), sRELOC_INDENT.c_str());

	int N_Vertices = m_environment.get_VertexCount();
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    fprintf(fw, "%s%s%sv%d\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), vertex_id);
	}

	int N_Robots = m_initial_arrangement.get_RobotCount();
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    fprintf(fw, "%s%s%sr%d\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), robot_id);
	}
	fprintf(fw, "%s%s)\n", indent.c_str(), sRELOC_INDENT.c_str());

	fprintf(fw, "%s%s(:init\n", indent.c_str(), sRELOC_INDENT.c_str());

	for (sUndirectedGraph::Edges_list::const_iterator edge = m_environment.m_Edges.begin(); edge != m_environment.m_Edges.end(); ++edge)
	{
	    fprintf(fw, "%s%s%s(adjacent v%d v%d)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), edge->m_arc_vu.m_target->m_id, edge->m_arc_uv.m_target->m_id);
	    fprintf(fw, "%s%s%s(adjacent v%d v%d)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), edge->m_arc_uv.m_target->m_id, edge->m_arc_vu.m_target->m_id);
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    fprintf(fw, "%s%s%s(robot_location r%d v%d)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), robot_id, m_initial_arrangement.get_RobotLocation(robot_id));
	}
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (robot_id == sRobotArrangement::VACANT_VERTEX)
	    {
		fprintf(fw, "%s%s%s(no_robot v%d)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), vertex_id);
	    }
	}
	fprintf(fw, "%s%s)\n", indent.c_str(), sRELOC_INDENT.c_str());

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    fprintf(fw, "%s%s(:goal (and \n", indent.c_str(), sRELOC_INDENT.c_str());
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		fprintf(fw, "%s%s%s(robot_location r%d v%d)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), robot_id, m_goal_arrangement.get_RobotLocation(robot_id));
	    }
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);
		
		if (robot_id == sRobotArrangement::VACANT_VERTEX)
		{
		    fprintf(fw, "%s%s%s(no_robot v%d)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), vertex_id);
		}
	    }
	    fprintf(fw, "%s%s))\n", indent.c_str(), sRELOC_INDENT.c_str());
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    fprintf(fw, "%s%s(:goal (and \n", indent.c_str(), sRELOC_INDENT.c_str());
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
		fprintf(fw, "%s%s%s(or ", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
		for (sRobotGoal::Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
		{
		    fprintf(fw, "(robot_location r%d v%d) ", robot_id, *goal_id);
		}
		fprintf(fw, ")\n");
	    }
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (robot_IDs.empty())
		{
		    fprintf(fw, "%s%s%s(no_robot v%d)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), vertex_id);
		}
	    }
	    fprintf(fw, "%s%s))\n", indent.c_str(), sRELOC_INDENT.c_str());
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}

	fprintf(fw, "%s)\n", indent.c_str());
    }


    void sMultirobotInstance::to_Stream_bgu(FILE *fw, const sString &indent, int instance_id) const
    {
	sASSERT(m_environment.m_Matrix != NULL);

	fprintf(fw, "%s%d\n", indent.c_str(), instance_id);
	fprintf(fw, "%sGrid:\n", indent.c_str());
	fprintf(fw, "%s%d,%d\n", indent.c_str(), m_environment.m_x_size, m_environment.m_y_size);

	for (int j = 0; j < m_environment.m_y_size; ++j)
	{
	    for (int i = 0; i < m_environment.m_x_size; ++i)
	    {
		if (m_environment.m_Matrix[j * m_environment.m_x_size + i] >= 0)
		{
		    fprintf(fw, ".");
		}
		else
		{
		    fprintf(fw, "@");
		}
	    }
	    fprintf(fw, "\n");
	}
	int N_Robots = m_initial_arrangement.get_RobotCount();

	fprintf(fw, "%sAgents:\n", indent.c_str());
	fprintf(fw, "%s%d\n", indent.c_str(), N_Robots);

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		fprintf(fw, "%s%d,", indent.c_str(), robot_id - 1);

		int goal_vertex_id = m_goal_arrangement.get_RobotLocation(robot_id);
		fprintf(fw, "%d,%d,",  m_environment.calc_GridRow(goal_vertex_id), m_environment.calc_GridColumn(goal_vertex_id));

		int init_vertex_id = m_initial_arrangement.get_RobotLocation(robot_id);
		fprintf(fw, "%d,%d\n",  m_environment.calc_GridRow(init_vertex_id), m_environment.calc_GridColumn(init_vertex_id));
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		fprintf(fw, "%s%d,", indent.c_str(), robot_id - 1);

		sASSERT(m_goal_specification.get_RobotGoal(robot_id).size() == 1);
		int goal_vertex_id = *m_goal_specification.get_RobotGoal(robot_id).begin();
		fprintf(fw, "%d,%d,",  m_environment.calc_GridRow(goal_vertex_id), m_environment.calc_GridColumn(goal_vertex_id));

		int init_vertex_id = m_initial_arrangement.get_RobotLocation(robot_id);
		fprintf(fw, "%d,%d\n",  m_environment.calc_GridRow(init_vertex_id), m_environment.calc_GridColumn(init_vertex_id));
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
    }


    void sMultirobotInstance::to_Stream_InverseCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool verbose) const
    {
	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;
 
	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableStateIdentifier vertex_occupancy(&encoding_context.m_variable_store, "vertex_occupancy", N_Robots + 1, sIntegerScope(0, N_Vertices), sIntegerScope(0, encoding_context.m_N_Layers - 1));
	encoding_context.m_vertex_occupancy = vertex_occupancy;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	    {
		Clause_cnt += encoding_context.m_clause_generator->count_Alignment(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
	    }
	}

	encoding_context.m_transition_Actions.resize(N_Vertices);
	
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    sIndexableStateIdentifier transition_action(&encoding_context.m_variable_store, "transition_action-" + sInt_32_to_String(vertex_id), 2 * m_environment.get_Vertex(vertex_id)->calc_NeighborCount() + 1, sIntegerScope(0, encoding_context.m_N_Layers - 1)); 
	    encoding_context.m_transition_Actions[vertex_id] = transition_action;
	    encoding_context.register_TranslateIdentifier(encoding_context.m_transition_Actions[vertex_id]);

	    for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	    {		    
		Clause_cnt += encoding_context.m_clause_generator->count_Alignment(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedStateIdentifier(&transition_action, sIntegerIndex(layer)));
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{	
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int N_vertex_Neighbors = m_environment.get_Vertex(vertex_id)->calc_NeighborCount();
		const sVertex::Neighbors_list &out_Neighbors =  m_environment.get_Vertex(vertex_id)->m_Neighbors;

		int out_neighbor_order = 0;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    int neighbor_id = (*out_neighbor)->m_target->m_id;
		    int N_neighbor_Neighbors = m_environment.get_Vertex(neighbor_id)->calc_NeighborCount();
		    int in_neighbor_order = m_environment.get_Vertex(neighbor_id)->calc_NeighborOrder(vertex_id);

		    Clause_cnt += encoding_context.m_clause_generator->count_ConditionalEquality(aux_Variable_cnt,
												 total_Literal_cnt,
												 sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[vertex_id], sIntegerIndex(layer)),
												 out_neighbor_order,
												 sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[neighbor_id], sIntegerIndex(layer)),
												 N_neighbor_Neighbors + in_neighbor_order,
												 sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(neighbor_id), sIntegerIndex(layer)),
												 0,
												 sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)),
												 0,
												 sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)),
												 sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		    
		    Clause_cnt += encoding_context.m_clause_generator->count_ConditionalEquality(aux_Variable_cnt,
												 total_Literal_cnt,
												 sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[vertex_id], sIntegerIndex(layer)),
												 N_vertex_Neighbors + out_neighbor_order,
												 sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[neighbor_id], sIntegerIndex(layer)),
												 in_neighbor_order);

		    ++out_neighbor_order;
		}
		
		Clause_cnt += encoding_context.m_clause_generator->count_ConditionalEquality(aux_Variable_cnt,
											     total_Literal_cnt,
											     sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[vertex_id], sIntegerIndex(layer)),
											     2 * N_vertex_Neighbors,
											     sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)),
											     sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);
	    Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
									      total_Literal_cnt,
									      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(0)), robot_id);
	}
/*
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);
	    Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt, total_Literal_cnt, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), robot_id);
	}
*/
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);
		Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
										  total_Literal_cnt,
										  sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), robot_id);
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    sStateClauseGenerator::SpecifiedStateIdentifiers_vector goal_case_disj_Identifiers;
		    sStateClauseGenerator::States_vector goal_case_disj_States;

		    goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    goal_case_disj_States.push_back(0);
		    
		    for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
		    {
			goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
			goal_case_disj_States.push_back(*robot_id);
		    }
		    Clause_cnt += encoding_context.m_clause_generator->count_DisjunctiveEquality(aux_Variable_cnt,
												 total_Literal_cnt,
												 goal_case_disj_Identifiers,
												 goal_case_disj_States);
		}
		else
		{
		    Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
										      total_Literal_cnt,
										      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), sRobotArrangement::VACANT_VERTEX);
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}

	int N_cnf_Variables = encoding_context.m_variable_store.get_Last_CNFVariable() + aux_Variable_cnt - 1;

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot inverse SAT encoding\n", sPRODUCT);
	fprintf(fw, "c %s\n", sCOPYRIGHT);
	fprintf(fw, "c number of layers = %d\n", encoding_context.m_N_Layers);
	fprintf(fw, "c number of visible variables = %d\n", N_cnf_Variables - aux_Variable_cnt);
	fprintf(fw, "c number of hidden variables = %d\n", aux_Variable_cnt);
	fprintf(fw, "c number of literals = %d\n", total_Literal_cnt);
	fprintf(fw, "c number of clauses = %d\n", Clause_cnt);
	fprintf(fw, "c number of propositional variables = %d\n", N_cnf_Variables);
	fprintf(fw, "c number of clauses / number of variables = %.3f\n", (double)Clause_cnt / N_cnf_Variables);
	fprintf(fw, "c number of literals / number of clauses = %.3f\n", (double)total_Literal_cnt /  Clause_cnt);
	to_Stream(fw, "c ");
#endif
	fprintf(fw, "p cnf %d %d\n", N_cnf_Variables, Clause_cnt);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	    {
		encoding_context.m_clause_generator->generate_Alignment(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)), verbose);
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	    {		    
		encoding_context.m_clause_generator->generate_Alignment(fw, sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[vertex_id], sIntegerIndex(layer)), verbose);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{	
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int N_vertex_Neighbors = m_environment.get_Vertex(vertex_id)->calc_NeighborCount();
		const sVertex::Neighbors_list &out_Neighbors =  m_environment.get_Vertex(vertex_id)->m_Neighbors;

		int out_neighbor_order = 0;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    int neighbor_id = (*out_neighbor)->m_target->m_id;
		    int N_neighbor_Neighbors = m_environment.get_Vertex(neighbor_id)->calc_NeighborCount();
		    int in_neighbor_order = m_environment.get_Vertex(neighbor_id)->calc_NeighborOrder(vertex_id);

		    encoding_context.m_clause_generator->generate_ConditionalEquality(fw,
										      sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[vertex_id], sIntegerIndex(layer)),
										      out_neighbor_order,
										      sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[neighbor_id], sIntegerIndex(layer)),
										      N_neighbor_Neighbors + in_neighbor_order,
										      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(neighbor_id), sIntegerIndex(layer)),
										      0,
										      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)),
										      0,
										      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)),
										      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										      verbose);
		    
		    encoding_context.m_clause_generator->generate_ConditionalEquality(fw,
										      sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[vertex_id], sIntegerIndex(layer)),
										      N_vertex_Neighbors + out_neighbor_order,
										      sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[neighbor_id], sIntegerIndex(layer)),
										      in_neighbor_order,
										      verbose);
		    
		    ++out_neighbor_order;
		}
		encoding_context.m_clause_generator->generate_ConditionalEquality(fw,
										  sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[vertex_id], sIntegerIndex(layer)),
										  2 * N_vertex_Neighbors,
										  sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)),
										  sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										  verbose);
	    }
	}
	
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);
	    encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(0)), robot_id, verbose);
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);
		encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), robot_id, verbose);
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    sStateClauseGenerator::SpecifiedStateIdentifiers_vector goal_case_disj_Identifiers;
		    sStateClauseGenerator::States_vector goal_case_disj_States;

		    goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    goal_case_disj_States.push_back(0);
		    
		    for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
		    {
			goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
			goal_case_disj_States.push_back(*robot_id);
		    }
		    encoding_context.m_clause_generator->generate_DisjunctiveEquality(fw, goal_case_disj_Identifiers, goal_case_disj_States, verbose);
		}
		else
		{
		    encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), sRobotArrangement::VACANT_VERTEX, verbose);
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
    }


    void sMultirobotInstance::to_Stream_AdvancedCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	encoding_context.switchTo_AdvancedGeneratingMode();
	to_Stream_InverseCNFsat(fw, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Stream_DifferentialCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool verbose) const
    {
	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableStateIdentifier robot_location(&encoding_context.m_variable_store, "robot_location", N_Vertices, sIntegerScope(1, N_Robots), sIntegerScope(0, encoding_context.m_N_Layers - 1));
	encoding_context.m_robot_location = robot_location;
	encoding_context.register_TranslateIdentifier(encoding_context.m_robot_location);

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	    {
		Clause_cnt += encoding_context.m_clause_generator->count_Alignment(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)));
	    }
	}
 
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    sStateClauseGenerator::SpecifiedStateIdentifiers_vector robot_diff_Identifiers;
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		robot_diff_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)));
	    }		
	    Clause_cnt += encoding_context.m_clause_generator->count_AllDifferenceConstraint(aux_Variable_cnt,
											     total_Literal_cnt,
											     robot_diff_Identifiers);
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		sStateClauseGenerator::SpecifiedStateIdentifiers_vector prev_robot_diff_Identifiers;
		int prev_robot_id;
		for (prev_robot_id = 1; prev_robot_id < robot_id; ++prev_robot_id)
		{
	 	    prev_robot_diff_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(prev_robot_id), sIntegerIndex(layer)));
		}
		for (++prev_robot_id; prev_robot_id <= N_Robots; ++prev_robot_id)
		{
		    prev_robot_diff_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(prev_robot_id), sIntegerIndex(layer)));
		}
		Clause_cnt += encoding_context.m_clause_generator->count_DifferenceConstraint(aux_Variable_cnt,
											      total_Literal_cnt,
											      sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer + 1)),
											      prev_robot_diff_Identifiers);

		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    sStateClauseGenerator::SpecifiedStateIdentifiers_vector robot_case_split_Identifiers;
		    sStateClauseGenerator::States_vector robot_case_split_States;

		    robot_case_split_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)));
		    robot_case_split_States.push_back(vertex_id);

		    robot_case_split_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer + 1)));
		    robot_case_split_States.push_back(vertex_id);

		    const sVertex::Neighbors_list &Neighbors =  m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
		    {
			int neighbor_id = (*neighbor)->m_target->m_id;

			robot_case_split_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer + 1)));
			robot_case_split_States.push_back(neighbor_id);
		    }
		    Clause_cnt += encoding_context.m_clause_generator->count_CaseSplitting(aux_Variable_cnt,
											   total_Literal_cnt,
											   robot_case_split_Identifiers,
											   robot_case_split_States);
		}
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    int vertex_id = m_initial_arrangement.get_RobotLocation(robot_id);
	    Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
									      total_Literal_cnt,
									      sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(0)),
									      vertex_id);
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		int vertex_id = m_goal_arrangement.get_RobotLocation(robot_id);
		Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
										  total_Literal_cnt,
										  sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(encoding_context.m_N_Layers - 1)),
										  vertex_id);
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		const sRobotGoal::Vertices_set &vertex_IDs = m_goal_specification.get_RobotGoal(robot_id);
		
		if (!vertex_IDs.empty())
		{
		    sStateClauseGenerator::SpecifiedStateIdentifiers_vector goal_case_disj_Identifiers;
		    sStateClauseGenerator::States_vector goal_case_disj_States;

		    goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    goal_case_disj_States.push_back(0);
		    
		    for (sRobotGoal::Vertices_set::const_iterator vertex_id = vertex_IDs.begin(); vertex_id != vertex_IDs.end(); ++vertex_id)
		    {
			goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
			goal_case_disj_States.push_back(*vertex_id);
		    }
		    Clause_cnt += encoding_context.m_clause_generator->count_DisjunctiveEquality(aux_Variable_cnt,
												 total_Literal_cnt,
												 goal_case_disj_Identifiers,
												 goal_case_disj_States);
		}
		else
		{
		    sASSERT(false);
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
	
	int N_cnf_Variables = encoding_context.m_variable_store.get_Last_CNFVariable() + aux_Variable_cnt - 1;

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot differential SAT encoding\n", sPRODUCT);
	fprintf(fw, "c %s\n", sCOPYRIGHT);
	fprintf(fw, "c number of layers = %d\n", encoding_context.m_N_Layers);
	fprintf(fw, "c number of visible variables = %d\n", N_cnf_Variables - aux_Variable_cnt);
	fprintf(fw, "c number of hidden variables = %d\n", aux_Variable_cnt);
	fprintf(fw, "c number of literals = %d\n", total_Literal_cnt);
	fprintf(fw, "c number of clauses = %d\n", Clause_cnt);
	fprintf(fw, "c number of propositional variables = %d\n", N_cnf_Variables);
	fprintf(fw, "c number of clauses / number of variables = %.3f\n", (double)Clause_cnt / N_cnf_Variables);
	fprintf(fw, "c number of literals / number of clauses = %.3f\n", (double)total_Literal_cnt /  Clause_cnt);
	to_Stream(fw, "c ");
#endif
	fprintf(fw, "p cnf %d %d\n", N_cnf_Variables, Clause_cnt);

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	    {
		fprintf(fw, "c label alignment layer=%d robot=%d\n", layer, robot_id);
		encoding_context.m_clause_generator->generate_Alignment(fw, sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)), verbose);
	    }
	}
 
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    sStateClauseGenerator::SpecifiedStateIdentifiers_vector robot_diff_Identifiers;
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		robot_diff_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)));
	    }
	    fprintf(fw, "c label alldifferent layer=%d\n", layer);
	    encoding_context.m_clause_generator->generate_AllDifferenceConstraint(fw, robot_diff_Identifiers, verbose);
	}
	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		sStateClauseGenerator::SpecifiedStateIdentifiers_vector prev_robot_diff_Identifiers;
		int prev_robot_id;
		for (prev_robot_id = 1; prev_robot_id < robot_id; ++prev_robot_id)
		{
		    prev_robot_diff_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(prev_robot_id), sIntegerIndex(layer)));
		}
		for (++prev_robot_id; prev_robot_id <= N_Robots; ++prev_robot_id)
		{
		    prev_robot_diff_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(prev_robot_id), sIntegerIndex(layer)));
		}
		fprintf(fw, "c label different layer=%d robot=%d\n", layer, robot_id);
		encoding_context.m_clause_generator->generate_DifferenceConstraint(fw, sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer + 1)), prev_robot_diff_Identifiers, verbose);

		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    sStateClauseGenerator::SpecifiedStateIdentifiers_vector robot_case_split_Identifiers;
		    sStateClauseGenerator::States_vector robot_case_split_States;

		    robot_case_split_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)));
		    robot_case_split_States.push_back(vertex_id);

		    robot_case_split_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer + 1)));
		    robot_case_split_States.push_back(vertex_id);

		    const sVertex::Neighbors_list &Neighbors =  m_environment.get_Vertex(vertex_id)->m_Neighbors;
 
		    for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
		    {
			int neighbor_id = (*neighbor)->m_target->m_id;

			robot_case_split_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer + 1)));
			robot_case_split_States.push_back(neighbor_id);
		    }
		    fprintf(fw, "c label case_split layer=%d robot=%d vertex=%d\n", layer, robot_id, vertex_id);
		    encoding_context.m_clause_generator->generate_CaseSplitting(fw, robot_case_split_Identifiers, robot_case_split_States, verbose);
		}
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    int vertex_id = m_initial_arrangement.get_RobotLocation(robot_id);

	    fprintf(fw, "c label initial robot=%d\n", robot_id);
	    encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(0)), vertex_id, verbose);
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		int vertex_id = m_goal_arrangement.get_RobotLocation(robot_id);
		fprintf(fw, "c label goal robot=%d\n", robot_id);
		encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), vertex_id, verbose);
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		const sRobotGoal::Vertices_set &vertex_IDs = m_goal_specification.get_RobotGoal(robot_id);
		
		if (!vertex_IDs.empty())
		{
		    sStateClauseGenerator::SpecifiedStateIdentifiers_vector goal_case_disj_Identifiers;
		    sStateClauseGenerator::States_vector goal_case_disj_States;

		    goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    goal_case_disj_States.push_back(0);
		    
		    for (sRobotGoal::Vertices_set::const_iterator vertex_id = vertex_IDs.begin(); vertex_id != vertex_IDs.end(); ++vertex_id)
		    {
			goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
			goal_case_disj_States.push_back(*vertex_id);
		    }
		    fprintf(fw, "c label goal robot=%d\n", robot_id);
		    encoding_context.m_clause_generator->generate_DisjunctiveEquality(fw, goal_case_disj_Identifiers, goal_case_disj_States, verbose);
		}
		else
		{
		    sASSERT(false);
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
    }


    void sMultirobotInstance::to_Stream_BijectionCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	encoding_context.switchTo_AdvancedGeneratingMode();
	to_Stream_DifferentialCNFsat(fw, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Stream_HeuristicDifferentialCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool verbose)
    {
	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	m_environment.calc_AllPairsShortestPaths();
	const sUndirectedGraph::Distances_2d_vector &all_pairs_Distances = m_environment.get_AllPairsShortestPaths();

#ifdef MUTEX
	m_environment.calc_AllPairsShortestCoopPaths();
	const sUndirectedGraph::Distances_4d_vector &all_pairs_coop_Distances = m_environment.get_AllPairsShortestCoopPaths();

	s_GlobalPhaseStatistics.enter_Phase("SAT");
#endif

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableStateIdentifier robot_location(&encoding_context.m_variable_store, "robot_location", N_Vertices, sIntegerScope(1, N_Robots), sIntegerScope(0, encoding_context.m_N_Layers - 1));
	encoding_context.m_robot_location = robot_location;
	encoding_context.register_TranslateIdentifier(encoding_context.m_robot_location);

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	    {
		Clause_cnt += encoding_context.m_clause_generator->count_Alignment(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)));
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
        {
	    sStateClauseGenerator::SpecifiedStateIdentifiers_vector robot_diff_Identifiers;
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		robot_diff_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)));
	    }
	    Clause_cnt += encoding_context.m_clause_generator->count_AllDifferenceConstraint(aux_Variable_cnt,
											     total_Literal_cnt,
											     robot_diff_Identifiers);
	}
	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		sStateClauseGenerator::SpecifiedStateIdentifiers_vector prev_robot_diff_Identifiers;
		int prev_robot_id;
		for (prev_robot_id = 1; prev_robot_id < robot_id; ++prev_robot_id)
		{
		    prev_robot_diff_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(prev_robot_id), sIntegerIndex(layer)));
		}
		for (++prev_robot_id; prev_robot_id <= N_Robots; ++prev_robot_id)
		{
		    prev_robot_diff_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(prev_robot_id), sIntegerIndex(layer)));
		}
		Clause_cnt += encoding_context.m_clause_generator->count_DifferenceConstraint(aux_Variable_cnt,
											      total_Literal_cnt,
											      sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer + 1)),
											      prev_robot_diff_Identifiers);

		sStateClauseGenerator::States_vector vertex_IDs;

		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    bool forbid_vertex = true;

		    switch (m_goal_type)
		    {
		    case GOAL_TYPE_ARRANGEMENT:
		    {
			if (all_pairs_Distances[vertex_id][m_goal_arrangement.get_RobotLocation(robot_id)] <= encoding_context.m_N_Layers - layer)
			{
			    forbid_vertex = false;
			}
			break;
		    }
		    case GOAL_TYPE_SPECIFICATION:
		    {
			const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
			for (sRobotGoal::Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
			{
			    if (all_pairs_Distances[vertex_id][*goal_id] <= encoding_context.m_N_Layers - layer)
			    {
				forbid_vertex = false;
				break;
			    }
			}
			break;
		    }
		    default:
		    {
			sASSERT(false);
			break;
		    }
		    }
		    if (forbid_vertex || all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] > layer)
		    {
			vertex_IDs.push_back(vertex_id);
		    }
		    else
		    {
			sStateClauseGenerator::SpecifiedStateIdentifiers_vector robot_case_split_Identifiers;
			sStateClauseGenerator::States_vector robot_case_split_States;
			
			robot_case_split_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)));
			robot_case_split_States.push_back(vertex_id);
			
			robot_case_split_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer + 1)));
			robot_case_split_States.push_back(vertex_id);
			
			const sVertex::Neighbors_list &Neighbors =  m_environment.get_Vertex(vertex_id)->m_Neighbors;
			
			for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
			{
			    int neighbor_id = (*neighbor)->m_target->m_id;
			    
			    robot_case_split_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer + 1)));
			    robot_case_split_States.push_back(neighbor_id);
			}
			Clause_cnt += encoding_context.m_clause_generator->count_CaseSplitting(aux_Variable_cnt,
											       total_Literal_cnt,
											       robot_case_split_Identifiers,
											       robot_case_split_States);

#ifdef MUTEX
			for (int vertex_2_id = vertex_id + 1; vertex_2_id < N_Vertices; ++vertex_2_id)
			{
			    for (int robot_2_id = 1; robot_2_id <= N_Robots; ++robot_2_id)
			    {
				if (all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_2_id)][vertex_2_id] <= layer && all_pairs_Distances[vertex_2_id][m_goal_arrangement.get_RobotLocation(robot_2_id)] <= encoding_context.m_N_Layers - layer)
				{

				    if (   all_pairs_coop_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][m_initial_arrangement.get_RobotLocation(robot_2_id)][vertex_id][vertex_2_id] > layer 
					|| all_pairs_coop_Distances[vertex_id][vertex_2_id][m_goal_arrangement.get_RobotLocation(robot_id)][m_goal_arrangement.get_RobotLocation(robot_2_id)] > encoding_context.m_N_Layers - layer)
				    {
					Clause_cnt += encoding_context.m_clause_generator->count_DisjunctiveDisequality(aux_Variable_cnt,
															total_Literal_cnt, 
															sSpecifiedStateIdentifier(&robot_location,
																		  sIntegerIndex(robot_id),
																		  sIntegerIndex(layer)),
															vertex_id,
															sSpecifiedStateIdentifier(&robot_location,
																		  sIntegerIndex(robot_2_id),
																		  sIntegerIndex(layer)),
															vertex_2_id);
				    }
				}
			    }
			}
#endif
		    }
		    //			Clause_cnt += encoding_context.m_clause_generator->count_Disequality(aux_Variable_cnt, total_Literal_cnt, sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)), vertex_id);
		}
		Clause_cnt += encoding_context.m_clause_generator->count_Disequalities(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)),
										       vertex_IDs);
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    int vertex_id = m_initial_arrangement.get_RobotLocation(robot_id);
	    Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
									      total_Literal_cnt,
									      sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(0)),
									      vertex_id);
	}
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		int vertex_id = m_goal_arrangement.get_RobotLocation(robot_id);
		Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
										  total_Literal_cnt,
										  sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(encoding_context.m_N_Layers - 1)),
										  vertex_id);
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		const sRobotGoal::Vertices_set &vertex_IDs = m_goal_specification.get_RobotGoal(robot_id);
		
		if (!vertex_IDs.empty())
		{
		    sStateClauseGenerator::SpecifiedStateIdentifiers_vector goal_case_disj_Identifiers;
		    sStateClauseGenerator::States_vector goal_case_disj_States;

		    goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    goal_case_disj_States.push_back(0);
		    
		    for (sRobotGoal::Vertices_set::const_iterator vertex_id = vertex_IDs.begin(); vertex_id != vertex_IDs.end(); ++vertex_id)
		    {
			goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
			goal_case_disj_States.push_back(*vertex_id);
		    }
		    Clause_cnt += encoding_context.m_clause_generator->count_DisjunctiveEquality(aux_Variable_cnt,
												 total_Literal_cnt,
												 goal_case_disj_Identifiers,
												 goal_case_disj_States);
		}
		else
		{
		    sASSERT(false);
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}


	int N_cnf_Variables = encoding_context.m_variable_store.get_Last_CNFVariable() + aux_Variable_cnt - 1;

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot differential SAT encoding\n", sPRODUCT);
	fprintf(fw, "c %s\n", sCOPYRIGHT);
	fprintf(fw, "c number of layers = %d\n", encoding_context.m_N_Layers);
	fprintf(fw, "c number of visible variables = %d\n", N_cnf_Variables - aux_Variable_cnt);
	fprintf(fw, "c number of hidden variables = %d\n", aux_Variable_cnt);
	fprintf(fw, "c number of literals = %d\n", total_Literal_cnt);
	fprintf(fw, "c number of clauses = %d\n", Clause_cnt);
	fprintf(fw, "c number of propositional variables = %d\n", N_cnf_Variables);
	fprintf(fw, "c number of clauses / number of variables = %.3f\n", (double)Clause_cnt / N_cnf_Variables);
	fprintf(fw, "c number of literals / number of clauses = %.3f\n", (double)total_Literal_cnt /  Clause_cnt);
	to_Stream(fw, "c ");
#endif
	fprintf(fw, "p cnf %d %d\n", N_cnf_Variables, Clause_cnt);

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	    {
		encoding_context.m_clause_generator->generate_Alignment(fw, sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)), verbose);
	    }
	}
 
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    sStateClauseGenerator::SpecifiedStateIdentifiers_vector robot_diff_Identifiers;
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		robot_diff_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)));
	    }		
	    encoding_context.m_clause_generator->generate_AllDifferenceConstraint(fw, robot_diff_Identifiers, verbose);
	}
	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		sStateClauseGenerator::SpecifiedStateIdentifiers_vector prev_robot_diff_Identifiers;
		int prev_robot_id;
		for (prev_robot_id = 1; prev_robot_id < robot_id; ++prev_robot_id)
		{
		    prev_robot_diff_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(prev_robot_id), sIntegerIndex(layer)));
		}
		for (++prev_robot_id; prev_robot_id <= N_Robots; ++prev_robot_id)
		{
		    prev_robot_diff_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(prev_robot_id), sIntegerIndex(layer)));
		}
		encoding_context.m_clause_generator->generate_DifferenceConstraint(fw, sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer + 1)), prev_robot_diff_Identifiers, verbose);

		sStateClauseGenerator::States_vector vertex_IDs;

		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    bool forbid_vertex = true;

		    switch (m_goal_type)
		    {
		    case GOAL_TYPE_ARRANGEMENT:
		    {
			if (all_pairs_Distances[vertex_id][m_goal_arrangement.get_RobotLocation(robot_id)] <= encoding_context.m_N_Layers - layer)
			{
			    forbid_vertex = false;
			}
			break;
		    }
		    case GOAL_TYPE_SPECIFICATION:
		    {
			const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
			for (sRobotGoal::Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
			{
			    if (all_pairs_Distances[vertex_id][*goal_id] <= encoding_context.m_N_Layers - layer)
			    {
				forbid_vertex = false;
				break;
			    }
			}
			break;
		    }
		    default:
		    {
			sASSERT(false);
			break;
		    }
		    }
		    if (forbid_vertex || all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] > layer)
		    {
			vertex_IDs.push_back(vertex_id);
		    }
		    else
		    {
		      sStateClauseGenerator::SpecifiedStateIdentifiers_vector robot_case_split_Identifiers;
			sStateClauseGenerator::States_vector robot_case_split_States;
			
			robot_case_split_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)));
			robot_case_split_States.push_back(vertex_id);
			
			robot_case_split_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer + 1)));
			robot_case_split_States.push_back(vertex_id);
			
			const sVertex::Neighbors_list &Neighbors =  m_environment.get_Vertex(vertex_id)->m_Neighbors;
			
			for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
			{
			    int neighbor_id = (*neighbor)->m_target->m_id;
			    
			    robot_case_split_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer + 1)));
			    robot_case_split_States.push_back(neighbor_id);
			}
			encoding_context.m_clause_generator->generate_CaseSplitting(fw, robot_case_split_Identifiers, robot_case_split_States, verbose);

#ifdef MUTEX
			for (int vertex_2_id = vertex_id + 1; vertex_2_id < N_Vertices; ++vertex_2_id)
			{
			    for (int robot_2_id = 1; robot_2_id <= N_Robots; ++robot_2_id)
			    {
				if (all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_2_id)][vertex_2_id] <= layer && all_pairs_Distances[vertex_2_id][m_goal_arrangement.get_RobotLocation(robot_2_id)] <= encoding_context.m_N_Layers - layer)
				{

				    if (   all_pairs_coop_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][m_initial_arrangement.get_RobotLocation(robot_2_id)][vertex_id][vertex_2_id] > layer 
					|| all_pairs_coop_Distances[vertex_id][vertex_2_id][m_goal_arrangement.get_RobotLocation(robot_id)][m_goal_arrangement.get_RobotLocation(robot_2_id)] > encoding_context.m_N_Layers - layer)
				    {
					encoding_context.m_clause_generator->generate_DisjunctiveDisequality(fw,
													     sSpecifiedStateIdentifier(&robot_location,
																       sIntegerIndex(robot_id),
																       sIntegerIndex(layer)),
													     vertex_id,
													     sSpecifiedStateIdentifier(&robot_location,
																       sIntegerIndex(robot_2_id),
																       sIntegerIndex(layer)),
													     vertex_2_id);
				    }
				}
			    }
			}
#endif
		    }
		}
		encoding_context.m_clause_generator->generate_Disequalities(fw, sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)), vertex_IDs, verbose);
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    int vertex_id = m_initial_arrangement.get_RobotLocation(robot_id);
	    encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(0)), vertex_id, verbose);
	}
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		int vertex_id = m_goal_arrangement.get_RobotLocation(robot_id);
		encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), vertex_id, verbose);
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		const sRobotGoal::Vertices_set &vertex_IDs = m_goal_specification.get_RobotGoal(robot_id);
		
		if (!vertex_IDs.empty())
		{
		    sStateClauseGenerator::SpecifiedStateIdentifiers_vector goal_case_disj_Identifiers;
		    sStateClauseGenerator::States_vector goal_case_disj_States;

		    goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    goal_case_disj_States.push_back(0);
		    
		    for (sRobotGoal::Vertices_set::const_iterator vertex_id = vertex_IDs.begin(); vertex_id != vertex_IDs.end(); ++vertex_id)
		    {
			goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
			goal_case_disj_States.push_back(*vertex_id);
		    }
		    encoding_context.m_clause_generator->generate_DisjunctiveEquality(fw, goal_case_disj_Identifiers, goal_case_disj_States, verbose);
		}
		else
		{
		    sASSERT(false);
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
    }


    void sMultirobotInstance::to_Stream_HeuristicBijectionCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	encoding_context.switchTo_AdvancedGeneratingMode();
	to_Stream_HeuristicDifferentialCNFsat(fw, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Stream_HeuristicAdvancedCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool verbose)
    {
	if (encoding_context.get_GeneratingMode() != sMultirobotEncodingContext_CNFsat::GENERATING_BITWISE)
	{
	    encoding_context.switchTo_AdvancedGeneratingMode();
	}
	m_environment.calc_AllPairsShortestPaths();
	const sUndirectedGraph::Distances_2d_vector &all_pairs_Distances = m_environment.get_AllPairsShortestPaths();

#ifdef MUTEX
	m_environment.calc_AllPairsShortestCoopPaths();
	const sUndirectedGraph::Distances_4d_vector &all_pairs_coop_Distances = m_environment.get_AllPairsShortestCoopPaths();

	s_GlobalPhaseStatistics.enter_Phase("SAT");
#endif

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;
 
	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableStateIdentifier vertex_occupancy(&encoding_context.m_variable_store, "vertex_occupancy", N_Robots + 1, sIntegerScope(0, N_Vertices), sIntegerScope(0, encoding_context.m_N_Layers - 1));
	encoding_context.m_vertex_occupancy = vertex_occupancy;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	    {
		Clause_cnt += encoding_context.m_clause_generator->count_Alignment(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
	    }
	}
	encoding_context.m_transition_Actions.resize(N_Vertices);
	
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    sIndexableStateIdentifier transition_action(&encoding_context.m_variable_store, "transition_action-" + sInt_32_to_String(vertex_id), 2 * m_environment.get_Vertex(vertex_id)->calc_NeighborCount() + 1, sIntegerScope(0, encoding_context.m_N_Layers - 1)); 
	    encoding_context.m_transition_Actions[vertex_id] = transition_action;
	    encoding_context.register_TranslateIdentifier(encoding_context.m_transition_Actions[vertex_id]);

	    for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	    {		    
		Clause_cnt += encoding_context.m_clause_generator->count_Alignment(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedStateIdentifier(&transition_action, sIntegerIndex(layer)));
		sStateClauseGenerator::States_vector robot_IDs;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    bool forbid_vertex = true;

		    switch (m_goal_type)
		    {
		    case GOAL_TYPE_ARRANGEMENT:
		    {
			if (all_pairs_Distances[vertex_id][m_goal_arrangement.get_RobotLocation(robot_id)] <= encoding_context.m_N_Layers - layer)
			{
			    forbid_vertex = false;
			}
			break;
		    }
		    case GOAL_TYPE_SPECIFICATION:
		    {
			const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
			for (sRobotGoal::Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
			{
			    if (all_pairs_Distances[vertex_id][*goal_id] <= encoding_context.m_N_Layers - layer)
			    {
				forbid_vertex = false;
				break;
			    }
			}
			break;
		    }
		    default:
		    {
			sASSERT(false);
			break;
		    }
		    }
		    if (forbid_vertex || all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] > layer)
		    {
			robot_IDs.push_back(robot_id);
		    }
		    else
		    {
#ifdef MUTEX
			for (int vertex_2_id = vertex_id + 1; vertex_2_id < N_Vertices; ++vertex_2_id)
			{
			    for (int robot_2_id = 1; robot_2_id <= N_Robots; ++robot_2_id)
			    {
				if (all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_2_id)][vertex_2_id] <= layer && all_pairs_Distances[vertex_2_id][m_goal_arrangement.get_RobotLocation(robot_2_id)] <= encoding_context.m_N_Layers - layer)
				{

				    if (   all_pairs_coop_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][m_initial_arrangement.get_RobotLocation(robot_2_id)][vertex_id][vertex_2_id] > layer 
					|| all_pairs_coop_Distances[vertex_id][vertex_2_id][m_goal_arrangement.get_RobotLocation(robot_id)][m_goal_arrangement.get_RobotLocation(robot_2_id)] > encoding_context.m_N_Layers - layer)
				    {
					Clause_cnt += encoding_context.m_clause_generator->count_DisjunctiveDisequality(aux_Variable_cnt,
															total_Literal_cnt,
															sSpecifiedStateIdentifier(&vertex_occupancy,
																		  sIntegerIndex(vertex_id),
																		  sIntegerIndex(layer)),
															robot_id,
															sSpecifiedStateIdentifier(&vertex_occupancy,
																		  sIntegerIndex(vertex_2_id),
																		  sIntegerIndex(layer)),
															robot_2_id);
				    }
				}
			    }
			}
#endif
		    }
		}
		Clause_cnt += encoding_context.m_clause_generator->count_Disequalities(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										       robot_IDs);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{	
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int N_vertex_Neighbors = m_environment.get_Vertex(vertex_id)->calc_NeighborCount();
		const sVertex::Neighbors_list &out_Neighbors =  m_environment.get_Vertex(vertex_id)->m_Neighbors;

		int out_neighbor_order = 0;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    int neighbor_id = (*out_neighbor)->m_target->m_id;
		    int N_neighbor_Neighbors = m_environment.get_Vertex(neighbor_id)->calc_NeighborCount();
		    int in_neighbor_order = m_environment.get_Vertex(neighbor_id)->calc_NeighborOrder(vertex_id);

		    Clause_cnt += encoding_context.m_clause_generator->count_ConditionalEquality(aux_Variable_cnt,
												 total_Literal_cnt,
												 sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[vertex_id], sIntegerIndex(layer)),
												 out_neighbor_order,
												 sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[neighbor_id], sIntegerIndex(layer)),
												 N_neighbor_Neighbors + in_neighbor_order,
												 sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(neighbor_id), sIntegerIndex(layer)),
												 0,
												 sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)),
												 0,
												 sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)),
												 sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		    
		    Clause_cnt += encoding_context.m_clause_generator->count_ConditionalEquality(aux_Variable_cnt,
												 total_Literal_cnt,
												 sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[vertex_id], sIntegerIndex(layer)),
												 N_vertex_Neighbors + out_neighbor_order,
												 sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[neighbor_id], sIntegerIndex(layer)),
												 in_neighbor_order);

		    ++out_neighbor_order;
		}
		
		Clause_cnt += encoding_context.m_clause_generator->count_ConditionalEquality(aux_Variable_cnt,
											     total_Literal_cnt,
											     sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[vertex_id], sIntegerIndex(layer)),
											     2 * N_vertex_Neighbors,
											     sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)),
											     sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);
	    Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
									      total_Literal_cnt,
									      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(0)),
									      robot_id);
	}
/*
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);
	    Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt, total_Literal_cnt, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), robot_id);
	}
*/
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);
		Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
										  total_Literal_cnt,
										  sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)),
										  robot_id);
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    sStateClauseGenerator::SpecifiedStateIdentifiers_vector goal_case_disj_Identifiers;
		    sStateClauseGenerator::States_vector goal_case_disj_States;
		    
		    goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    goal_case_disj_States.push_back(0);
		    
		    for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
		    {
			goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
			goal_case_disj_States.push_back(*robot_id);
		    }
		    Clause_cnt += encoding_context.m_clause_generator->count_DisjunctiveEquality(aux_Variable_cnt,
												 total_Literal_cnt,
												 goal_case_disj_Identifiers,
												 goal_case_disj_States);
		}
		else
		{
		    Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
										      total_Literal_cnt,
										      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)),
										      sRobotArrangement::VACANT_VERTEX);
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}

	int N_cnf_Variables = encoding_context.m_variable_store.get_Last_CNFVariable() + aux_Variable_cnt - 1;

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot advanced SAT encoding\n", sPRODUCT);
	fprintf(fw, "c %s\n", sCOPYRIGHT);
	fprintf(fw, "c number of layers = %d\n", encoding_context.m_N_Layers);
	fprintf(fw, "c number of visible variables = %d\n", N_cnf_Variables - aux_Variable_cnt);
	fprintf(fw, "c number of hidden variables = %d\n", aux_Variable_cnt);
	fprintf(fw, "c number of literals = %d\n", total_Literal_cnt);
	fprintf(fw, "c number of clauses = %d\n", Clause_cnt);
	fprintf(fw, "c number of propositional variables = %d\n", N_cnf_Variables);
	fprintf(fw, "c number of clauses / number of variables = %.3f\n", (double)Clause_cnt / N_cnf_Variables);
	fprintf(fw, "c number of literals / number of clauses = %.3f\n", (double)total_Literal_cnt /  Clause_cnt);
	to_Stream(fw, "c ");
#endif
	fprintf(fw, "p cnf %d %d\n", N_cnf_Variables, Clause_cnt);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	    {
		encoding_context.m_clause_generator->generate_Alignment(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)), verbose);
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	    {		    
		encoding_context.m_clause_generator->generate_Alignment(fw, sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[vertex_id], sIntegerIndex(layer)), verbose);
		sStateClauseGenerator::States_vector robot_IDs;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    bool forbid_vertex = true;

		    switch (m_goal_type)
		    {
		    case GOAL_TYPE_ARRANGEMENT:
		    {
			if (all_pairs_Distances[vertex_id][m_goal_arrangement.get_RobotLocation(robot_id)] <= encoding_context.m_N_Layers - layer)
			{
			    forbid_vertex = false;
			}
			break;
		    }
		    case GOAL_TYPE_SPECIFICATION:
		    {
			const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
			sASSERT(!goal_IDs.empty());

			for (sRobotGoal::Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
			{
			    if (all_pairs_Distances[vertex_id][*goal_id] <= encoding_context.m_N_Layers - layer)
			    {
				forbid_vertex = false;
				break;
			    }
			}
			break;
		    }
		    default:
		    {
			sASSERT(false);
			break;
		    }
		    }
		    if (forbid_vertex || all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] > layer)
		    {
			robot_IDs.push_back(robot_id);
		    }
		    else
		    {
#ifdef MUTEX
			for (int vertex_2_id = vertex_id + 1; vertex_2_id < N_Vertices; ++vertex_2_id)
			{
			    for (int robot_2_id = 1; robot_2_id <= N_Robots; ++robot_2_id)
			    {
				if (all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_2_id)][vertex_2_id] <= layer && all_pairs_Distances[vertex_2_id][m_goal_arrangement.get_RobotLocation(robot_2_id)] <= encoding_context.m_N_Layers - layer)
				{

				    if (   all_pairs_coop_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][m_initial_arrangement.get_RobotLocation(robot_2_id)][vertex_id][vertex_2_id] > layer 
					|| all_pairs_coop_Distances[vertex_id][vertex_2_id][m_goal_arrangement.get_RobotLocation(robot_id)][m_goal_arrangement.get_RobotLocation(robot_2_id)] > encoding_context.m_N_Layers - layer)
				    {
					encoding_context.m_clause_generator->generate_DisjunctiveDisequality(fw,
													     sSpecifiedStateIdentifier(&vertex_occupancy,
																       sIntegerIndex(vertex_id),
																       sIntegerIndex(layer)),
													     robot_id,
													     sSpecifiedStateIdentifier(&vertex_occupancy,
																       sIntegerIndex(vertex_2_id),
																       sIntegerIndex(layer)),
													     robot_2_id);
				    }
				}
			    }
			}
#endif
		    }
		}
		encoding_context.m_clause_generator->generate_Disequalities(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)), robot_IDs);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{	
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int N_vertex_Neighbors = m_environment.get_Vertex(vertex_id)->calc_NeighborCount();
		const sVertex::Neighbors_list &out_Neighbors =  m_environment.get_Vertex(vertex_id)->m_Neighbors;

		int out_neighbor_order = 0;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    int neighbor_id = (*out_neighbor)->m_target->m_id;
		    int N_neighbor_Neighbors = m_environment.get_Vertex(neighbor_id)->calc_NeighborCount();
		    int in_neighbor_order = m_environment.get_Vertex(neighbor_id)->calc_NeighborOrder(vertex_id);

		    encoding_context.m_clause_generator->generate_ConditionalEquality(fw,
										      sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[vertex_id], sIntegerIndex(layer)),
										      out_neighbor_order,
										      sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[neighbor_id], sIntegerIndex(layer)),
										      N_neighbor_Neighbors + in_neighbor_order,
										      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(neighbor_id), sIntegerIndex(layer)),
										      0,
										      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)),
										      0,
										      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)),
										      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										      verbose);
		    
		    encoding_context.m_clause_generator->generate_ConditionalEquality(fw,
										      sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[vertex_id], sIntegerIndex(layer)),
										      N_vertex_Neighbors + out_neighbor_order,
										      sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[neighbor_id], sIntegerIndex(layer)),
										      in_neighbor_order,
										      verbose);
		    
		    ++out_neighbor_order;
		}
		encoding_context.m_clause_generator->generate_ConditionalEquality(fw,
										  sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[vertex_id], sIntegerIndex(layer)),
										  2 * N_vertex_Neighbors,
										  sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)),
										  sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										  verbose);
	    }
	}
	
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);
	    encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(0)), robot_id, verbose);
	}
/*
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);
	    encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), robot_id, verbose);
	} 
*/
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);
		encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), robot_id, verbose);
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    sStateClauseGenerator::SpecifiedStateIdentifiers_vector goal_case_disj_Identifiers;
		    sStateClauseGenerator::States_vector goal_case_disj_States;
		    
		    goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    goal_case_disj_States.push_back(0);
		    
		    for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
		    {
			goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
			goal_case_disj_States.push_back(*robot_id);
		    }
		    encoding_context.m_clause_generator->generate_DisjunctiveEquality(fw, goal_case_disj_Identifiers, goal_case_disj_States, verbose);
		}
		else
		{
		    encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), sRobotArrangement::VACANT_VERTEX, verbose);
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
    }


    void sMultirobotInstance::to_Stream_PuzzleCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool verbose) const
    {
	encoding_context.switchTo_AdvancedGeneratingMode();

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Edges = m_environment.get_EdgeCount();

	sIndexableStateIdentifier vertex_occupancy(&encoding_context.m_variable_store, "vertex_occupancy", N_Robots + 1, sIntegerScope(0, N_Vertices), sIntegerScope(0, encoding_context.m_N_Layers - 1));
	encoding_context.m_vertex_occupancy = vertex_occupancy;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy);

	sIndexableStateIdentifier transition_action(&encoding_context.m_variable_store, "transition_action", 1 + 2 * N_Edges, sIntegerScope(0, encoding_context.m_N_Layers - 1));
	encoding_context.m_transition_action = transition_action;
	encoding_context.register_TranslateIdentifier(encoding_context.m_transition_action);

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		Clause_cnt += encoding_context.m_clause_generator->count_Alignment(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
	    }
	    Clause_cnt += encoding_context.m_clause_generator->count_Alignment(aux_Variable_cnt,
									       total_Literal_cnt,
									       sSpecifiedStateIdentifier(&transition_action, sIntegerIndex(layer)));
	}
         
	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{	
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sVertex::Neighbors_list &out_Neighbors =  m_environment.get_Vertex(vertex_id)->m_Neighbors;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    int neighbor_id = (*out_neighbor)->m_target->m_id;
		    int action_id;

		    if ((*out_neighbor)->m_edge->m_arc_uv.m_target->m_id == vertex_id)
		    {
			action_id = (*out_neighbor)->m_edge->m_id;
		    }
		    else
		    {
			action_id = (*out_neighbor)->m_edge->m_id + N_Edges;
		    }

		    sStateClauseGenerator::SpecifiedStateIdentifiers_vector occupancy_Identifiers_A;
		    sStateClauseGenerator::SpecifiedStateIdentifiers_vector occupancy_Identifiers_B;

		    occupancy_Identifiers_A.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)));
		    occupancy_Identifiers_B.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));

		    for (int remain_vertex_id = 0; remain_vertex_id < N_Vertices; ++remain_vertex_id)
		    {
			if (remain_vertex_id != vertex_id && remain_vertex_id != neighbor_id)
			{
			    occupancy_Identifiers_A.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(remain_vertex_id), sIntegerIndex(layer + 1)));
			    occupancy_Identifiers_B.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(remain_vertex_id), sIntegerIndex(layer)));
			}
		    }
		    
		    Clause_cnt += encoding_context.m_clause_generator->count_LargeConditionalEquality(aux_Variable_cnt,
												      total_Literal_cnt,
												      sSpecifiedStateIdentifier(&encoding_context.m_transition_action, sIntegerIndex(layer)),
												      action_id,
												      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)),
												      0,
												      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(neighbor_id), sIntegerIndex(layer)),
												      0,
												      occupancy_Identifiers_A,
												      occupancy_Identifiers_B);
		}
		sStateClauseGenerator::SpecifiedStateIdentifiers_vector occupancy_Identifiers_A;
		sStateClauseGenerator::SpecifiedStateIdentifiers_vector occupancy_Identifiers_B;

		for (int remain_vertex_id = 0; remain_vertex_id < N_Vertices; ++remain_vertex_id)
		{
		    occupancy_Identifiers_A.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(remain_vertex_id), sIntegerIndex(layer + 1)));
		    occupancy_Identifiers_B.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(remain_vertex_id), sIntegerIndex(layer)));
		}

		Clause_cnt += encoding_context.m_clause_generator->count_LargeConditionalEquality(aux_Variable_cnt,
												  total_Literal_cnt,
												  sSpecifiedStateIdentifier(&encoding_context.m_transition_action, sIntegerIndex(layer)),
												  2 * N_Edges,
												  occupancy_Identifiers_A,
												  occupancy_Identifiers_B);
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);
	    Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
									      total_Literal_cnt,
									      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(0)),
									      robot_id);
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);
	    Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
									      total_Literal_cnt,
									      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)),
									      robot_id);
	}

	int N_cnf_Variables = encoding_context.m_variable_store.get_Last_CNFVariable() + aux_Variable_cnt - 1;

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot inverse SAT encoding\n", sPRODUCT);
	fprintf(fw, "c %s\n", sCOPYRIGHT);
	fprintf(fw, "c number of layers = %d\n", encoding_context.m_N_Layers);
	fprintf(fw, "c number of visible variables = %d\n", N_cnf_Variables - aux_Variable_cnt);
	fprintf(fw, "c number of hidden variables = %d\n", aux_Variable_cnt);
	fprintf(fw, "c number of literals = %d\n", total_Literal_cnt);
	fprintf(fw, "c number of clauses = %d\n", Clause_cnt);
	fprintf(fw, "c number of propositional variables = %d\n", N_cnf_Variables);
	fprintf(fw, "c number of clauses / number of variables = %.3f\n", (double)Clause_cnt / N_cnf_Variables);
	fprintf(fw, "c number of literals / number of clauses = %.3f\n", (double)total_Literal_cnt /  Clause_cnt);
	to_Stream(fw, "c ");
#endif
	fprintf(fw, "p cnf %d %d\n", N_cnf_Variables, Clause_cnt);

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {

		encoding_context.m_clause_generator->generate_Alignment(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)), verbose);
	    }
	    encoding_context.m_clause_generator->generate_Alignment(fw, sSpecifiedStateIdentifier(&transition_action, sIntegerIndex(layer)), verbose);
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{	
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sVertex::Neighbors_list &out_Neighbors =  m_environment.get_Vertex(vertex_id)->m_Neighbors;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    int neighbor_id = (*out_neighbor)->m_target->m_id;

		    int action_id;
		    if ((*out_neighbor)->m_edge->m_arc_uv.m_target->m_id == vertex_id)
		    {
			action_id = (*out_neighbor)->m_edge->m_id;
		    }
		    else
		    {
			action_id = (*out_neighbor)->m_edge->m_id + N_Edges;
		    }

		    sStateClauseGenerator::SpecifiedStateIdentifiers_vector occupancy_Identifiers_A;
		    sStateClauseGenerator::SpecifiedStateIdentifiers_vector occupancy_Identifiers_B;

		    occupancy_Identifiers_A.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)));
		    occupancy_Identifiers_B.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));

		    for (int remain_vertex_id = 0; remain_vertex_id < N_Vertices; ++remain_vertex_id)
		    {
			if (remain_vertex_id != vertex_id && remain_vertex_id != neighbor_id)
			{
			    occupancy_Identifiers_A.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(remain_vertex_id), sIntegerIndex(layer + 1)));
			    occupancy_Identifiers_B.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(remain_vertex_id), sIntegerIndex(layer)));
			}
		    }
		    
		    encoding_context.m_clause_generator->generate_LargeConditionalEquality(fw,
											   sSpecifiedStateIdentifier(&encoding_context.m_transition_action, sIntegerIndex(layer)),
											   action_id,
											   sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)),
											   0,
											   sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(neighbor_id), sIntegerIndex(layer)),
											   0,
											   occupancy_Identifiers_A,
											   occupancy_Identifiers_B,
											   verbose);
		}
		sStateClauseGenerator::SpecifiedStateIdentifiers_vector occupancy_Identifiers_A;
		sStateClauseGenerator::SpecifiedStateIdentifiers_vector occupancy_Identifiers_B;

		for (int remain_vertex_id = 0; remain_vertex_id < N_Vertices; ++remain_vertex_id)
		{
		    occupancy_Identifiers_A.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(remain_vertex_id), sIntegerIndex(layer + 1)));
		    occupancy_Identifiers_B.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(remain_vertex_id), sIntegerIndex(layer)));
		}

		encoding_context.m_clause_generator->generate_LargeConditionalEquality(fw,
										       sSpecifiedStateIdentifier(&encoding_context.m_transition_action, sIntegerIndex(layer)),
										       2 * N_Edges,
										       occupancy_Identifiers_A,
										       occupancy_Identifiers_B,
										       verbose);
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);
	    encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(0)), robot_id, verbose);
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);
	    encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), robot_id, verbose);
	} 	
    }


    void sMultirobotInstance::to_Stream_BitwiseCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	encoding_context.switchTo_BitwiseGeneratingMode();
	to_Stream_HeuristicAdvancedCNFsat(fw, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Stream_FlowCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose)) const
    {
	encoding_context.switchTo_AdvancedGeneratingMode();

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableBitIdentifier vertex_occupancy_by_robot(&encoding_context.m_variable_store, "vertex_occupancy_by_robot", sIntegerScope(1, N_Robots), sIntegerScope(0, N_Vertices - 1), sIntegerScope(0, encoding_context.m_N_Layers - 1));
	sIndexableBitIdentifier vertex_occupancy_by_water(&encoding_context.m_variable_store, "vertex_occupancy_by_water", sIntegerScope(0, N_Vertices - 1), sIntegerScope(0, encoding_context.m_N_Layers - 1));

	
	encoding_context.m_vertex_occupancy_by_robot = vertex_occupancy_by_robot;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_robot);

	encoding_context.m_edge_occupancy_by_water.resize(N_Vertices);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    sIndexableBitIdentifier edge_occupancy_by_water(&encoding_context.m_variable_store, "edge_occupancy_by_water-" + sInt_32_to_String(vertex_id),
							    sIntegerScope(0, m_environment.get_Vertex(vertex_id)->calc_NeighborCount()),
							    sIntegerScope(0, encoding_context.m_N_Layers - 2));
	    encoding_context.m_edge_occupancy_by_water[vertex_id] = edge_occupancy_by_water;
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_source_Identifiers;

		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));

		    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
										      total_Literal_cnt,
										      sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										      sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											 total_Literal_cnt,
											 mutex_source_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector target_Identifiers;
		target_Identifiers.push_back(sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

		int out_neighbor_index = 1;
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    int neighbor_id = (*out_neighbor)->m_target->m_id;
		    target_Identifiers.push_back(sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)));

		    Clause_cnt += encoding_context.m_bit_generator->count_BiangleMutex(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)),
										       sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(neighbor_id), sIntegerIndex(layer)));
		    ++out_neighbor_index;
		}
		Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										       target_Identifiers);
	    }	    
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;

		Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
										  total_Literal_cnt,
										  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)),
										  sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										  sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));
		mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)));

		int out_neighbor_index = 1;
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
										      total_Literal_cnt,
										      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)),
										      sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										      sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex((*out_neighbor)->m_target->m_id), sIntegerIndex(layer + 1)));
		    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)));

		    ++out_neighbor_index;
		}
		Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										       mutex_target_Identifiers);
		Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											 total_Literal_cnt, 
											 mutex_target_Identifiers);

		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_source_Identifiers;

		const sVertex::Neighbors_list &in_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;
		mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)));

		for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		{
		    int in_neighbor_id = (*in_neighbor)->m_target->m_id;

		    int out_neighbor_index = 1;
		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(in_neighbor_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int out_neighbor_id = (*out_neighbor)->m_target->m_id;
			if (out_neighbor_id == vertex_id)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[in_neighbor_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)));
			}
			++out_neighbor_index;
		    }
		}
		Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)),
										       mutex_source_Identifiers);
		Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											 total_Literal_cnt,
											 mutex_source_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_Effect(aux_Variable_cnt,
										 total_Literal_cnt,
										 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)),
										 sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										 sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

		    int out_neighbor_index = 1;
		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_Effect(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)),
										     sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										     sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex((*out_neighbor)->m_target->m_id), sIntegerIndex(layer + 1)));
			
			++out_neighbor_index;
		    }
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    for (int robot_id = 1; robot_id <= N_Robots;  ++robot_id)
	    {
		if (robot_id == init_robot_id)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										 total_Literal_cnt,
										 sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(0)));
		}
		else
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(0)));
		}
	    }
	    if (init_robot_id > 0)
	    {
		Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
									     total_Literal_cnt,
									     sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(0)));
	    }
	    else
	    {
		Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
									       total_Literal_cnt,
									       sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(0)));
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);
		
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    if (robot_id == goal_robot_id)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		if (goal_robot_id > 0)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										 total_Literal_cnt,
										 sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		}
		else
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_bit_disj_Identifiers;

		    
		    for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
		    {
			goal_bit_disj_Identifiers.push_back(sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(*robot_id), sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    Clause_cnt += encoding_context.m_bit_generator->count_Disjunction(aux_Variable_cnt,
										      total_Literal_cnt,
										      goal_bit_disj_Identifiers);

		    if (robot_IDs.size() == 1)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		}

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    if (robot_IDs.find(robot_id) == robot_IDs.end())
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}

	int N_cnf_Variables = encoding_context.m_variable_store.get_Last_CNFVariable() + aux_Variable_cnt - 1;

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot flow SAT encoding\n", sPRODUCT);
	fprintf(fw, "c %s\n", sCOPYRIGHT);
	fprintf(fw, "c number of layers = %d\n", encoding_context.m_N_Layers);
	fprintf(fw, "c number of visible variables = %d\n", N_cnf_Variables - aux_Variable_cnt);
	fprintf(fw, "c number of hidden variables = %d\n", aux_Variable_cnt);
	fprintf(fw, "c number of literals = %d\n", total_Literal_cnt);
	fprintf(fw, "c number of clauses = %d\n", Clause_cnt);
	fprintf(fw, "c number of propositional variables = %d\n", N_cnf_Variables);
	fprintf(fw, "c number of clauses / number of variables = %.3f\n", (double)Clause_cnt / N_cnf_Variables);
	fprintf(fw, "c number of literals / number of clauses = %.3f\n", (double)total_Literal_cnt /  Clause_cnt);
	to_Stream(fw, "c ");
#endif
	fprintf(fw, "p cnf %d %d\n", N_cnf_Variables, Clause_cnt);

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_source_Identifiers;

		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));

		    encoding_context.m_bit_generator->generate_Implication(fw,
									   sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)),
									   sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		encoding_context.m_bit_generator->generate_AllMutexConstraint(fw, mutex_source_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector target_Identifiers;
		target_Identifiers.push_back(sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

		int out_neighbor_index = 1;
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    int neighbor_id = (*out_neighbor)->m_target->m_id;
		    target_Identifiers.push_back(sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)));

		    encoding_context.m_bit_generator->generate_BiangleMutex(fw,
									    sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)),
									    sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(neighbor_id), sIntegerIndex(layer)));
		    ++out_neighbor_index;
		}
		encoding_context.m_bit_generator->generate_MultiImplication(fw,
									    sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
									    target_Identifiers);
	    }	    
	}
       
	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;

		encoding_context.m_bit_generator->generate_Implication(fw,
								       sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)),
								       sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
								       sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));
		mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)));

		int out_neighbor_index = 1;
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    encoding_context.m_bit_generator->generate_Implication(fw,
									   sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)),
									   sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
									   sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex((*out_neighbor)->m_target->m_id), sIntegerIndex(layer + 1)));
		    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)));

		    ++out_neighbor_index;
		}
		encoding_context.m_bit_generator->generate_MultiImplication(fw,
									    sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
									    mutex_target_Identifiers);
		encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,  mutex_target_Identifiers);

		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_source_Identifiers;

		const sVertex::Neighbors_list &in_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;
		mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)));

		for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		{
		    int in_neighbor_id = (*in_neighbor)->m_target->m_id;

		    int out_neighbor_index = 1;
		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(in_neighbor_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int out_neighbor_id = (*out_neighbor)->m_target->m_id;
			if (out_neighbor_id == vertex_id)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[in_neighbor_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)));
			}
			++out_neighbor_index;
		    }
		}
		encoding_context.m_bit_generator->generate_MultiImplication(fw,
									    sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)),
									    mutex_source_Identifiers);

		encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,  mutex_source_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    encoding_context.m_bit_generator->generate_Effect(fw,
								      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)),
								      sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)),
								      sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

		    int out_neighbor_index = 1;
		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			encoding_context.m_bit_generator->generate_Effect(fw,
									  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)),
									  sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)),
									  sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex((*out_neighbor)->m_target->m_id), sIntegerIndex(layer + 1)));
			
			++out_neighbor_index;
		    }
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    for (int robot_id = 1; robot_id <= N_Robots;  ++robot_id)
	    {
		if (robot_id == init_robot_id)
		{
		    encoding_context.m_bit_generator->generate_BitSet(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(0)));
		}
		else
		{
		    encoding_context.m_bit_generator->generate_BitUnset(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(0)));
		}
	    }
	    if (init_robot_id > 0)
	    {
		encoding_context.m_bit_generator->generate_BitSet(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(0)));
	    }
	    else
	    {
		encoding_context.m_bit_generator->generate_BitUnset(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(0)));
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    if (robot_id == goal_robot_id)
		    {
			encoding_context.m_bit_generator->generate_BitSet(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    else
		    {
			encoding_context.m_bit_generator->generate_BitUnset(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		if (goal_robot_id > 0)
		{
		    encoding_context.m_bit_generator->generate_BitSet(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		}
		else
		{
		    encoding_context.m_bit_generator->generate_BitUnset(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_bit_disj_Identifiers;

		    for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
		    {
			goal_bit_disj_Identifiers.push_back(sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(*robot_id), sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    encoding_context.m_bit_generator->generate_Disjunction(fw, goal_bit_disj_Identifiers);

		    if (robot_IDs.size() == 1)
		    {
			encoding_context.m_bit_generator->generate_BitSet(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
		    encoding_context.m_bit_generator->generate_BitUnset(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		}
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    if (robot_IDs.find(robot_id) == robot_IDs.end())
		    {
			encoding_context.m_bit_generator->generate_BitUnset(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
    }


    void sMultirobotInstance::to_Stream_MatchingCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose)) const
    {
	encoding_context.switchTo_AdvancedGeneratingMode();

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableBitIdentifier vertex_occupancy_by_water(&encoding_context.m_variable_store, "vertex_occupancy_by_water", sIntegerScope(0, N_Vertices - 1), sIntegerScope(0, encoding_context.m_N_Layers - 1));
	sIndexableStateIdentifier vertex_occupancy(&encoding_context.m_variable_store, "vertex_occupancy", N_Robots + 1, sIntegerScope(0, N_Vertices), sIntegerScope(0, encoding_context.m_N_Layers - 1));

	encoding_context.m_vertex_occupancy = vertex_occupancy;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	    {
		Clause_cnt += encoding_context.m_clause_generator->count_Alignment(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
	    }
	}

	encoding_context.m_edge_occupancy_by_water.resize(N_Vertices);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    sIndexableBitIdentifier edge_occupancy_by_water(&encoding_context.m_variable_store, "edge_occupancy_by_water-" + sInt_32_to_String(vertex_id),
							    sIntegerScope(0, m_environment.get_Vertex(vertex_id)->calc_NeighborCount()),
							    sIntegerScope(0, encoding_context.m_N_Layers - 2));
	    encoding_context.m_edge_occupancy_by_water[vertex_id] = edge_occupancy_by_water;
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		Clause_cnt += encoding_context.m_bit_generator->count_NonzeroImplication(aux_Variable_cnt,
											 total_Literal_cnt,
											 sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
											 sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector target_Identifiers;
		target_Identifiers.push_back(sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

		int out_neighbor_index = 1;
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    int neighbor_id = (*out_neighbor)->m_target->m_id;
		    target_Identifiers.push_back(sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)));

		    Clause_cnt += encoding_context.m_bit_generator->count_BiangleMutex(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)),
										       sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(neighbor_id), sIntegerIndex(layer)));
		    ++out_neighbor_index;
		}
		/*
		Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										       target_Identifiers);
		*/
	    }	    
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;

		Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
										  total_Literal_cnt,
										  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)),
										  sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										  sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));
		mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)));

		int out_neighbor_index = 1;
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
										      total_Literal_cnt,
										      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)),
										      sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										      sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex((*out_neighbor)->m_target->m_id), sIntegerIndex(layer + 1)));
		    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)));

		    ++out_neighbor_index;
		}
		Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										       mutex_target_Identifiers);
		Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											 total_Literal_cnt,
											 mutex_target_Identifiers);

		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_source_Identifiers;

		const sVertex::Neighbors_list &in_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;
		mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)));

		for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		{
		    int in_neighbor_id = (*in_neighbor)->m_target->m_id;

		    int out_neighbor_index = 1;
		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(in_neighbor_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int out_neighbor_id = (*out_neighbor)->m_target->m_id;
			if (out_neighbor_id == vertex_id)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[in_neighbor_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)));
			}
			++out_neighbor_index;
		    }
		}
		Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)),
										       mutex_source_Identifiers);
		Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											 total_Literal_cnt,
											 mutex_source_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
											  total_Literal_cnt,
											  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)),
											  sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
											  sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

		int out_neighbor_index = 1;
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
											      total_Literal_cnt,
											      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)),
											      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
											      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex((*out_neighbor)->m_target->m_id), sIntegerIndex(layer + 1)));
		    ++out_neighbor_index;
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);
	    Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
									      total_Literal_cnt,
									      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(0)),
									      init_robot_id);

	    if (init_robot_id > 0)
	    {
		Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
									     total_Literal_cnt,
									     sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(0)));
	    }
	    else
	    {
		Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
									       total_Literal_cnt,
									       sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(0)));
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);
		Clause_cnt += encoding_context.m_bit_generator->count_Equality(aux_Variable_cnt,
									       total_Literal_cnt,
									       sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)),
									       goal_robot_id);

		if (goal_robot_id > 0)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										 total_Literal_cnt,
										 sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		}
		else
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    if (robot_IDs.size() > 1)
		    {
			sStateClauseGenerator::SpecifiedStateIdentifiers_vector goal_case_disj_Identifiers;
			sStateClauseGenerator::States_vector goal_case_disj_States;
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_bit_disj_Identifiers;
		    
			goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
			goal_case_disj_States.push_back(0);
			
			for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
			{
			    goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
			    goal_case_disj_States.push_back(*robot_id);
			}
			Clause_cnt += encoding_context.m_clause_generator->count_DisjunctiveEquality(aux_Variable_cnt,
												     total_Literal_cnt,
												     goal_case_disj_Identifiers,
												     goal_case_disj_States);
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
											  total_Literal_cnt,
											  sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)),
											  *robot_IDs.begin());
			Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
		    Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
										      total_Literal_cnt,
										      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)),
										      sRobotArrangement::VACANT_VERTEX);

		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}

	int N_cnf_Variables = encoding_context.m_variable_store.get_Last_CNFVariable() + aux_Variable_cnt - 1;

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot matching SAT encoding\n", sPRODUCT);
	fprintf(fw, "c %s\n", sCOPYRIGHT);
	fprintf(fw, "c number of layers = %d\n", encoding_context.m_N_Layers);
	fprintf(fw, "c number of visible variables = %d\n", N_cnf_Variables - aux_Variable_cnt);
	fprintf(fw, "c number of hidden variables = %d\n", aux_Variable_cnt);
	fprintf(fw, "c number of literals = %d\n", total_Literal_cnt);
	fprintf(fw, "c number of clauses = %d\n", Clause_cnt);
	fprintf(fw, "c number of propositional variables = %d\n", N_cnf_Variables);
	fprintf(fw, "c number of clauses / number of variables = %.3f\n", (double)Clause_cnt / N_cnf_Variables);
	fprintf(fw, "c number of literals / number of clauses = %.3f\n", (double)total_Literal_cnt /  Clause_cnt);
	to_Stream(fw, "c ");
#endif
	fprintf(fw, "p cnf %d %d\n", N_cnf_Variables, Clause_cnt);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	    {
//		fprintf(fw, "c label alignment layer=%d vertex=%d\n", layer, vertex_id);
		encoding_context.m_clause_generator->generate_Alignment(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
//		fprintf(fw, "c label nonzero layer=%d vertex=%d\n", layer, vertex_id);
		encoding_context.m_bit_generator->generate_NonzeroImplication(fw,
									      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
									      sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector target_Identifiers;
		target_Identifiers.push_back(sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

		int out_neighbor_index = 1;
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    int neighbor_id = (*out_neighbor)->m_target->m_id;
		    target_Identifiers.push_back(sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)));

//		    fprintf(fw, "c label biangle_mutex layer=%d vertex=%d\n", layer, vertex_id);
		    encoding_context.m_bit_generator->generate_BiangleMutex(fw,
									    sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)),
									    sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(neighbor_id), sIntegerIndex(layer)));
		    ++out_neighbor_index;
		}
		/*
		fprintf(fw, "c label target_implication layer=%d vertex=%d\n", layer, vertex_id);
		encoding_context.m_bit_generator->generate_MultiImplication(fw,
									    sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
									    target_Identifiers);
		*/
	    }	    
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;

//		fprintf(fw, "c label water_implication layer=%d vertex=%d\n", layer, vertex_id);
		encoding_context.m_bit_generator->generate_Implication(fw,
								       sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)),
								       sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
								       sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));
		mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)));

		int out_neighbor_index = 1;
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
//		    fprintf(fw, "c label water_target_implication layer=%d vertex=%d\n", layer, vertex_id);
		    encoding_context.m_bit_generator->generate_Implication(fw,
									   sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)),
									   sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
									   sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex((*out_neighbor)->m_target->m_id), sIntegerIndex(layer + 1)));
		    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)));

		    ++out_neighbor_index;
		}

//		fprintf(fw, "c label water_target_implication layer=%d vertex=%d\n", layer, vertex_id);
		encoding_context.m_bit_generator->generate_MultiImplication(fw,
									    sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
									    mutex_target_Identifiers);

//		fprintf(fw, "c label mutex_target layer=%d vertex=%d\n", layer, vertex_id);
		encoding_context.m_bit_generator->generate_AllMutexConstraint(fw, mutex_target_Identifiers);

		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_source_Identifiers;

		const sVertex::Neighbors_list &in_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;
		mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)));

		for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		{
		    int in_neighbor_id = (*in_neighbor)->m_target->m_id;

		    int out_neighbor_index = 1;
		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(in_neighbor_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int out_neighbor_id = (*out_neighbor)->m_target->m_id;
			if (out_neighbor_id == vertex_id)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[in_neighbor_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)));
			}
			++out_neighbor_index;
		    }
		}
		
//		fprintf(fw, "c label water_source_implication layer=%d vertex=%d\n", layer, vertex_id);
		encoding_context.m_bit_generator->generate_MultiImplication(fw,
									    sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)),
									    mutex_source_Identifiers);
//		fprintf(fw, "c label mutex_source layer=%d vertex=%d\n", layer, vertex_id);
		encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,  mutex_source_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
//		fprintf(fw, "c label occupancy_water_implication_1 layer=%d vertex=%d\n", layer, vertex_id);
		encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
									       sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)),
									       sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
									       sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

		int out_neighbor_index = 1;
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
//		    fprintf(fw, "c label occupancy_water_implication_2 layer=%d vertex=%d\n", layer, vertex_id);
		    encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
										   sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)),
										   sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										   sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex((*out_neighbor)->m_target->m_id), sIntegerIndex(layer + 1)));
		    
		    ++out_neighbor_index;
		}
	    }
	}
	
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);
//	    fprintf(fw, "c label init_occupancy vertex=%d\n", vertex_id);
	    encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(0)), init_robot_id);

	    if (init_robot_id > 0)
	    {
//		fprintf(fw, "c label init_water_set vertex=%d\n", vertex_id);
		encoding_context.m_bit_generator->generate_BitSet(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(0)));
	    }
	    else
	    {
//		fprintf(fw, "c label init_water_unset vertex=%d\n", vertex_id);
		encoding_context.m_bit_generator->generate_BitUnset(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(0)));
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);
//		fprintf(fw, "c label goal_occupancy vertex=%d\n", vertex_id);
		encoding_context.m_bit_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), goal_robot_id);

		if (goal_robot_id > 0)
		{
//		    fprintf(fw, "c label goal_water_set vertex=%d\n", vertex_id);
		    encoding_context.m_bit_generator->generate_BitSet(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		}
		else
		{
//		    fprintf(fw, "c label goal_water_unset vertex=%d\n", vertex_id);
		    encoding_context.m_bit_generator->generate_BitUnset(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    if (robot_IDs.size() > 1)
		    {
			sStateClauseGenerator::SpecifiedStateIdentifiers_vector goal_case_disj_Identifiers;
			sStateClauseGenerator::States_vector goal_case_disj_States;
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_bit_disj_Identifiers;

			goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
			goal_case_disj_States.push_back(0);
			
			for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
			{
			    goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
			    goal_case_disj_States.push_back(*robot_id);
			}
//			fprintf(fw, "c label goal_occupancy_disjunction vertex=%d\n", vertex_id);
			encoding_context.m_clause_generator->generate_DisjunctiveEquality(fw, goal_case_disj_Identifiers, goal_case_disj_States);
		    }
		    else
		    {
//			fprintf(fw, "c label goal_occupancy vertex=%d\n", vertex_id);
			encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), *robot_IDs.begin());
//			fprintf(fw, "c label goal_water_set vertex=%d\n", vertex_id);
			encoding_context.m_bit_generator->generate_BitSet(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
//		    fprintf(fw, "c label goal_occupancy vertex=%d\n", vertex_id);
		    encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), sRobotArrangement::VACANT_VERTEX);

//		    fprintf(fw, "c label goal_water_unset vertex=%d\n", vertex_id);
		    encoding_context.m_bit_generator->generate_BitUnset(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
    }



    void sMultirobotInstance::to_Stream_HeuristicMatchingCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();

	m_environment.calc_AllPairsShortestPaths();
	const sUndirectedGraph::Distances_2d_vector &all_pairs_Distances = m_environment.get_AllPairsShortestPaths();

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableBitIdentifier vertex_occupancy_by_water(&encoding_context.m_variable_store, "vertex_occupancy_by_water", sIntegerScope(0, N_Vertices - 1), sIntegerScope(0, encoding_context.m_N_Layers - 1));
	sIndexableStateIdentifier vertex_occupancy(&encoding_context.m_variable_store, "vertex_occupancy", N_Robots + 1, sIntegerScope(0, N_Vertices), sIntegerScope(0, encoding_context.m_N_Layers - 1));

	encoding_context.m_vertex_occupancy = vertex_occupancy;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	    {
		/*
		Clause_cnt += encoding_context.m_clause_generator->count_Alignment(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		*/
	    }
	}

	encoding_context.m_edge_occupancy_by_water.resize(N_Vertices);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    sIndexableBitIdentifier edge_occupancy_by_water(&encoding_context.m_variable_store, "edge_occupancy_by_water-" + sInt_32_to_String(vertex_id),
							    sIntegerScope(0, m_environment.get_Vertex(vertex_id)->calc_NeighborCount()),
							    sIntegerScope(0, encoding_context.m_N_Layers - 2));
	    encoding_context.m_edge_occupancy_by_water[vertex_id] = edge_occupancy_by_water;
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		Clause_cnt += encoding_context.m_bit_generator->count_NonzeroImplication(aux_Variable_cnt,
											 total_Literal_cnt,
											 sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
											 sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
/*
		Clause_cnt += encoding_context.m_bit_generator->count_ZeroImplication(aux_Variable_cnt, total_Literal_cnt,
										      sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
*/
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector target_Identifiers;
		target_Identifiers.push_back(sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

		int out_neighbor_index = 1;
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    int neighbor_id = (*out_neighbor)->m_target->m_id;
		    target_Identifiers.push_back(sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)));

		    Clause_cnt += encoding_context.m_bit_generator->count_BiangleMutex(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)),
										       sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(neighbor_id), sIntegerIndex(layer)));
		    ++out_neighbor_index;
		}
		/*
		Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										       target_Identifiers);
		*/
	    }	    
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;

		Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
										  total_Literal_cnt,
										  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)),
										  sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										  sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));
		mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)));

		int out_neighbor_index = 1;
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
										      total_Literal_cnt,
										      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)),
										      sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										      sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex((*out_neighbor)->m_target->m_id), sIntegerIndex(layer + 1)));
		    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)));

		    ++out_neighbor_index;
		}
		Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										       mutex_target_Identifiers);
		Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											 total_Literal_cnt, 
											 mutex_target_Identifiers);

		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_source_Identifiers;

		const sVertex::Neighbors_list &in_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;
		mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)));

		for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		{
		    int in_neighbor_id = (*in_neighbor)->m_target->m_id;

		    int out_neighbor_index = 1;
		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(in_neighbor_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int out_neighbor_id = (*out_neighbor)->m_target->m_id;
			if (out_neighbor_id == vertex_id)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[in_neighbor_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)));
			}
			++out_neighbor_index;
		    }
		}
		Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)),
										       mutex_source_Identifiers);
		Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											 total_Literal_cnt,
											 mutex_source_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
											  total_Literal_cnt,
											  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)),
											  sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
											  sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

		int out_neighbor_index = 1;
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
											      total_Literal_cnt,
											      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)),
											      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
											      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex((*out_neighbor)->m_target->m_id), sIntegerIndex(layer + 1)));
		    ++out_neighbor_index;
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);
	    Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
									      total_Literal_cnt,
									      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(0)), init_robot_id);

	    if (init_robot_id > 0)
	    {
		Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
									     total_Literal_cnt,
									     sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(0)));
	    }
	    else
	    {
		Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
									       total_Literal_cnt,
									       sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(0)));
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);
		Clause_cnt += encoding_context.m_bit_generator->count_Equality(aux_Variable_cnt,
									       total_Literal_cnt,
									       sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)),
									       goal_robot_id);

		if (goal_robot_id > 0)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										 total_Literal_cnt,
										 sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		}
		else
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    if (robot_IDs.size() > 1)
		    {
			sStateClauseGenerator::SpecifiedStateIdentifiers_vector goal_case_disj_Identifiers;
			sStateClauseGenerator::States_vector goal_case_disj_States;
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_bit_disj_Identifiers;
			
			goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
			goal_case_disj_States.push_back(0);
			
			for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
			{
			    goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
			    goal_case_disj_States.push_back(*robot_id);
			}
			Clause_cnt += encoding_context.m_clause_generator->count_DisjunctiveEquality(aux_Variable_cnt,
												     total_Literal_cnt,
												     goal_case_disj_Identifiers,
												     goal_case_disj_States);
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
											  total_Literal_cnt,
											  sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)),
											  *robot_IDs.begin());
			Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
		    Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
										      total_Literal_cnt,
										      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)),
										      sRobotArrangement::VACANT_VERTEX);

		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	    {		    
		sStateClauseGenerator::States_vector robot_IDs;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    bool forbid_vertex = true;

		    switch (m_goal_type)
		    {
		    case GOAL_TYPE_ARRANGEMENT:
		    {
			if (all_pairs_Distances[vertex_id][m_goal_arrangement.get_RobotLocation(robot_id)] <= encoding_context.m_N_Layers - layer)
			{
			    forbid_vertex = false;
			}
			break;
		    }
		    case GOAL_TYPE_SPECIFICATION:
		    {
			const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
			for (sRobotGoal::Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
			{
			    if (all_pairs_Distances[vertex_id][*goal_id] <= encoding_context.m_N_Layers - layer)
			    {
				forbid_vertex = false;
				break;
			    }
			}
			break;
		    }
		    default:
		    {
			sASSERT(false);
			break;
		    }
		    }
		    if (forbid_vertex || all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] > layer)
		    {
			robot_IDs.push_back(robot_id);
		    }
		}
		Clause_cnt += encoding_context.m_clause_generator->count_Disequalities(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										       robot_IDs);
	    }
	}


	int N_cnf_Variables = encoding_context.m_variable_store.get_Last_CNFVariable() + aux_Variable_cnt - 1;

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot heuristic matching SAT encoding\n", sPRODUCT);
	fprintf(fw, "c %s\n", sCOPYRIGHT);
	fprintf(fw, "c number of layers = %d\n", encoding_context.m_N_Layers);
	fprintf(fw, "c number of visible variables = %d\n", N_cnf_Variables - aux_Variable_cnt);
	fprintf(fw, "c number of hidden variables = %d\n", aux_Variable_cnt);
	fprintf(fw, "c number of literals = %d\n", total_Literal_cnt);
	fprintf(fw, "c number of clauses = %d\n", Clause_cnt);
	fprintf(fw, "c number of propositional variables = %d\n", N_cnf_Variables);
	fprintf(fw, "c number of clauses / number of variables = %.3f\n", (double)Clause_cnt / N_cnf_Variables);
	fprintf(fw, "c number of literals / number of clauses = %.3f\n", (double)total_Literal_cnt /  Clause_cnt);
	to_Stream(fw, "c ");
#endif
	fprintf(fw, "p cnf %d %d\n", N_cnf_Variables, Clause_cnt);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	    {
		/*
		encoding_context.m_clause_generator->generate_Alignment(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		*/
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		encoding_context.m_bit_generator->generate_NonzeroImplication(fw,
									      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
									      sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)));

/*
		encoding_context.m_bit_generator->generate_ZeroImplication(fw,
									   sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
									   sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
*/
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector target_Identifiers;
		target_Identifiers.push_back(sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

		int out_neighbor_index = 1;
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    int neighbor_id = (*out_neighbor)->m_target->m_id;
		    target_Identifiers.push_back(sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)));

		    encoding_context.m_bit_generator->generate_BiangleMutex(fw,
									    sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)),
									    sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(neighbor_id), sIntegerIndex(layer)));
		    ++out_neighbor_index;
		}
		/*
		encoding_context.m_bit_generator->generate_MultiImplication(fw,
									    sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
									    target_Identifiers);
		*/
	    }	    
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;

		encoding_context.m_bit_generator->generate_Implication(fw,
								       sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)),
								       sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
								       sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));
		mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)));

		int out_neighbor_index = 1;
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    encoding_context.m_bit_generator->generate_Implication(fw,
									   sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)),
									   sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
									   sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex((*out_neighbor)->m_target->m_id), sIntegerIndex(layer + 1)));
		    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)));

		    ++out_neighbor_index;
		}
		encoding_context.m_bit_generator->generate_MultiImplication(fw,
									    sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
									    mutex_target_Identifiers);

		encoding_context.m_bit_generator->generate_AllMutexConstraint(fw, mutex_target_Identifiers);

		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_source_Identifiers;

		const sVertex::Neighbors_list &in_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;
		mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)));

		for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		{
		    int in_neighbor_id = (*in_neighbor)->m_target->m_id;

		    int out_neighbor_index = 1;
		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(in_neighbor_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int out_neighbor_id = (*out_neighbor)->m_target->m_id;
			if (out_neighbor_id == vertex_id)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[in_neighbor_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)));
			}
			++out_neighbor_index;
		    }
		}
		encoding_context.m_bit_generator->generate_MultiImplication(fw,
									    sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)),
									    mutex_source_Identifiers);

		encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,  mutex_source_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
									       sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)),
									       sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
									       sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

		int out_neighbor_index = 1;
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
										   sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)),
										   sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										   sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex((*out_neighbor)->m_target->m_id), sIntegerIndex(layer + 1)));
		    
		    ++out_neighbor_index;
		}
	    }
	}
	
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);
	    encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(0)), init_robot_id);

	    if (init_robot_id > 0)
	    {
		encoding_context.m_bit_generator->generate_BitSet(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(0)));
	    }
	    else
	    {
		encoding_context.m_bit_generator->generate_BitUnset(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(0)));
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);
		encoding_context.m_bit_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), goal_robot_id);

		if (goal_robot_id > 0)
		{
		    encoding_context.m_bit_generator->generate_BitSet(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		}
		else
		{
		    encoding_context.m_bit_generator->generate_BitUnset(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    if (robot_IDs.size() > 1)
		    {
			sStateClauseGenerator::SpecifiedStateIdentifiers_vector goal_case_disj_Identifiers;
			sStateClauseGenerator::States_vector goal_case_disj_States;
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_bit_disj_Identifiers;
			
			goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
			goal_case_disj_States.push_back(0);
			
			for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
			{
			    goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
			    goal_case_disj_States.push_back(*robot_id);
			}
			encoding_context.m_clause_generator->generate_DisjunctiveEquality(fw, goal_case_disj_Identifiers, goal_case_disj_States);
		    }
		    else
		    {
			encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), *robot_IDs.begin());
			encoding_context.m_bit_generator->generate_BitSet(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
		    encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), sRobotArrangement::VACANT_VERTEX);
		    encoding_context.m_bit_generator->generate_BitUnset(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	    {		    
		sStateClauseGenerator::States_vector robot_IDs;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    bool forbid_vertex = true;

		    switch (m_goal_type)
		    {
		    case GOAL_TYPE_ARRANGEMENT:
		    {
			if (all_pairs_Distances[vertex_id][m_goal_arrangement.get_RobotLocation(robot_id)] <= encoding_context.m_N_Layers - layer)
			{
			    forbid_vertex = false;
			}
			break;
		    }
		    case GOAL_TYPE_SPECIFICATION:
		    {
			const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
			for (sRobotGoal::Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
			{
			    if (all_pairs_Distances[vertex_id][*goal_id] <= encoding_context.m_N_Layers - layer)
			    {
				forbid_vertex = false;
				break;
			    }
			}
			break;
		    }
		    default:
		    {
			sASSERT(false);
			break;
		    }
		    }
		    if (forbid_vertex || all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] > layer)
		    {			
			robot_IDs.push_back(robot_id);
		    }
		}
		encoding_context.m_clause_generator->generate_Disequalities(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)), robot_IDs);
	    }
	}
    }


    void sMultirobotInstance::to_Stream_DirectCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose)) const
    {
	encoding_context.switchTo_AdvancedGeneratingMode();

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableBitIdentifier robot_location_in_vertex(&encoding_context.m_variable_store, "robot_location_in_vertex", sIntegerScope(1, N_Robots), sIntegerScope(0, N_Vertices - 1), sIntegerScope(0, encoding_context.m_N_Layers - 1));

	encoding_context.m_robot_location_in_vertex = robot_location_in_vertex;
	encoding_context.register_TranslateIdentifier(encoding_context.m_robot_location_in_vertex);

	// robot is placed in exactly one vertex
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector robot_Identifiers;

		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											 total_Literal_cnt,
											 robot_Identifiers);
		Clause_cnt += encoding_context.m_bit_generator->count_Disjunction(aux_Variable_cnt,
										  total_Literal_cnt,
										  robot_Identifiers);
	    }
	}

	// there is at most one robot in a vertex
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector robot_mutex_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    robot_mutex_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											 total_Literal_cnt,
											 robot_mutex_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector target_robot_Identifiers;
		    target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int neighbor_id = (*out_neighbor)->m_target->m_id;
			target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)));
		    }
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&robot_location_in_vertex,
														   sIntegerIndex(robot_id),
														   sIntegerIndex(vertex_id),
														   sIntegerIndex(layer)),
											   target_robot_Identifiers);
		}
	    }	    
	}

	for (int layer = 1; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector source_robot_Identifiers;
		    source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer - 1)));

		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int neighbor_id = (*out_neighbor)->m_target->m_id;
			source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer - 1)));
		    }
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&robot_location_in_vertex,
														   sIntegerIndex(robot_id),
														   sIntegerIndex(vertex_id),
														   sIntegerIndex(layer)),
											   source_robot_Identifiers);
		}
	    }	    
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int neighbor_id = (*out_neighbor)->m_target->m_id;

			sBitClauseGenerator::SpecifiedBitIdentifiers_vector unset_robots_Identifiers;
			for (int unset_robot_id = 1; unset_robot_id <= N_Robots; ++unset_robot_id)
			{
			    unset_robots_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
										       sIntegerIndex(unset_robot_id),
										       sIntegerIndex(vertex_id),
										       sIntegerIndex(layer + 1)));
			    unset_robots_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
										       sIntegerIndex(unset_robot_id),
										       sIntegerIndex(neighbor_id),
										       sIntegerIndex(layer)));
			}

			Clause_cnt += encoding_context.m_bit_generator->count_SwapConstraint(aux_Variable_cnt,
											     total_Literal_cnt,
											     sSpecifiedBitIdentifier(&robot_location_in_vertex,
														     sIntegerIndex(robot_id),
														     sIntegerIndex(vertex_id),
														     sIntegerIndex(layer)),
											     sSpecifiedBitIdentifier(&robot_location_in_vertex,
														     sIntegerIndex(robot_id),
														     sIntegerIndex(neighbor_id),
														     sIntegerIndex(layer + 1)),
											     unset_robots_Identifiers);
		    }
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id == 0)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&robot_location_in_vertex,
													   sIntegerIndex(robot_id),
													   sIntegerIndex(vertex_id),
													   sIntegerIndex(0)));
		}
	    }
	    else
	    {
		for (int robot_id = 1; robot_id < init_robot_id; ++robot_id)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&robot_location_in_vertex,
													   sIntegerIndex(robot_id),
													   sIntegerIndex(vertex_id),
													   sIntegerIndex(0)));
		}
		Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
									     total_Literal_cnt,
									     sSpecifiedBitIdentifier(&robot_location_in_vertex,
												     sIntegerIndex(init_robot_id),
												     sIntegerIndex(vertex_id),
												     sIntegerIndex(0)));

		for (int robot_id = init_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&robot_location_in_vertex,
													   sIntegerIndex(robot_id),
													   sIntegerIndex(vertex_id),
													   sIntegerIndex(0)));
		}
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id == 0)
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id < goal_robot_id; ++robot_id)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										 total_Literal_cnt,
										 sSpecifiedBitIdentifier(&robot_location_in_vertex,
													 sIntegerIndex(goal_robot_id),
													 sIntegerIndex(vertex_id),
													 sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    
		    for (int robot_id = goal_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_robot_Identifiers;

		    for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
		    {
			goal_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
										 sIntegerIndex(*robot_id),
										 sIntegerIndex(vertex_id),
										 sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    Clause_cnt += encoding_context.m_bit_generator->count_Disjunction(aux_Variable_cnt,
										      total_Literal_cnt,
										      goal_robot_Identifiers);

		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			if (robot_IDs.find(robot_id) == robot_IDs.end())
			{
			    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&robot_location_in_vertex,
														   sIntegerIndex(robot_id),
														   sIntegerIndex(vertex_id),
														   sIntegerIndex(encoding_context.m_N_Layers - 1)));			    
			}
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}

	int N_cnf_Variables = encoding_context.m_variable_store.get_Last_CNFVariable() + aux_Variable_cnt - 1;

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot direct SAT encoding\n", sPRODUCT);
	fprintf(fw, "c %s\n", sCOPYRIGHT);
	fprintf(fw, "c number of layers = %d\n", encoding_context.m_N_Layers);
	fprintf(fw, "c number of visible variables = %d\n", N_cnf_Variables - aux_Variable_cnt);
	fprintf(fw, "c number of hidden variables = %d\n", aux_Variable_cnt);
	fprintf(fw, "c number of literals = %d\n", total_Literal_cnt);
	fprintf(fw, "c number of clauses = %d\n", Clause_cnt);
	fprintf(fw, "c number of propositional variables = %d\n", N_cnf_Variables);
	fprintf(fw, "c number of clauses / number of variables = %.3f\n", (double)Clause_cnt / N_cnf_Variables);
	fprintf(fw, "c number of literals / number of clauses = %.3f\n", (double)total_Literal_cnt /  Clause_cnt);
	to_Stream(fw, "c ");
#endif
	fprintf(fw, "p cnf %d %d\n", N_cnf_Variables, Clause_cnt);

	// robot is placed in exactly one vertex
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector robot_Identifiers;

		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		encoding_context.m_bit_generator->generate_AllMutexConstraint(fw, robot_Identifiers);
		encoding_context.m_bit_generator->generate_Disjunction(fw, robot_Identifiers);
	    }
	}

	// there is at most one robot in a vertex
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector robot_mutex_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    robot_mutex_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		encoding_context.m_bit_generator->generate_AllMutexConstraint(fw, robot_mutex_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector target_robot_Identifiers;
		    target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int neighbor_id = (*out_neighbor)->m_target->m_id;
			target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)));
		    }
		    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
											      sSpecifiedBitIdentifier(&robot_location_in_vertex,
														      sIntegerIndex(robot_id),
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      target_robot_Identifiers);
		}
	    }	    
	}

	for (int layer = 1; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector source_robot_Identifiers;
		    source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer - 1)));

		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int neighbor_id = (*out_neighbor)->m_target->m_id;
			source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer - 1)));
		    }
		    encoding_context.m_bit_generator->generate_MultiImplication(fw,
										sSpecifiedBitIdentifier(&robot_location_in_vertex,
													sIntegerIndex(robot_id),
													sIntegerIndex(vertex_id),
													sIntegerIndex(layer)),
										source_robot_Identifiers);
		}
	    }	    
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int neighbor_id = (*out_neighbor)->m_target->m_id;

			sBitClauseGenerator::SpecifiedBitIdentifiers_vector unset_robots_Identifiers;
			for (int unset_robot_id = 1; unset_robot_id <= N_Robots; ++unset_robot_id)
			{
			    unset_robots_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
										       sIntegerIndex(unset_robot_id),
										       sIntegerIndex(vertex_id),
										       sIntegerIndex(layer + 1)));
			    unset_robots_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
										       sIntegerIndex(unset_robot_id),
										       sIntegerIndex(neighbor_id),
										       sIntegerIndex(layer)));
			}

			encoding_context.m_bit_generator->generate_SwapConstraint(fw,
										  sSpecifiedBitIdentifier(&robot_location_in_vertex,
													  sIntegerIndex(robot_id),
													  sIntegerIndex(vertex_id),
													  sIntegerIndex(layer)),
										  sSpecifiedBitIdentifier(&robot_location_in_vertex,
													  sIntegerIndex(robot_id),
													  sIntegerIndex(neighbor_id),
													  sIntegerIndex(layer + 1)),
										  unset_robots_Identifiers);
		    }
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id == 0)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    encoding_context.m_bit_generator->generate_BitUnset(fw,
									sSpecifiedBitIdentifier(&robot_location_in_vertex,
												sIntegerIndex(robot_id),
												sIntegerIndex(vertex_id),
												sIntegerIndex(0)));
		}
	    }
	    else
	    {
		for (int robot_id = 1; robot_id < init_robot_id; ++robot_id)
		{
		    encoding_context.m_bit_generator->generate_BitUnset(fw,
									sSpecifiedBitIdentifier(&robot_location_in_vertex,
												sIntegerIndex(robot_id),
												sIntegerIndex(vertex_id),
												sIntegerIndex(0)));
		}
		encoding_context.m_bit_generator->generate_BitSet(fw,
								  sSpecifiedBitIdentifier(&robot_location_in_vertex,
											  sIntegerIndex(init_robot_id),
											  sIntegerIndex(vertex_id),
											  sIntegerIndex(0)));
		
		for (int robot_id = init_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		{
		    encoding_context.m_bit_generator->generate_BitUnset(fw,
									sSpecifiedBitIdentifier(&robot_location_in_vertex,
												sIntegerIndex(robot_id),
												sIntegerIndex(vertex_id),
												sIntegerIndex(0)));
		}
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id == 0)
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id < goal_robot_id; ++robot_id)
		    {
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    encoding_context.m_bit_generator->generate_BitSet(fw,
								      sSpecifiedBitIdentifier(&robot_location_in_vertex,
											      sIntegerIndex(goal_robot_id),
											      sIntegerIndex(vertex_id),
											      sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    
		    for (int robot_id = goal_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		    {
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_robot_Identifiers;

		    for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
		    {
			goal_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
										 sIntegerIndex(*robot_id),
										 sIntegerIndex(vertex_id),
										 sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    encoding_context.m_bit_generator->generate_Disjunction(fw, goal_robot_Identifiers);

		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			if (robot_IDs.find(robot_id) == robot_IDs.end())
			{
			    encoding_context.m_bit_generator->generate_BitUnset(fw,
										sSpecifiedBitIdentifier(&robot_location_in_vertex,
													sIntegerIndex(robot_id),
													sIntegerIndex(vertex_id),
													sIntegerIndex(encoding_context.m_N_Layers - 1)));			    
			}
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
    }


    void sMultirobotInstance::to_Stream_HeuristicDirectCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();

	m_environment.calc_AllPairsShortestPaths();
	const sUndirectedGraph::Distances_2d_vector &all_pairs_Distances = m_environment.get_AllPairsShortestPaths();

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableBitIdentifier robot_location_in_vertex(&encoding_context.m_variable_store, "robot_location_in_vertex", sIntegerScope(1, N_Robots), sIntegerScope(0, N_Vertices - 1), sIntegerScope(0, encoding_context.m_N_Layers - 1));

	encoding_context.m_robot_location_in_vertex = robot_location_in_vertex;
	encoding_context.register_TranslateIdentifier(encoding_context.m_robot_location_in_vertex);

	// robot is placed in exactly one vertex
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector robot_Identifiers;

		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											 total_Literal_cnt,
											 robot_Identifiers);
		Clause_cnt += encoding_context.m_bit_generator->count_Disjunction(aux_Variable_cnt,
										  total_Literal_cnt,
										  robot_Identifiers);
	    }
	}

	// there is at most one robot in a vertex
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector robot_mutex_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    bool forbid_vertex = true;
		    
		    switch (m_goal_type)
		    {
		    case GOAL_TYPE_ARRANGEMENT:
		    {
			if (all_pairs_Distances[vertex_id][m_goal_arrangement.get_RobotLocation(robot_id)] <= encoding_context.m_N_Layers - layer)
			{
			    forbid_vertex = false;
			}
			break;
		    }
		    case GOAL_TYPE_SPECIFICATION:
		    {
			const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
			for (sRobotGoal::Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
			{
			    if (all_pairs_Distances[vertex_id][*goal_id] <= encoding_context.m_N_Layers - layer)
			    {
				forbid_vertex = false;
				break;
			    }
			}
			break;
		    }
		    default:
		    {
			sASSERT(false);
			break;
		    }
		    }
		    if (forbid_vertex || all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] > layer)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(layer)));
		    }

		    robot_mutex_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											 total_Literal_cnt,
											 robot_mutex_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    bool forbid_vertex = true;
		    
		    switch (m_goal_type)
		    {
		    case GOAL_TYPE_ARRANGEMENT:
		    {
			if (all_pairs_Distances[vertex_id][m_goal_arrangement.get_RobotLocation(robot_id)] <= encoding_context.m_N_Layers - layer)
			{
			    forbid_vertex = false;
			}
			break;
		    }
		    case GOAL_TYPE_SPECIFICATION:
		    {
			const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
			for (sRobotGoal::Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
			{
			    if (all_pairs_Distances[vertex_id][*goal_id] <= encoding_context.m_N_Layers - layer)
			    {
				forbid_vertex = false;
				break;
			    }
			}
			break;
		    }
		    default:
		    {
			sASSERT(false);
			break;
		    }
		    }
		    if (!forbid_vertex && all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] <= layer)
		    {
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector target_robot_Identifiers;
			target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;
			
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    int neighbor_id = (*out_neighbor)->m_target->m_id;
			    target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)));
			}
			Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&robot_location_in_vertex,
														       sIntegerIndex(robot_id),
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(layer)),
											       target_robot_Identifiers);
		    }
		}
	    }	    
	}

	for (int layer = 1; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    bool forbid_vertex = true;
		    
		    switch (m_goal_type)
		    {
		    case GOAL_TYPE_ARRANGEMENT:
		    {
			if (all_pairs_Distances[vertex_id][m_goal_arrangement.get_RobotLocation(robot_id)] <= encoding_context.m_N_Layers - layer)
			{
			    forbid_vertex = false;
			}
			break;
		    }
		    case GOAL_TYPE_SPECIFICATION:
		    {
			const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
			for (sRobotGoal::Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
			{
			    if (all_pairs_Distances[vertex_id][*goal_id] <= encoding_context.m_N_Layers - layer)
			    {
				forbid_vertex = false;
				break;
			    }
			}
			break;
		    }
		    default:
		    {
			sASSERT(false);
			break;
		    }
		    }
		    if (!forbid_vertex && all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] <= layer)
		    {
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector source_robot_Identifiers;
			source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer - 1)));
		    
			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    int neighbor_id = (*out_neighbor)->m_target->m_id;
			    source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer - 1)));
			}
			Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&robot_location_in_vertex,
														       sIntegerIndex(robot_id),
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(layer)),
											       source_robot_Identifiers);
		    }
		}
	    }	    
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    bool forbid_vertex = true;
		    
		    switch (m_goal_type)
		    {
		    case GOAL_TYPE_ARRANGEMENT:
		    {
			if (all_pairs_Distances[vertex_id][m_goal_arrangement.get_RobotLocation(robot_id)] <= encoding_context.m_N_Layers - layer)
			{
			    forbid_vertex = false;
			}
			break;
		    }
		    case GOAL_TYPE_SPECIFICATION:
		    {
			const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
			for (sRobotGoal::Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
			{
			    if (all_pairs_Distances[vertex_id][*goal_id] <= encoding_context.m_N_Layers - layer)
			    {
				forbid_vertex = false;
				break;
			    }
			}
			break;
		    }
		    default:
		    {
			sASSERT(false);
			break;
		    }
		    }
		    if (!forbid_vertex && all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] <= layer)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    int neighbor_id = (*out_neighbor)->m_target->m_id;
			
			    sBitClauseGenerator::SpecifiedBitIdentifiers_vector unset_robots_Identifiers;
			    for (int unset_robot_id = 1; unset_robot_id <= N_Robots; ++unset_robot_id)
			    {
				unset_robots_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
											   sIntegerIndex(unset_robot_id),
											   sIntegerIndex(vertex_id),
											   sIntegerIndex(layer + 1)));
				unset_robots_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
											   sIntegerIndex(unset_robot_id),
											   sIntegerIndex(neighbor_id),
											   sIntegerIndex(layer)));
			    }

			    Clause_cnt += encoding_context.m_bit_generator->count_SwapConstraint(aux_Variable_cnt,
												 total_Literal_cnt,
												 sSpecifiedBitIdentifier(&robot_location_in_vertex,
															 sIntegerIndex(robot_id),
															 sIntegerIndex(vertex_id),
															 sIntegerIndex(layer)),
												 sSpecifiedBitIdentifier(&robot_location_in_vertex,
															 sIntegerIndex(robot_id),
															 sIntegerIndex(neighbor_id),
															 sIntegerIndex(layer + 1)),
												 unset_robots_Identifiers);
			}
		    }
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id == 0)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&robot_location_in_vertex,
													   sIntegerIndex(robot_id),
													   sIntegerIndex(vertex_id),
													   sIntegerIndex(0)));
		}
	    }
	    else
	    {
		for (int robot_id = 1; robot_id < init_robot_id; ++robot_id)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&robot_location_in_vertex,
													   sIntegerIndex(robot_id),
													   sIntegerIndex(vertex_id),
													   sIntegerIndex(0)));
		}
		Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
									     total_Literal_cnt,
									     sSpecifiedBitIdentifier(&robot_location_in_vertex,
												     sIntegerIndex(init_robot_id),
												     sIntegerIndex(vertex_id),
												     sIntegerIndex(0)));

		for (int robot_id = init_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&robot_location_in_vertex,
													   sIntegerIndex(robot_id),
													   sIntegerIndex(vertex_id),
													   sIntegerIndex(0)));
		}
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id == 0)
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id < goal_robot_id; ++robot_id)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										 total_Literal_cnt,
										 sSpecifiedBitIdentifier(&robot_location_in_vertex,
													 sIntegerIndex(goal_robot_id),
													 sIntegerIndex(vertex_id),
													 sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    
		    for (int robot_id = goal_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_robot_Identifiers;

		    for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
		    {
			goal_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
										 sIntegerIndex(*robot_id),
										 sIntegerIndex(vertex_id),
										 sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    Clause_cnt += encoding_context.m_bit_generator->count_Disjunction(aux_Variable_cnt,
										      total_Literal_cnt,
										      goal_robot_Identifiers);

		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			if (robot_IDs.find(robot_id) == robot_IDs.end())
			{
			    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&robot_location_in_vertex,
														   sIntegerIndex(robot_id),
														   sIntegerIndex(vertex_id),
														   sIntegerIndex(encoding_context.m_N_Layers - 1)));			    
			}
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}

	int N_cnf_Variables = encoding_context.m_variable_store.get_Last_CNFVariable() + aux_Variable_cnt - 1;

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot direct SAT encoding\n", sPRODUCT);
	fprintf(fw, "c %s\n", sCOPYRIGHT);
	fprintf(fw, "c number of layers = %d\n", encoding_context.m_N_Layers);
	fprintf(fw, "c number of visible variables = %d\n", N_cnf_Variables - aux_Variable_cnt);
	fprintf(fw, "c number of hidden variables = %d\n", aux_Variable_cnt);
	fprintf(fw, "c number of literals = %d\n", total_Literal_cnt);
	fprintf(fw, "c number of clauses = %d\n", Clause_cnt);
	fprintf(fw, "c number of propositional variables = %d\n", N_cnf_Variables);
	fprintf(fw, "c number of clauses / number of variables = %.3f\n", (double)Clause_cnt / N_cnf_Variables);
	fprintf(fw, "c number of literals / number of clauses = %.3f\n", (double)total_Literal_cnt /  Clause_cnt);
	to_Stream(fw, "c ");
#endif
	fprintf(fw, "p cnf %d %d\n", N_cnf_Variables, Clause_cnt);
	
	// robot is placed in exactly one vertex
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector robot_Identifiers;

		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		encoding_context.m_bit_generator->generate_AllMutexConstraint(fw, robot_Identifiers);
		encoding_context.m_bit_generator->generate_Disjunction(fw, robot_Identifiers);
	    }
	}

	// there is at most one robot in a vertex
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector robot_mutex_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    bool forbid_vertex = true;
		    
		    switch (m_goal_type)
		    {
		    case GOAL_TYPE_ARRANGEMENT:
		    {
			if (all_pairs_Distances[vertex_id][m_goal_arrangement.get_RobotLocation(robot_id)] <= encoding_context.m_N_Layers - layer)
			{
			    forbid_vertex = false;
			}
			break;
		    }
		    case GOAL_TYPE_SPECIFICATION:
		    {
			const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
			for (sRobotGoal::Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
			{
			    if (all_pairs_Distances[vertex_id][*goal_id] <= encoding_context.m_N_Layers - layer)
			    {
				forbid_vertex = false;
				break;
			    }
			}
			break;
		    }
		    default:
		    {
			sASSERT(false);
			break;
		    }
		    }
		    if (forbid_vertex || all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] > layer)
		    {
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(layer)));
		    }
		    robot_mutex_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		encoding_context.m_bit_generator->generate_AllMutexConstraint(fw, robot_mutex_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    bool forbid_vertex = true;
		    
		    switch (m_goal_type)
		    {
		    case GOAL_TYPE_ARRANGEMENT:
		    {
			if (all_pairs_Distances[vertex_id][m_goal_arrangement.get_RobotLocation(robot_id)] <= encoding_context.m_N_Layers - layer)
			{
			    forbid_vertex = false;
			}
			break;
		    }
		    case GOAL_TYPE_SPECIFICATION:
		    {
			const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
			for (sRobotGoal::Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
			{
			    if (all_pairs_Distances[vertex_id][*goal_id] <= encoding_context.m_N_Layers - layer)
			    {
				forbid_vertex = false;
				break;
			    }
			}
			break;
		    }
		    default:
		    {
			sASSERT(false);
			break;
		    }
		    }
		    if (!forbid_vertex && all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] <= layer)
		    {
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector target_robot_Identifiers;
			target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));
			
			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;
			
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    int neighbor_id = (*out_neighbor)->m_target->m_id;
			    target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)));
			}
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												  sSpecifiedBitIdentifier(&robot_location_in_vertex,
															  sIntegerIndex(robot_id),
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  target_robot_Identifiers);
		    }
		}
	    }	    
	}

	for (int layer = 1; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    bool forbid_vertex = true;
		    
		    switch (m_goal_type)
		    {
		    case GOAL_TYPE_ARRANGEMENT:
		    {
			if (all_pairs_Distances[vertex_id][m_goal_arrangement.get_RobotLocation(robot_id)] <= encoding_context.m_N_Layers - layer)
			{
			    forbid_vertex = false;
			}
			break;
		    }
		    case GOAL_TYPE_SPECIFICATION:
		    {
			const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
			for (sRobotGoal::Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
			{
			    if (all_pairs_Distances[vertex_id][*goal_id] <= encoding_context.m_N_Layers - layer)
			    {
				forbid_vertex = false;
				break;
			    }
			}
			break;
		    }
		    default:
		    {
			sASSERT(false);
			break;
		    }
		    }
		    if (!forbid_vertex && all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] <= layer)
		    {
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector source_robot_Identifiers;
			source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer - 1)));

			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    int neighbor_id = (*out_neighbor)->m_target->m_id;
			    source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer - 1)));
			}
			encoding_context.m_bit_generator->generate_MultiImplication(fw,
										    sSpecifiedBitIdentifier(&robot_location_in_vertex,
													    sIntegerIndex(robot_id),
													    sIntegerIndex(vertex_id),
													    sIntegerIndex(layer)),
										    source_robot_Identifiers);
		    }
		}
	    }	    
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    bool forbid_vertex = true;
		    
		    switch (m_goal_type)
		    {
		    case GOAL_TYPE_ARRANGEMENT:
		    {
			if (all_pairs_Distances[vertex_id][m_goal_arrangement.get_RobotLocation(robot_id)] <= encoding_context.m_N_Layers - layer)
			{
			    forbid_vertex = false;
			}
			break;
		    }
		    case GOAL_TYPE_SPECIFICATION:
		    {
			const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
			for (sRobotGoal::Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
			{
			    if (all_pairs_Distances[vertex_id][*goal_id] <= encoding_context.m_N_Layers - layer)
			    {
				forbid_vertex = false;
				break;
			    }
			}
			break;
		    }
		    default:
		    {
			sASSERT(false);
			break;
		    }
		    }
		    if (!forbid_vertex && all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] <= layer)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    int neighbor_id = (*out_neighbor)->m_target->m_id;
			    
			    sBitClauseGenerator::SpecifiedBitIdentifiers_vector unset_robots_Identifiers;
			    for (int unset_robot_id = 1; unset_robot_id <= N_Robots; ++unset_robot_id)
			    {
				unset_robots_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
											   sIntegerIndex(unset_robot_id),
											   sIntegerIndex(vertex_id),
											   sIntegerIndex(layer + 1)));
				unset_robots_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
											   sIntegerIndex(unset_robot_id),
											   sIntegerIndex(neighbor_id),
											   sIntegerIndex(layer)));
			    }
			    
			    encoding_context.m_bit_generator->generate_SwapConstraint(fw,
										      sSpecifiedBitIdentifier(&robot_location_in_vertex,
													      sIntegerIndex(robot_id),
													      sIntegerIndex(vertex_id),
													      sIntegerIndex(layer)),
										      sSpecifiedBitIdentifier(&robot_location_in_vertex,
													      sIntegerIndex(robot_id),
													      sIntegerIndex(neighbor_id),
													      sIntegerIndex(layer + 1)),
										      unset_robots_Identifiers);
			}
		    }
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id == 0)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    encoding_context.m_bit_generator->generate_BitUnset(fw,
									sSpecifiedBitIdentifier(&robot_location_in_vertex,
												sIntegerIndex(robot_id),
												sIntegerIndex(vertex_id),
												sIntegerIndex(0)));
		}
	    }
	    else
	    {
		for (int robot_id = 1; robot_id < init_robot_id; ++robot_id)
		{
		    encoding_context.m_bit_generator->generate_BitUnset(fw,
									sSpecifiedBitIdentifier(&robot_location_in_vertex,
												sIntegerIndex(robot_id),
												sIntegerIndex(vertex_id),
												sIntegerIndex(0)));
		}
		encoding_context.m_bit_generator->generate_BitSet(fw,
								  sSpecifiedBitIdentifier(&robot_location_in_vertex,
											  sIntegerIndex(init_robot_id),
											  sIntegerIndex(vertex_id),
											  sIntegerIndex(0)));
		
		for (int robot_id = init_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		{
		    encoding_context.m_bit_generator->generate_BitUnset(fw,
									sSpecifiedBitIdentifier(&robot_location_in_vertex,
												sIntegerIndex(robot_id),
												sIntegerIndex(vertex_id),
												sIntegerIndex(0)));
		}
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id == 0)
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id < goal_robot_id; ++robot_id)
		    {
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    encoding_context.m_bit_generator->generate_BitSet(fw,
								      sSpecifiedBitIdentifier(&robot_location_in_vertex,
											      sIntegerIndex(goal_robot_id),
											      sIntegerIndex(vertex_id),
											      sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    
		    for (int robot_id = goal_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		    {
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_robot_Identifiers;

		    for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
		    {
			goal_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
										 sIntegerIndex(*robot_id),
										 sIntegerIndex(vertex_id),
										 sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    encoding_context.m_bit_generator->generate_Disjunction(fw, goal_robot_Identifiers);

		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			if (robot_IDs.find(robot_id) == robot_IDs.end())
			{
			    encoding_context.m_bit_generator->generate_BitUnset(fw,
										sSpecifiedBitIdentifier(&robot_location_in_vertex,
													sIntegerIndex(robot_id),
													sIntegerIndex(vertex_id),
													sIntegerIndex(encoding_context.m_N_Layers - 1)));			    
			}
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
    }


    void sMultirobotInstance::to_Stream_SimplicialCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose)) const
    {
	encoding_context.switchTo_AdvancedGeneratingMode();

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableBitIdentifier robot_location_in_vertex(&encoding_context.m_variable_store, "robot_location_in_vertex", sIntegerScope(1, N_Robots), sIntegerScope(0, N_Vertices - 1), sIntegerScope(0, encoding_context.m_N_Layers - 1));
	sIndexableBitIdentifier empty_vertex(&encoding_context.m_variable_store, "empty_vertex", sIntegerScope(0, N_Vertices - 1), sIntegerScope(0, encoding_context.m_N_Layers - 1));

	encoding_context.m_robot_location_in_vertex = robot_location_in_vertex;
	encoding_context.register_TranslateIdentifier(encoding_context.m_robot_location_in_vertex);

	// there is at most one robot in a vertex
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector robot_mutex_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    robot_mutex_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											 total_Literal_cnt,
											 robot_mutex_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector no_robot_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    no_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		Clause_cnt += encoding_context.m_bit_generator->count_MultiNegativeImplication(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&empty_vertex,
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(layer)),
											       no_robot_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector target_robot_Identifiers;
		    target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int neighbor_id = (*out_neighbor)->m_target->m_id;
			target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)));
		    }
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&robot_location_in_vertex,
														   sIntegerIndex(robot_id),
														   sIntegerIndex(vertex_id),
														   sIntegerIndex(layer)),
											   target_robot_Identifiers);
		}
	    }	    
	}
	/*
	for (int layer = 1; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector source_robot_Identifiers;
		    source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer - 1)));

		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int neighbor_id = (*out_neighbor)->m_target->m_id;
			source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer - 1)));
		    }
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&robot_location_in_vertex,
														   sIntegerIndex(robot_id),
														   sIntegerIndex(vertex_id),
														   sIntegerIndex(layer)),
											   source_robot_Identifiers);
		}
	    }	    
	}
	*/

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int neighbor_id = (*out_neighbor)->m_target->m_id;

			Clause_cnt += encoding_context.m_bit_generator->count_NegativeSwapConstraint(aux_Variable_cnt,
												     total_Literal_cnt,
												     sSpecifiedBitIdentifier(&robot_location_in_vertex,
															     sIntegerIndex(robot_id),
															     sIntegerIndex(vertex_id),
															     sIntegerIndex(layer)),
												     sSpecifiedBitIdentifier(&robot_location_in_vertex,
															     sIntegerIndex(robot_id),
															     sIntegerIndex(neighbor_id),
															     sIntegerIndex(layer + 1)),
												     sSpecifiedBitIdentifier(&empty_vertex,
															     sIntegerIndex(vertex_id),
															     sIntegerIndex(layer + 1)),
												     sSpecifiedBitIdentifier(&empty_vertex,
															     sIntegerIndex(neighbor_id),
															     sIntegerIndex(layer)));
		    }
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id == 0)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&robot_location_in_vertex,
													   sIntegerIndex(robot_id),
													   sIntegerIndex(vertex_id),
													   sIntegerIndex(0)));
		}
	    }
	    else
	    {
		for (int robot_id = 1; robot_id < init_robot_id; ++robot_id)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&robot_location_in_vertex,
													   sIntegerIndex(robot_id),
													   sIntegerIndex(vertex_id),
													   sIntegerIndex(0)));
		}
		Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
									     total_Literal_cnt,
									     sSpecifiedBitIdentifier(&robot_location_in_vertex,
												     sIntegerIndex(init_robot_id),
												     sIntegerIndex(vertex_id),
												     sIntegerIndex(0)));

		for (int robot_id = init_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&robot_location_in_vertex,
													   sIntegerIndex(robot_id),
													   sIntegerIndex(vertex_id),
													   sIntegerIndex(0)));
		}
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id == 0)
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id < goal_robot_id; ++robot_id)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										 total_Literal_cnt,
										 sSpecifiedBitIdentifier(&robot_location_in_vertex,
													 sIntegerIndex(goal_robot_id),
													 sIntegerIndex(vertex_id),
													 sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    
		    for (int robot_id = goal_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_robot_Identifiers;

		    for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
		    {
			goal_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
										 sIntegerIndex(*robot_id),
										 sIntegerIndex(vertex_id),
										 sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    Clause_cnt += encoding_context.m_bit_generator->count_Disjunction(aux_Variable_cnt,
										      total_Literal_cnt,
										      goal_robot_Identifiers);

		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			if (robot_IDs.find(robot_id) == robot_IDs.end())
			{
			    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&robot_location_in_vertex,
														   sIntegerIndex(robot_id),
														   sIntegerIndex(vertex_id),
														   sIntegerIndex(encoding_context.m_N_Layers - 1)));			    
			}
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}

	int N_cnf_Variables = encoding_context.m_variable_store.get_Last_CNFVariable() + aux_Variable_cnt - 1;

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot simplicial SAT encoding\n", sPRODUCT);
	fprintf(fw, "c %s\n", sCOPYRIGHT);
	fprintf(fw, "c number of layers = %d\n", encoding_context.m_N_Layers);
	fprintf(fw, "c number of visible variables = %d\n", N_cnf_Variables - aux_Variable_cnt);
	fprintf(fw, "c number of hidden variables = %d\n", aux_Variable_cnt);
	fprintf(fw, "c number of literals = %d\n", total_Literal_cnt);
	fprintf(fw, "c number of clauses = %d\n", Clause_cnt);
	fprintf(fw, "c number of propositional variables = %d\n", N_cnf_Variables);
	fprintf(fw, "c number of clauses / number of variables = %.3f\n", (double)Clause_cnt / N_cnf_Variables);
	fprintf(fw, "c number of literals / number of clauses = %.3f\n", (double)total_Literal_cnt /  Clause_cnt);
	to_Stream(fw, "c ");
#endif
	fprintf(fw, "p cnf %d %d\n", N_cnf_Variables, Clause_cnt);

	// there is at most one robot in a vertex
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector robot_mutex_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    robot_mutex_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		fprintf(fw, "c label allmutex layer=%d vertex=%d\n", layer, vertex_id);
		encoding_context.m_bit_generator->generate_AllMutexConstraint(fw, robot_mutex_Identifiers);
	    }
	}
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector no_robot_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    no_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}

		fprintf(fw, "c label multi_negative layer=%d vertex=%d\n", layer, vertex_id);
		Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegativeImplication(fw,
												  sSpecifiedBitIdentifier(&empty_vertex,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  no_robot_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector target_robot_Identifiers;
		    target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int neighbor_id = (*out_neighbor)->m_target->m_id;
			target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)));
		    }

		    fprintf(fw, "c label multi_target_implication layer=%d vertex=%d robot=%d\n", layer, vertex_id, robot_id);
		    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
											      sSpecifiedBitIdentifier(&robot_location_in_vertex,
														      sIntegerIndex(robot_id),
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      target_robot_Identifiers);
		}
	    }	    
	}

	/*
	for (int layer = 1; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector source_robot_Identifiers;
		    source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer - 1)));

		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int neighbor_id = (*out_neighbor)->m_target->m_id;
			source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer - 1)));
		    }
		    encoding_context.m_bit_generator->generate_MultiImplication(fw,
										sSpecifiedBitIdentifier(&robot_location_in_vertex,
													sIntegerIndex(robot_id),
													sIntegerIndex(vertex_id),
													sIntegerIndex(layer)),
										source_robot_Identifiers);
		}
	    }	    
	}
	*/

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int neighbor_id = (*out_neighbor)->m_target->m_id;

			fprintf(fw, "c label negative_swap layer=%d vertex=%d robot=%d\n", layer, vertex_id, robot_id);
			Clause_cnt += encoding_context.m_bit_generator->generate_NegativeSwapConstraint(fw,
													sSpecifiedBitIdentifier(&robot_location_in_vertex,
																sIntegerIndex(robot_id),
																sIntegerIndex(vertex_id),
																sIntegerIndex(layer)),
													sSpecifiedBitIdentifier(&robot_location_in_vertex,
																sIntegerIndex(robot_id),
																sIntegerIndex(neighbor_id),
																sIntegerIndex(layer + 1)),
													sSpecifiedBitIdentifier(&empty_vertex,
																sIntegerIndex(vertex_id),
																sIntegerIndex(layer + 1)),
													sSpecifiedBitIdentifier(&empty_vertex,
																sIntegerIndex(neighbor_id),
																sIntegerIndex(layer)));
		    }
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id == 0)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    fprintf(fw, "c label init_bit_unset vertex=%d robot=%d\n", vertex_id, robot_id);
		    encoding_context.m_bit_generator->generate_BitUnset(fw,
									sSpecifiedBitIdentifier(&robot_location_in_vertex,
												sIntegerIndex(robot_id),
												sIntegerIndex(vertex_id),
												sIntegerIndex(0)));
		}
	    }
	    else
	    {
		for (int robot_id = 1; robot_id < init_robot_id; ++robot_id)
		{
		    fprintf(fw, "c label init_bit_unset vertex=%d robot=%d\n", vertex_id, robot_id);
		    encoding_context.m_bit_generator->generate_BitUnset(fw,
									sSpecifiedBitIdentifier(&robot_location_in_vertex,
												sIntegerIndex(robot_id),
												sIntegerIndex(vertex_id),
												sIntegerIndex(0)));
		}
		fprintf(fw, "c label init_bit_set vertex=%d\n", vertex_id);
		encoding_context.m_bit_generator->generate_BitSet(fw,
								  sSpecifiedBitIdentifier(&robot_location_in_vertex,
											  sIntegerIndex(init_robot_id),
											  sIntegerIndex(vertex_id),
											  sIntegerIndex(0)));
		
		for (int robot_id = init_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		{
		    fprintf(fw, "c label init_bit_unset vertex=%d robot=%d\n", vertex_id, robot_id);
		    encoding_context.m_bit_generator->generate_BitUnset(fw,
									sSpecifiedBitIdentifier(&robot_location_in_vertex,
												sIntegerIndex(robot_id),
												sIntegerIndex(vertex_id),
												sIntegerIndex(0)));
		}
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id == 0)
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			fprintf(fw, "c label goal_bit_unset vertex=%d robot=%d\n", vertex_id, robot_id);
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id < goal_robot_id; ++robot_id)
		    {
			fprintf(fw, "c label goal_bit_unset vertex=%d robot=%d\n", vertex_id, robot_id);
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    fprintf(fw, "c label goal_bit_set vertex=%d\n", vertex_id);
		    encoding_context.m_bit_generator->generate_BitSet(fw,
								      sSpecifiedBitIdentifier(&robot_location_in_vertex,
											      sIntegerIndex(goal_robot_id),
											      sIntegerIndex(vertex_id),
											      sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    
		    for (int robot_id = goal_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		    {
			fprintf(fw, "c label goal_bit_unset vertex=%d robot=%d\n", vertex_id, robot_id);
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_robot_Identifiers;

		    for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
		    {
			goal_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
										 sIntegerIndex(*robot_id),
										 sIntegerIndex(vertex_id),
										 sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    fprintf(fw, "c label goal_disjunction vertex=%d\n", vertex_id);
		    encoding_context.m_bit_generator->generate_Disjunction(fw, goal_robot_Identifiers);

		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			if (robot_IDs.find(robot_id) == robot_IDs.end())
			{
			    fprintf(fw, "c label goal_bit_unset vertex=%d robot=%d\n", vertex_id, robot_id);
			    encoding_context.m_bit_generator->generate_BitUnset(fw,
										sSpecifiedBitIdentifier(&robot_location_in_vertex,
													sIntegerIndex(robot_id),
													sIntegerIndex(vertex_id),
													sIntegerIndex(encoding_context.m_N_Layers - 1)));			    
			}
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			fprintf(fw, "c label goal_bit_unset vertex=%d robot=%d\n", vertex_id, robot_id);
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
    }


    void sMultirobotInstance::to_Stream_HeuristicSimplicialCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();

	m_environment.calc_AllPairsShortestPaths();
	const sUndirectedGraph::Distances_2d_vector &all_pairs_Distances = m_environment.get_AllPairsShortestPaths();

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableBitIdentifier robot_location_in_vertex(&encoding_context.m_variable_store, "robot_location_in_vertex", sIntegerScope(1, N_Robots), sIntegerScope(0, N_Vertices - 1), sIntegerScope(0, encoding_context.m_N_Layers - 1));
	sIndexableBitIdentifier empty_vertex(&encoding_context.m_variable_store, "empty_vertex", sIntegerScope(0, N_Vertices - 1), sIntegerScope(0, encoding_context.m_N_Layers - 1));

	encoding_context.m_robot_location_in_vertex = robot_location_in_vertex;
	encoding_context.register_TranslateIdentifier(encoding_context.m_robot_location_in_vertex);

	// there is at most one robot in a vertex
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector robot_mutex_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    bool forbid_vertex = true;
		    
		    switch (m_goal_type)
		    {
		    case GOAL_TYPE_ARRANGEMENT:
		    {
			if (all_pairs_Distances[vertex_id][m_goal_arrangement.get_RobotLocation(robot_id)] <= encoding_context.m_N_Layers - layer)
			{
			    forbid_vertex = false;
			}
			break;
		    }
		    case GOAL_TYPE_SPECIFICATION:
		    {
			const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
			for (sRobotGoal::Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
			{
			    if (all_pairs_Distances[vertex_id][*goal_id] <= encoding_context.m_N_Layers - layer)
			    {
				forbid_vertex = false;
				break;
			    }
			}
			break;
		    }
		    default:
		    {
			sASSERT(false);
			break;
		    }
		    }
		    if (forbid_vertex || all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] > layer)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(layer)));
		    }

		    robot_mutex_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											 total_Literal_cnt,
											 robot_mutex_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector no_robot_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    no_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		Clause_cnt += encoding_context.m_bit_generator->count_MultiNegativeImplication(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&empty_vertex,
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(layer)),
											       no_robot_Identifiers);
	    }
	}
	/*
	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    bool forbid_vertex = true;
		    
		    switch (m_goal_type)
		    {
		    case GOAL_TYPE_ARRANGEMENT:
		    {
			if (all_pairs_Distances[vertex_id][m_goal_arrangement.get_RobotLocation(robot_id)] <= encoding_context.m_N_Layers - layer)
			{
			    forbid_vertex = false;
			}
			break;
		    }
		    case GOAL_TYPE_SPECIFICATION:
		    {
			const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
			for (sRobotGoal::Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
			{
			    if (all_pairs_Distances[vertex_id][*goal_id] <= encoding_context.m_N_Layers - layer)
			    {
				forbid_vertex = false;
				break;
			    }
			}
			break;
		    }
		    default:
		    {
			sASSERT(false);
			break;
		    }
		    }
		    if (!forbid_vertex && all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] <= layer)
		    {
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector target_robot_Identifiers;
			target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;
			
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    int neighbor_id = (*out_neighbor)->m_target->m_id;
			    target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)));
			}
			Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&robot_location_in_vertex,
														       sIntegerIndex(robot_id),
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(layer)),
											       target_robot_Identifiers);
		    }
		}
	    }	    
	}
	*/
	for (int layer = 1; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    bool forbid_vertex = true;
		    
		    switch (m_goal_type)
		    {
		    case GOAL_TYPE_ARRANGEMENT:
		    {
			if (all_pairs_Distances[vertex_id][m_goal_arrangement.get_RobotLocation(robot_id)] <= encoding_context.m_N_Layers - layer)
			{
			    forbid_vertex = false;
			}
			break;
		    }
		    case GOAL_TYPE_SPECIFICATION:
		    {
			const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
			for (sRobotGoal::Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
			{
			    if (all_pairs_Distances[vertex_id][*goal_id] <= encoding_context.m_N_Layers - layer)
			    {
				forbid_vertex = false;
				break;
			    }
			}
			break;
		    }
		    default:
		    {
			sASSERT(false);
			break;
		    }
		    }
		    if (!forbid_vertex && all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] <= layer)
		    {
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector source_robot_Identifiers;
			source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer - 1)));
		    
			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    int neighbor_id = (*out_neighbor)->m_target->m_id;
			    source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer - 1)));
			}
			Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&robot_location_in_vertex,
														       sIntegerIndex(robot_id),
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(layer)),
											       source_robot_Identifiers);
		    }
		}
	    }	    
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    bool forbid_vertex = true;
		    
		    switch (m_goal_type)
		    {
		    case GOAL_TYPE_ARRANGEMENT:
		    {
			if (all_pairs_Distances[vertex_id][m_goal_arrangement.get_RobotLocation(robot_id)] <= encoding_context.m_N_Layers - layer)
			{
			    forbid_vertex = false;
			}
			break;
		    }
		    case GOAL_TYPE_SPECIFICATION:
		    {
			const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
			for (sRobotGoal::Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
			{
			    if (all_pairs_Distances[vertex_id][*goal_id] <= encoding_context.m_N_Layers - layer)
			    {
				forbid_vertex = false;
				break;
			    }
			}
			break;
		    }
		    default:
		    {
			sASSERT(false);
			break;
		    }
		    }
		    if (!forbid_vertex && all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] <= layer)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    int neighbor_id = (*out_neighbor)->m_target->m_id;

			    Clause_cnt += encoding_context.m_bit_generator->count_NegativeSwapConstraint(aux_Variable_cnt,
													 total_Literal_cnt,
													 sSpecifiedBitIdentifier(&robot_location_in_vertex,
																 sIntegerIndex(robot_id),
																 sIntegerIndex(vertex_id),
																 sIntegerIndex(layer)),
													 sSpecifiedBitIdentifier(&robot_location_in_vertex,
																 sIntegerIndex(robot_id),
																 sIntegerIndex(neighbor_id),
																 sIntegerIndex(layer + 1)),
													 sSpecifiedBitIdentifier(&empty_vertex,
																 sIntegerIndex(vertex_id),
																 sIntegerIndex(layer + 1)),
													 sSpecifiedBitIdentifier(&empty_vertex,
																 sIntegerIndex(neighbor_id),
																 sIntegerIndex(layer)));
			}
		    }
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id == 0)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&robot_location_in_vertex,
													   sIntegerIndex(robot_id),
													   sIntegerIndex(vertex_id),
													   sIntegerIndex(0)));
		}
	    }
	    else
	    {
		for (int robot_id = 1; robot_id < init_robot_id; ++robot_id)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&robot_location_in_vertex,
													   sIntegerIndex(robot_id),
													   sIntegerIndex(vertex_id),
													   sIntegerIndex(0)));
		}
		Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
									     total_Literal_cnt,
									     sSpecifiedBitIdentifier(&robot_location_in_vertex,
												     sIntegerIndex(init_robot_id),
												     sIntegerIndex(vertex_id),
												     sIntegerIndex(0)));

		for (int robot_id = init_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&robot_location_in_vertex,
													   sIntegerIndex(robot_id),
													   sIntegerIndex(vertex_id),
													   sIntegerIndex(0)));
		}
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id == 0)
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id < goal_robot_id; ++robot_id)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										 total_Literal_cnt,
										 sSpecifiedBitIdentifier(&robot_location_in_vertex,
													 sIntegerIndex(goal_robot_id),
													 sIntegerIndex(vertex_id),
													 sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    
		    for (int robot_id = goal_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_robot_Identifiers;

		    for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
		    {
			goal_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
										 sIntegerIndex(*robot_id),
										 sIntegerIndex(vertex_id),
										 sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    Clause_cnt += encoding_context.m_bit_generator->count_Disjunction(aux_Variable_cnt,
										      total_Literal_cnt,
										      goal_robot_Identifiers);

		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			if (robot_IDs.find(robot_id) == robot_IDs.end())
			{
			    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&robot_location_in_vertex,
														   sIntegerIndex(robot_id),
														   sIntegerIndex(vertex_id),
														   sIntegerIndex(encoding_context.m_N_Layers - 1)));			    
			}
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}

	int N_cnf_Variables = encoding_context.m_variable_store.get_Last_CNFVariable() + aux_Variable_cnt - 1;

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot simplicial SAT encoding\n", sPRODUCT);
	fprintf(fw, "c %s\n", sCOPYRIGHT);
	fprintf(fw, "c number of layers = %d\n", encoding_context.m_N_Layers);
	fprintf(fw, "c number of visible variables = %d\n", N_cnf_Variables - aux_Variable_cnt);
	fprintf(fw, "c number of hidden variables = %d\n", aux_Variable_cnt);
	fprintf(fw, "c number of literals = %d\n", total_Literal_cnt);
	fprintf(fw, "c number of clauses = %d\n", Clause_cnt);
	fprintf(fw, "c number of propositional variables = %d\n", N_cnf_Variables);
	fprintf(fw, "c number of clauses / number of variables = %.3f\n", (double)Clause_cnt / N_cnf_Variables);
	fprintf(fw, "c number of literals / number of clauses = %.3f\n", (double)total_Literal_cnt /  Clause_cnt);
	to_Stream(fw, "c ");
#endif
	fprintf(fw, "p cnf %d %d\n", N_cnf_Variables, Clause_cnt);
	
	// there is at most one robot in a vertex
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector robot_mutex_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    bool forbid_vertex = true;
		    
		    switch (m_goal_type)
		    {
		    case GOAL_TYPE_ARRANGEMENT:
		    {
			if (all_pairs_Distances[vertex_id][m_goal_arrangement.get_RobotLocation(robot_id)] <= encoding_context.m_N_Layers - layer)
			{
			    forbid_vertex = false;
			}
			break;
		    }
		    case GOAL_TYPE_SPECIFICATION:
		    {
			const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
			for (sRobotGoal::Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
			{
			    if (all_pairs_Distances[vertex_id][*goal_id] <= encoding_context.m_N_Layers - layer)
			    {
				forbid_vertex = false;
				break;
			    }
			}
			break;
		    }
		    default:
		    {
			sASSERT(false);
			break;
		    }
		    }
		    if (forbid_vertex || all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] > layer)
		    {
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(layer)));
		    }
		    robot_mutex_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		encoding_context.m_bit_generator->generate_AllMutexConstraint(fw, robot_mutex_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector no_robot_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    no_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegativeImplication(fw,
												  sSpecifiedBitIdentifier(&empty_vertex,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  no_robot_Identifiers);
	    }
	}
	/*
	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    bool forbid_vertex = true;
		    
		    switch (m_goal_type)
		    {
		    case GOAL_TYPE_ARRANGEMENT:
		    {
			if (all_pairs_Distances[vertex_id][m_goal_arrangement.get_RobotLocation(robot_id)] <= encoding_context.m_N_Layers - layer)
			{
			    forbid_vertex = false;
			}
			break;
		    }
		    case GOAL_TYPE_SPECIFICATION:
		    {
			const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
			for (sRobotGoal::Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
			{
			    if (all_pairs_Distances[vertex_id][*goal_id] <= encoding_context.m_N_Layers - layer)
			    {
				forbid_vertex = false;
				break;
			    }
			}
			break;
		    }
		    default:
		    {
			sASSERT(false);
			break;
		    }
		    }
		    if (!forbid_vertex && all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] <= layer)
		    {
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector target_robot_Identifiers;
			target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));
			
			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;
			
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    int neighbor_id = (*out_neighbor)->m_target->m_id;
			    target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)));
			}
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												  sSpecifiedBitIdentifier(&robot_location_in_vertex,
															  sIntegerIndex(robot_id),
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  target_robot_Identifiers);
		    }
		}
	    }	    
	}
	*/
	for (int layer = 1; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    bool forbid_vertex = true;
		    
		    switch (m_goal_type)
		    {
		    case GOAL_TYPE_ARRANGEMENT:
		    {
			if (all_pairs_Distances[vertex_id][m_goal_arrangement.get_RobotLocation(robot_id)] <= encoding_context.m_N_Layers - layer)
			{
			    forbid_vertex = false;
			}
			break;
		    }
		    case GOAL_TYPE_SPECIFICATION:
		    {
			const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
			for (sRobotGoal::Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
			{
			    if (all_pairs_Distances[vertex_id][*goal_id] <= encoding_context.m_N_Layers - layer)
			    {
				forbid_vertex = false;
				break;
			    }
			}
			break;
		    }
		    default:
		    {
			sASSERT(false);
			break;
		    }
		    }
		    if (!forbid_vertex && all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] <= layer)
		    {
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector source_robot_Identifiers;
			source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer - 1)));

			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    int neighbor_id = (*out_neighbor)->m_target->m_id;
			    source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer - 1)));
			}
			encoding_context.m_bit_generator->generate_MultiImplication(fw,
										    sSpecifiedBitIdentifier(&robot_location_in_vertex,
													    sIntegerIndex(robot_id),
													    sIntegerIndex(vertex_id),
													    sIntegerIndex(layer)),
										    source_robot_Identifiers);
		    }
		}
	    }	    
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    bool forbid_vertex = true;
		    
		    switch (m_goal_type)
		    {
		    case GOAL_TYPE_ARRANGEMENT:
		    {
			if (all_pairs_Distances[vertex_id][m_goal_arrangement.get_RobotLocation(robot_id)] <= encoding_context.m_N_Layers - layer)
			{
			    forbid_vertex = false;
			}
			break;
		    }
		    case GOAL_TYPE_SPECIFICATION:
		    {
			const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
			for (sRobotGoal::Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
			{
			    if (all_pairs_Distances[vertex_id][*goal_id] <= encoding_context.m_N_Layers - layer)
			    {
				forbid_vertex = false;
				break;
			    }
			}
			break;
		    }
		    default:
		    {
			sASSERT(false);
			break;
		    }
		    }
		    if (!forbid_vertex && all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] <= layer)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    int neighbor_id = (*out_neighbor)->m_target->m_id;
			    
			    Clause_cnt += encoding_context.m_bit_generator->generate_NegativeSwapConstraint(fw,
													    sSpecifiedBitIdentifier(&robot_location_in_vertex,
																    sIntegerIndex(robot_id),
																    sIntegerIndex(vertex_id),
																    sIntegerIndex(layer)),
													    sSpecifiedBitIdentifier(&robot_location_in_vertex,
																    sIntegerIndex(robot_id),
																    sIntegerIndex(neighbor_id),
																    sIntegerIndex(layer + 1)),
													    sSpecifiedBitIdentifier(&empty_vertex,
																    sIntegerIndex(vertex_id),
																    sIntegerIndex(layer + 1)),
													    sSpecifiedBitIdentifier(&empty_vertex,
																    sIntegerIndex(neighbor_id),
																    sIntegerIndex(layer)));
			}
		    }
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id == 0)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    encoding_context.m_bit_generator->generate_BitUnset(fw,
									sSpecifiedBitIdentifier(&robot_location_in_vertex,
												sIntegerIndex(robot_id),
												sIntegerIndex(vertex_id),
												sIntegerIndex(0)));
		}
	    }
	    else
	    {
		for (int robot_id = 1; robot_id < init_robot_id; ++robot_id)
		{
		    encoding_context.m_bit_generator->generate_BitUnset(fw,
									sSpecifiedBitIdentifier(&robot_location_in_vertex,
												sIntegerIndex(robot_id),
												sIntegerIndex(vertex_id),
												sIntegerIndex(0)));
		}
		encoding_context.m_bit_generator->generate_BitSet(fw,
								  sSpecifiedBitIdentifier(&robot_location_in_vertex,
											  sIntegerIndex(init_robot_id),
											  sIntegerIndex(vertex_id),
											  sIntegerIndex(0)));
		
		for (int robot_id = init_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		{
		    encoding_context.m_bit_generator->generate_BitUnset(fw,
									sSpecifiedBitIdentifier(&robot_location_in_vertex,
												sIntegerIndex(robot_id),
												sIntegerIndex(vertex_id),
												sIntegerIndex(0)));
		}
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id == 0)
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id < goal_robot_id; ++robot_id)
		    {
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    encoding_context.m_bit_generator->generate_BitSet(fw,
								      sSpecifiedBitIdentifier(&robot_location_in_vertex,
											      sIntegerIndex(goal_robot_id),
											      sIntegerIndex(vertex_id),
											      sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    
		    for (int robot_id = goal_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		    {
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_robot_Identifiers;

		    for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
		    {
			goal_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
										 sIntegerIndex(*robot_id),
										 sIntegerIndex(vertex_id),
										 sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    encoding_context.m_bit_generator->generate_Disjunction(fw, goal_robot_Identifiers);

		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			if (robot_IDs.find(robot_id) == robot_IDs.end())
			{
			    encoding_context.m_bit_generator->generate_BitUnset(fw,
										sSpecifiedBitIdentifier(&robot_location_in_vertex,
													sIntegerIndex(robot_id),
													sIntegerIndex(vertex_id),
													sIntegerIndex(encoding_context.m_N_Layers - 1)));			    
			}
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
    }


    void sMultirobotInstance::to_Stream_SingularCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose)) const
    {
	encoding_context.switchTo_AdvancedGeneratingMode();

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableStateIdentifier vertex_occupancy(&encoding_context.m_variable_store, "vertex_occupancy", N_Robots + 1, sIntegerScope(0, N_Vertices - 1));
	encoding_context.m_vertex_occupancy = vertex_occupancy;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy);

	sIndexableBitIdentifier vertex_occupancy_by_water(&encoding_context.m_variable_store, "vertex_occupancy_by_water", sIntegerScope(0, N_Vertices - 1));
	encoding_context.m_vertex_occupancy_by_water = vertex_occupancy_by_water;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_water);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    Clause_cnt += encoding_context.m_clause_generator->count_Alignment(aux_Variable_cnt,
									       total_Literal_cnt,
									       sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id)));
	}

	encoding_context.m_edge_occupancy_by_water.resize(N_Vertices);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    sIndexableBitIdentifier edge_occupancy_by_water(&encoding_context.m_variable_store, "edge_occupancy_by_water-" + sInt_32_to_String(vertex_id),
							    sIntegerScope(0, m_environment.get_Vertex(vertex_id)->calc_NeighborCount() - 1));
	    encoding_context.m_edge_occupancy_by_water[vertex_id] = edge_occupancy_by_water;
	    encoding_context.register_TranslateIdentifier(encoding_context.m_edge_occupancy_by_water[vertex_id]);

	    Clause_cnt += encoding_context.m_bit_generator->count_NonzeroImplication(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id)),
										     sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id)));
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
	    
	    int out_neighbor_index = 0;
	    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		if (m_goal_arrangement.get_VertexOccupancy(vertex_id) == 0)
		{
		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											  total_Literal_cnt,
											  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id],
														  sIntegerIndex(out_neighbor_index)),
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id)),
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex((*out_neighbor)->m_target->m_id)));
			Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
												  total_Literal_cnt,
												  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id],
															  sIntegerIndex(out_neighbor_index)),
												  sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex(vertex_id)),
												  sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex((*out_neighbor)->m_target->m_id)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index)));
		    
			++out_neighbor_index;
		    }
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id)),
											   mutex_target_Identifiers);
		    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											     total_Literal_cnt,
											     mutex_target_Identifiers);   
		}
		else
		{
		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index)));
		    
			++out_neighbor_index;
		    }
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiNegation(aux_Variable_cnt,
											total_Literal_cnt,
											mutex_target_Identifiers);
		}
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		if (m_goal_specification.get_GoalCompatibility(vertex_id).empty())
		{
		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											  total_Literal_cnt,
											  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id],
														  sIntegerIndex(out_neighbor_index)),
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id)),
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex((*out_neighbor)->m_target->m_id)));
			Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
												  total_Literal_cnt,
												  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id],
															  sIntegerIndex(out_neighbor_index)),
												  sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex(vertex_id)),
												  sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex((*out_neighbor)->m_target->m_id)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index)));
		    
			++out_neighbor_index;
		    }
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id)),
											   mutex_target_Identifiers);
		    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											     total_Literal_cnt,
											     mutex_target_Identifiers);   
		}
		else
		{
		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index)));
		    
			++out_neighbor_index;
		    }
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiNegation(aux_Variable_cnt,
											total_Literal_cnt,
											mutex_target_Identifiers);
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_source_Identifiers;

	    const sVertex::Neighbors_list &in_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

	    for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
	    {
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;

		int in_neighbor_index = 0;
		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    if ((*out_neighbor)->m_target->m_id == vertex_id)
		    {
			break;
		    }
		    ++in_neighbor_index;
		}
		mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[(*in_neighbor)->m_target->m_id], sIntegerIndex(in_neighbor_index)));
	    }
	    if (m_initial_arrangement.get_VertexOccupancy(vertex_id) == 0)
	    {
		Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											 total_Literal_cnt,
											 mutex_source_Identifiers);
		Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id)),
										       mutex_source_Identifiers);
	    }
	    else
	    {
		Clause_cnt += encoding_context.m_bit_generator->count_MultiNegation(aux_Variable_cnt,
										    total_Literal_cnt,
										    mutex_source_Identifiers);
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id > 0)
	    {
		Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
										  total_Literal_cnt,
										  sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id)),
										  init_robot_id);
		Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
									     total_Literal_cnt,
									     sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id)));
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id > 0)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_Equality(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id)),
										   goal_robot_id);
		    Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										 total_Literal_cnt,
										 sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id)));
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    if (robot_IDs.size() > 1)
		    {
			sASSERT(false);
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
											  total_Literal_cnt,
											  sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id)),
											  *robot_IDs.begin());
			Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id)));
		    }
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}

	int N_cnf_Variables = encoding_context.m_variable_store.get_Last_CNFVariable() + aux_Variable_cnt - 1;

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot matching SAT encoding\n", sPRODUCT);
	fprintf(fw, "c %s\n", sCOPYRIGHT);
	fprintf(fw, "c number of layers = %d\n", encoding_context.m_N_Layers);
	fprintf(fw, "c number of visible variables = %d\n", N_cnf_Variables - aux_Variable_cnt);
	fprintf(fw, "c number of hidden variables = %d\n", aux_Variable_cnt);
	fprintf(fw, "c number of literals = %d\n", total_Literal_cnt);
	fprintf(fw, "c number of clauses = %d\n", Clause_cnt);
	fprintf(fw, "c number of propositional variables = %d\n", N_cnf_Variables);
	fprintf(fw, "c number of clauses / number of variables = %.3f\n", (double)Clause_cnt / N_cnf_Variables);
	fprintf(fw, "c number of literals / number of clauses = %.3f\n", (double)total_Literal_cnt /  Clause_cnt);
	to_Stream(fw, "c ");
#endif
	fprintf(fw, "p cnf %d %d\n", N_cnf_Variables, Clause_cnt);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    Clause_cnt += encoding_context.m_clause_generator->generate_Alignment(fw,
										  sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id)));
	}
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    Clause_cnt += encoding_context.m_bit_generator->generate_NonzeroImplication(fw,
											sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id)),
											sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id)));
	}
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
	    
	    int out_neighbor_index = 0;
	    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		if (m_goal_arrangement.get_VertexOccupancy(vertex_id) == 0)
		{
		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id],
														     sIntegerIndex(out_neighbor_index)),
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex(vertex_id)),
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex((*out_neighbor)->m_target->m_id)));
			Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id],
															  sIntegerIndex(out_neighbor_index)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex(vertex_id)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex((*out_neighbor)->m_target->m_id)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index)));		    
			++out_neighbor_index;
		    }

		    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id)),
											      mutex_target_Identifiers);
		    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												mutex_target_Identifiers);
		}
		else
		{
		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index)));
		    
			++out_neighbor_index;
		    }
		    Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegation(fw, mutex_target_Identifiers);   
		}
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		if (m_goal_specification.get_GoalCompatibility(vertex_id).empty())
		{
		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id],
														     sIntegerIndex(out_neighbor_index)),
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex(vertex_id)),
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex((*out_neighbor)->m_target->m_id)));
			Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id],
															  sIntegerIndex(out_neighbor_index)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex(vertex_id)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex((*out_neighbor)->m_target->m_id)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index)));
		    
			++out_neighbor_index;
		    }
		    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id)),
											      mutex_target_Identifiers);
		    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												mutex_target_Identifiers);   
		}
		else
		{
		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index)));
		    
			++out_neighbor_index;
		    }
		    Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegation(fw, mutex_target_Identifiers);   
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_source_Identifiers;
  	    const sVertex::Neighbors_list &in_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;
	    
	    for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
	    {
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;

		int in_neighbor_index = 0;
		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    if ((*out_neighbor)->m_target->m_id == vertex_id)
		    {
			break;
		    }
		    ++in_neighbor_index;
		}
		mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[(*in_neighbor)->m_target->m_id], sIntegerIndex(in_neighbor_index)));
	    }
	    if (m_initial_arrangement.get_VertexOccupancy(vertex_id) == 0)
	    {
		Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id)),
											  mutex_source_Identifiers);
		Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
											    mutex_source_Identifiers);
	    }
	    else
	    {
		Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegation(fw,
										       mutex_source_Identifiers);
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{

	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id > 0)
	    {
		Clause_cnt += encoding_context.m_clause_generator->generate_Equality(fw,
										     sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id)),
										     init_robot_id);
		Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
										sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id)));
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id > 0)
		{
		    Clause_cnt += encoding_context.m_bit_generator->generate_Equality(fw,
										      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id)),
										      goal_robot_id);
		    Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
										    sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id)));
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    if (robot_IDs.size() > 1)
		    {
			sASSERT(false);
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_clause_generator->generate_Equality(fw,
											     sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id)),
											     *robot_IDs.begin());
			Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
											sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id)));
		    }
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
    }


    void sMultirobotInstance::to_Stream_PluralCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose)) const
    {
	encoding_context.switchTo_AdvancedGeneratingMode();

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableStateIdentifier vertex_occupancy(&encoding_context.m_variable_store,
						   "vertex_occupancy",
						   N_Robots + 1,
						   sIntegerScope(0, N_Vertices - 1),
						   sIntegerScope(0, encoding_context.m_N_Layers - 1));
	encoding_context.m_vertex_occupancy = vertex_occupancy;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy);

	sIndexableBitIdentifier vertex_occupancy_by_water(&encoding_context.m_variable_store,
							  "vertex_occupancy_by_water",
							  sIntegerScope(0, N_Vertices - 1),
							  sIntegerScope(0, encoding_context.m_N_Layers - 1));
	encoding_context.m_vertex_occupancy_by_water = vertex_occupancy_by_water;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_water);

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		/*
		Clause_cnt += encoding_context.m_clause_generator->count_Alignment(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedStateIdentifier(&vertex_occupancy,
													     sIntegerIndex(vertex_id),
													     sIntegerIndex(layer)));
		*/
	    }
	}
	encoding_context.m_edge_occupancy_by_water_.resize(encoding_context.m_N_Layers);

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    encoding_context.m_edge_occupancy_by_water_[layer].resize(N_Vertices);


	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sIndexableBitIdentifier edge_occupancy_by_water_(&encoding_context.m_variable_store,
								 "edge_occupancy_by_water-" + sInt_32_to_String(vertex_id) + "_" + sInt_32_to_String(layer),
								 sIntegerScope(0, m_environment.get_Vertex(vertex_id)->calc_NeighborCount()));
		encoding_context.m_edge_occupancy_by_water_[layer][vertex_id] = edge_occupancy_by_water_;
		encoding_context.register_TranslateIdentifier(encoding_context.m_edge_occupancy_by_water_[layer][vertex_id]);
		
		Clause_cnt += encoding_context.m_bit_generator->count_NonzeroImplication(aux_Variable_cnt,
											 total_Literal_cnt,
											 sSpecifiedStateIdentifier(&vertex_occupancy,
														   sIntegerIndex(vertex_id),
														   sIntegerIndex(layer)),
											 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														 sIntegerIndex(vertex_id),
														 sIntegerIndex(layer)));
	    }
	}
	encoding_context.m_edge_occupancy_by_water_[encoding_context.m_N_Layers - 1].resize(N_Vertices);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    sIndexableBitIdentifier edge_occupancy_by_water_(&encoding_context.m_variable_store,
							    "edge_occupancy_by_water-" + sInt_32_to_String(vertex_id) + "_" + sInt_32_to_String(encoding_context.m_N_Layers - 1),
							    sIntegerScope(0, m_environment.get_Vertex(vertex_id)->calc_NeighborCount() - 1));
	    encoding_context.m_edge_occupancy_by_water_[encoding_context.m_N_Layers - 1][vertex_id] = edge_occupancy_by_water_;
	    encoding_context.register_TranslateIdentifier(encoding_context.m_edge_occupancy_by_water_[encoding_context.m_N_Layers - 1][vertex_id]);

	    Clause_cnt += encoding_context.m_bit_generator->count_NonzeroImplication(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedStateIdentifier(&vertex_occupancy,
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)),
										     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													     sIntegerIndex(vertex_id),
													     sIntegerIndex(encoding_context.m_N_Layers - 1)));
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
		
		int out_neighbor_index = 0;
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;
		
		switch (m_goal_type)
		{
		case GOAL_TYPE_ARRANGEMENT:
		{
		    if (layer == encoding_context.m_N_Layers - 1)
		    {
			if (m_goal_arrangement.get_VertexOccupancy(vertex_id) != 0)
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
											   sIntegerIndex(out_neighbor_index)));
				
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->count_MultiNegation(aux_Variable_cnt,
												total_Literal_cnt,
												mutex_target_Identifiers);
			}
			else
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
												  total_Literal_cnt,
												  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															  sIntegerIndex(out_neighbor_index)),
												  /*
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  */
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex((*out_neighbor)->m_target->m_id),
															  sIntegerIndex(layer)));
				Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
													  total_Literal_cnt,
													  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																  sIntegerIndex(out_neighbor_index)),
													  sSpecifiedStateIdentifier(&vertex_occupancy,
																    sIntegerIndex(vertex_id),
																    sIntegerIndex(layer)),
													  sSpecifiedStateIdentifier(&vertex_occupancy,
																    sIntegerIndex((*out_neighbor)->m_target->m_id),
																    sIntegerIndex(layer)));
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index)));
			    
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
												   total_Literal_cnt,
												   sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															   sIntegerIndex(vertex_id),
															   sIntegerIndex(layer)),
												   mutex_target_Identifiers);
			    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
												     total_Literal_cnt,
												     mutex_target_Identifiers);   
			}
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											  total_Literal_cnt,
											  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														  sIntegerIndex(0)),
											  /*
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer)),
											  */
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer + 1)));
			Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
												  total_Literal_cnt,
												  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															  sIntegerIndex(0)),
												  sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex(vertex_id),
															    sIntegerIndex(layer)),
												  sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex(vertex_id),
															    sIntegerIndex(layer + 1)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(0)));

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											      total_Literal_cnt,
											      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														      sIntegerIndex(out_neighbor_index + 1)),
											      /*
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      */
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex((*out_neighbor)->m_target->m_id),
														      sIntegerIndex(layer)));
			    Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
												      total_Literal_cnt,
												      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															      sIntegerIndex(out_neighbor_index + 1)),
												      sSpecifiedStateIdentifier(&vertex_occupancy,
																sIntegerIndex(vertex_id),
																sIntegerIndex(layer)),
												      sSpecifiedStateIdentifier(&vertex_occupancy,
																sIntegerIndex((*out_neighbor)->m_target->m_id),
																sIntegerIndex(layer)));
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(layer)),
											       mutex_target_Identifiers);
			Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
												 total_Literal_cnt,
												 mutex_target_Identifiers);   
		    }
		    break;
		}
		case GOAL_TYPE_SPECIFICATION:
		{
		    if (layer == encoding_context.m_N_Layers - 1)
		    {
			if (!m_goal_specification.get_GoalCompatibility(vertex_id).empty())
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
											   sIntegerIndex(out_neighbor_index)));
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->count_MultiNegation(aux_Variable_cnt,
												total_Literal_cnt,
												mutex_target_Identifiers);
			}
			else
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
												  total_Literal_cnt,
												  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															  sIntegerIndex(out_neighbor_index)),
												  /*
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  */
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex((*out_neighbor)->m_target->m_id),
															  sIntegerIndex(layer)));
				Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
													  total_Literal_cnt,
													  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																  sIntegerIndex(out_neighbor_index)),
													  sSpecifiedStateIdentifier(&vertex_occupancy,
																    sIntegerIndex(vertex_id),
																    sIntegerIndex(layer)),
													  sSpecifiedStateIdentifier(&vertex_occupancy,
																    sIntegerIndex((*out_neighbor)->m_target->m_id),
																    sIntegerIndex(layer)));
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index)));
			    
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
												   total_Literal_cnt,
												   sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															   sIntegerIndex(vertex_id),
															   sIntegerIndex(layer)),
												   mutex_target_Identifiers);
			    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
												     total_Literal_cnt,
												     mutex_target_Identifiers);   
			}
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											  total_Literal_cnt,
											  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														  sIntegerIndex(0)),
											  /*
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer)),
											  */
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer + 1)));
			Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
												  total_Literal_cnt,
												  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															  sIntegerIndex(0)),
												  sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex(vertex_id),
															    sIntegerIndex(layer)),
												  sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex(vertex_id),
															    sIntegerIndex(layer + 1)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(0)));

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											      total_Literal_cnt,
											      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														      sIntegerIndex(out_neighbor_index + 1)),
											      /*
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      */
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex((*out_neighbor)->m_target->m_id),
														      sIntegerIndex(layer)));
			    Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
												      total_Literal_cnt,
												      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															      sIntegerIndex(out_neighbor_index + 1)),
												      sSpecifiedStateIdentifier(&vertex_occupancy,
																sIntegerIndex(vertex_id),
																sIntegerIndex(layer)),
												      sSpecifiedStateIdentifier(&vertex_occupancy,
																sIntegerIndex((*out_neighbor)->m_target->m_id),
																sIntegerIndex(layer)));
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(layer)),
											       mutex_target_Identifiers);
			Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
												 total_Literal_cnt,
												 mutex_target_Identifiers);   
		    }
		    break;
		}
		default:
		{
		    sASSERT(false);
		    break;
		}
		}
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_source_Identifiers;

		if (layer > 0)
		{
		    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer - 1][vertex_id],
									       sIntegerIndex(0)));
		    const sVertex::Neighbors_list &in_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;
		    
			int in_neighbor_index = 0;
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    if ((*out_neighbor)->m_target->m_id == vertex_id)
			    {
				break;
			    }
			    ++in_neighbor_index;
			}
			if (layer == encoding_context.m_N_Layers - 1)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index)));
			}
			else
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index + 1)));
			}
		    }
		}
		else
		{
		    const sVertex::Neighbors_list &in_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;
		
		    for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;
		    
			int in_neighbor_index = 0;
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    if ((*out_neighbor)->m_target->m_id == vertex_id)
			    {
				break;
			    }
			    ++in_neighbor_index;
			}
			if (layer == encoding_context.m_N_Layers - 1)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index)));
			}
			else
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index + 1)));
			}
		    }
		}
		if (layer == 0)
		{
		    if (m_initial_arrangement.get_VertexOccupancy(vertex_id) == 0)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
												 total_Literal_cnt,
												 mutex_source_Identifiers);
			
			Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(layer)),
											       mutex_source_Identifiers);
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_MultiNegation(aux_Variable_cnt,
											    total_Literal_cnt,
											    mutex_source_Identifiers);
		    }
		}
		else
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											     total_Literal_cnt,
											     mutex_source_Identifiers);
		    
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														   sIntegerIndex(vertex_id),
														   sIntegerIndex(layer)),
											   mutex_source_Identifiers);
		}

	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id > 0)
	    {
		Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
										  total_Literal_cnt,
										  sSpecifiedStateIdentifier(&vertex_occupancy,
													    sIntegerIndex(vertex_id),
													    sIntegerIndex(0)),
										  init_robot_id);
		Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
									     total_Literal_cnt,
									     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
												     sIntegerIndex(vertex_id),
												     sIntegerIndex(0)));
	    }
	}
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id > 0)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_Equality(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedStateIdentifier(&vertex_occupancy,
													     sIntegerIndex(vertex_id),
													     sIntegerIndex(encoding_context.m_N_Layers - 1)),
										   goal_robot_id);
		    Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										 total_Literal_cnt,
										 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													 sIntegerIndex(vertex_id),
													 sIntegerIndex(encoding_context.m_N_Layers - 1)));
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    if (robot_IDs.size() > 1)
		    {
			sASSERT(false);
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
											  total_Literal_cnt,
											  sSpecifiedStateIdentifier(&vertex_occupancy,
														    sIntegerIndex(vertex_id),
														    sIntegerIndex(encoding_context.m_N_Layers - 1)),
											  *robot_IDs.begin());
			Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													     sIntegerIndex(vertex_id),
													     sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
	int N_cnf_Variables = encoding_context.m_variable_store.get_Last_CNFVariable() + aux_Variable_cnt - 1;

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot plural SAT encoding\n", sPRODUCT);
	fprintf(fw, "c %s\n", sCOPYRIGHT);
	fprintf(fw, "c number of layers = %d\n", encoding_context.m_N_Layers);
	fprintf(fw, "c number of visible variables = %d\n", N_cnf_Variables - aux_Variable_cnt);
	fprintf(fw, "c number of hidden variables = %d\n", aux_Variable_cnt);
	fprintf(fw, "c number of literals = %d\n", total_Literal_cnt);
	fprintf(fw, "c number of clauses = %d\n", Clause_cnt);
	fprintf(fw, "c number of propositional variables = %d\n", N_cnf_Variables);
	fprintf(fw, "c number of clauses / number of variables = %.3f\n", (double)Clause_cnt / N_cnf_Variables);
	fprintf(fw, "c number of literals / number of clauses = %.3f\n", (double)total_Literal_cnt /  Clause_cnt);
	to_Stream(fw, "c ");
#endif
	fprintf(fw, "p cnf %d %d\n", N_cnf_Variables, Clause_cnt);
	
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		/*
		Clause_cnt += encoding_context.m_clause_generator->generate_Alignment(fw,
										      sSpecifiedStateIdentifier(&vertex_occupancy,
													     sIntegerIndex(vertex_id),
													     sIntegerIndex(layer)));
		*/
	    }
	}
	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		Clause_cnt += encoding_context.m_bit_generator->generate_NonzeroImplication(fw,
											    sSpecifiedStateIdentifier(&vertex_occupancy,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											    sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														    sIntegerIndex(vertex_id),
														    sIntegerIndex(layer)));
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    Clause_cnt += encoding_context.m_bit_generator->generate_NonzeroImplication(fw,
											sSpecifiedStateIdentifier(&vertex_occupancy,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(encoding_context.m_N_Layers - 1)),
											sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														sIntegerIndex(vertex_id),
														sIntegerIndex(encoding_context.m_N_Layers - 1)));
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
		
		int out_neighbor_index = 0;
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;
		
		switch (m_goal_type)
		{
		case GOAL_TYPE_ARRANGEMENT:
		{
		    if (layer == encoding_context.m_N_Layers - 1)
		    {
			if (m_goal_arrangement.get_VertexOccupancy(vertex_id) != 0)
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
											   sIntegerIndex(out_neighbor_index)));
				
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegation(fw,
												   mutex_target_Identifiers);
			}
			else
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															     sIntegerIndex(out_neighbor_index)),
												     /*
												     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															     sIntegerIndex(vertex_id),
															     sIntegerIndex(layer)),
												     */
												     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															     sIntegerIndex((*out_neighbor)->m_target->m_id),
															     sIntegerIndex(layer)));
				Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
													     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																     sIntegerIndex(out_neighbor_index)),
													     sSpecifiedStateIdentifier(&vertex_occupancy,
																       sIntegerIndex(vertex_id),
																       sIntegerIndex(layer)),
													     sSpecifiedStateIdentifier(&vertex_occupancy,
																       sIntegerIndex((*out_neighbor)->m_target->m_id),
																       sIntegerIndex(layer)));
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index)));
			    
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															      sIntegerIndex(vertex_id),
															      sIntegerIndex(layer)),
												      mutex_target_Identifiers);			    
			    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
													mutex_target_Identifiers);   
			}
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														     sIntegerIndex(0)),
											     /*
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex(vertex_id),
														     sIntegerIndex(layer)),
											     */
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex(vertex_id),
														     sIntegerIndex(layer + 1)));
			Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															     sIntegerIndex(0)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex(vertex_id),
															       sIntegerIndex(layer)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex(vertex_id),
															       sIntegerIndex(layer + 1)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(0)));

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															 sIntegerIndex(out_neighbor_index + 1)),
												 /*
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex(vertex_id),
															 sIntegerIndex(layer)),
												 */
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex((*out_neighbor)->m_target->m_id),
															 sIntegerIndex(layer)));
			    Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
													 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																 sIntegerIndex(out_neighbor_index + 1)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex(vertex_id),
																   sIntegerIndex(layer)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex((*out_neighbor)->m_target->m_id),
																   sIntegerIndex(layer)));
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  mutex_target_Identifiers);
			Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												    mutex_target_Identifiers);
		    }
		    break;
		}
		case GOAL_TYPE_SPECIFICATION:
		{
		    if (layer == encoding_context.m_N_Layers - 1)
		    {
			if (!m_goal_specification.get_GoalCompatibility(vertex_id).empty())
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
											   sIntegerIndex(out_neighbor_index)));
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegation(fw,
												   mutex_target_Identifiers);
			}
			else
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															     sIntegerIndex(out_neighbor_index)),
												     /*
												     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															     sIntegerIndex(vertex_id),
															     sIntegerIndex(layer)),
												     */
												     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															     sIntegerIndex((*out_neighbor)->m_target->m_id),
															     sIntegerIndex(layer)));
				Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
													     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																     sIntegerIndex(out_neighbor_index)),
													     sSpecifiedStateIdentifier(&vertex_occupancy,
																       sIntegerIndex(vertex_id),
																       sIntegerIndex(layer)),
													     sSpecifiedStateIdentifier(&vertex_occupancy,
																       sIntegerIndex((*out_neighbor)->m_target->m_id),
																       sIntegerIndex(layer)));
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index)));
			    
				++out_neighbor_index;
			    }

			    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															      sIntegerIndex(vertex_id),
															      sIntegerIndex(layer)),
												      mutex_target_Identifiers);
			    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
													mutex_target_Identifiers);
			}
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														     sIntegerIndex(0)),
											     /*
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex(vertex_id),
														     sIntegerIndex(layer)),
											     */
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex(vertex_id),
														     sIntegerIndex(layer + 1)));

			Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															     sIntegerIndex(0)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex(vertex_id),
															       sIntegerIndex(layer)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex(vertex_id),
															       sIntegerIndex(layer + 1)));

			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(0)));

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															 sIntegerIndex(out_neighbor_index + 1)),
												 /*
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex(vertex_id),
															 sIntegerIndex(layer)),*/
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex((*out_neighbor)->m_target->m_id),
															 sIntegerIndex(layer)));

			    Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
													 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																 sIntegerIndex(out_neighbor_index + 1)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex(vertex_id),
																   sIntegerIndex(layer)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex((*out_neighbor)->m_target->m_id),
																   sIntegerIndex(layer)));

			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  mutex_target_Identifiers);
			Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												    mutex_target_Identifiers);
		    }
		    break;
		}
		default:
		{
		    sASSERT(false);
		    break;
		}
		}
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_source_Identifiers;

		if (layer > 0)
		{
		    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer - 1][vertex_id],
									       sIntegerIndex(0)));
		    const sVertex::Neighbors_list &in_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;
		    
			int in_neighbor_index = 0;
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    if ((*out_neighbor)->m_target->m_id == vertex_id)
			    {
				break;
			    }
			    ++in_neighbor_index;
			}
			if (layer == encoding_context.m_N_Layers - 1)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index)));
			}
			else
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index + 1)));
			}
		    }
		}
		else
		{
		    const sVertex::Neighbors_list &in_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;
		
		    for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;
		    
			int in_neighbor_index = 0;
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    if ((*out_neighbor)->m_target->m_id == vertex_id)
			    {
				break;
			    }
			    ++in_neighbor_index;
			}
			if (layer == encoding_context.m_N_Layers - 1)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index)));
			}
			else
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index + 1)));
			}
		    }
		}
		if (layer == 0)
		{
		    if (m_initial_arrangement.get_VertexOccupancy(vertex_id) == 0)
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												    mutex_source_Identifiers);
			
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  mutex_source_Identifiers);
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegation(fw,
											       mutex_source_Identifiers);
		    }
		}
		else
		{
		    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												mutex_source_Identifiers);
		    
		    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      mutex_source_Identifiers);
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id > 0)
	    {
		Clause_cnt += encoding_context.m_clause_generator->generate_Equality(fw,
										     sSpecifiedStateIdentifier(&vertex_occupancy,
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(0)),
										     init_robot_id);
		Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
										sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													sIntegerIndex(vertex_id),
													sIntegerIndex(0)));
	    }
	}

//	int robot_limit = 0;

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    printf("ARANGEMENT\n");
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id > 0)
		{
		    Clause_cnt += encoding_context.m_bit_generator->generate_Equality(fw,
										      sSpecifiedStateIdentifier(&vertex_occupancy,
														sIntegerIndex(vertex_id),
														sIntegerIndex(encoding_context.m_N_Layers - 1)),
										      goal_robot_id);
		    Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
										    sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													    sIntegerIndex(vertex_id),
													    sIntegerIndex(encoding_context.m_N_Layers - 1)));
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    printf("SPECIFICATION A1\n");
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    if (robot_IDs.size() > 1)
		    {
			// no goal setup
			sASSERT(false);
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_clause_generator->generate_Equality(fw,
											     sSpecifiedStateIdentifier(&vertex_occupancy,
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(encoding_context.m_N_Layers - 1)),
											     *robot_IDs.begin());
			Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
											sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														sIntegerIndex(vertex_id),
														sIntegerIndex(encoding_context.m_N_Layers - 1)));


/*
			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

			int out_neighbor_index = 0;
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[encoding_context.m_N_Layers - 1][vertex_id],
										       sIntegerIndex(out_neighbor_index)));
			    ++out_neighbor_index;
			}
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegation(fw,
											       mutex_target_Identifiers);
*/
		    }
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
    }


    void sMultirobotInstance::to_Stream_Plural2CNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableStateIdentifier vertex_occupancy(&encoding_context.m_variable_store,
						   "vertex_occupancy",
						   N_Robots + 1,
						   sIntegerScope(0, N_Vertices - 1),
						   sIntegerScope(0, encoding_context.m_N_Layers - 1));
	encoding_context.m_vertex_occupancy = vertex_occupancy;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy);

	sIndexableBitIdentifier vertex_occupancy_by_water(&encoding_context.m_variable_store,
							  "vertex_occupancy_by_water",
							  sIntegerScope(0, N_Vertices - 1),
							  sIntegerScope(0, encoding_context.m_N_Layers - 1));
	encoding_context.m_vertex_occupancy_by_water = vertex_occupancy_by_water;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_water);

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		Clause_cnt += encoding_context.m_clause_generator->count_Alignment(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedStateIdentifier(&vertex_occupancy,
													     sIntegerIndex(vertex_id),
													     sIntegerIndex(layer)));
	    }
	}
	encoding_context.m_edge_occupancy_by_water_.resize(encoding_context.m_N_Layers);

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    encoding_context.m_edge_occupancy_by_water_[layer].resize(N_Vertices);


	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sIndexableBitIdentifier edge_occupancy_by_water_(&encoding_context.m_variable_store,
								 "edge_occupancy_by_water-" + sInt_32_to_String(vertex_id) + "_" + sInt_32_to_String(layer),
								 sIntegerScope(0, m_sparse_environment.get_Vertex(vertex_id)->calc_NeighborCount() + m_environment.get_Vertex(vertex_id)->calc_NeighborCount()));
		encoding_context.m_edge_occupancy_by_water_[layer][vertex_id] = edge_occupancy_by_water_;
		encoding_context.register_TranslateIdentifier(encoding_context.m_edge_occupancy_by_water_[layer][vertex_id]);
	
		Clause_cnt += encoding_context.m_bit_generator->count_NonzeroImplication(aux_Variable_cnt,
											 total_Literal_cnt,
											 sSpecifiedStateIdentifier(&vertex_occupancy,
														   sIntegerIndex(vertex_id),
														   sIntegerIndex(layer)),
											 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														 sIntegerIndex(vertex_id),
														 sIntegerIndex(layer)));
	    }
	}
	encoding_context.m_edge_occupancy_by_water_[encoding_context.m_N_Layers - 1].resize(N_Vertices);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    sIndexableBitIdentifier edge_occupancy_by_water_(&encoding_context.m_variable_store,
							     "edge_occupancy_by_water-" + sInt_32_to_String(vertex_id) + "_" + sInt_32_to_String(encoding_context.m_N_Layers - 1),
							     sIntegerScope(0, m_sparse_environment.get_Vertex(vertex_id)->calc_NeighborCount() - 1));
	    encoding_context.m_edge_occupancy_by_water_[encoding_context.m_N_Layers - 1][vertex_id] = edge_occupancy_by_water_;
	    encoding_context.register_TranslateIdentifier(encoding_context.m_edge_occupancy_by_water_[encoding_context.m_N_Layers - 1][vertex_id]);

	    Clause_cnt += encoding_context.m_bit_generator->count_NonzeroImplication(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedStateIdentifier(&vertex_occupancy,
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)),
										     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													     sIntegerIndex(vertex_id),
													     sIntegerIndex(encoding_context.m_N_Layers - 1)));
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
		
		int out_neighbor_index = 0;
		const sVertex::Neighbors_list &out_Neighbors = m_sparse_environment.get_Vertex(vertex_id)->m_Neighbors;
		const sVertex::Neighbors_list &full_out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;
		
		switch (m_goal_type)
		{
		case GOAL_TYPE_ARRANGEMENT:
		{
		    if (layer == encoding_context.m_N_Layers - 1)
		    {
			if (m_goal_arrangement.get_VertexOccupancy(vertex_id) != 0)
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
											   sIntegerIndex(out_neighbor_index)));
				
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->count_MultiNegation(aux_Variable_cnt,
												total_Literal_cnt,
												mutex_target_Identifiers);
			}
			else
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
												  total_Literal_cnt,
												  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															  sIntegerIndex(out_neighbor_index)),
												  /*
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  */
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex((*out_neighbor)->m_target->m_id),
															  sIntegerIndex(layer)));
				Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
													  total_Literal_cnt,
													  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																  sIntegerIndex(out_neighbor_index)),
													  sSpecifiedStateIdentifier(&vertex_occupancy,
																    sIntegerIndex(vertex_id),
																    sIntegerIndex(layer)),
													  sSpecifiedStateIdentifier(&vertex_occupancy,
																    sIntegerIndex((*out_neighbor)->m_target->m_id),
																    sIntegerIndex(layer)));
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index)));
			    
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
												   total_Literal_cnt,
												   sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															   sIntegerIndex(vertex_id),
															   sIntegerIndex(layer)),
												   mutex_target_Identifiers);
			    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
												     total_Literal_cnt,
												     mutex_target_Identifiers);   
			}
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											  total_Literal_cnt,
											  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														  sIntegerIndex(0)),
											  /*
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer)),
											  */
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer + 1)));
			Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
												  total_Literal_cnt,
												  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															  sIntegerIndex(0)),
												  sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex(vertex_id),
															    sIntegerIndex(layer)),
												  sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex(vertex_id),
															    sIntegerIndex(layer + 1)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(0)));

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											      total_Literal_cnt,
											      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														      sIntegerIndex(out_neighbor_index + 1)),
											      /*
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      */
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex((*out_neighbor)->m_target->m_id),
														      sIntegerIndex(layer)));
			    Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
												      total_Literal_cnt,
												      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															      sIntegerIndex(out_neighbor_index + 1)),
												      sSpecifiedStateIdentifier(&vertex_occupancy,
																sIntegerIndex(vertex_id),
																sIntegerIndex(layer)),
												      sSpecifiedStateIdentifier(&vertex_occupancy,
																sIntegerIndex((*out_neighbor)->m_target->m_id),
																sIntegerIndex(layer)));
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			for (sVertex::Neighbors_list::const_iterator out_neighbor = full_out_Neighbors.begin(); out_neighbor != full_out_Neighbors.end(); ++out_neighbor)
			{
			    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											      total_Literal_cnt,
											      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														      sIntegerIndex(out_neighbor_index + 1)),
											      /*
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      */
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex((*out_neighbor)->m_target->m_id),
														      sIntegerIndex(layer + 1)));
			    Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
												      total_Literal_cnt,
												      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															      sIntegerIndex(out_neighbor_index + 1)),
												      sSpecifiedStateIdentifier(&vertex_occupancy,
																sIntegerIndex(vertex_id),
																sIntegerIndex(layer)),
												      sSpecifiedStateIdentifier(&vertex_occupancy,
																sIntegerIndex((*out_neighbor)->m_target->m_id),
																sIntegerIndex(layer + 1)));
			    Clause_cnt += encoding_context.m_bit_generator->count_BiangleMutex(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														       sIntegerIndex(out_neighbor_index + 1)),
											       sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														       sIntegerIndex((*out_neighbor)->m_target->m_id),
														       sIntegerIndex(layer + 1)));
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(layer)),
											       mutex_target_Identifiers);
			Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
												 total_Literal_cnt,
												 mutex_target_Identifiers);   
		    }
		    break;
		}
		case GOAL_TYPE_SPECIFICATION:
		{
		    if (layer == encoding_context.m_N_Layers - 1)
		    {
			if (!m_goal_specification.get_GoalCompatibility(vertex_id).empty())
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
											   sIntegerIndex(out_neighbor_index)));
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->count_MultiNegation(aux_Variable_cnt,
												total_Literal_cnt,
												mutex_target_Identifiers);
			}
			else
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
												  total_Literal_cnt,
												  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															  sIntegerIndex(out_neighbor_index)),
												  /*
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  */
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex((*out_neighbor)->m_target->m_id),
															  sIntegerIndex(layer)));
				Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
													  total_Literal_cnt,
													  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																  sIntegerIndex(out_neighbor_index)),
													  sSpecifiedStateIdentifier(&vertex_occupancy,
																    sIntegerIndex(vertex_id),
																    sIntegerIndex(layer)),
													  sSpecifiedStateIdentifier(&vertex_occupancy,
																    sIntegerIndex((*out_neighbor)->m_target->m_id),
																    sIntegerIndex(layer)));
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index)));
			    
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
												   total_Literal_cnt,
												   sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															   sIntegerIndex(vertex_id),
															   sIntegerIndex(layer)),
												   mutex_target_Identifiers);
			    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
												     total_Literal_cnt,
												     mutex_target_Identifiers);   
			}
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											  total_Literal_cnt,
											  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														  sIntegerIndex(0)),
											  /*
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer)),
											  */
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer + 1)));
			Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
												  total_Literal_cnt,
												  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															  sIntegerIndex(0)),
												  sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex(vertex_id),
															    sIntegerIndex(layer)),
												  sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex(vertex_id),
															    sIntegerIndex(layer + 1)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(0)));

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											      total_Literal_cnt,
											      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														      sIntegerIndex(out_neighbor_index + 1)),
											      /*
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      */
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex((*out_neighbor)->m_target->m_id),
														      sIntegerIndex(layer)));
			    Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
												      total_Literal_cnt,
												      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															      sIntegerIndex(out_neighbor_index + 1)),
												      sSpecifiedStateIdentifier(&vertex_occupancy,
																sIntegerIndex(vertex_id),
																sIntegerIndex(layer)),
												      sSpecifiedStateIdentifier(&vertex_occupancy,
																sIntegerIndex((*out_neighbor)->m_target->m_id),
																sIntegerIndex(layer)));
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			for (sVertex::Neighbors_list::const_iterator out_neighbor = full_out_Neighbors.begin(); out_neighbor != full_out_Neighbors.end(); ++out_neighbor)
			{
			    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											      total_Literal_cnt,
											      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														      sIntegerIndex(out_neighbor_index + 1)),
											      /*
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      */
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex((*out_neighbor)->m_target->m_id),
														      sIntegerIndex(layer + 1)));
			    Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
												      total_Literal_cnt,
												      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															      sIntegerIndex(out_neighbor_index + 1)),
												      sSpecifiedStateIdentifier(&vertex_occupancy,
																sIntegerIndex(vertex_id),
																sIntegerIndex(layer)),
												      sSpecifiedStateIdentifier(&vertex_occupancy,
																sIntegerIndex((*out_neighbor)->m_target->m_id),
																sIntegerIndex(layer + 1)));
			    Clause_cnt += encoding_context.m_bit_generator->count_BiangleMutex(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														       sIntegerIndex(out_neighbor_index + 1)),
											       sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														       sIntegerIndex((*out_neighbor)->m_target->m_id),
														       sIntegerIndex(layer + 1)));
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(layer)),
											       mutex_target_Identifiers);
			Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
												 total_Literal_cnt,
												 mutex_target_Identifiers);   
		    }
		    break;
		}
		default:
		{
		    sASSERT(false);
		    break;
		}
		}
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_source_Identifiers;

		if (layer > 0)
		{
		    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer - 1][vertex_id],
									       sIntegerIndex(0)));
		    const sVertex::Neighbors_list &in_Neighbors = m_sparse_environment.get_Vertex(vertex_id)->m_in_Neighbors;
		    const sVertex::Neighbors_list &full_in_Neighbors = m_environment.get_Vertex(vertex_id)->m_in_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_sparse_environment.get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;
		    
			int in_neighbor_index = 0;
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    if ((*out_neighbor)->m_target->m_id == vertex_id)
			    {
				break;
			    }
			    ++in_neighbor_index;
			}
			if (layer == encoding_context.m_N_Layers - 1)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index)));
			}
			else
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index + 1)));
			}
		    }
		    for (sVertex::Neighbors_list::const_iterator in_neighbor = full_in_Neighbors.begin(); in_neighbor != full_in_Neighbors.end(); ++in_neighbor)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_sparse_environment.get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;
			const sVertex::Neighbors_list &full_out_Neighbors = m_environment.get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;
			int out_neighbor_count = out_Neighbors.size();
		    
			int in_neighbor_index = 0;
			for (sVertex::Neighbors_list::const_iterator out_neighbor = full_out_Neighbors.begin(); out_neighbor != full_out_Neighbors.end(); ++out_neighbor)
			{
			    if ((*out_neighbor)->m_target->m_id == vertex_id)
			    {
				break;
			    }
			    ++in_neighbor_index;
			}
			mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer - 1][(*in_neighbor)->m_target->m_id],
										   sIntegerIndex(out_neighbor_count + in_neighbor_index + 1)));
		    }
		}
		else
		{
		    const sVertex::Neighbors_list &in_Neighbors = m_sparse_environment.get_Vertex(vertex_id)->m_Neighbors;
		
		    for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_sparse_environment.get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;
		    
			int in_neighbor_index = 0;
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    if ((*out_neighbor)->m_target->m_id == vertex_id)
			    {
				break;
			    }
			    ++in_neighbor_index;
			}
			if (layer == encoding_context.m_N_Layers - 1)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index)));
			}
			else
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index + 1)));
			}
		    }
		}

		if (layer == 0)
		{
		    if (m_initial_arrangement.get_VertexOccupancy(vertex_id) == 0)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
												 total_Literal_cnt,
												 mutex_source_Identifiers);
			
			Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(layer)),
											       mutex_source_Identifiers);
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_MultiNegation(aux_Variable_cnt,
											    total_Literal_cnt,
											    mutex_source_Identifiers);
		    }
		}
		else
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											     total_Literal_cnt,
											     mutex_source_Identifiers);
		    
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														   sIntegerIndex(vertex_id),
														   sIntegerIndex(layer)),
											   mutex_source_Identifiers);
		}

	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id > 0)
	    {
		Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
										  total_Literal_cnt,
										  sSpecifiedStateIdentifier(&vertex_occupancy,
													    sIntegerIndex(vertex_id),
													    sIntegerIndex(0)),
										  init_robot_id);
		Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
									     total_Literal_cnt,
									     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
												     sIntegerIndex(vertex_id),
												     sIntegerIndex(0)));
	    }
	}
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id > 0)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_Equality(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedStateIdentifier(&vertex_occupancy,
													     sIntegerIndex(vertex_id),
													     sIntegerIndex(encoding_context.m_N_Layers - 1)),
										   goal_robot_id);
		    Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										 total_Literal_cnt,
										 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													 sIntegerIndex(vertex_id),
													 sIntegerIndex(encoding_context.m_N_Layers - 1)));
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    if (robot_IDs.size() > 1)
		    {
			sASSERT(false);
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
											  total_Literal_cnt,
											  sSpecifiedStateIdentifier(&vertex_occupancy,
														    sIntegerIndex(vertex_id),
														    sIntegerIndex(encoding_context.m_N_Layers - 1)),
											  *robot_IDs.begin());
			Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													     sIntegerIndex(vertex_id),
													     sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}       
	int N_cnf_Variables = encoding_context.m_variable_store.get_Last_CNFVariable() + aux_Variable_cnt - 1;

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot plural-2 SAT encoding\n", sPRODUCT);
	fprintf(fw, "c %s\n", sCOPYRIGHT);
	fprintf(fw, "c number of layers = %d\n", encoding_context.m_N_Layers);
	fprintf(fw, "c number of visible variables = %d\n", N_cnf_Variables - aux_Variable_cnt);
	fprintf(fw, "c number of hidden variables = %d\n", aux_Variable_cnt);
	fprintf(fw, "c number of literals = %d\n", total_Literal_cnt);
	fprintf(fw, "c number of clauses = %d\n", Clause_cnt);
	fprintf(fw, "c number of propositional variables = %d\n", N_cnf_Variables);
	fprintf(fw, "c number of clauses / number of variables = %.3f\n", (double)Clause_cnt / N_cnf_Variables);
	fprintf(fw, "c number of literals / number of clauses = %.3f\n", (double)total_Literal_cnt /  Clause_cnt);
	to_Stream(fw, "c ");
#endif
	fprintf(fw, "p cnf %d %d\n", N_cnf_Variables, Clause_cnt);
	
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {	       
		Clause_cnt += encoding_context.m_clause_generator->generate_Alignment(fw,
										      sSpecifiedStateIdentifier(&vertex_occupancy,
													     sIntegerIndex(vertex_id),
													     sIntegerIndex(layer)));
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		Clause_cnt += encoding_context.m_bit_generator->generate_NonzeroImplication(fw,
											    sSpecifiedStateIdentifier(&vertex_occupancy,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											    sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														    sIntegerIndex(vertex_id),
														    sIntegerIndex(layer)));
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    Clause_cnt += encoding_context.m_bit_generator->generate_NonzeroImplication(fw,
											sSpecifiedStateIdentifier(&vertex_occupancy,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(encoding_context.m_N_Layers - 1)),
											sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														sIntegerIndex(vertex_id),
														sIntegerIndex(encoding_context.m_N_Layers - 1)));
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
		
		int out_neighbor_index = 0;
		const sVertex::Neighbors_list &out_Neighbors = m_sparse_environment.get_Vertex(vertex_id)->m_Neighbors;
		const sVertex::Neighbors_list &full_out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;
		
		switch (m_goal_type)
		{
		case GOAL_TYPE_ARRANGEMENT:
		{
		    if (layer == encoding_context.m_N_Layers - 1)
		    {
			if (m_goal_arrangement.get_VertexOccupancy(vertex_id) != 0)
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
											   sIntegerIndex(out_neighbor_index)));
				
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegation(fw,
												   mutex_target_Identifiers);
			}
			else
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															     sIntegerIndex(out_neighbor_index)),
												     /**/
												     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															     sIntegerIndex(vertex_id),
															     sIntegerIndex(layer)),
												     /**/
												     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															     sIntegerIndex((*out_neighbor)->m_target->m_id),
															     sIntegerIndex(layer)));
				Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
													     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																     sIntegerIndex(out_neighbor_index)),
													     sSpecifiedStateIdentifier(&vertex_occupancy,
																       sIntegerIndex(vertex_id),
																       sIntegerIndex(layer)),
													     sSpecifiedStateIdentifier(&vertex_occupancy,
																       sIntegerIndex((*out_neighbor)->m_target->m_id),
																       sIntegerIndex(layer)));
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index)));
			    
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															      sIntegerIndex(vertex_id),
															      sIntegerIndex(layer)),
												      mutex_target_Identifiers);			    
			    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
													mutex_target_Identifiers);   
			}
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														     sIntegerIndex(0)),
											     /**/
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex(vertex_id),
														     sIntegerIndex(layer)),
											     /**/
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex(vertex_id),
														     sIntegerIndex(layer + 1)));
			Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															     sIntegerIndex(0)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex(vertex_id),
															       sIntegerIndex(layer)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex(vertex_id),
															       sIntegerIndex(layer + 1)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(0)));

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															 sIntegerIndex(out_neighbor_index + 1)),
												 /**/
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex(vertex_id),
															 sIntegerIndex(layer)),
												 /**/
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex((*out_neighbor)->m_target->m_id),
															 sIntegerIndex(layer)));
			    Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
													 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																 sIntegerIndex(out_neighbor_index + 1)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex(vertex_id),
																   sIntegerIndex(layer)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex((*out_neighbor)->m_target->m_id),
																   sIntegerIndex(layer)));
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			for (sVertex::Neighbors_list::const_iterator out_neighbor = full_out_Neighbors.begin(); out_neighbor != full_out_Neighbors.end(); ++out_neighbor)
			{
			    Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															 sIntegerIndex(out_neighbor_index + 1)),
												 /**/
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex(vertex_id),
															 sIntegerIndex(layer)),
												 /**/
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex((*out_neighbor)->m_target->m_id),
															 sIntegerIndex(layer + 1)));
			    Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
													 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																 sIntegerIndex(out_neighbor_index + 1)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex(vertex_id),
																   sIntegerIndex(layer)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex((*out_neighbor)->m_target->m_id),
																   sIntegerIndex(layer + 1)));			    
			    Clause_cnt += encoding_context.m_bit_generator->generate_BiangleMutex(fw,
												  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															  sIntegerIndex(out_neighbor_index + 1)),
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer + 1)));

			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  mutex_target_Identifiers);
			Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												    mutex_target_Identifiers);
		    }
		    break;
		}
		case GOAL_TYPE_SPECIFICATION:
		{
		    if (layer == encoding_context.m_N_Layers - 1)
		    {
			if (!m_goal_specification.get_GoalCompatibility(vertex_id).empty())
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
											   sIntegerIndex(out_neighbor_index)));
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegation(fw,
												   mutex_target_Identifiers);
			}
			else
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															     sIntegerIndex(out_neighbor_index)),
												     /**/
												     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															     sIntegerIndex(vertex_id),
															     sIntegerIndex(layer)),
												     /**/
												     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															     sIntegerIndex((*out_neighbor)->m_target->m_id),
															     sIntegerIndex(layer)));
				Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
													     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																     sIntegerIndex(out_neighbor_index)),
													     sSpecifiedStateIdentifier(&vertex_occupancy,
																       sIntegerIndex(vertex_id),
																       sIntegerIndex(layer)),
													     sSpecifiedStateIdentifier(&vertex_occupancy,
																       sIntegerIndex((*out_neighbor)->m_target->m_id),
																       sIntegerIndex(layer)));
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index)));
			    
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															      sIntegerIndex(vertex_id),
															      sIntegerIndex(layer)),
												      mutex_target_Identifiers);
			    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
													mutex_target_Identifiers);
			}
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														     sIntegerIndex(0)),
											     /**/
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex(vertex_id),
														     sIntegerIndex(layer)),
											     /**/
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex(vertex_id),
														     sIntegerIndex(layer + 1)));
			Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															     sIntegerIndex(0)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex(vertex_id),
															       sIntegerIndex(layer)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex(vertex_id),
															       sIntegerIndex(layer + 1)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(0)));

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															 sIntegerIndex(out_neighbor_index + 1)),
												 /**/
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex(vertex_id),
															 sIntegerIndex(layer)),
												 /**/
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex((*out_neighbor)->m_target->m_id),
															 sIntegerIndex(layer)));
			    Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
													 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																 sIntegerIndex(out_neighbor_index + 1)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex(vertex_id),
																   sIntegerIndex(layer)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex((*out_neighbor)->m_target->m_id),
																   sIntegerIndex(layer)));
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			for (sVertex::Neighbors_list::const_iterator out_neighbor = full_out_Neighbors.begin(); out_neighbor != full_out_Neighbors.end(); ++out_neighbor)
			{
			    Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															 sIntegerIndex(out_neighbor_index + 1)),
												 /**/
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex(vertex_id),
															 sIntegerIndex(layer)),
												 /**/
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex((*out_neighbor)->m_target->m_id),
															 sIntegerIndex(layer + 1)));
			    Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
													 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																 sIntegerIndex(out_neighbor_index + 1)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex(vertex_id),
																   sIntegerIndex(layer)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex((*out_neighbor)->m_target->m_id),
																   sIntegerIndex(layer + 1)));
			    Clause_cnt += encoding_context.m_bit_generator->generate_BiangleMutex(fw,
												  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															  sIntegerIndex(out_neighbor_index + 1)),
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer + 1)));

			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  mutex_target_Identifiers);
			Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												    mutex_target_Identifiers);
		    }
		    break;
		}
		default:
		{
		    sASSERT(false);
		    break;
		}
		}
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_source_Identifiers;

		if (layer > 0)
		{
		    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer - 1][vertex_id],
									       sIntegerIndex(0)));
		    const sVertex::Neighbors_list &in_Neighbors = m_sparse_environment.get_Vertex(vertex_id)->m_in_Neighbors;
		    const sVertex::Neighbors_list &full_in_Neighbors = m_environment.get_Vertex(vertex_id)->m_in_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_sparse_environment.get_Vertex((*in_neighbor)->m_source->m_id)->m_out_Neighbors;
		    
			int in_neighbor_index = 0;
#ifdef sDEBUG
			bool breaked = false;
#endif
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    if ((*out_neighbor)->m_target->m_id == vertex_id)
//			    if ((*out_neighbor)->m_target->m_id == vertex_id)
			    {
#ifdef sDEBUG
				breaked = true;
#endif	
				break;
			    }
			    ++in_neighbor_index;
			}
			if (!out_Neighbors.empty())
			{
			    sASSERT(breaked);
			}

			if (layer == encoding_context.m_N_Layers - 1)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_source->m_id],
										       sIntegerIndex(in_neighbor_index)));
			}
			else
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_source->m_id],
										       sIntegerIndex(in_neighbor_index + 1)));
			}
		    }

		    for (sVertex::Neighbors_list::const_iterator in_neighbor = full_in_Neighbors.begin(); in_neighbor != full_in_Neighbors.end(); ++in_neighbor)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_sparse_environment.get_Vertex((*in_neighbor)->m_source->m_id)->m_out_Neighbors;
			const sVertex::Neighbors_list &full_out_Neighbors = m_environment.get_Vertex((*in_neighbor)->m_source->m_id)->m_out_Neighbors;
			int out_neighbor_count = out_Neighbors.size();
		    
			int in_neighbor_index = 0;
#ifdef sDEBUG
			bool breaked = false;
#endif
			for (sVertex::Neighbors_list::const_iterator out_neighbor = full_out_Neighbors.begin(); out_neighbor != full_out_Neighbors.end(); ++out_neighbor)
			{
			    if ((*out_neighbor)->m_target->m_id == vertex_id)
			    {
#ifdef sDEBUG
				breaked = true;
#endif	
				break;
			    }
			    ++in_neighbor_index;
			}
			if (!full_out_Neighbors.empty())
			{
			    sASSERT(breaked);
			}

			mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer - 1][(*in_neighbor)->m_source->m_id],
										   sIntegerIndex(out_neighbor_count + in_neighbor_index + 1)));
		    }
		}
		else
		{
		    const sVertex::Neighbors_list &in_Neighbors = m_sparse_environment.get_Vertex(vertex_id)->m_in_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_sparse_environment.get_Vertex((*in_neighbor)->m_source->m_id)->m_out_Neighbors;
		    
			int in_neighbor_index = 0;
#ifdef sDEBUG
			bool breaked = false;
#endif
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
//			    if ((*out_neighbor)->m_edge->m_arc_vu.m_target->m_id == vertex_id)
			    if ((*out_neighbor)->m_target->m_id == vertex_id)
			    {
#ifdef sDEBUG
				breaked = true;
#endif	
				break;
			    }
			    ++in_neighbor_index;
			}
			if (!out_Neighbors.empty())
			{
			    sASSERT(breaked);
			}

			if (layer == encoding_context.m_N_Layers - 1)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_source->m_id],
										       sIntegerIndex(in_neighbor_index)));
			}
			else
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_source->m_id],
										       sIntegerIndex(in_neighbor_index + 1)));
			}
		    }
		}
		if (layer == 0)
		{
		    if (m_initial_arrangement.get_VertexOccupancy(vertex_id) == 0)
		    {
			if (!mutex_source_Identifiers.empty())
			{
			    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
													mutex_source_Identifiers);
			    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															      sIntegerIndex(vertex_id),
															      sIntegerIndex(layer)),
												      mutex_source_Identifiers);
			}
		    }
		    else
		    {
			if (!mutex_source_Identifiers.empty())
			{
			    Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegation(fw,
												   mutex_source_Identifiers);
			}
		    }
		}
		else
		{
		    if (!mutex_source_Identifiers.empty())
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												    mutex_source_Identifiers);
			
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  mutex_source_Identifiers);
		    }
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id > 0)
	    {
		Clause_cnt += encoding_context.m_clause_generator->generate_Equality(fw,
										     sSpecifiedStateIdentifier(&vertex_occupancy,
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(0)),
										     init_robot_id);

		Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
										sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													sIntegerIndex(vertex_id),
													sIntegerIndex(0)));
	    }
	    else
	    {
/*
		Clause_cnt += encoding_context.m_clause_generator->generate_Equality(fw,
										     sSpecifiedStateIdentifier(&vertex_occupancy,
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(0)),
										     init_robot_id);

		Clause_cnt += encoding_context.m_bit_generator->generate_BitUnset(fw,
										sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													sIntegerIndex(vertex_id),
													sIntegerIndex(0)));
*/
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id > 0)
		{
		    Clause_cnt += encoding_context.m_bit_generator->generate_Equality(fw,
										      sSpecifiedStateIdentifier(&vertex_occupancy,
														sIntegerIndex(vertex_id),
														sIntegerIndex(encoding_context.m_N_Layers - 1)),
										      goal_robot_id);

		    Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
										    sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													    sIntegerIndex(vertex_id),
													    sIntegerIndex(encoding_context.m_N_Layers - 1)));
		}
		else
		{
/*
		    Clause_cnt += encoding_context.m_bit_generator->generate_Equality(fw,
										      sSpecifiedStateIdentifier(&vertex_occupancy,
														sIntegerIndex(vertex_id),
														sIntegerIndex(encoding_context.m_N_Layers - 1)),
										      goal_robot_id);

		    Clause_cnt += encoding_context.m_bit_generator->generate_BitUnset(fw,
										    sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													    sIntegerIndex(vertex_id),
													    sIntegerIndex(encoding_context.m_N_Layers - 1)));
*/
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    if (robot_IDs.size() > 1)
		    {
			sASSERT(false);
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_clause_generator->generate_Equality(fw,
											     sSpecifiedStateIdentifier(&vertex_occupancy,
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(encoding_context.m_N_Layers - 1)),
											     *robot_IDs.begin());
			Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
											sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														sIntegerIndex(vertex_id),
														sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
/*
		    Clause_cnt += encoding_context.m_clause_generator->generate_Equality(fw,
											 sSpecifiedStateIdentifier(&vertex_occupancy,
														   sIntegerIndex(vertex_id),
														   sIntegerIndex(encoding_context.m_N_Layers - 1)),
											 *robot_IDs.begin());
		    Clause_cnt += encoding_context.m_bit_generator->generate_BitUnset(fw,
										      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													      sIntegerIndex(vertex_id),
													      sIntegerIndex(encoding_context.m_N_Layers - 1)));
*/
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
    }


    void sMultirobotInstance::to_Stream_HeightedCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	
	build_HeightedEnvironments_(encoding_context.m_max_total_cost);

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	int N_Layers = m_heighted_Environments.size();

	sIndexableStateIdentifier vertex_occupancy(&encoding_context.m_variable_store,
						   "vertex_occupancy",
						   N_Robots + 1,
						   sIntegerScope(0, N_Vertices - 1),
						   sIntegerScope(0, N_Layers - 1));
	encoding_context.m_vertex_occupancy = vertex_occupancy;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy);

	sIndexableBitIdentifier vertex_occupancy_by_water(&encoding_context.m_variable_store,
							  "vertex_occupancy_by_water",
							  sIntegerScope(0, N_Vertices - 1),
							  sIntegerScope(0, N_Layers - 1));
	encoding_context.m_vertex_occupancy_by_water = vertex_occupancy_by_water;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_water);

	for (int layer = 0; layer < N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		/*
		Clause_cnt += encoding_context.m_clause_generator->count_Alignment(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedStateIdentifier(&vertex_occupancy,
													     sIntegerIndex(vertex_id),
													     sIntegerIndex(layer)));
		*/
	    }
	}
	encoding_context.m_edge_occupancy_by_water_.resize(N_Layers);

	for (int layer = 0; layer < N_Layers - 1; ++layer)
	{
	    encoding_context.m_edge_occupancy_by_water_[layer].resize(N_Vertices);


	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sIndexableBitIdentifier edge_occupancy_by_water_(&encoding_context.m_variable_store,
								 "edge_occupancy_by_water-" + sInt_32_to_String(vertex_id) + "_" + sInt_32_to_String(layer),
								 sIntegerScope(0, m_heighted_Environments[layer].get_Vertex(vertex_id)->calc_NeighborCount()));
		encoding_context.m_edge_occupancy_by_water_[layer][vertex_id] = edge_occupancy_by_water_;
		encoding_context.register_TranslateIdentifier(encoding_context.m_edge_occupancy_by_water_[layer][vertex_id]);
		
		Clause_cnt += encoding_context.m_bit_generator->count_NonzeroImplication(aux_Variable_cnt,
											 total_Literal_cnt,
											 sSpecifiedStateIdentifier(&vertex_occupancy,
														   sIntegerIndex(vertex_id),
														   sIntegerIndex(layer)),
											 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														 sIntegerIndex(vertex_id),
														 sIntegerIndex(layer)));
	    }
	}
	encoding_context.m_edge_occupancy_by_water_[N_Layers - 1].resize(N_Vertices);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    sIndexableBitIdentifier edge_occupancy_by_water_(&encoding_context.m_variable_store,
							    "edge_occupancy_by_water-" + sInt_32_to_String(vertex_id) + "_" + sInt_32_to_String(N_Layers - 1),
							    sIntegerScope(0, m_heighted_Environments[N_Layers - 1].get_Vertex(vertex_id)->calc_NeighborCount() - 1));
	    encoding_context.m_edge_occupancy_by_water_[N_Layers - 1][vertex_id] = edge_occupancy_by_water_;
	    encoding_context.register_TranslateIdentifier(encoding_context.m_edge_occupancy_by_water_[N_Layers - 1][vertex_id]);

	    Clause_cnt += encoding_context.m_bit_generator->count_NonzeroImplication(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedStateIdentifier(&vertex_occupancy,
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(N_Layers - 1)),
										     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													     sIntegerIndex(vertex_id),
													     sIntegerIndex(N_Layers - 1)));
	}
/*
	int soft_Clause_cnt = 0;

	for (int layer = 0; layer < N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int neighbor_idx = 0; neighbor_idx < m_heighted_Environments[layer].get_Vertex(vertex_id)->calc_NeighborCount(); ++neighbor_idx)
		{
		    int cnt = encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										 total_Literal_cnt,
										 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
													 sIntegerIndex(neighbor_idx)));
		    Clause_cnt += cnt;
		    soft_Clause_cnt += cnt;
		}
	    }
	}
*/
	for (int layer = 0; layer < N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
		
		int out_neighbor_index = 0;
		const sVertex::Neighbors_list &out_Neighbors = m_heighted_Environments[layer].get_Vertex(vertex_id)->m_Neighbors;
		
		switch (m_goal_type)
		{
		case GOAL_TYPE_ARRANGEMENT:
		{
		    if (layer == N_Layers - 1)
		    {
			if (m_goal_arrangement.get_VertexOccupancy(vertex_id) != 0)
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
											   sIntegerIndex(out_neighbor_index)));
				
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->count_MultiNegation(aux_Variable_cnt,
												total_Literal_cnt,
												mutex_target_Identifiers);
			}
			else
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
												  total_Literal_cnt,
												  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															  sIntegerIndex(out_neighbor_index)),
												  /*
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  */
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex((*out_neighbor)->m_target->m_id),
															  sIntegerIndex(layer)));
				Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
													  total_Literal_cnt,
													  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																  sIntegerIndex(out_neighbor_index)),
													  sSpecifiedStateIdentifier(&vertex_occupancy,
																    sIntegerIndex(vertex_id),
																    sIntegerIndex(layer)),
													  sSpecifiedStateIdentifier(&vertex_occupancy,
																    sIntegerIndex((*out_neighbor)->m_target->m_id),
																    sIntegerIndex(layer)));
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index)));
			    
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
												   total_Literal_cnt,
												   sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															   sIntegerIndex(vertex_id),
															   sIntegerIndex(layer)),
												   mutex_target_Identifiers);
			    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
												     total_Literal_cnt,
												     mutex_target_Identifiers);   
			}
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											  total_Literal_cnt,
											  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														  sIntegerIndex(0)),
											  /*
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer)),
											  */
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer + 1)));
			Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
												  total_Literal_cnt,
												  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															  sIntegerIndex(0)),
												  sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex(vertex_id),
															    sIntegerIndex(layer)),
												  sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex(vertex_id),
															    sIntegerIndex(layer + 1)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(0)));

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											      total_Literal_cnt,
											      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														      sIntegerIndex(out_neighbor_index + 1)),
											      /*
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      */
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex((*out_neighbor)->m_target->m_id),
														      sIntegerIndex(layer)));
			    Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
												      total_Literal_cnt,
												      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															      sIntegerIndex(out_neighbor_index + 1)),
												      sSpecifiedStateIdentifier(&vertex_occupancy,
																sIntegerIndex(vertex_id),
																sIntegerIndex(layer)),
												      sSpecifiedStateIdentifier(&vertex_occupancy,
																sIntegerIndex((*out_neighbor)->m_target->m_id),
																sIntegerIndex(layer)));
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(layer)),
											       mutex_target_Identifiers);
			Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
												 total_Literal_cnt,
												 mutex_target_Identifiers);   
		    }
		    break;
		}
		case GOAL_TYPE_SPECIFICATION:
		{
		    if (layer == N_Layers - 1)
		    {
			if (!m_goal_specification.get_GoalCompatibility(vertex_id).empty())
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
											   sIntegerIndex(out_neighbor_index)));
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->count_MultiNegation(aux_Variable_cnt,
												total_Literal_cnt,
												mutex_target_Identifiers);
			}
			else
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
												  total_Literal_cnt,
												  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															  sIntegerIndex(out_neighbor_index)),
												  /*
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  */
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex((*out_neighbor)->m_target->m_id),
															  sIntegerIndex(layer)));
				Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
													  total_Literal_cnt,
													  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																  sIntegerIndex(out_neighbor_index)),
													  sSpecifiedStateIdentifier(&vertex_occupancy,
																    sIntegerIndex(vertex_id),
																    sIntegerIndex(layer)),
													  sSpecifiedStateIdentifier(&vertex_occupancy,
																    sIntegerIndex((*out_neighbor)->m_target->m_id),
																    sIntegerIndex(layer)));
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index)));
			    
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
												   total_Literal_cnt,
												   sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															   sIntegerIndex(vertex_id),
															   sIntegerIndex(layer)),
												   mutex_target_Identifiers);
			    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
												     total_Literal_cnt,
												     mutex_target_Identifiers);   
			}
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											  total_Literal_cnt,
											  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														  sIntegerIndex(0)),
											  /*
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer)),
											  */
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer + 1)));
			Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
												  total_Literal_cnt,
												  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															  sIntegerIndex(0)),
												  sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex(vertex_id),
															    sIntegerIndex(layer)),
												  sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex(vertex_id),
															    sIntegerIndex(layer + 1)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(0)));

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											      total_Literal_cnt,
											      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														      sIntegerIndex(out_neighbor_index + 1)),
											      /*
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      */
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex((*out_neighbor)->m_target->m_id),
														      sIntegerIndex(layer)));
			    Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
												      total_Literal_cnt,
												      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															      sIntegerIndex(out_neighbor_index + 1)),
												      sSpecifiedStateIdentifier(&vertex_occupancy,
																sIntegerIndex(vertex_id),
																sIntegerIndex(layer)),
												      sSpecifiedStateIdentifier(&vertex_occupancy,
																sIntegerIndex((*out_neighbor)->m_target->m_id),
																sIntegerIndex(layer)));
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(layer)),
											       mutex_target_Identifiers);
			Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
												 total_Literal_cnt,
												 mutex_target_Identifiers);   
		    }
		    break;
		}
		default:
		{
		    sASSERT(false);
		    break;
		}
		}
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_source_Identifiers;

		if (layer > 0)
		{
		    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer - 1][vertex_id],
									       sIntegerIndex(0)));
		    const sVertex::Neighbors_list &in_Neighbors = m_heighted_Environments[layer].get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_heighted_Environments[layer].get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;
		    
			int in_neighbor_index = 0;
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    if ((*out_neighbor)->m_target->m_id == vertex_id)
			    {
				break;
			    }
			    ++in_neighbor_index;
			}
			if (layer == N_Layers - 1)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index)));
			}
			else
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index + 1)));
			}
		    }
		}
		else
		{
		    const sVertex::Neighbors_list &in_Neighbors = m_heighted_Environments[layer].get_Vertex(vertex_id)->m_Neighbors;
		
		    for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_heighted_Environments[layer].get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;
		    
			int in_neighbor_index = 0;
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    if ((*out_neighbor)->m_target->m_id == vertex_id)
			    {
				break;
			    }
			    ++in_neighbor_index;
			}
			if (layer == N_Layers - 1)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index)));
			}
			else
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index + 1)));
			}
		    }
		}
		if (layer == 0)
		{
		    if (m_initial_arrangement.get_VertexOccupancy(vertex_id) == 0)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
												 total_Literal_cnt,
												 mutex_source_Identifiers);
			
			Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(layer)),
											       mutex_source_Identifiers);
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_MultiNegation(aux_Variable_cnt,
											    total_Literal_cnt,
											    mutex_source_Identifiers);
		    }
		}
		else
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											     total_Literal_cnt,
											     mutex_source_Identifiers);
		    
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														   sIntegerIndex(vertex_id),
														   sIntegerIndex(layer)),
											   mutex_source_Identifiers);
		}

	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id > 0)
	    {
		Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
										  total_Literal_cnt,
										  sSpecifiedStateIdentifier(&vertex_occupancy,
													    sIntegerIndex(vertex_id),
													    sIntegerIndex(0)),
										  init_robot_id);
		Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
									     total_Literal_cnt,
									     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
												     sIntegerIndex(vertex_id),
												     sIntegerIndex(0)));
	    }
	}
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id > 0)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_Equality(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedStateIdentifier(&vertex_occupancy,
													     sIntegerIndex(vertex_id),
													     sIntegerIndex(N_Layers - 1)),
										   goal_robot_id);
		    Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										 total_Literal_cnt,
										 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													 sIntegerIndex(vertex_id),
													 sIntegerIndex(N_Layers - 1)));
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    if (robot_IDs.size() > 1)
		    {
			sASSERT(false);
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
											  total_Literal_cnt,
											  sSpecifiedStateIdentifier(&vertex_occupancy,
														    sIntegerIndex(vertex_id),
														    sIntegerIndex(N_Layers - 1)),
											  *robot_IDs.begin());
			Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													     sIntegerIndex(vertex_id),
													     sIntegerIndex(N_Layers - 1)));
		    }
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
	int N_cnf_Variables = encoding_context.m_variable_store.get_Last_CNFVariable() + aux_Variable_cnt - 1;

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot plural SAT encoding\n", sPRODUCT);
	fprintf(fw, "c %s\n", sCOPYRIGHT);
	fprintf(fw, "c number of layers = %d\n", N_Layers);
	fprintf(fw, "c number of visible variables = %d\n", N_cnf_Variables - aux_Variable_cnt);
	fprintf(fw, "c number of hidden variables = %d\n", aux_Variable_cnt);
	fprintf(fw, "c number of literals = %d\n", total_Literal_cnt);
	fprintf(fw, "c number of clauses = %d\n", Clause_cnt);
	fprintf(fw, "c number of propositional variables = %d\n", N_cnf_Variables);
	fprintf(fw, "c number of clauses / number of variables = %.3f\n", (double)Clause_cnt / N_cnf_Variables);
	fprintf(fw, "c number of literals / number of clauses = %.3f\n", (double)total_Literal_cnt /  Clause_cnt);
	to_Stream(fw, "c ");
#endif
	fprintf(fw, "p cnf %d %d\n", N_cnf_Variables, Clause_cnt);
	
	for (int layer = 0; layer < N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		/*
		Clause_cnt += encoding_context.m_clause_generator->generate_Alignment(fw,
										      sSpecifiedStateIdentifier(&vertex_occupancy,
													     sIntegerIndex(vertex_id),
													     sIntegerIndex(layer)));
		*/
	    }
	}
	for (int layer = 0; layer < N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		Clause_cnt += encoding_context.m_bit_generator->generate_NonzeroImplication(fw,
											    sSpecifiedStateIdentifier(&vertex_occupancy,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											    sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														    sIntegerIndex(vertex_id),
														    sIntegerIndex(layer)));
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    Clause_cnt += encoding_context.m_bit_generator->generate_NonzeroImplication(fw,
											sSpecifiedStateIdentifier(&vertex_occupancy,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(N_Layers - 1)),
											sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														sIntegerIndex(vertex_id),
														sIntegerIndex(N_Layers - 1)));
	}

	for (int layer = 0; layer < N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
		
		int out_neighbor_index = 0;
		const sVertex::Neighbors_list &out_Neighbors = m_heighted_Environments[layer].get_Vertex(vertex_id)->m_Neighbors;
		
		switch (m_goal_type)
		{
		case GOAL_TYPE_ARRANGEMENT:
		{
		    if (layer == N_Layers - 1)
		    {
			if (m_goal_arrangement.get_VertexOccupancy(vertex_id) != 0)
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
											   sIntegerIndex(out_neighbor_index)));
				
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegation(fw,
												   mutex_target_Identifiers);
			}
			else
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															     sIntegerIndex(out_neighbor_index)),
												     /*
												     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															     sIntegerIndex(vertex_id),
															     sIntegerIndex(layer)),
												     */
												     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															     sIntegerIndex((*out_neighbor)->m_target->m_id),
															     sIntegerIndex(layer)));
				Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
													     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																     sIntegerIndex(out_neighbor_index)),
													     sSpecifiedStateIdentifier(&vertex_occupancy,
																       sIntegerIndex(vertex_id),
																       sIntegerIndex(layer)),
													     sSpecifiedStateIdentifier(&vertex_occupancy,
																       sIntegerIndex((*out_neighbor)->m_target->m_id),
																       sIntegerIndex(layer)));
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index)));
			    
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															      sIntegerIndex(vertex_id),
															      sIntegerIndex(layer)),
												      mutex_target_Identifiers);			    
			    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
													mutex_target_Identifiers);   
			}
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														     sIntegerIndex(0)),
											     /*
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex(vertex_id),
														     sIntegerIndex(layer)),
											     */
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex(vertex_id),
														     sIntegerIndex(layer + 1)));
			Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															     sIntegerIndex(0)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex(vertex_id),
															       sIntegerIndex(layer)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex(vertex_id),
															       sIntegerIndex(layer + 1)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(0)));

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															 sIntegerIndex(out_neighbor_index + 1)),
												 /*
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex(vertex_id),
															 sIntegerIndex(layer)),
												 */
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex((*out_neighbor)->m_target->m_id),
															 sIntegerIndex(layer)));
			    Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
													 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																 sIntegerIndex(out_neighbor_index + 1)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex(vertex_id),
																   sIntegerIndex(layer)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex((*out_neighbor)->m_target->m_id),
																   sIntegerIndex(layer)));
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  mutex_target_Identifiers);
			Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												    mutex_target_Identifiers);
		    }
		    break;
		}
		case GOAL_TYPE_SPECIFICATION:
		{
		    if (layer == N_Layers - 1)
		    {
			if (!m_goal_specification.get_GoalCompatibility(vertex_id).empty())
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
											   sIntegerIndex(out_neighbor_index)));
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegation(fw,
												   mutex_target_Identifiers);
			}
			else
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															     sIntegerIndex(out_neighbor_index)),
												     /*
												     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															     sIntegerIndex(vertex_id),
															     sIntegerIndex(layer)),
												     */
												     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															     sIntegerIndex((*out_neighbor)->m_target->m_id),
															     sIntegerIndex(layer)));
				Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
													     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																     sIntegerIndex(out_neighbor_index)),
													     sSpecifiedStateIdentifier(&vertex_occupancy,
																       sIntegerIndex(vertex_id),
																       sIntegerIndex(layer)),
													     sSpecifiedStateIdentifier(&vertex_occupancy,
																       sIntegerIndex((*out_neighbor)->m_target->m_id),
																       sIntegerIndex(layer)));
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index)));
			    
				++out_neighbor_index;
			    }

			    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															      sIntegerIndex(vertex_id),
															      sIntegerIndex(layer)),
												      mutex_target_Identifiers);
			    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
													mutex_target_Identifiers);
			}
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														     sIntegerIndex(0)),
											     /*
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex(vertex_id),
														     sIntegerIndex(layer)),
											     */
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex(vertex_id),
														     sIntegerIndex(layer + 1)));

			Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															     sIntegerIndex(0)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex(vertex_id),
															       sIntegerIndex(layer)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex(vertex_id),
															       sIntegerIndex(layer + 1)));

			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(0)));

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															 sIntegerIndex(out_neighbor_index + 1)),
												 /*
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex(vertex_id),
															 sIntegerIndex(layer)),*/
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex((*out_neighbor)->m_target->m_id),
															 sIntegerIndex(layer)));

			    Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
													 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																 sIntegerIndex(out_neighbor_index + 1)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex(vertex_id),
																   sIntegerIndex(layer)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex((*out_neighbor)->m_target->m_id),
																   sIntegerIndex(layer)));

			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  mutex_target_Identifiers);
			Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												    mutex_target_Identifiers);
		    }
		    break;
		}
		default:
		{
		    sASSERT(false);
		    break;
		}
		}
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_source_Identifiers;

		if (layer > 0)
		{
		    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer - 1][vertex_id],
									       sIntegerIndex(0)));
		    const sVertex::Neighbors_list &in_Neighbors = m_heighted_Environments[layer].get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_heighted_Environments[layer].get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;
		    
			int in_neighbor_index = 0;
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    if ((*out_neighbor)->m_target->m_id == vertex_id)
			    {
				break;
			    }
			    ++in_neighbor_index;
			}
			if (layer == N_Layers - 1)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index)));
			}
			else
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index + 1)));
			}
		    }
		}
		else
		{
		    const sVertex::Neighbors_list &in_Neighbors = m_heighted_Environments[layer].get_Vertex(vertex_id)->m_Neighbors;
		
		    for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_heighted_Environments[layer].get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;
		    
			int in_neighbor_index = 0;
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    if ((*out_neighbor)->m_target->m_id == vertex_id)
			    {
				break;
			    }
			    ++in_neighbor_index;
			}
			if (layer == N_Layers - 1)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index)));
			}
			else
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index + 1)));
			}
		    }
		}
		if (layer == 0)
		{
		    if (m_initial_arrangement.get_VertexOccupancy(vertex_id) == 0)
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												    mutex_source_Identifiers);
			
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  mutex_source_Identifiers);
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegation(fw,
											       mutex_source_Identifiers);
		    }
		}
		else
		{
		    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												mutex_source_Identifiers);
		    
		    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      mutex_source_Identifiers);
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id > 0)
	    {
		Clause_cnt += encoding_context.m_clause_generator->generate_Equality(fw,
										     sSpecifiedStateIdentifier(&vertex_occupancy,
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(0)),
										     init_robot_id);
		Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
										sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													sIntegerIndex(vertex_id),
													sIntegerIndex(0)));
	    }
	}

//	int robot_limit = 0;

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    printf("ARANGEMENT\n");
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id > 0)
		{
		    Clause_cnt += encoding_context.m_bit_generator->generate_Equality(fw,
										      sSpecifiedStateIdentifier(&vertex_occupancy,
														sIntegerIndex(vertex_id),
														sIntegerIndex(N_Layers - 1)),
										      goal_robot_id);
		    Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
										    sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													    sIntegerIndex(vertex_id),
													    sIntegerIndex(N_Layers - 1)));
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    printf("SPECIFICATION A1\n");
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    if (robot_IDs.size() > 1)
		    {
			// no goal setup
			sASSERT(false);
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_clause_generator->generate_Equality(fw,
											     sSpecifiedStateIdentifier(&vertex_occupancy,
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(N_Layers - 1)),
											     *robot_IDs.begin());
			Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
											sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														sIntegerIndex(vertex_id),
														sIntegerIndex(N_Layers - 1)));


/*
			const sVertex::Neighbors_list &out_Neighbors = m_heighted_Environments[layer].get_Vertex(vertex_id)->m_Neighbors;

			int out_neighbor_index = 0;
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[N_Layers - 1][vertex_id],
										       sIntegerIndex(out_neighbor_index)));
			    ++out_neighbor_index;
			}
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegation(fw,
											       mutex_target_Identifiers);
*/
		    }
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
	printf("generation finished\n");
    }


    void sMultirobotInstance::to_Stream_MddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	int extra_cost;
	MDD_vector MDD, extra_MDD;

	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	int mdd_depth = construct_MDD(encoding_context.m_max_total_cost, MDD, extra_cost, extra_MDD);
	//s_GlobalPhaseStatistics.leave_Phase();

	if (encoding_context.m_extra_cost >= 0)
	{
	    to_Stream_MddCNFsat(fw, encoding_context, encoding_context.m_extra_cost, mdd_depth, MDD, extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Stream_MddCNFsat(fw, encoding_context, extra_cost, mdd_depth, MDD, extra_MDD, indent, verbose);
	}
    }


    void sMultirobotInstance::to_Stream_MmddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	MDD_vector MDD;

	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	--encoding_context.m_N_Layers;
	construct_MakespanMDD(encoding_context.m_N_Layers, MDD);
	//s_GlobalPhaseStatistics.leave_Phase();

	to_Stream_MmddCNFsat(fw, encoding_context, MDD, indent, verbose);
    }


    void sMultirobotInstance::to_Stream_RXMddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	int extra_cost;
	MDD_vector MDD, extra_MDD;

	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	int mdd_depth = construct_MDD(encoding_context.m_max_total_cost, MDD, extra_cost, extra_MDD);
	// s_GlobalPhaseStatistics.leave_Phase();

	to_Stream_RXMddCNFsat(fw, encoding_context, extra_cost, mdd_depth, MDD, extra_MDD, indent, verbose);
    }


    void sMultirobotInstance::to_Stream_NoMddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	int extra_cost;
	MDD_vector MDD, extra_MDD;

	int mdd_depth = construct_NoMDD(encoding_context.m_max_total_cost, MDD, extra_cost, extra_MDD);

	if (encoding_context.m_extra_cost >= 0)
	{
	    to_Stream_MddCNFsat(fw, encoding_context, encoding_context.m_extra_cost, mdd_depth, MDD, extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Stream_MddCNFsat(fw, encoding_context, extra_cost, mdd_depth, MDD, extra_MDD, indent, verbose);
	}
    }


    void sMultirobotInstance::to_Stream_RXNoMddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	int extra_cost;
	MDD_vector MDD, extra_MDD;

	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	int mdd_depth = construct_NoMDD(encoding_context.m_max_total_cost, MDD, extra_cost, extra_MDD);
	// s_GlobalPhaseStatistics.leave_Phase();

	to_Stream_RXMddCNFsat(fw, encoding_context, extra_cost, mdd_depth, MDD, extra_MDD, indent, verbose);
    }


    void sMultirobotInstance::to_Stream_MddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, int extra_cost, int mdd_depth, const MDD_vector &MDD, const MDD_vector &extra_MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = mdd_depth;

	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
	encoding_context.m_vertex_water_cardinality_.resize(N_Robots + 1);
	encoding_context.m_edge_occupancy_by_water__.resize(N_Robots + 1);

	//	s_GlobalPhaseStatistics.enter_Phase("Counting");

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_vertex_occupancy_by_water_[robot_id].resize(N_Layers + 1);
	    encoding_context.m_vertex_water_cardinality_[robot_id].resize(N_Layers + 1);
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sIndexableBitIdentifier vertex_occupancy_by_water_(&encoding_context.m_variable_store,
								   "vertex_occupancy_by_water-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer),
								   sIntegerScope(0, MDD[robot_id][layer].size() - 1));
		encoding_context.m_vertex_occupancy_by_water_[robot_id][layer] = vertex_occupancy_by_water_;
		encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_water_[robot_id][layer]);

		if (!extra_MDD[robot_id][layer].empty())
		{
		    sASSERT(extra_MDD[robot_id][layer].size() == 1);

		    sIndexableBitIdentifier vertex_water_cardinality_(&encoding_context.m_variable_store,
								      "vertex_water_cardinality-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer),
								      sIntegerScope(0, 0));
		    encoding_context.m_vertex_water_cardinality_[robot_id][layer] = vertex_water_cardinality_;
		    encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_water_cardinality_[robot_id][layer]);

		    for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		    {
			if (extra_MDD[robot_id][layer][0] != MDD[robot_id][layer][u])
			{
			    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											      total_Literal_cnt,
											      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)),
											      sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][layer], sIntegerIndex(0)));
			}
		    }
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector prev_cardinality_Identifiers;

		    for (int prev_layer = 0; prev_layer < layer; ++prev_layer)
		    {
			if (!extra_MDD[robot_id][prev_layer].empty())
			{
			    prev_cardinality_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][prev_layer], sIntegerIndex(0)));
			}
		    }
		    if (!prev_cardinality_Identifiers.empty())
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][layer], sIntegerIndex(0)),
											       prev_cardinality_Identifiers);
		    }
		}
	    }
	}
	sBitClauseGenerator::SpecifiedBitIdentifiers_vector cardinality_Identifiers;
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		if (!extra_MDD[robot_id][layer].empty())
		{
		    cardinality_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][layer], sIntegerIndex(0)));
		}
	    }
	}
	if (!cardinality_Identifiers.empty())
	{
	    Clause_cnt += encoding_context.m_bit_generator->count_Cardinality(aux_Variable_cnt,total_Literal_cnt, cardinality_Identifiers, extra_cost);
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_edge_occupancy_by_water__[robot_id].resize(N_Layers);

	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		encoding_context.m_edge_occupancy_by_water__[robot_id][layer].resize(MDD[robot_id][layer].size());

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    int N_neighbors = 0;

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    ++N_neighbors;
			}
		    }
		    sIndexableBitIdentifier edge_occupancy_by_water__(&encoding_context.m_variable_store,
								      "edge_occupancy_by_water-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer) + "_" + sInt_32_to_String(MDD[robot_id][layer][u]),
								      sIntegerScope(0, N_neighbors - 1));
		    encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u] = edge_occupancy_by_water__;
		    encoding_context.register_TranslateIdentifier(encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u]);
		}
	    }
	}
	/*
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    typedef std::map<int, int> VerticesMDD_map;
	    typedef std::vector<VerticesMDD_map> LayerVerticesMDD_vector;
	    LayerVerticesMDD_vector layer_Vertices_MDD;
	    layer_Vertices_MDD.resize(N_Layers + 1);

	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
	      for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		  layer_Vertices_MDD[layer][MDD[robot_id][layer][u]] = u;
		}
	    }

	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
		    int neighbor_index = 0;

		    const sVertex *vertex = m_environment.get_Vertex(MDD[robot_id][layer][u]);
		    for (sVertex::Neighbors_list::const_iterator neighbor = vertex->m_Neighbors.begin(); neighbor != vertex->m_Neighbors.end(); ++neighbor)
		      {
			VerticesMDD_map::const_iterator v_iter = layer_Vertices_MDD[layer + 1].find((*neighbor)->m_target->m_id);
			if (v_iter != layer_Vertices_MDD[layer + 1].end())
			  {
			    int v = v_iter->second;

			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(neighbor_index)));

			    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											      total_Literal_cnt,
											      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														      sIntegerIndex(neighbor_index)),
											      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
														      sIntegerIndex(v)));
			    neighbor_index++;
			}
		    }
		    VerticesMDD_map::const_iterator v_iter = layer_Vertices_MDD[layer + 1].find(vertex->m_id);

		    if (v_iter != layer_Vertices_MDD[layer + 1].end())
		      {
			  int v = v_iter->second;

			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(neighbor_index)));

			    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											      total_Literal_cnt,
											      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														      sIntegerIndex(neighbor_index)),
											      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
														      sIntegerIndex(v)));
			    neighbor_index++;
		      }
		       
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));	
	    
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
														   sIntegerIndex(u)),
											   mutex_target_Identifiers);

		    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											     total_Literal_cnt,
											     mutex_target_Identifiers);   
		}

		/* Believed to be O.K.
		Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											 total_Literal_cnt,
											 mutex_vertex_Identifiers);
	*/
	/*
	    }
	}
        */
	/*
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
	    for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers], sIntegerIndex(u)));
	    }
	    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
										     total_Literal_cnt,
										     mutex_vertex_Identifiers);
	}

	for (int layer = 0; layer <= N_Layers; ++layer)
	  {
	    typedef std::vector<sBitClauseGenerator::SpecifiedBitIdentifiers_vector> VertexSpecifiedBitIdentifiers_vector;
	    VertexSpecifiedBitIdentifiers_vector vertex_mutex_occupancy_Identifiers;
	    vertex_mutex_occupancy_Identifiers.resize(N_Vertices);

	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	      {
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		  {
		    vertex_mutex_occupancy_Identifiers[MDD[robot_id][layer][u]].push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));
		  }
	      }

	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	      {
		if (vertex_mutex_occupancy_Identifiers[vertex_id].size() > 1)
		  {
		    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											     total_Literal_cnt,
											     vertex_mutex_occupancy_Identifiers[vertex_id]);
		  }
	      }
	  }

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_occupancy_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		    {
			if (MDD[robot_id][layer][u] == vertex_id)
			{
			    mutex_occupancy_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));
			}
		    }
		}
		if (mutex_occupancy_Identifiers.size() > 1)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											     total_Literal_cnt,
											     mutex_occupancy_Identifiers);
		}
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    int neighbor_index = 0;
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    if (MDD[robot_id][layer][u] != MDD[robot_id][layer + 1][v])
			    {
				for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
				{
				    if (other_robot_id != robot_id)
				    {
					for (int vv = 0; vv < MDD[other_robot_id][layer].size(); ++vv)
					{
					    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer][vv])
					    {
						Clause_cnt += encoding_context.m_bit_generator->count_BiangleMutex(aux_Variable_cnt,
														   total_Literal_cnt,
														   sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																	   sIntegerIndex(neighbor_index)),
														   sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer],
																	   sIntegerIndex(vv)));
					    }
					}
				    }
				}
			    }
			    ++neighbor_index;
			}
		    }
		}
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int u = 0; u < MDD[robot_id][0].size(); ++u)
	    {
		if (MDD[robot_id][0][u] == m_initial_arrangement.get_RobotLocation(robot_id))
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										 total_Literal_cnt,
										 sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][0],
													 sIntegerIndex(u)));
		}
	    }
	}
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
		{
		    if (MDD[robot_id][N_Layers][u] == m_goal_arrangement.get_RobotLocation(robot_id))
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers],
													     sIntegerIndex(u)));
		    }
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
		{
		    sASSERT(m_goal_specification.get_RobotGoal(robot_id).size() == 1);

		    if (MDD[robot_id][N_Layers][u] == *m_goal_specification.get_RobotGoal(robot_id).begin())
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers],
													     sIntegerIndex(u)));
		    }
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
	*/

	int N_cnf_Variables = encoding_context.m_variable_store.get_Last_CNFVariable() + aux_Variable_cnt - 1;

	//	s_GlobalPhaseStatistics.leave_Phase();
	//	s_GlobalPhaseStatistics.enter_Phase("Writing");

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot MDD SAT encoding\n", sPRODUCT);
	fprintf(fw, "c %s\n", sCOPYRIGHT);
	fprintf(fw, "c number of layers = %d\n", N_Layers);
	fprintf(fw, "c number of visible variables = %d\n", N_cnf_Variables - aux_Variable_cnt);
	fprintf(fw, "c number of hidden variables = %d\n", aux_Variable_cnt);
	fprintf(fw, "c number of literals = %d\n", total_Literal_cnt);
	fprintf(fw, "c number of clauses = %d\n", Clause_cnt);
	fprintf(fw, "c number of propositional variables = %d\n", N_cnf_Variables);
	fprintf(fw, "c number of clauses / number of variables = %.3f\n", (double)Clause_cnt / N_cnf_Variables);
	fprintf(fw, "c number of literals / number of clauses = %.3f\n", (double)total_Literal_cnt /  Clause_cnt);
	to_Stream(fw, "c ");
#endif
	fprintf(fw, "p cnf %d %d\n", N_cnf_Variables, Clause_cnt);

	//	s_GlobalPhaseStatistics.enter_Phase("Pregen 1");

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		if (!extra_MDD[robot_id][layer].empty())
		{
		    sASSERT(extra_MDD[robot_id][layer].size() == 1);

		    for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		    {
			if (extra_MDD[robot_id][layer][0] != MDD[robot_id][layer][u])
			{
			    Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												 sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)),
												 sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][layer], sIntegerIndex(0)));
			}
		    }
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector prev_cardinality_Identifiers;

		    for (int prev_layer = 0; prev_layer < layer; ++prev_layer)
		    {
			if (!extra_MDD[robot_id][prev_layer].empty())
			{
			    prev_cardinality_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][prev_layer], sIntegerIndex(0)));
			}
		    }
		    if (!prev_cardinality_Identifiers.empty())
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												  sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][layer], sIntegerIndex(0)),
												  prev_cardinality_Identifiers);
		    }
		}
	    }
	}
	//	s_GlobalPhaseStatistics.leave_Phase();
	//	s_GlobalPhaseStatistics.enter_Phase("Pregen 2");

	if (!cardinality_Identifiers.empty())
	{
	    printf("----> Cardinality: %d, %d, %d <----\n", cardinality_Identifiers.size(), extra_cost, encoding_context.m_max_total_cost);
	    Clause_cnt += encoding_context.m_bit_generator->generate_Cardinality(fw, cardinality_Identifiers, extra_cost);
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    typedef std::map<int, int> VerticesMDD_map;
	    typedef std::vector<VerticesMDD_map> LayerVerticesMDD_vector;
	    LayerVerticesMDD_vector layer_Vertices_MDD;
	    layer_Vertices_MDD.resize(N_Layers + 1);

	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
	      for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		  layer_Vertices_MDD[layer][MDD[robot_id][layer][u]] = u;
		}
	    }

	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
		    int neighbor_index = 0;
		    
		    const sVertex *vertex = m_environment.get_Vertex(MDD[robot_id][layer][u]);
		    for (sVertex::Neighbors_list::const_iterator neighbor = vertex->m_Neighbors.begin(); neighbor != vertex->m_Neighbors.end(); ++neighbor)
		      {
			VerticesMDD_map::const_iterator v_iter = layer_Vertices_MDD[layer + 1].find((*neighbor)->m_target->m_id);
			if (v_iter != layer_Vertices_MDD[layer + 1].end())
			  {
			    int v = v_iter->second;

			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(neighbor_index)));

			    Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
															 sIntegerIndex(neighbor_index)),
												 sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
															 sIntegerIndex(v)));
			    neighbor_index++;
			  }
		      }
		    {
		    VerticesMDD_map::const_iterator v_iter = layer_Vertices_MDD[layer + 1].find(vertex->m_id);

		    if (v_iter != layer_Vertices_MDD[layer + 1].end())
			{
			  int v = v_iter->second;
			  
			  mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(neighbor_index)));
			  
			  Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
											       sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														       sIntegerIndex(neighbor_index)),
											       sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
														       sIntegerIndex(v)));
			  neighbor_index++;
			}
		    }
		    
		    /*		    
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(neighbor_index)));

			    Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
															 sIntegerIndex(neighbor_index)),
												 sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
															 sIntegerIndex(v)));
			    neighbor_index++;
			}
		    }
		    */
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));	
	    
		    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
											      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
														      sIntegerIndex(u)),
											      mutex_target_Identifiers);

		   if (mutex_target_Identifiers.size() > 1)
		     {
		       Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												   mutex_target_Identifiers);   
		     }
		}
		/* Believed to be O.K. */
		/*
		if (mutex_vertex_Identifiers.size() > 1)
		{
		  Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
											      mutex_vertex_Identifiers);
		}
		*/
	    }
	}
	//	s_GlobalPhaseStatistics.leave_Phase();
	//s_GlobalPhaseStatistics.enter_Phase("Pregen 3");

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
	    for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers], sIntegerIndex(u)));
	    }
	    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
											mutex_vertex_Identifiers);
	}
	//	s_GlobalPhaseStatistics.leave_Phase();
	//s_GlobalPhaseStatistics.enter_Phase("Pregen 4");

	for (int layer = 0; layer <= N_Layers; ++layer)
	  {
	    typedef std::vector<sBitClauseGenerator::SpecifiedBitIdentifiers_vector> VertexSpecifiedBitIdentifiers_vector;
	    VertexSpecifiedBitIdentifiers_vector vertex_mutex_occupancy_Identifiers;
	    vertex_mutex_occupancy_Identifiers.resize(N_Vertices);

	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	      {
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		  {
		    vertex_mutex_occupancy_Identifiers[MDD[robot_id][layer][u]].push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));
		  }
	      }

	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	      {
		if (vertex_mutex_occupancy_Identifiers[vertex_id].size() > 1)
		  {
		    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												vertex_mutex_occupancy_Identifiers[vertex_id]);
		  }
	      }
	  }

	//s_GlobalPhaseStatistics.leave_Phase();
	//s_GlobalPhaseStatistics.enter_Phase("Pregen 5");

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    int neighbor_index = 0;
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    if (MDD[robot_id][layer][u] != MDD[robot_id][layer + 1][v])
			    {
				for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
				{
				    if (other_robot_id != robot_id)
				    {
					for (int vv = 0; vv < MDD[other_robot_id][layer].size(); ++vv)
					{
					    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer][vv])
					    {
						Clause_cnt += encoding_context.m_bit_generator->generate_BiangleMutex(fw,
														      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																	      sIntegerIndex(neighbor_index)),
														      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer],
																	      sIntegerIndex(vv)));
					    }
					}
				    }
				}
			    }
			    ++neighbor_index;
			}
		    }
		}
	    }
	}

	//	s_GlobalPhaseStatistics.leave_Phase();
	//	s_GlobalPhaseStatistics.enter_Phase("Goaling");

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int u = 0; u < MDD[robot_id][0].size(); ++u)
	    {
		if (MDD[robot_id][0][u] == m_initial_arrangement.get_RobotLocation(robot_id))
		{
		    Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
										    sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][0],
													    sIntegerIndex(u)));
		}
	    }
	}
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
		{
		    if (MDD[robot_id][N_Layers][u] == m_goal_arrangement.get_RobotLocation(robot_id))
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
											sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers],
														sIntegerIndex(u)));
		    }
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
		{
		    sASSERT(m_goal_specification.get_RobotGoal(robot_id).size() == 1);

		    if (MDD[robot_id][N_Layers][u] == *m_goal_specification.get_RobotGoal(robot_id).begin())
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
											sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers],
														sIntegerIndex(u)));
		    }
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
	//	s_GlobalPhaseStatistics.leave_Phase();
	//	s_GlobalPhaseStatistics.leave_Phase();
    }



    void sMultirobotInstance::to_Stream_MmddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const MDD_vector &MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = encoding_context.m_N_Layers;

	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
	encoding_context.m_edge_occupancy_by_water__.resize(N_Robots + 1);

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_vertex_occupancy_by_water_[robot_id].resize(N_Layers + 1);
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sIndexableBitIdentifier vertex_occupancy_by_water_(&encoding_context.m_variable_store,
								   "vertex_occupancy_by_water-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer),
								   sIntegerScope(0, MDD[robot_id][layer].size() - 1));
		encoding_context.m_vertex_occupancy_by_water_[robot_id][layer] = vertex_occupancy_by_water_;
		encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_water_[robot_id][layer]);
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_edge_occupancy_by_water__[robot_id].resize(N_Layers);

	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		encoding_context.m_edge_occupancy_by_water__[robot_id][layer].resize(MDD[robot_id][layer].size());

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    int N_neighbors = 0;

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    ++N_neighbors;
			}
		    }
		    sIndexableBitIdentifier edge_occupancy_by_water__(&encoding_context.m_variable_store,
								      "edge_occupancy_by_water-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer) + "_" + sInt_32_to_String(MDD[robot_id][layer][u]),
								      sIntegerScope(0, N_neighbors - 1));
		    encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u] = edge_occupancy_by_water__;
		    encoding_context.register_TranslateIdentifier(encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u]);
		}
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;

		    int neighbor_index = 0;
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(neighbor_index)));

			    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											      total_Literal_cnt,
											      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														      sIntegerIndex(neighbor_index)),
											      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
														      sIntegerIndex(v)));
			    neighbor_index++;
			}
		    }
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));	
	    
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
														   sIntegerIndex(u)),
											   mutex_target_Identifiers);

		    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											     total_Literal_cnt,
											     mutex_target_Identifiers);   
		}
		Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											 total_Literal_cnt,
											 mutex_vertex_Identifiers);
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
	    for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers], sIntegerIndex(u)));
	    }
	    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
										     total_Literal_cnt,
										     mutex_vertex_Identifiers);
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_occupancy_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		    {
			if (MDD[robot_id][layer][u] == vertex_id)
			{
			    mutex_occupancy_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));
			}
		    }
		}
		if (mutex_occupancy_Identifiers.size() > 1)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											     total_Literal_cnt,
											     mutex_occupancy_Identifiers);
		}
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    int neighbor_index = 0;
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    if (MDD[robot_id][layer][u] != MDD[robot_id][layer + 1][v])
			    {
				for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
				{
				    if (other_robot_id != robot_id)
				    {
					for (int vv = 0; vv < MDD[other_robot_id][layer].size(); ++vv)
					{
					    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer][vv])
					    {
						Clause_cnt += encoding_context.m_bit_generator->count_BiangleMutex(aux_Variable_cnt,
														   total_Literal_cnt,
														   sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																	   sIntegerIndex(neighbor_index)),
														   sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer],
																	   sIntegerIndex(vv)));
					    }
					}
				    }
				}
			    }
			    ++neighbor_index;
			}
		    }
		}
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int u = 0; u < MDD[robot_id][0].size(); ++u)
	    {
		if (MDD[robot_id][0][u] == m_initial_arrangement.get_RobotLocation(robot_id))
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										 total_Literal_cnt,
										 sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][0],
													 sIntegerIndex(u)));
		}
	    }
	}
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
		{
		    if (MDD[robot_id][N_Layers][u] == m_goal_arrangement.get_RobotLocation(robot_id))
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers],
													     sIntegerIndex(u)));
		    }
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
		{
		    sASSERT(m_goal_specification.get_RobotGoal(robot_id).size() == 1);

		    if (MDD[robot_id][N_Layers][u] == *m_goal_specification.get_RobotGoal(robot_id).begin())
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers],
													     sIntegerIndex(u)));
		    }
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}

	int N_cnf_Variables = encoding_context.m_variable_store.get_Last_CNFVariable() + aux_Variable_cnt - 1;

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot MDD SAT encoding\n", sPRODUCT);
	fprintf(fw, "c %s\n", sCOPYRIGHT);
	fprintf(fw, "c number of layers = %d\n", N_Layers);
	fprintf(fw, "c number of visible variables = %d\n", N_cnf_Variables - aux_Variable_cnt);
	fprintf(fw, "c number of hidden variables = %d\n", aux_Variable_cnt);
	fprintf(fw, "c number of literals = %d\n", total_Literal_cnt);
	fprintf(fw, "c number of clauses = %d\n", Clause_cnt);
	fprintf(fw, "c number of propositional variables = %d\n", N_cnf_Variables);
	fprintf(fw, "c number of clauses / number of variables = %.3f\n", (double)Clause_cnt / N_cnf_Variables);
	fprintf(fw, "c number of literals / number of clauses = %.3f\n", (double)total_Literal_cnt /  Clause_cnt);
	to_Stream(fw, "c ");
#endif
	fprintf(fw, "p cnf %d %d\n", N_cnf_Variables, Clause_cnt);

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;

		    int neighbor_index = 0;
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(neighbor_index)));

			    Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
															 sIntegerIndex(neighbor_index)),
												 sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
															 sIntegerIndex(v)));
			    neighbor_index++;
			}
		    }
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));	
	    
		    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
											      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
														      sIntegerIndex(u)),
											      mutex_target_Identifiers);

		    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												mutex_target_Identifiers);   
		}
		Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
											    mutex_vertex_Identifiers);
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
	    for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers], sIntegerIndex(u)));
	    }
	    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
											mutex_vertex_Identifiers);
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_occupancy_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		    {
			if (MDD[robot_id][layer][u] == vertex_id)
			{
			    mutex_occupancy_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));
			}
		    }
		}
		if (mutex_occupancy_Identifiers.size() > 1)
		{
		    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												mutex_occupancy_Identifiers);
		}
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    int neighbor_index = 0;
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    if (MDD[robot_id][layer][u] != MDD[robot_id][layer + 1][v])
			    {
				for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
				{
				    if (other_robot_id != robot_id)
				    {
					for (int vv = 0; vv < MDD[other_robot_id][layer].size(); ++vv)
					{
					    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer][vv])
					    {
						Clause_cnt += encoding_context.m_bit_generator->generate_BiangleMutex(fw,
														      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																	      sIntegerIndex(neighbor_index)),
														      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer],
																	      sIntegerIndex(vv)));
					    }
					}
				    }
				}
			    }
			    ++neighbor_index;
			}
		    }
		}
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int u = 0; u < MDD[robot_id][0].size(); ++u)
	    {
		if (MDD[robot_id][0][u] == m_initial_arrangement.get_RobotLocation(robot_id))
		{
		    Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
										    sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][0],
													    sIntegerIndex(u)));
		}
	    }
	}
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
		{
		    if (MDD[robot_id][N_Layers][u] == m_goal_arrangement.get_RobotLocation(robot_id))
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
											sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers],
														sIntegerIndex(u)));
		    }
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
		{
		    sASSERT(m_goal_specification.get_RobotGoal(robot_id).size() == 1);

		    if (MDD[robot_id][N_Layers][u] == *m_goal_specification.get_RobotGoal(robot_id).begin())
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
											sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers],
														sIntegerIndex(u)));
		    }
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
    }


    void sMultirobotInstance::to_Stream_RXMddCNFsat(FILE                              *fw,
						    sMultirobotEncodingContext_CNFsat &encoding_context,
						    int                                extra_cost,
						    int                                mdd_depth,
						    const MDD_vector                  &MDD,
						    const MDD_vector                  &extra_MDD,
						    const sString                     &sUNUSED(indent),
						    bool                               sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = mdd_depth;

	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
	encoding_context.m_vertex_water_cardinality_.resize(N_Robots + 1);
	encoding_context.m_edge_occupancy_by_water__.resize(N_Robots + 1);

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_vertex_occupancy_by_water_[robot_id].resize(N_Layers + 1);
	    encoding_context.m_vertex_water_cardinality_[robot_id].resize(N_Layers + 1);
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sIndexableBitIdentifier vertex_occupancy_by_water_(&encoding_context.m_variable_store,
								   "vertex_occupancy_by_water-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer),
								   sIntegerScope(0, MDD[robot_id][layer].size() - 1));
		encoding_context.m_vertex_occupancy_by_water_[robot_id][layer] = vertex_occupancy_by_water_;
		encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_water_[robot_id][layer]);

		if (!extra_MDD[robot_id][layer].empty())
		{
		    sASSERT(extra_MDD[robot_id][layer].size() == 1);

		    sIndexableBitIdentifier vertex_water_cardinality_(&encoding_context.m_variable_store,
								      "vertex_water_cardinality-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer),
								      sIntegerScope(0, 0));
		    encoding_context.m_vertex_water_cardinality_[robot_id][layer] = vertex_water_cardinality_;
		    encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_water_cardinality_[robot_id][layer]);

		    for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		    {
			if (extra_MDD[robot_id][layer][0] != MDD[robot_id][layer][u])
			{
			    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											      total_Literal_cnt,
											      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)),
											      sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][layer], sIntegerIndex(0)));
			}
		    }
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector prev_cardinality_Identifiers;

		    for (int prev_layer = 0; prev_layer < layer; ++prev_layer)
		    {
			if (!extra_MDD[robot_id][prev_layer].empty())
			{
			    prev_cardinality_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][prev_layer], sIntegerIndex(0)));
			}
		    }
		    if (!prev_cardinality_Identifiers.empty())
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][layer], sIntegerIndex(0)),
											       prev_cardinality_Identifiers);
		    }
		}
	    }
	}
	sBitClauseGenerator::SpecifiedBitIdentifiers_vector cardinality_Identifiers;
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		if (!extra_MDD[robot_id][layer].empty())
		{
		    cardinality_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][layer], sIntegerIndex(0)));
		}
	    }
	}
/*
	if (!cardinality_Identifiers.empty())
	{
	    Clause_cnt += encoding_context.m_bit_generator->count_Cardinality(aux_Variable_cnt,total_Literal_cnt, cardinality_Identifiers, extra_cost);
	}
*/
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_edge_occupancy_by_water__[robot_id].resize(N_Layers);

	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		encoding_context.m_edge_occupancy_by_water__[robot_id][layer].resize(MDD[robot_id][layer].size());

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    int N_neighbors = 0;

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    ++N_neighbors;
			}
		    }
		    sIndexableBitIdentifier edge_occupancy_by_water__(&encoding_context.m_variable_store,
								      "edge_occupancy_by_water-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer) + "_" + sInt_32_to_String(MDD[robot_id][layer][u]),
								      sIntegerScope(0, N_neighbors - 1));
		    encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u] = edge_occupancy_by_water__;
		    encoding_context.register_TranslateIdentifier(encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u]);
		}
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;

		    int neighbor_index = 0;
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(neighbor_index)));

			    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											      total_Literal_cnt,
											      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														      sIntegerIndex(neighbor_index)),
											      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
														      sIntegerIndex(v)));
			    neighbor_index++;
			}
		    }
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));	
	    
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
														   sIntegerIndex(u)),
											   mutex_target_Identifiers);

		    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											     total_Literal_cnt,
											     mutex_target_Identifiers);   
		}
		Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											 total_Literal_cnt,
											 mutex_vertex_Identifiers);
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
	    for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers], sIntegerIndex(u)));
	    }
	    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
										     total_Literal_cnt,
										     mutex_vertex_Identifiers);
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_occupancy_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		    {
			if (MDD[robot_id][layer][u] == vertex_id)
			{
			    mutex_occupancy_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));
			}
		    }
		}
		if (mutex_occupancy_Identifiers.size() > 1)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											     total_Literal_cnt,
											     mutex_occupancy_Identifiers);
		}
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    int neighbor_index = 0;
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    if (MDD[robot_id][layer][u] != MDD[robot_id][layer + 1][v])
			    {
				for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
				{
				    if (other_robot_id != robot_id)
				    {
					for (int vv = 0; vv < MDD[other_robot_id][layer].size(); ++vv)
					{
					    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer][vv])
					    {
						Clause_cnt += encoding_context.m_bit_generator->count_BiangleMutex(aux_Variable_cnt,
														   total_Literal_cnt,
														   sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																	   sIntegerIndex(neighbor_index)),
														   sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer],
																	   sIntegerIndex(vv)));
					    }
					}
				    }
				}
			    }
			    ++neighbor_index;
			}
		    }
		}
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int u = 0; u < MDD[robot_id][0].size(); ++u)
	    {
		if (MDD[robot_id][0][u] == m_initial_arrangement.get_RobotLocation(robot_id))
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										 total_Literal_cnt,
										 sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][0],
													 sIntegerIndex(u)));
		}
	    }
	}
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
		{
		    if (MDD[robot_id][N_Layers][u] == m_goal_arrangement.get_RobotLocation(robot_id))
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers],
													     sIntegerIndex(u)));
		    }
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
		{
		    sASSERT(m_goal_specification.get_RobotGoal(robot_id).size() == 1);

		    if (MDD[robot_id][N_Layers][u] == *m_goal_specification.get_RobotGoal(robot_id).begin())
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers],
													     sIntegerIndex(u)));
		    }
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}

	int N_cnf_Variables = encoding_context.m_variable_store.get_Last_CNFVariable() + aux_Variable_cnt - 1;

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot MDD SAT encoding\n", sPRODUCT);
	fprintf(fw, "c %s\n", sCOPYRIGHT);
	fprintf(fw, "c number of layers = %d\n", N_Layers);
	fprintf(fw, "c number of visible variables = %d\n", N_cnf_Variables - aux_Variable_cnt);
	fprintf(fw, "c number of hidden variables = %d\n", aux_Variable_cnt);
	fprintf(fw, "c number of literals = %d\n", total_Literal_cnt);
	fprintf(fw, "c number of clauses = %d\n", Clause_cnt);
	fprintf(fw, "c number of propositional variables = %d\n", N_cnf_Variables);
	fprintf(fw, "c number of clauses / number of variables = %.3f\n", (double)Clause_cnt / N_cnf_Variables);
	fprintf(fw, "c number of literals / number of clauses = %.3f\n", (double)total_Literal_cnt /  Clause_cnt);
	to_Stream(fw, "c ");
#endif
	fprintf(fw, "p cnf %d %d\n", N_cnf_Variables, Clause_cnt);

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		if (!extra_MDD[robot_id][layer].empty())
		{
		    sASSERT(extra_MDD[robot_id][layer].size() == 1);

		    for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		    {
			if (extra_MDD[robot_id][layer][0] != MDD[robot_id][layer][u])
			{
			    Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												 sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)),
												 sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][layer], sIntegerIndex(0)));
			}
		    }
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector prev_cardinality_Identifiers;

		    for (int prev_layer = 0; prev_layer < layer; ++prev_layer)
		    {
			if (!extra_MDD[robot_id][prev_layer].empty())
			{
			    prev_cardinality_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][prev_layer], sIntegerIndex(0)));
			}
		    }
		    if (!prev_cardinality_Identifiers.empty())
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												  sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][layer], sIntegerIndex(0)),
												  prev_cardinality_Identifiers);
		    }
		}
	    }
	}
/*
	if (!cardinality_Identifiers.empty())
	{
	    printf("----> Cardinality: %d, %d, %d <----\n", cardinality_Identifiers.size(), extra_cost, encoding_context.m_max_total_cost);
	    Clause_cnt += encoding_context.m_bit_generator->generate_Cardinality(fw, cardinality_Identifiers, extra_cost);
	}
*/
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;

		    int neighbor_index = 0;
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(neighbor_index)));

			    Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
															 sIntegerIndex(neighbor_index)),
												 sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
															 sIntegerIndex(v)));
			    neighbor_index++;
			}
		    }
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));	
	    
		    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
											      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
														      sIntegerIndex(u)),
											      mutex_target_Identifiers);

		    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												mutex_target_Identifiers);   
		}
		Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
											    mutex_vertex_Identifiers);
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
	    for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers], sIntegerIndex(u)));
	    }
	    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
											mutex_vertex_Identifiers);
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_occupancy_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		    {
			if (MDD[robot_id][layer][u] == vertex_id)
			{
			    mutex_occupancy_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));
			}
		    }
		}
		if (mutex_occupancy_Identifiers.size() > 1)
		{
		    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												mutex_occupancy_Identifiers);
		}
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    int neighbor_index = 0;
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    if (MDD[robot_id][layer][u] != MDD[robot_id][layer + 1][v])
			    {
				for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
				{
				    if (other_robot_id != robot_id)
				    {
					for (int vv = 0; vv < MDD[other_robot_id][layer].size(); ++vv)
					{
					    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer][vv])
					    {
						Clause_cnt += encoding_context.m_bit_generator->generate_BiangleMutex(fw,
														      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																	      sIntegerIndex(neighbor_index)),
														      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer],
																	      sIntegerIndex(vv)));
					    }
					}
				    }
				}
			    }
			    ++neighbor_index;
			}
		    }
		}
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int u = 0; u < MDD[robot_id][0].size(); ++u)
	    {
		if (MDD[robot_id][0][u] == m_initial_arrangement.get_RobotLocation(robot_id))
		{
		    Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
										    sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][0],
													    sIntegerIndex(u)));
		}
	    }
	}
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
		{
		    if (MDD[robot_id][N_Layers][u] == m_goal_arrangement.get_RobotLocation(robot_id))
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
											sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers],
														sIntegerIndex(u)));
		    }
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
		{
		    sASSERT(m_goal_specification.get_RobotGoal(robot_id).size() == 1);

		    if (MDD[robot_id][N_Layers][u] == *m_goal_specification.get_RobotGoal(robot_id).begin())
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
											sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers],
														sIntegerIndex(u)));
		    }
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
    }


/*----------------------------------------------------------------------------*/
// sMultirobotSolution

    const sMultirobotSolution::Move sMultirobotSolution::UNDEFINED_MOVE = sMultirobotSolution::Move(0, 0, 0);


/*----------------------------------------------------------------------------*/

    sMultirobotSolution::Move::Move(int robot_id, int src_vrtx_id, int dest_vrtx_id)
	: m_robot_id(robot_id)
	, m_src_vrtx_id(src_vrtx_id)
	, m_dest_vrtx_id(dest_vrtx_id)
	, m_crt_time(0)
    {
	// nothing
    }


    sMultirobotSolution::Move::Move(int robot_id, int src_vrtx_id, int dest_vrtx_id, int crt_time)
	: m_robot_id(robot_id)
	, m_src_vrtx_id(src_vrtx_id)
	, m_dest_vrtx_id(dest_vrtx_id)
	, m_crt_time(crt_time)
    {
	// nothing
    }


    bool sMultirobotSolution::Move::is_Undefined(void) const
    {
	return (m_robot_id <= 0);
    }


    bool sMultirobotSolution::Move::is_Dependent(const Move &move) const
    {
	return (   m_src_vrtx_id == move.m_src_vrtx_id
		|| m_src_vrtx_id == move.m_dest_vrtx_id
		|| m_dest_vrtx_id == move.m_src_vrtx_id
		|| m_dest_vrtx_id == move.m_dest_vrtx_id);
    }



/*----------------------------------------------------------------------------*/

    sMultirobotSolution::Step::Step(int time)
	: m_time(time)
    {
	// nothing
    }


/*----------------------------------------------------------------------------*/

    sMultirobotSolution::sMultirobotSolution()
	: m_Moves_cnt(0)
    {
	// nothing
    }


    sMultirobotSolution::sMultirobotSolution(int start_step, const sMultirobotSolution &sub_solution)
	: m_Moves_cnt(0)
    {
	int N_Steps = sub_solution.m_Steps.size();
	for (int i = 0; i < N_Steps; ++i)
	{
	    const Step &step = sub_solution.m_Steps[i];

	    for (Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	    {
		add_Move(i + start_step, *move);
	    }
	}	
    }


    sMultirobotSolution::sMultirobotSolution(const sMultirobotSolution &sub_solution_1, const sMultirobotSolution sub_solution_2)
	: m_Moves_cnt(0)
    {
	int N_Steps_1 = sub_solution_1.m_Steps.size();
	for (int i = 0; i < N_Steps_1; ++i)
	{
	    const Step &step = sub_solution_1.m_Steps[i];

	    for (Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	    {
		add_Move(i, *move);
	    }
	}

	int N_Steps_2 = sub_solution_2.m_Steps.size();
	for (int i = 0; i < N_Steps_2; ++i)
	{
	    const Step &step = sub_solution_2.m_Steps[i];

	    for (Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	    {
		add_Move(i, *move);
	    }
	}
    }


    sMultirobotSolution::sMultirobotSolution(const sMultirobotSolution &multirobot_solution)
	: m_Moves_cnt(multirobot_solution.m_Moves_cnt)
	, m_Steps(multirobot_solution.m_Steps)
    {
	// nothing
    }


    const sMultirobotSolution& sMultirobotSolution::operator=(const sMultirobotSolution &multirobot_solution)
    {
	m_Moves_cnt = multirobot_solution.m_Moves_cnt;
	m_Steps = multirobot_solution.m_Steps;

	return *this;
    }


    bool sMultirobotSolution::is_Null(void) const
    {
	return (m_Steps.empty());
    }


    int sMultirobotSolution::get_MoveCount(void) const
    {
	return m_Moves_cnt;
    }


    int sMultirobotSolution::get_StepCount(void) const
    {
	return m_Steps.size();
    }


    void sMultirobotSolution::add_Move(int time, const Move &move)
    {
	int push_cnt = time - m_Steps.size();

	while (push_cnt-- >= 0)
	{
	    m_Steps.push_back(Step(m_Steps.size()));
	}
	m_Steps[time].m_Moves.push_back(move);
	++m_Moves_cnt;
    }


    int sMultirobotSolution::calc_EmptySteps(void) const
    {
	int N_empty_Steps = 0;
	int N_Steps = m_Steps.size();
	for (int i = 0; i < N_Steps; ++i)
	{
	    const Step &step = m_Steps[i];
	    if (step.m_Moves.empty())
	    {
		++N_empty_Steps;
	    }
	}
	return N_empty_Steps;
    }


    void sMultirobotSolution::remove_EmptySteps(void)
    {
	int time = 0;
	Steps_vector clean_Steps;

	int N_Steps = m_Steps.size();
	for (int i = 0; i < N_Steps; ++i)
	{
	    Step &step = m_Steps[i];

	    if (!step.m_Moves.empty())
	    {
		step.m_time = time++;
		clean_Steps.push_back(step);
	    }
	}
	m_Steps = clean_Steps;
    }


    sMultirobotSolution sMultirobotSolution::extract_Subsolution(int start_step, int final_step) const
    {
	sMultirobotSolution subsolution;

	for (int i = start_step; i <= final_step; ++i)
	{
	    const Step &step = m_Steps[i];

	    for (Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	    {
		subsolution.add_Move(i, *move);
	    }
	}	

	return subsolution;
    }


    void sMultirobotSolution::execute_Solution(const sRobotArrangement &initial_arrangement,
					       sRobotArrangement       &final_arrangement,
					       int                      N_Steps) const
    {
	final_arrangement = initial_arrangement;

	if (N_Steps == N_STEPS_UNDEFINED)
	{
	    for (sMultirobotSolution::Steps_vector::const_iterator step =  m_Steps.begin(); step != m_Steps.end(); ++step)
	    {
		for (sMultirobotSolution::Moves_list::const_iterator move = step->m_Moves.begin(); move != step->m_Moves.end(); ++move)
		{
		    final_arrangement.move_Robot(move->m_robot_id, move->m_dest_vrtx_id);
		}	    
	    }
	}
	else
	{
	    for (sMultirobotSolution::Steps_vector::const_iterator step =  m_Steps.begin(); step != m_Steps.end(); ++step)
	    {
		for (sMultirobotSolution::Moves_list::const_iterator move = step->m_Moves.begin(); move != step->m_Moves.end(); ++move)
		{
		    final_arrangement.move_Robot(move->m_robot_id, move->m_dest_vrtx_id);
		}	    
		if (--N_Steps <= 0)
		{
		    return;
		}
	    }
	}
    }


    void sMultirobotSolution::execute_Step(const sRobotArrangement &current_arrangement,
					   sRobotArrangement       &final_arrangement,
					   int                      step_idx) const
    {
	final_arrangement = current_arrangement;

	const Step &step = m_Steps[step_idx];

	for (sMultirobotSolution::Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	{
	    final_arrangement.move_Robot(move->m_robot_id, move->m_dest_vrtx_id);
	}	    
    }


    void sMultirobotSolution::execute_Solution(sRobotArrangement &arrangement,
					       int                N_Steps) const
    {
	if (N_Steps == N_STEPS_UNDEFINED)
	{
	    for (sMultirobotSolution::Steps_vector::const_iterator step =  m_Steps.begin(); step != m_Steps.end(); ++step)
	    {
		for (sMultirobotSolution::Moves_list::const_iterator move = step->m_Moves.begin(); move != step->m_Moves.end(); ++move)
		{
		    arrangement.move_Robot(move->m_robot_id, move->m_dest_vrtx_id);
		}	    
	    }
	}
	else
	{
	    for (sMultirobotSolution::Steps_vector::const_iterator step =  m_Steps.begin(); step != m_Steps.end(); ++step)
	    {
		for (sMultirobotSolution::Moves_list::const_iterator move = step->m_Moves.begin(); move != step->m_Moves.end(); ++move)
		{
		    arrangement.move_Robot(move->m_robot_id, move->m_dest_vrtx_id);
		}	    
		if (--N_Steps <= 0)
		{
		    return;
		}
	    }
	}
    }


    void sMultirobotSolution::execute_Step(sRobotArrangement &arrangement,
					   int                step_idx) const
    {
	 const Step &step = m_Steps[step_idx];

	for (sMultirobotSolution::Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	{
	    arrangement.move_Robot(move->m_robot_id, move->m_dest_vrtx_id);
	}	    
    }


    bool sMultirobotSolution::verify_Step(const sRobotArrangement &arrangement,
					  int                      step_idx) const
    {
	const Step &step = m_Steps[step_idx];

	for (sMultirobotSolution::Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	{
	    if (!arrangement.verify_Move(move->m_robot_id, move->m_dest_vrtx_id))
	    {
		return false;
	    }
	}
	return true;
    }


    bool sMultirobotSolution::check_Step(const sRobotArrangement &arrangement,
					 int                      step_idx) const
    {
	const Step &step = m_Steps[step_idx];

	for (sMultirobotSolution::Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	{
	    if (!arrangement.check_Move(move->m_robot_id, move->m_dest_vrtx_id))
	    {
		return false;
	    }
	}
	return true;
    }


    void sMultirobotSolution::filter_Solution(const sRobotArrangement &initial_arrangement,
					      const sRobotGoal        &goal_arrangement,
					      sMultirobotSolution     &filter_solution) const
    {
	sRobotArrangement robot_arrangement = initial_arrangement;

	for (sMultirobotSolution::Steps_vector::const_iterator step = m_Steps.begin(); step != m_Steps.end(); ++step)
	{
	    for (sMultirobotSolution::Moves_list::const_iterator move = step->m_Moves.begin(); move != step->m_Moves.end(); ++move)
	    {
		robot_arrangement.move_Robot(move->m_robot_id, move->m_dest_vrtx_id);
		filter_solution.add_Move(step->m_time, *move);

		if (goal_arrangement.is_Satisfied(robot_arrangement))
		{
		    break;
		}
	    }	    
	}
    }


    int sMultirobotSolution::calc_CriticalTimes(void)
    {
	int max_crt_time = 0;

	for (sMultirobotSolution::Steps_vector::iterator step =  m_Steps.begin(); step != m_Steps.end(); ++step)
	{
	    for (sMultirobotSolution::Moves_list::iterator move = step->m_Moves.begin(); move != step->m_Moves.end(); ++move)
	    {
		move->m_crt_time = 0;

		for (sMultirobotSolution::Steps_vector::const_iterator prev_step =  m_Steps.begin(); prev_step != step; ++prev_step)
		{
		    for (sMultirobotSolution::Moves_list::const_iterator prev_move = prev_step->m_Moves.begin(); prev_move != prev_step->m_Moves.end(); ++prev_move)
		    {
			if (move->is_Dependent(*prev_move))
			{
			    if (prev_move->m_crt_time >= move->m_crt_time)
			    {
				move->m_crt_time = prev_move->m_crt_time + 1;
				max_crt_time = sMAX(max_crt_time, move->m_crt_time);
			    }
			}
		    }
		}
		for (sMultirobotSolution::Moves_list::const_iterator prev_move = step->m_Moves.begin(); prev_move != move; ++prev_move)
		{
		    if (move->is_Dependent(*prev_move))
		    {
			if (prev_move->m_crt_time >= move->m_crt_time)
			{
			    move->m_crt_time = prev_move->m_crt_time + 1;
			    max_crt_time = sMAX(max_crt_time, move->m_crt_time);
			}
		    }
		}
	    }
	}

	return max_crt_time;
    }


    void sMultirobotSolution::criticalize_Solution(sMultirobotSolution &critical_solution)
    {
	calc_CriticalTimes();

	for (sMultirobotSolution::Steps_vector::iterator step =  m_Steps.begin(); step != m_Steps.end(); ++step)
	{
	    for (sMultirobotSolution::Moves_list::iterator move = step->m_Moves.begin(); move != step->m_Moves.end(); ++move)
	    {
		critical_solution.add_Move(move->m_crt_time, *move);
	    }
	}
    }


/*----------------------------------------------------------------------------*/

    void sMultirobotSolution::to_Screen(const sString &indent) const
    {
	to_Stream(stdout, indent);
    }


    void sMultirobotSolution::to_Stream(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sMulirobot solution: (|moves| = %d, paralellism = %.3f) [\n", indent.c_str(), m_Moves_cnt, (double)m_Moves_cnt / m_Steps.size());

	int N_Steps = m_Steps.size();
	for (int i = 0; i < N_Steps; ++i)
	{
	    const Step &step = m_Steps[i];
	    fprintf(fw, "%s%sStep %d: ", indent.c_str(), sRELOC_INDENT.c_str(), step.m_time);

	    for (Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	    {
		fprintf(fw, "%d#%d->%d ", move->m_robot_id, move->m_src_vrtx_id, move->m_dest_vrtx_id);
	    }
	    fprintf(fw, "\n");
	}

	fprintf(fw, "%s]\n", indent.c_str());
    }


    sResult sMultirobotSolution::to_File_multirobot(const sString &filename, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_SOLUTION_OPEN_ERROR;
	}
	
	to_Stream_multirobot(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    void sMultirobotSolution::to_Stream_multirobot(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sFine solution\n", indent.c_str());
	fprintf(fw, "%sLength:%d\n", indent.c_str(), get_MoveCount());
	int N_Steps = m_Steps.size();
	for (int i = 0; i < N_Steps; ++i)
	{
	    const Step &step = m_Steps[i];
	    for (Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	    {
		fprintf(fw, "%s%d # %d ---> %d (%d)\n", indent.c_str(), move->m_robot_id, move->m_src_vrtx_id, move->m_dest_vrtx_id, i);
	    }
	}
    }


    sResult sMultirobotSolution::from_File_multirobot(const sString &filename)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sMULTIROBOT_SOLUTION_OPEN_ERROR;
	}
	
	result = from_Stream_multirobot(fr);
	if (sFAILED(result))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolution::from_Stream_multirobot(FILE *fr)
    {
	int N_Moves;
	int c = fgetc(fr);

	while (c != 'F')
	{
	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
	}

	fscanf(fr, "ine solution\nLength:%d\n", &N_Moves);
        #ifdef sDEBUG
	printf("Length:%d\n", N_Moves);
	#endif

	for (int i = 0; i < N_Moves; ++i)
	{
	    int robot_id, src_vertex_id, dest_vertex_id, step;
	    fscanf(fr, "%d # %d ---> %d (%d)\n", &robot_id, &src_vertex_id, &dest_vertex_id, &step);
	    #ifdef sDEBUG
	    printf("%d # %d ---> %d (%d)\n", robot_id, src_vertex_id, dest_vertex_id, step);
	    #endif
	    add_Move(step, Move(robot_id, src_vertex_id, dest_vertex_id));
	}

	return sRESULT_SUCCESS;
    }




/*----------------------------------------------------------------------------*/
// sMultirobotFlowModel

    sMultirobotFlowModel::sMultirobotFlowModel(const sMultirobotInstance &instance)
	: m_instance(instance)
    {
	// nothing
    }


    void sMultirobotFlowModel::build_Network(int N_steps)
    {
	for (int step = 0; step < N_steps; ++step)
	{
	    sString step_label = sInt_32_to_String(step);
	    for (sUndirectedGraph::Vertices_vector::const_iterator vertex = m_instance.m_environment.m_Vertices.begin(); vertex != m_instance.m_environment.m_Vertices.end(); ++vertex)
	    {
		sString vertex_label = sInt_32_to_String(vertex->m_id);
		m_flow_network.add_Vertex(vertex_label + "_" + step_label);
	    }
	}
	for (int step = 0; step < N_steps - 1; ++step)
	{
	    sString step_label = sInt_32_to_String(step);	    
	    sString next_step_label = sInt_32_to_String(step + 1);

	    for (sUndirectedGraph::Vertices_vector::const_iterator vertex = m_instance.m_environment.m_Vertices.begin(); vertex != m_instance.m_environment.m_Vertices.end(); ++vertex)
	    {
		sString vertex_label = sInt_32_to_String(vertex->m_id);
	
		for (sVertex::Neighbors_list::const_iterator neighbor = vertex->m_Neighbors.begin(); neighbor != vertex->m_Neighbors.end(); ++neighbor)
		{
		    sString next_vertex_label = sInt_32_to_String((*neighbor)->m_target->m_id);		    
		    m_flow_network.add_Arc(vertex_label + "_" + step_label, next_vertex_label + "_" + next_step_label, 1, 0);
		}
		m_flow_network.add_Arc(vertex_label + "_" + step_label, vertex_label + "_" + next_step_label, 1, 0);
	    }	    
	}

	m_network_source = m_flow_network.add_Vertex("source");
	m_network_sink = m_flow_network.add_Vertex("sink");

	sString first_step_label = sInt_32_to_String(0);
	sString last_step_label = sInt_32_to_String(N_steps - 1);

	int N_robots = m_instance.m_initial_arrangement.get_RobotCount();
	sASSERT(N_robots == m_instance.m_goal_arrangement.get_RobotCount());

	for (int r = 1; r <= N_robots; ++r)
	{
	    sString init_vertex_label = sInt_32_to_String(m_instance.m_initial_arrangement.get_RobotLocation(r));
	    sString goal_vertex_label = sInt_32_to_String(m_instance.m_goal_arrangement.get_RobotLocation(r));

	    m_flow_network.add_Arc("source", init_vertex_label + "_" + first_step_label, 1, 0);
	    m_flow_network.add_Arc(goal_vertex_label + "_" + last_step_label, "sink", 1, 0);
	}
    }


    void sMultirobotFlowModel::build_Network(const Robots_vector &robot_selection, int N_steps)
    {	
	for (int step = 0; step < N_steps; ++step)
	{
	    sString step_label = sInt_32_to_String(step);
	    for (sUndirectedGraph::Vertices_vector::const_iterator vertex = m_instance.m_environment.m_Vertices.begin(); vertex != m_instance.m_environment.m_Vertices.end(); ++vertex)
	    {
		sString vertex_label = sInt_32_to_String(vertex->m_id);
		m_flow_network.add_Vertex(vertex_label + "_" + step_label);
	    }
	}
	for (int step = 0; step < N_steps - 1; ++step)
	{
	    sString step_label = sInt_32_to_String(step);	    
	    sString next_step_label = sInt_32_to_String(step + 1);

	    for (sUndirectedGraph::Vertices_vector::const_iterator vertex = m_instance.m_environment.m_Vertices.begin(); vertex != m_instance.m_environment.m_Vertices.end(); ++vertex)
	    {
		sString vertex_label = sInt_32_to_String(vertex->m_id);
	
		for (sVertex::Neighbors_list::const_iterator neighbor = vertex->m_Neighbors.begin(); neighbor != vertex->m_Neighbors.end(); ++neighbor)
		{
		    sString next_vertex_label = sInt_32_to_String((*neighbor)->m_target->m_id);		    
		    m_flow_network.add_Arc(vertex_label + "_" + step_label, next_vertex_label + "_" + next_step_label, 1, 0);
		}
		m_flow_network.add_Arc(vertex_label + "_" + step_label, vertex_label + "_" + next_step_label, 1, 0);
	    }	    
	}

	m_network_source = m_flow_network.add_Vertex("source");
	m_network_sink = m_flow_network.add_Vertex("sink");

	sString first_step_label = sInt_32_to_String(0);
	sString last_step_label = sInt_32_to_String(N_steps - 1);

	int N_robots = m_instance.m_initial_arrangement.get_RobotCount();
	sASSERT(N_robots == m_instance.m_goal_arrangement.get_RobotCount());

	for (Robots_vector::const_iterator r = robot_selection.begin(); r != robot_selection.end(); ++r)
	{
	    sString init_vertex_label = sInt_32_to_String(m_instance.m_initial_arrangement.get_RobotLocation(*r));
	    sString goal_vertex_label = sInt_32_to_String(m_instance.m_goal_arrangement.get_RobotLocation(*r));

	    m_flow_network.add_Arc("source", init_vertex_label + "_" + first_step_label, 1, 0);
	    m_flow_network.add_Arc(goal_vertex_label + "_" + last_step_label, "sink", 1, 0);
	}
    }


    void sMultirobotFlowModel::destroy_Network(void)
    {
	m_flow_network.clean_Graph();
    }


    int sMultirobotFlowModel::compute_Relocation(void)
    {
	sGoldberg goldberg;
	return goldberg.compute_Flow(&m_flow_network, &m_network_source->second, &m_network_sink->second);
    }


    int sMultirobotFlowModel::compute_Distance(void)
    {
	int N_robots = m_instance.m_initial_arrangement.get_RobotCount();
	sASSERT(N_robots == m_instance.m_goal_arrangement.get_RobotCount());

	int N_steps = 1;

	while (true)
	{
	    destroy_Network();
	    build_Network(N_steps);

	    int relocation = compute_Relocation();
	    if (relocation == N_robots)
	    {
		return N_steps;
	    }
	    ++N_steps;
	}
	return -1;
    }


    int sMultirobotFlowModel::compute_Distance(const Robots_vector &robot_selection)
    {
	int N_robots = robot_selection.size();
	int N_steps = 1;

	while (true)
	{
	    destroy_Network();
	    build_Network(robot_selection, N_steps);

	    int relocation = compute_Relocation();
	    if (relocation == N_robots)
	    {
		return N_steps;
	    }
	    ++N_steps;
	}
	return -1;
    }


    int sMultirobotFlowModel::compute_Distance(int *N_tries)
    {
	int N_robots = m_instance.m_initial_arrangement.get_RobotCount();
	sASSERT(N_robots == m_instance.m_goal_arrangement.get_RobotCount());

	int max_N_steps = 1;

	for (int N_r = 1; N_r <= N_robots; ++N_r)
	{
	    for (int t = 0; t < N_tries[N_r-1]; ++t)
	    {
		Robots_vector robot_selection;
		for (int r = 1; r <= N_robots; ++r)
		{
		    robot_selection.push_back(r);
		}
		int selection_size = N_robots;
		while (selection_size > N_r)
		{
		    int rnd_r = rand() % selection_size;
		    robot_selection[rnd_r] = robot_selection[selection_size - 1];

		    --selection_size;
		}
		robot_selection.resize(N_r);

		for (Robots_vector::const_iterator sr = robot_selection.begin(); sr != robot_selection.end(); ++sr)
		{
		    printf("%d,", *sr);
		}
		printf("\n");

		int N_steps = 1;
		while (true)
		{
		    destroy_Network();
		    build_Network(robot_selection, N_steps);
	    
		    int relocation = compute_Relocation();
		    if (relocation == N_r)
		    {
			break;
		    }
		    ++N_steps;
		}
		if (N_steps > max_N_steps)
		{
		    max_N_steps = N_steps;
		}
		printf("----> %d\n", N_steps);
	    }
	}
	return max_N_steps;
    }


/*----------------------------------------------------------------------------*/

} // namespace sReloc
