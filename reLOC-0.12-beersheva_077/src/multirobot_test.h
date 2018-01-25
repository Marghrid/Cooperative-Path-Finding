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
// multirobot_test.h / 0.12-beersheva_077
/*----------------------------------------------------------------------------*/
//
// Multirobot coordinated path-finding - testing program.
//
/*----------------------------------------------------------------------------*/


#ifndef __MULTIROBOT_TEST_H__
#define __MULTIROBOT_TEST_H__

#include "hierarch.h"

using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{


/*----------------------------------------------------------------------------*/

    void test_robot_arrangement_1(void);
    void test_robot_goal_1(void);

    void test_multirobot_instance_1(void);

    void test_multirobot_solution_1(void);

    void test_multirobot_solver_ID_1(void);
    void test_multirobot_solver_ID_Astar_1(int x, int y, int N_Robots);

    void test_multirobot_solver_Astar_1(int x, int y, int N_Robots);
    void test_multirobot_solver_Astar_2(int x, int y, int N_Robots);
    void test_multirobot_solver_Astar_3(int x, int y, int N_Robots);
    void test_multirobot_solver_Astar_4(int x, int y, int N_Robots);

    void test_multirobot_solver_IDAstar_1(int x, int y, int N_Robots);

    void test_multirobot_PDDL_1(void);
    void test_multirobot_PDDL_2(void);
    void test_multirobot_PDDL_3(void);

    void test_multirobot_CNF_1(void);
    void test_multirobot_CNF_2(int N_Layers);
    void test_multirobot_CNF_3(int x, int y, int N_Robots, int N_Layers);

    void test_multirobot_solver_HCAstar_1(int x, int y, int N_Robots);
    void test_multirobot_solver_HCAstar_2(int x, int y, int N_Robots);

    void test_graph_1(void);
    void test_graph_2(void);
    void test_graph_3(void);
    void test_graph_4(void);
    void test_graph_5(void);
    void test_graph_6(int x, int y, int N_Robots);

    
/*----------------------------------------------------------------------------*/

} // namespace sReloc


#endif /* __MULTIROBOT_TEST_H__ */