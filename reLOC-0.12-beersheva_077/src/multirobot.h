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
// multirobot.h / 0.12-beersheva_077
/*----------------------------------------------------------------------------*/
//
// Multirobot coordinated path-finding solving package.
//
/*----------------------------------------------------------------------------*/


#ifndef __MULTIROBOT_H__
#define __MULTIROBOT_H__

#include <vector>
#include <list>
#include <set>
#include <map>

#include "types.h"
#include "result.h"
#include "reloc.h"
#include "cnf.h"
#include "graph.h"


using namespace std;
using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{


/*----------------------------------------------------------------------------*/
// sRobotArrangement

    class sRobotArrangement
    {
    public:
	static const int VACANT_VERTEX = 0; /* Robots are numbered starting with 1 */
	static const int UNDEFINED_LOCATION = -1;

	static const int RANDOM_WALK_LENGTH = sDEFAULT_RANDOM_WALK_LENGTH;

    public:
	typedef std::vector<int> Robots_vector;
	typedef std::vector<int> Vertices_vector;
	typedef std::vector<int> VertexIDs_vector;

    public:
	sRobotArrangement();
	sRobotArrangement(int N_Vertices, int N_Robots, bool random = false);
	sRobotArrangement(const sRobotArrangement &initial_arrangement, int N_Vertices, int N_Robots, bool random = false);
	sRobotArrangement(const sRobotArrangement &initial_arrangement, int N_Vertices, int N_Robots, int N_fixed, bool random = false);
	sRobotArrangement(const sRobotArrangement &robot_arrangement);
	const sRobotArrangement& operator=(const sRobotArrangement &robot_arrangement);

	bool operator==(const sRobotArrangement &robot_arrangement) const;
	bool operator<(const sRobotArrangement &robot_arrangement) const;

	int get_RobotCount(void) const;
	int get_VertexCount(void) const;

	int get_RobotLocation(int robot_id) const;
	int get_VertexOccupancy(int vertex_id) const;

	void place_Robot(int robot_id, int vertex_id);
	void remove_Robot(int robot_id);
	void clean_Vertex(int vertex_id);

	void move_Robot(int robot_id, int dest_vertex_id);
	bool verify_Move(int robot_id, int dest_vertex_id) const;
	bool check_Move(int robot_id, int dest_vertex_id) const;
	bool verify_Move(int robot_id, int dest_vertex_id, const sUndirectedGraph &graph) const;

	void randomize(void);

	void generate_Walk(const sRobotArrangement &initial_arrangement, const sUndirectedGraph &environment);
	void generate_Walk(const sRobotArrangement &initial_arrangement, const sUndirectedGraph &environment, int N_fixed);

	void generate_Equidistant(const sRobotArrangement &initial_arrangement, sUndirectedGraph &environment, int distance);

    public:
	virtual void to_Screen(const sString &indent = "") const;
	virtual void to_Stream(FILE *fw, const sString &indent = "") const;

	virtual sResult to_File_multirobot(const sString &filename, const sString &indent = "") const;
	virtual void to_Stream_multirobot(FILE *fw, const sString &indent = "") const;

	virtual sResult from_File_multirobot(const sString &filename, int component = 0);
	virtual sResult from_Stream_multirobot(FILE *fr, int component = 0);

    public:
	Robots_vector m_robot_Locs;
	Vertices_vector m_vertex_Occups;
    };


/*----------------------------------------------------------------------------*/
// sRobotGoal

    class sRobotGoal
    {
    public:
	static const int UNDEFINED_LOCATION = sRobotArrangement::UNDEFINED_LOCATION;

    public:
	typedef std::set<int> Vertices_set;
	typedef std::vector<Vertices_set> Goals_vector;

	typedef std::set<int> Robots_set;
	typedef std::vector<Robots_set> Compats_vector;

    public:
	sRobotGoal();
	sRobotGoal(int N_Vertices, int N_Robots);
	sRobotGoal(int N_Vertices, int N_Robots, int N_Goals);
	sRobotGoal(const sRobotArrangement &initial_arrangement, int N_Vertices, int N_Robots, int N_Goals);
	sRobotGoal(const sRobotArrangement &robot_arrangement);
	sRobotGoal(const sRobotGoal &robot_goal);
	const sRobotGoal& operator=(const sRobotGoal &robot_goal);

	int get_RobotCount(void) const;
	int get_VertexCount(void) const;

	const Vertices_set& get_RobotGoal(int robot_id) const;
	Vertices_set& provide_RobotGoal(int robot_id);

	const Robots_set& get_GoalCompatibility(int goal_id) const;
	Robots_set& provide_GoalCompatibility(int goal_id);

	void charge_Robot(int robot_id, int goal_id);
	void charge_Robot(int robot_id, const Vertices_set &goal_IDs);

	void assign_Goal(int goal_id, int robot_id);
	void assign_Goal(int goal_id, const Robots_set &robot_IDs);

	void discharge_Robot(int robot_id);
	void discharge_Robot(int robot_id, int goal_id);
	void discharge_Robot(int robot_id, const Vertices_set &goal_IDs);

	bool is_Satisfied(const sRobotArrangement &robot_arrangement) const;

    public:
	virtual void to_Screen(const sString &indent = "") const;
	virtual void to_Stream(FILE *fw, const sString &indent = "") const;

	virtual sResult to_File_multirobot(const sString &filename, const sString &indent = "") const;
	virtual void to_Stream_multirobot(FILE *fw, const sString &indent = "") const;
	virtual void to_Stream_multirobot(FILE *fw, const Robots_set &robot_IDs, const sString &indent = "") const;

	virtual sResult from_File_multirobot(const sString &filename, int component = 0);
	virtual sResult from_Stream_multirobot(FILE *fr, int component = 0);
	virtual sResult from_Stream_multirobot(FILE *fr, Robots_set &robot_IDs);

    public:
	Goals_vector m_robot_Goals;
	Compats_vector m_goal_Compats;
    };


/*----------------------------------------------------------------------------*/
// sMultirobotCNFEncodingContext
    
    class sMultirobotEncodingContext_CNFsat
    {	
    public:
	static const int UNDEFINED_LAYER_COUNT;

    public:
	enum GeneratingMode
	{
	    GENERATING_STANDARD,
	    GENERATING_ADVANCED,
	    GENERATING_BITWISE
	};

    public:
	typedef std::vector<sIndexableStateIdentifier> StateIdentifiers_vector;
	typedef std::vector<sIndexableBitIdentifier> BitIdentifiers_vector;
	typedef std::vector<StateIdentifiers_vector> StateIdentifiers_2d_vector;
	typedef std::vector<BitIdentifiers_vector> BitIdentifiers_2d_vector;
	typedef std::vector<BitIdentifiers_2d_vector> BitIdentifiers_3d_vector;

	typedef std::map<int, sIndexableStateIdentifier*, std::less<int> > StateIdentifiers_map;
	typedef std::map<int, sIndexableBitIdentifier*, std::less<int> > BitIdentifiers_map;

    public:
	sMultirobotEncodingContext_CNFsat();
	sMultirobotEncodingContext_CNFsat(int N_Layers);
	sMultirobotEncodingContext_CNFsat(const sMultirobotEncodingContext_CNFsat &encoding_context);
	const sMultirobotEncodingContext_CNFsat& operator=(const sMultirobotEncodingContext_CNFsat &encoding_context);

	GeneratingMode get_GeneratingMode(void) const;
	
	void switch_GeneratingMode(GeneratingMode generating_mode);
	void switchTo_StandardGeneratingMode(void);
	void switchTo_AdvancedGeneratingMode(void);
	void switchTo_BitwiseGeneratingMode(void);

	void register_InternalIdentifiers(void);
	void register_TranslateIdentifier(sIndexableStateIdentifier &state_identifier);
	void register_TranslateIdentifier(sIndexableBitIdentifier &bit_identifier);

	sSpecifiedIdentifier translate_CNF_Variable(int cnf_variable) const;

    public:
	int m_N_Layers;
	int m_max_total_cost;
	int m_extra_cost;
	sVariableStore_CNF m_variable_store;

	GeneratingMode m_generating_mode;

	sStateClauseGenerator *m_clause_generator;
	sStateClauseGenerator m_state_clause_generator;
	sAdvancedClauseGenerator m_advanced_clause_generator;
	sBitwiseClauseGenerator m_bitwise_clause_generator;

	sBitClauseGenerator *m_bit_generator;
	sBitClauseGenerator m_bit_clause_generator;

	sIndexableStateIdentifier m_vertex_occupancy;
	sIndexableStateIdentifier m_robot_location;
	sIndexableStateIdentifier m_transition_action;
	sIndexableBitIdentifier m_vertex_occupancy_by_water;
	BitIdentifiers_2d_vector m_vertex_occupancy_by_water_;
	BitIdentifiers_2d_vector m_vertex_water_cardinality_;

	sIndexableBitIdentifier m_vertex_occupancy_by_robot;
	sIndexableBitIdentifier m_robot_location_in_vertex;

	BitIdentifiers_vector m_edge_occupancy_by_water;
	BitIdentifiers_2d_vector m_edge_occupancy_by_water_;
	BitIdentifiers_3d_vector m_edge_occupancy_by_water__;
	StateIdentifiers_vector m_transition_Actions;

	StateIdentifiers_map m_state_Identifiers;
	BitIdentifiers_map m_bit_Identifiers;
    };


/*----------------------------------------------------------------------------*/
// sMultirobotInstance

    class sMultirobotInstance
    {
    public:
	enum GoalType
	{
	    GOAL_TYPE_UNDEFINED,
	    GOAL_TYPE_ARRANGEMENT,
	    GOAL_TYPE_SPECIFICATION
	};

	typedef std::vector<sUndirectedGraph> Environments_vector;

	typedef std::vector<int> VertexIDs_vector;
	typedef std::vector<VertexIDs_vector> RobotMDD_vector;
	typedef std::vector<RobotMDD_vector> MDD_vector;

    public:
	sMultirobotInstance();
	sMultirobotInstance(const sUndirectedGraph &environment, const sRobotArrangement &initial_arrangement, const sRobotArrangement &goal_arrangement);

	sMultirobotInstance(const sUndirectedGraph  &environment,
			    const sUndirectedGraph  &sparse_environment,
			    const sRobotArrangement &initial_arrangement,
			    const sRobotArrangement &goal_arrangement);

	sMultirobotInstance(const sUndirectedGraph &environment, const sRobotArrangement &initial_arrangement, const sRobotGoal &goal_specification);

	sMultirobotInstance(const sUndirectedGraph  &environment,
			    const sUndirectedGraph  &sparse_environment,
			    const sRobotArrangement &initial_arrangement,
			    const sRobotGoal        &goal_specification);

	sMultirobotInstance(const sMultirobotInstance &multirobot_instance);
	const sMultirobotInstance& operator=(const sMultirobotInstance &multirobot_instance);

    public:

	int analyze_EdgeHeights(int max_total_cost);
	int analyze_EdgeHeights_(int max_total_cost, int &max_vertex_height);
	int analyze_EdgeHeights_(int max_total_cost, int &max_vertex_height, sUndirectedGraph::VertexIDs_set &reachable_IDs);

	int build_HeightedEnvironments(int max_total_cost);
	int build_HeightedEnvironments(int max_total_cost, Environments_vector &heighted_Environments);

	int build_HeightedEnvironments_(int max_total_cost);
	int build_HeightedEnvironments_(int max_total_cost, Environments_vector &heighted_Environments);
        /*----------------------------------------------------------------------------*/

	void collect_Endpoints(VertexIDs_vector &source_IDs, VertexIDs_vector &goal_IDs);

	int estimate_TotalCost(int &max_individual_cost);
	int construct_MDD(int max_total_cost, MDD_vector &MDD, int &extra_cost, MDD_vector &extra_MDD);
	int construct_NoMDD(int max_total_cost, MDD_vector &MDD, int &extra_cost, MDD_vector &extra_MDD);

	void construct_MakespanMDD(int max_makespan, MDD_vector &MDD);
        /*----------------------------------------------------------------------------*/

	virtual void to_Screen(const sString &indent = "") const;
	virtual void to_Screen_multirobot(const sString &indent = "") const;
	virtual void to_Screen_domainPDDL(const sString &indent = "") const;
	virtual void to_Screen_problemPDDL(const sString &indent = "") const;
	virtual void to_Screen_bgu(const sString &indent = "", int instance_id = -1) const;
	virtual void to_Screen_InverseCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false) const;
	virtual void to_Screen_AdvancedCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false) const;
	virtual void to_Screen_DifferentialCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false) const;
	virtual void to_Screen_BijectionCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false) const;
	virtual void to_Screen_HeuristicDifferentialCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual void to_Screen_HeuristicBijectionCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual void to_Screen_HeuristicAdvancedCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual void to_Screen_PuzzleCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false) const;
	virtual void to_Screen_BitwiseCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual void to_Screen_FlowCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false) const;
	virtual void to_Screen_MatchingCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false) const;
	virtual void to_Screen_HeuristicMatchingCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual void to_Screen_DirectCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false) const;
	virtual void to_Screen_HeuristicDirectCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual void to_Screen_SimplicialCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false) const;
	virtual void to_Screen_HeuristicSimplicialCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual void to_Screen_SingularCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false) const;
	virtual void to_Screen_PluralCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false) const;
      	virtual void to_Screen_Plural2CNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual void to_Screen_HeightedCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual void to_Screen_MddCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual void to_Screen_MmddCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual void to_Screen_RXMddCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual void to_Screen_NoMddCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual void to_Screen_RXNoMddCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);

	virtual sResult to_File(const sString &filename, const sString &indent = "") const;
	virtual sResult to_File_multirobot(const sString &filename, const sString &indent = "") const;
	virtual sResult to_File_domainPDDL(const sString &filename, const sString &indent = "") const;
	virtual sResult to_File_problemPDDL(const sString &filename, const sString &indent = "") const;
	virtual sResult to_File_bgu(const sString &filename, const sString &indent = "", int instance_id = -1) const;
	virtual sResult to_File_InverseCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false) const;
	virtual sResult to_File_AdvancedCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false) const;
	virtual sResult to_File_DifferentialCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false) const;
	virtual sResult to_File_BijectionCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false) const;
	virtual sResult to_File_HeuristicDifferentialCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual sResult to_File_HeuristicBijectionCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual sResult to_File_HeuristicAdvancedCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual sResult to_File_PuzzleCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false) const;
	virtual sResult to_File_BitwiseCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual sResult to_File_FlowCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false) const;
	virtual sResult to_File_MatchingCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false) const;
	virtual sResult to_File_HeuristicMatchingCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual sResult to_File_DirectCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false) const;
	virtual sResult to_File_HeuristicDirectCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual sResult to_File_SimplicialCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false) const;
	virtual sResult to_File_HeuristicSimplicialCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual sResult to_File_SingularCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false) const;
	virtual sResult to_File_PluralCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false) const;
	virtual sResult to_File_Plural2CNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual sResult to_File_HeightedCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual sResult to_File_MddCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual sResult to_File_MmddCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual sResult to_File_RXMddCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual sResult to_File_NoMddCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual sResult to_File_RXNoMddCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);

	virtual void to_Stream(FILE *fw, const sString &indent = "") const;
	virtual void to_Stream_multirobot(FILE *fw, const sString &indent = "") const;
	virtual void to_Stream_domainPDDL(FILE *fw, const sString &indent = "") const;
	virtual void to_Stream_problemPDDL(FILE *fw, const sString &indent = "") const;
	virtual void to_Stream_bgu(FILE *fw, const sString &indent = "", int instance_id = -1) const;
	virtual void to_Stream_InverseCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false) const;
	virtual void to_Stream_AdvancedCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false) const;
	virtual void to_Stream_DifferentialCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false) const;
	virtual void to_Stream_BijectionCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false) const;
	virtual void to_Stream_HeuristicDifferentialCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual void to_Stream_HeuristicBijectionCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual void to_Stream_HeuristicAdvancedCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual void to_Stream_PuzzleCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false) const;
	virtual void to_Stream_BitwiseCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual void to_Stream_FlowCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false) const;
	virtual void to_Stream_MatchingCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false) const;
	virtual void to_Stream_HeuristicMatchingCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual void to_Stream_DirectCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false) const;
	virtual void to_Stream_HeuristicDirectCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual void to_Stream_SimplicialCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false) const;
	virtual void to_Stream_HeuristicSimplicialCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual void to_Stream_SingularCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false) const;
	virtual void to_Stream_PluralCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false) const;
	virtual void to_Stream_Plural2CNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual void to_Stream_HeightedCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual void to_Stream_MddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual void to_Stream_MmddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual void to_Stream_RXMddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual void to_Stream_NoMddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);
	virtual void to_Stream_RXNoMddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent = "", bool verbose = false);

	virtual void to_Stream_MddCNFsat(FILE                              *fw,
					 sMultirobotEncodingContext_CNFsat &encoding_context,
					 int                                extra_cost,
					 int                                mdd_depth,
					 const MDD_vector                  &MDD,
					 const MDD_vector                  &extra_MDD,
					 const sString                     &indent = "",
					 bool                              verbose = false);

	virtual void to_Stream_MmddCNFsat(FILE                              *fw,
					  sMultirobotEncodingContext_CNFsat &encoding_context,
					  const MDD_vector                  &MDD,
					  const sString                     &indent = "",
					  bool                              verbose = false);

	virtual void to_Stream_RXMddCNFsat(FILE                              *fw,
					   sMultirobotEncodingContext_CNFsat &encoding_context,
					   int                                extra_cost,
					   int                                mdd_depth,
					   const MDD_vector                  &MDD,
					   const MDD_vector                  &extra_MDD,
					   const sString                     &indent = "",
					   bool                              verbose = false);

    public:
	GoalType m_goal_type;

	sUndirectedGraph m_environment;
	sUndirectedGraph m_sparse_environment;

	Environments_vector m_heighted_Environments;

	sRobotArrangement m_initial_arrangement;
	sRobotArrangement m_goal_arrangement;
	sRobotGoal m_goal_specification;
    };


/*----------------------------------------------------------------------------*/
// sMultirobotSolution

    class sMultirobotSolution
    {
    public:
	static const int N_STEPS_UNDEFINED = -1;

    public:
	struct Move
	{
	    Move(int robot_id, int src_vrtx_id, int dest_vrtx_id);
	    Move(int robot_id, int src_vrtx_id, int dest_vrtx_id, int crt_time);

	    bool is_Undefined(void) const;
	    bool is_Dependent(const Move &move) const;

	    int m_robot_id;
	    int m_src_vrtx_id;
	    int m_dest_vrtx_id;

	    int m_crt_time;
	};

	static const Move UNDEFINED_MOVE;

	typedef std::list<Move> Moves_list;

	struct Step
	{
	    Step(int time);

	    int m_time;
	    Moves_list m_Moves;
	};

	typedef std::vector<Step> Steps_vector;
	
    public:
	sMultirobotSolution();
	sMultirobotSolution(int start_step, const sMultirobotSolution &sub_solution);
	sMultirobotSolution(const sMultirobotSolution &sub_solution_1, const sMultirobotSolution sub_solution_2);
	sMultirobotSolution(const sMultirobotSolution &multirobot_solution);
	const sMultirobotSolution& operator=(const sMultirobotSolution &multirobot_solution);

	bool is_Null(void) const;

	int get_MoveCount(void) const;
	int get_StepCount(void) const;

	void add_Move(int time, const Move &move);

	int calc_EmptySteps(void) const;
	void remove_EmptySteps(void);

	sMultirobotSolution extract_Subsolution(int start_step, int final_step) const;

	void execute_Solution(const sRobotArrangement &initial_arrangement,
			      sRobotArrangement       &final_arrangement,
			      int                      N_Steps = N_STEPS_UNDEFINED) const;

	void execute_Step(const sRobotArrangement &current_arrangement,
			  sRobotArrangement       &final_arrangement,
			  int                      step) const;

	void execute_Solution(sRobotArrangement &arrangement,
			      int                N_Steps = N_STEPS_UNDEFINED) const;

	void execute_Step(sRobotArrangement &final_arrangement,
			  int                step) const;

	bool verify_Step(const sRobotArrangement &arrangement,
			 int                      step) const;

	bool check_Step(const sRobotArrangement &arrangement,
			int                      step) const;

	void filter_Solution(const sRobotArrangement &initial_arrangement,
			     const sRobotGoal        &goal_arrangement,
			     sMultirobotSolution     &filter_solution) const;

	int calc_CriticalTimes(void);
	void criticalize_Solution(sMultirobotSolution &critical_solution);

    public:
	virtual void to_Screen(const sString &indent = "") const;
	virtual void to_Stream(FILE *fw, const sString &indent = "") const;

	virtual sResult to_File_multirobot(const sString &filename, const sString &indent = "") const;
	virtual void to_Stream_multirobot(FILE *fw, const sString &indent = "") const;

	virtual sResult from_File_multirobot(const sString &filename);
	virtual sResult from_Stream_multirobot(FILE *fr);

    public:
	int m_Moves_cnt;
	Steps_vector m_Steps;
    };


/*----------------------------------------------------------------------------*/
// sMultirobotFlowModel

    class sMultirobotFlowModel
    {
    public:
	typedef std::vector<int> Robots_vector;

    public:
	sMultirobotFlowModel(const sMultirobotInstance &instance);

	void build_Network(int N_steps);
	void build_Network(const Robots_vector &robot_selection, int N_steps);
	void destroy_Network(void);

	int compute_Relocation(void);

	int compute_Distance(void);
	int compute_Distance(const Robots_vector &robot_selection);
	int compute_Distance(int *N_tries);

    public:
	sMultirobotInstance m_instance;
	sDigraph m_flow_network;

	sDigraph::Vertices_map::iterator m_network_source;
	sDigraph::Vertices_map::iterator m_network_sink;
    };



/*----------------------------------------------------------------------------*/

} // namespace sReloc

#endif /* __MULTIROBOT_H__ */
