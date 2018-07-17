cmake_minimum_required(VERSION 3.9)
project(CPFSolver)

set(CMAKE_CXX_STANDARD 17)

include_directories (CPFSolver/src glucose CPFSolver/src/encoders CPFSolver/src/searches CPFSolver/src/exceptions)
link_directories(glucose/simp)

add_executable(CPFSolver
        CPFSolver/src/encoders/Simplified.cpp
        CPFSolver/src/encoders/Simplified.h
        CPFSolver/src/searches/BinarySearch.h
        CPFSolver/src/searches/SAT_UNSATSearch.h
        CPFSolver/src/searches/UNSAT_SATSearch.h
        CPFSolver/src/Agent.h
        CPFSolver/src/CPFSolver.cpp
        CPFSolver/src/CPFSolver.h
        CPFSolver/src/Encoder.h
        CPFSolver/src/Graph.cpp
        CPFSolver/src/Graph.h
        CPFSolver/src/Instance.cpp
        CPFSolver/src/Instance.h
        CPFSolver/src/main.cpp
        CPFSolver/src/Parser.cpp
        CPFSolver/src/Parser.h
        CPFSolver/src/Search.h
        CPFSolver/src/Solution.cpp
        CPFSolver/src/Solution.h
        CPFSolver/src/exceptions/OutOfMemoryException.h
        CPFSolver/src/exceptions/TimeoutException.h CPFSolver/src/Group.h CPFSolver/src/Group.cpp CPFSolver/src/encoders/SimplifiedID.cpp CPFSolver/src/encoders/SimplifiedID.h CPFSolver/src/EncoderID.h)

target_link_libraries(CPFSolver
        _standard)
