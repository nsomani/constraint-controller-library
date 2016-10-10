cmake_minimum_required(VERSION 2.8.3)
project(constraint-controller-library)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ".")
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

find_package(OpenGL REQUIRED)
set(QT_USE_IMPORTED_TARGETS ON)
find_package(Qt4 COMPONENTS QtCore QtGui QtNetwork QtOpenGL)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(QT_USE_QTMAIN ON)
include(${QT_USE_FILE})

find_package(RL 0.6.2 COMPONENTS MATH HAL MDL REQUIRED)
find_package(LibXml2 REQUIRED)
find_package(Eigen REQUIRED)
find_package(Bullet REQUIRED)

ADD_DEFINITIONS(
  ${RL_DEFINITIONS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(
    ${QT_INCLUDES}
    ${Boost_INCLUDE_DIR}
    ${EIGEN_INCLUDE_DIRS}
    ${LIBXML2_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIR}
    ${RL_INCLUDE_DIRS}
)

set (HDRS
    src/constraint/ConstraintUtils.h
    src/constraint/ObjectModel.h
    src/constraint/Constraint.h
    src/constraint/GeometricConstraints.h
    src/constraint/RobotConstraints.h
)

set (SRCS
    src/constraint/ObjectModel.cpp
    src/constraint/Constraint.cpp
    src/constraint/GeometricConstraints.cpp
    src/constraint/RobotConstraints.cpp
)

add_library(
    constraints
    SHARED
    ${SRCS}
    ${HDRS}
)

set (SOLVER_HDRS
    src/solver/SolverUtils.h
    src/solver/SolverNLOpt.h
    src/solver/SolverNLOptGenericKinematic.h
    src/solver/SolverNLOptGenericGeometric.h
    src/RobotControl.h
)

set (SOLVER_SRCS
    src/solver/SolverUtils.cpp
    src/solver/SolverNLOpt.cpp
    src/solver/SolverNLOptGenericKinematic.cpp
    src/solver/SolverNLOptGenericGeometric.cpp
    src/RobotControl.cpp
)

add_library(
    constraint_solvers
    SHARED
    ${SOLVER_SRCS}
    ${SOLVER_HDRS}
)

set (SKILL_HDRS
    src/skill/ConstraintSkill.h
    src/skill/CompositeSkill.h
    src/skill/CompositePrioritySkill.h
    src/skill/CollisionAvoidanceSkill.h
    src/ConstraintParser.h
)

set (SKILL_SRCS
    src/skill/ConstraintSkill.cpp
    src/skill/CompositeSkill.cpp
    src/skill/CompositePrioritySkill.cpp
    src/skill/CollisionAvoidanceSkill.cpp
    src/ConstraintParser.cpp
)

add_library(
    constraint_skills
    SHARED
    ${SKILL_SRCS}
    ${SKILL_HDRS}
)

target_link_libraries(constraints
  ${RL_LIBRARIES}
  ${QT_QTCORE_LIBRARY}
  ${Boost_LIBRARIES}
  ${LIBXML2_LIBRARIES}
  jsoncpp
)

target_link_libraries(constraint_solvers
  constraints
  ${RL_LIBRARIES}
  ${QT_QTCORE_LIBRARY}
  ${Boost_LIBRARIES}
  ${LIBXML2_LIBRARIES}
  nlopt
  jsoncpp
)

target_link_libraries(constraint_skills
  constraints
  constraint_solvers
  ${RL_LIBRARIES}
  ${QT_QTCORE_LIBRARY}
  ${Boost_LIBRARIES}
  ${LIBXML2_LIBRARIES}
  nlopt
  jsoncpp
)

QT4_WRAP_UI( UI_HEADERS test/JoggingGUI.ui)
QT4_WRAP_CPP( UI_SRCS test/JoggingGUI.h)

add_executable(run_demo test/testGUI.cpp test/JoggingGUI.h ${UI_HEADERS} ${UI_SRCS} test/JoggingGUI.cpp test/Evaluation.h test/Evaluation.cpp)

target_link_libraries(run_demo
  constraints
  constraint_solvers
  constraint_skills
  ${QT_LIBRARIES}
  ${RL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${LIBXML2_LIBRARIES}
  nlopt
  jsoncpp
)

add_executable(demo_eval test/eval_robot_control.cpp test/Evaluation.h test/Evaluation.cpp)

target_link_libraries(demo_eval
  constraints
  constraint_solvers
  constraint_skills
  ${QT_LIBRARIES}
  ${RL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${LIBXML2_LIBRARIES}
  nlopt
  jsoncpp
)

execute_process(
        COMMAND ${CMAKE_COMMAND} -E create_symlink
        ${CMAKE_CURRENT_SOURCE_DIR}/models
        ${CMAKE_CURRENT_BINARY_DIR}/models
)
