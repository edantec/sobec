set(${PROJECT_NAME}_FACTORY_TEST
  factory/activation.hpp
  factory/cost.hpp
  factory/pinocchio_model.hpp
  factory/random_generator.hpp
  factory/diff-action.hpp
  factory/actuation.hpp
  factory/activation.cpp
  factory/pinocchio_model.cpp
  factory/diff-action.cpp
  factory/actuation.cpp
  )

add_library(${PROJECT_NAME}_unittest SHARED ${${PROJECT_NAME}_FACTORY_TEST})
target_link_libraries(${PROJECT_NAME}_unittest PUBLIC ${PROJECT_NAME} crocoddyl::crocoddyl)

ADD_UNIT_TEST(test_diff_actions test_diff_actions.cpp)
target_link_libraries(test_diff_actions PUBLIC ${PROJECT_NAME}_unittest)

ADD_UNIT_TEST(test_walk_params test_walk_params.cpp)
target_link_libraries(test_walk_params PUBLIC ${PROJECT_NAME}_unittest)
target_compile_definitions(test_walk_params PRIVATE PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

if(BUILD_PYTHON_INTERFACE)
  ADD_UNIT_TEST(test_init_shooting_problem test_init_shooting_problem.cpp)
  target_link_libraries(test_init_shooting_problem PUBLIC ${PROJECT_NAME}_py2cpp)
  target_compile_definitions(test_init_shooting_problem PRIVATE PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
  #set_tests_properties(test_init_shooting_problem PROPERTIES ENVIRONMENT
  #   "PYTHONPATH=${PROJECT_SOURCE_DIR}/mpc:${PROJECT_BINARY_DIR}/python")

  add_subdirectory(python)
endif()
