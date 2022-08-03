set(${PROJECT_NAME}_FACTORY_TEST
  factory/activation.hpp
  factory/pinocchio_model.hpp
  factory/random_generator.hpp
  factory/activation.cpp
  factory/pinocchio_model.cpp
  )

add_library(${PROJECT_NAME}_unittest SHARED ${${PROJECT_NAME}_FACTORY_TEST})
target_link_libraries(${PROJECT_NAME}_unittest PUBLIC ${PROJECT_NAME} crocoddyl::crocoddyl)

