cmake_minimum_required(VERSION 3.5)
project(robotiq_hand-e)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)


ament_export_dependencies(urdf)
ament_package()

# Copy all model files to correct path setup for ROS
install(DIRECTORY meshes
    DESTINATION share/${PROJECT_NAME}/models/${PROJECT_NAME}
)
install(FILES model.config
    DESTINATION share/${PROJECT_NAME}/models/${PROJECT_NAME}
)
