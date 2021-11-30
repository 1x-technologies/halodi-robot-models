cmake_minimum_required(VERSION 3.5)

project(eve_r3_description)


find_package(ament_cmake QUIET)
find_package(urdf QUIET)


find_program(GZ NAMES gz)
find_program(XMLLINT NAMES xmllint)
find_program(XSLTPROC NAMES xsltproc)

if(GZ)
    message(STATUS "Found gazebo: ${GZ}")
else()
    message(FATAL_ERROR "Cannot find Gazebo. Install gazebo9.")
endif()

if(XMLLINT)
    message(STATUS "Found xmllint: ${XMLLINT}")
else()
    message(FATAL_ERROR "Cannot find xmllint. Install libxml2-utils.")
endif()

if(XSLTPROC)
    message(STATUS "Found xsltproc: ${XSLTPROC}")
else()
    message(FATAL_ERROR "Cannot find xsltproc. Install xsltproc.")
endif()

SET(URDF_IN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/urdf.in")
SET(URDF_DIR "${CMAKE_CURRENT_SOURCE_DIR}/urdf")
SET(SDF_DIR "${CMAKE_CURRENT_SOURCE_DIR}/sdf")

add_custom_target(urdf_directory ALL
    COMMAND ${CMAKE_COMMAND} -E make_directory "${URDF_DIR}")

add_custom_target(sdf_directory ALL
    COMMAND ${CMAKE_COMMAND} -E make_directory "${SDF_DIR}")

    
function(CONVERT_URDF MODEL_NAME)
    
    SET(URDF_IN "${URDF_IN_DIR}/${MODEL_NAME}.in.urdf")
    SET(URDF_OUT "${URDF_DIR}/${MODEL_NAME}.urdf")
    SET(SDF_OUT "${SDF_DIR}/${MODEL_NAME}.sdf")

    FILE ( GLOB URDF_INPUTS "${URDF_IN_DIR}/*.urdf")

    add_custom_command(
    OUTPUT "${URDF_OUT}"
    COMMAND "${XMLLINT}" "${URDF_IN}" "--xinclude" "--output" "${URDF_OUT}" "--noent" "--format" 
    DEPENDS "${URDF_INPUTS}" urdf_directory)
    
    add_custom_command(
    OUTPUT "${SDF_OUT}"
    COMMAND "${GZ}" "sdf" "-v" "1.5" "-p" "${URDF_OUT}" "|" "${XSLTPROC}" "-o" "${SDF_OUT}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake/DeleteMaterial.xsl" "-"
    DEPENDS "${URDF_OUT}" sdf_directory)
    
    
    add_custom_target("${MODEL_NAME}" ALL DEPENDS "${SDF_OUT}")

endfunction(CONVERT_URDF)


CONVERT_URDF("eve_r3")
CONVERT_URDF("eve_r3_upper_body")
CONVERT_URDF("eve_r3_robotiq_2f_85")
CONVERT_URDF("eve_r3_robotiq_hand-e")


if(ament_cmake_FOUND)

    # Copy all model files to correct path setup for ROS
    install(DIRECTORY meshes urdf sdf 
    DESTINATION share/${PROJECT_NAME}/models/${PROJECT_NAME}
    )

    install(DIRECTORY meshes 
    DESTINATION share/${PROJECT_NAME}
    )
    install(FILES model.config
    DESTINATION share/${PROJECT_NAME}/models/${PROJECT_NAME}
    )


    ament_export_dependencies(urdf)
    ament_package()
endif()
