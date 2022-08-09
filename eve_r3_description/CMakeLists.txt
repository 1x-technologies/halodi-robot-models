cmake_minimum_required(VERSION 3.5)

project(eve_r3_description)

find_package(ament_cmake QUIET)
find_package(urdf QUIET)

find_program(GZ NAMES gz)
find_program(XMLLINT NAMES xmllint)
find_program(XSLTPROC NAMES xsltproc)
find_package(xacro REQUIRED)

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
SET(XACRO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/urdf/xacro")
SET(SDF_DIR "${CMAKE_CURRENT_SOURCE_DIR}/sdf")

add_custom_target(urdf_directory ALL
    COMMAND ${CMAKE_COMMAND} -E make_directory "${URDF_DIR}")

add_custom_target(xacro_directory ALL
    COMMAND ${CMAKE_COMMAND} -E make_directory "${XACRO_DIR}")

add_custom_target(sdf_directory ALL
    COMMAND ${CMAKE_COMMAND} -E make_directory "${SDF_DIR}")

function(CONVERT_URDF MODEL_NAME USE_XACRO)
    SET(URDF_IN "${URDF_IN_DIR}/${MODEL_NAME}.in.urdf")
    SET(XACRO_OUT "${XACRO_DIR}/${MODEL_NAME}.urdf.xacro")
    SET(URDF_OUT "${URDF_DIR}/${MODEL_NAME}.urdf")
    SET(SDF_OUT "${SDF_DIR}/${MODEL_NAME}.sdf")

    FILE(GLOB URDF_INPUTS "${URDF_IN_DIR}/*.urdf")

    if(NOT USE_XACRO)
        set(XACRO_OUT "${URDF_OUT}")
    endif()

    add_custom_command(
        OUTPUT "${XACRO_OUT}"
        COMMAND "${XMLLINT}" "${URDF_IN}" "--xinclude" "--output" "${XACRO_OUT}" "--noent" "--format"
        DEPENDS "${URDF_INPUTS}" urdf_directory)

    if(USE_XACRO)
        xacro_add_files("${XACRO_OUT}" OUTPUT "${URDF_OUT}" TARGET "${MODEL_NAME}" DEPENDS xacro_directory)
    endif()

    add_custom_command(
        OUTPUT "${SDF_OUT}"
        COMMAND "${GZ}" "sdf" "-v" "1.5" "-p" "${URDF_OUT}" "|" "${XSLTPROC}" "-o" "${SDF_OUT}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake/DeleteMaterial.xsl" "-"
        DEPENDS "${URDF_OUT}" sdf_directory)

    add_custom_target("${MODEL_NAME}" ALL DEPENDS "${SDF_OUT}")
endfunction(CONVERT_URDF MODEL_NAME)

CONVERT_URDF("eve_r3" FALSE)
CONVERT_URDF("eve_r3.collision" TRUE)
CONVERT_URDF("eve_r3_upper_body" FALSE)
CONVERT_URDF("eve_r3_robotiq_2f_85" FALSE)
CONVERT_URDF("eve_r3_robotiq_hand-e" FALSE)

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
    install(DIRECTORY launch rviz
        DESTINATION share/${PROJECT_NAME}
    )

    # for compatibility with reading the mesh path from the urdf
    install(DIRECTORY meshes
        DESTINATION share/${PROJECT_NAME}
    )

    ament_export_dependencies(urdf)
    ament_package()
endif()
