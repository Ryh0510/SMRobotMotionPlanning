cmake_minimum_required(VERSION 3.20)

set(${TARGET_NAME}_RequiredLibsPublic
    SMRobotMotionPlanning::MotionPlanningCore
    SMRobotPlatform::SimulationProject
    SMRobotPlatform::SimulationRuntime
    osqpstatic
)

set(${TARGET_NAME}_RequiredLibsPrivate
    nlohmann_json::nlohmann_json
    SMRobotMotionPlanning::MotionPlanningOmpl
)
