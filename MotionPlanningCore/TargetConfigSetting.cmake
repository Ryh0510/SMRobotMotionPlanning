cmake_minimum_required(VERSION 3.20)

set(${TARGET_NAME}_RequiredLibsPublic
    SMRobotCore::RobotTrajectoryCore
)

set(${TARGET_NAME}_RequiredLibsPrivate
    SMRobotPlatform::SimulationProject
    nlohmann_json::nlohmann_json
)
