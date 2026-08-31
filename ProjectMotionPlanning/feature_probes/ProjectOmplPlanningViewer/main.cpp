#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <CameraCore/CameraFactory.h>
#include <GLRuntime/GLRuntime.h>
#include <ProjectMotionPlanning/ProjectMotionPlanning.h>
#include <RenderCore/ShaderLibrary.h>
#include <RobotRenderBridge/CollisionRenderBridge.h>
#include <RobotRenderBridge/ProjectSceneRenderBridge.h>
#include <SceneCore/CameraNode.h>
#include <SceneCore/CollisionOverlayRenderConfig.h>
#include <SceneCore/DefaultLighting.h>
#include <SceneCore/Renderer.h>
#include <SceneCore/RenderPass/AxisPass.h>
#include <SceneCore/RenderPass/GridPass.h>
#include <SceneCore/RenderPass/MeshPass.h>
#include <SceneCore/RenderPass/PrimitivePass.h>
#include <SceneCore/RenderPass/TrajectoryPass.h>
#include <SimulationProject/ProjectIo.h>

#include <data_path.h>

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
    struct Options
    {
        std::filesystem::path projectPath = std::filesystem::path(PROJECT_SOURCE_PATH) /
            "config/projects/420-red4600-tool.sys.json";
        std::string robotId = "Red4600";
        std::string detectorId = "default_collision";
        std::vector<double> start = {
            0.659405553148032,
            -0.919209792140321,
            -0.844791768088929,
            -4.8446095729532,
            -1.72245218831655,
            2.56813378610512
        };
        std::vector<double> goal = {
            -0.934876835266256,
            -0.633262433922526,
            1.11553781428474,
            -5.91669352284723,
            1.09495329977677,
            5.42211279330075
        };
        std::uint32_t seed = 4204600u;
        double timeoutSeconds = 8.0;
        bool noWindow = false;
        bool hidden = false;
        int maxFrames = 0;
        std::filesystem::path saveTrajectoryPath;
    };

    struct AppContext
    {
        cameracore::CameraControllerPtr controller;
        double lastX = 0.0;
        double lastY = 0.0;
        bool firstMouse = true;
    };

    std::vector<double> parseVector(const std::string& text)
    {
        std::vector<double> values;
        std::stringstream stream(text);
        std::string token;
        while (std::getline(stream, token, ','))
            values.push_back(std::stod(token));
        return values;
    }

    void printUsage()
    {
        std::cout
            << "ProjectOmplPlanningViewer options:\n"
            << "  --project <file>\n"
            << "  --robot <id>\n"
            << "  --detector <id>\n"
            << "  --start <q1,q2,...>\n"
            << "  --goal <q1,q2,...>\n"
            << "  --seed <integer>\n"
            << "  --timeout <seconds>\n"
            << "  --no-window\n"
            << "  --hidden\n"
            << "  --max-frames <count>\n"
            << "  --save-trajectory <csv-file>\n";
    }

    bool parseOptions(int argc, char** argv, Options& options)
    {
        try
        {
            for (int index = 1; index < argc; ++index)
            {
                const std::string argument = argv[index];
                const auto value = [&](const char* name) -> std::string {
                    if (index + 1 >= argc)
                        throw std::runtime_error(std::string("Missing value for ") + name);
                    return argv[++index];
                };

                if (argument == "--help" || argument == "-h")
                {
                    printUsage();
                    return false;
                }
                if (argument == "--project")
                    options.projectPath = std::filesystem::u8path(value("--project"));
                else if (argument == "--robot")
                    options.robotId = value("--robot");
                else if (argument == "--detector")
                    options.detectorId = value("--detector");
                else if (argument == "--start")
                    options.start = parseVector(value("--start"));
                else if (argument == "--goal")
                    options.goal = parseVector(value("--goal"));
                else if (argument == "--seed")
                    options.seed = static_cast<std::uint32_t>(std::stoul(value("--seed")));
                else if (argument == "--timeout")
                    options.timeoutSeconds = std::stod(value("--timeout"));
                else if (argument == "--no-window")
                    options.noWindow = true;
                else if (argument == "--hidden")
                    options.hidden = true;
                else if (argument == "--max-frames")
                    options.maxFrames = std::max(0, std::stoi(value("--max-frames")));
                else if (argument == "--save-trajectory")
                    options.saveTrajectoryPath = std::filesystem::u8path(value("--save-trajectory"));
                else
                    throw std::runtime_error("Unknown option: " + argument);
            }
        }
        catch (const std::exception& exception)
        {
            std::cerr << exception.what() << "\n";
            printUsage();
            return false;
        }
        return true;
    }

    bool saveTrajectoryCsv(
        const std::filesystem::path& path,
        const std::vector<std::string>& jointNames,
        const robottrajectory::JointTrajectory& trajectory,
        std::string& errorMessage)
    {
        std::ofstream output(path);
        if (!output)
        {
            errorMessage = "Failed to open trajectory output: " + path.generic_u8string();
            return false;
        }
        output << "time";
        for (const std::string& jointName : jointNames)
            output << ',' << jointName;
        output << '\n';
        for (const robottrajectory::TimedJointPoint& point : trajectory.points)
        {
            output << point.time;
            for (double value : point.q)
                output << ',' << value;
            output << '\n';
        }
        return true;
    }

    void mouseCallback(GLFWwindow* window, double x, double y)
    {
        auto* app = static_cast<AppContext*>(glfwGetWindowUserPointer(window));
        if (app == nullptr || !app->controller)
            return;
        if (app->firstMouse)
        {
            app->lastX = x;
            app->lastY = y;
            app->firstMouse = false;
        }
        const float dx = static_cast<float>(x - app->lastX) * 0.8f;
        const float dy = static_cast<float>(app->lastY - y) * 0.8f;
        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
            app->controller->onMouseMove(dx, dy, 0);
        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS)
            app->controller->onMouseMove(dx, dy, 1);
        app->lastX = x;
        app->lastY = y;
    }

    void scrollCallback(GLFWwindow* window, double, double offset)
    {
        auto* app = static_cast<AppContext*>(glfwGetWindowUserPointer(window));
        if (app != nullptr && app->controller)
            app->controller->onScroll(static_cast<float>(offset) * 0.8f);
    }

    std::shared_ptr<scenecore::CameraNode> createCamera(scenecore::SceneGraph& graph)
    {
        auto camera = cameracore::CameraFactory::createOrbitCamera();
        camera->setUpAxis(cameracore::UpAxis::Z_UP);
        camera->lookAt(
            Eigen::Vector3f(4.3f, -4.3f, 3.0f),
            Eigen::Vector3f(1.0f, 0.0f, 0.7f),
            Eigen::Vector3f::UnitZ());
        auto node = std::make_shared<scenecore::CameraNode>(camera, "planningCamera");
        graph.addNode(node);
        graph.registerNode(node);
        return node;
    }

    glm::vec3 toGlm(const Eigen::Vector3d& value)
    {
        return glm::vec3(
            static_cast<float>(value.x()),
            static_cast<float>(value.y()),
            static_cast<float>(value.z()));
    }
}

int main(int argc, char** argv)
{
    Options options;
    if (!parseOptions(argc, argv, options))
        return 2;

    simulation_project::ProjectDocument document;
    std::string errorMessage;
    if (!simulation_project::loadProjectDocument(options.projectPath, document, &errorMessage))
    {
        std::cerr << "Project load failed: " << errorMessage << "\n";
        return 3;
    }

    motion_planning::ProjectPlanningRequest request;
    request.robotId = options.robotId;
    request.start = options.start;
    request.goal = options.goal;
    request.collisionDetectorIds = { options.detectorId };
    request.planner.randomSeed = options.seed;
    request.planner.timeoutSeconds = options.timeoutSeconds;
    request.validation.maxJointStep = 0.04;
    request.postProcess.duration = 6.0;
    request.postProcess.minimumWaypointCount = 80;

    std::unique_ptr<motion_planning::ProjectPlanningSceneSnapshot> scene =
        motion_planning::ProjectPlanningSceneBuilder::build(
            document,
            options.projectPath.parent_path(),
            request,
            &errorMessage);
    if (!scene)
    {
        std::cerr << "Planning scene build failed: " << errorMessage << "\n";
        return 4;
    }
    request.jointNames = scene->jointNames();

    const motion_planning::StateValidationResult startValidation = scene->validateState(request.start);
    if (!startValidation.valid)
    {
        std::cerr << "Start state invalid: " << startValidation.message << "\n";
        return 5;
    }
    const Eigen::Vector3d startTool = scene->attachmentWorldTransform("420_tool_attachment").translation();
    const motion_planning::StateValidationResult goalValidation = scene->validateState(request.goal);
    if (!goalValidation.valid)
    {
        std::cerr << "Goal state invalid: " << goalValidation.message << "\n";
        return 6;
    }
    const Eigen::Vector3d goalTool = scene->attachmentWorldTransform("420_tool_attachment").translation();
    if (scene->validateMotion(request.start, request.goal, request.validation).valid)
    {
        std::cerr << "Direct interpolation is collision-free; this fixture does not prove obstacle avoidance.\n";
        return 7;
    }

    const motion_planning::ProjectMotionPlanningService service;
    const motion_planning::MotionPlanningResult planning = service.plan(*scene, request);
    if (!planning.succeeded())
    {
        const std::string detail = planning.diagnostics.empty()
            ? "unknown planning failure"
            : planning.diagnostics.front().message;
        std::cerr << "Planning failed: " << detail << "\n";
        return 8;
    }

    for (std::size_t index = 1; index < planning.trajectory.points.size(); ++index)
    {
        if (!scene->validateMotion(
                planning.trajectory.points[index - 1].q,
                planning.trajectory.points[index].q,
                request.validation).valid)
        {
            std::cerr << "Final trajectory revalidation failed.\n";
            return 9;
        }
    }

    if (!options.saveTrajectoryPath.empty() &&
        !saveTrajectoryCsv(options.saveTrajectoryPath, scene->jointNames(), planning.trajectory, errorMessage))
    {
        std::cerr << errorMessage << "\n";
        return 10;
    }

    std::vector<Eigen::Vector3d> toolPath;
    toolPath.reserve(planning.trajectory.points.size());
    for (const robottrajectory::TimedJointPoint& point : planning.trajectory.points)
    {
        if (!scene->setState(point.q, &errorMessage))
        {
            std::cerr << "Path visualization state failed: " << errorMessage << "\n";
            return 11;
        }
        toolPath.push_back(scene->attachmentWorldTransform("420_tool_attachment").translation());
    }
    scene->setState(request.start, nullptr);

    std::cout << "Project OMPL planning succeeded without Qt.\n"
              << "  project=" << options.projectPath.generic_u8string() << "\n"
              << "  detector=" << options.detectorId << "\n"
              << "  waypoints=" << planning.trajectory.points.size() << "\n"
              << "  sampledStates=" << planning.sampledStateCount << "\n"
              << "  planningSeconds=" << planning.planningTimeSeconds << "\n"
              << "  startTool=" << startTool.transpose() << "\n"
              << "  goalTool=" << goalTool.transpose() << "\n";

    if (options.noWindow)
        return 0;

    if (!glfwInit())
    {
        std::cerr << "GLFW initialization failed.\n";
        return 12;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    if (options.hidden)
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);

    GLFWwindow* window = glfwCreateWindow(1280, 800, "Project OMPL Planning Viewer", nullptr, nullptr);
    if (window == nullptr)
    {
        glfwTerminate();
        std::cerr << "GLFW window creation failed.\n";
        return 13;
    }
    glfwMakeContextCurrent(window);
    gladLoadGL();
    if (!GLRuntime::instance().initialize())
    {
        glfwDestroyWindow(window);
        glfwTerminate();
        std::cerr << "OpenGL runtime initialization failed.\n";
        return 14;
    }

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);

    rendercore::ShaderLibrary::initialize();
    scenecore::SceneGraph sceneGraph;
    scenecore::Renderer renderer;
    renderer.initialize();
    const std::shared_ptr<scenecore::CameraNode> cameraNode = createCamera(sceneGraph);
    scenecore::DefaultLighting::createDefaultLighting(sceneGraph, 0.6f);

    auto collisionMeshPass = std::make_shared<scenecore::MeshPass>(
        rendercore::ShaderLibrary::getShader("MLRobotUBO"));
    auto collisionPrimitivePass = std::make_shared<scenecore::PrimitivePass>();
    auto collisionLinePass = std::make_shared<scenecore::TrajectoryPass>();
    scenecore::CollisionOverlayRenderConfig overlayConfig;
    overlayConfig.depthMode = scenecore::OverlayDepthMode::DepthTested;
    collisionMeshPass->setCollisionOverlayConfig(overlayConfig);
    collisionPrimitivePass->setCollisionOverlayConfig(overlayConfig);
    collisionLinePass->setCollisionOverlayConfig(overlayConfig);

    renderer.addPass(std::make_shared<scenecore::GridPass>(scenecore::GridType::FadedInfiniteGrid));
    renderer.addPass(std::make_shared<scenecore::MeshPass>(
        rendercore::ShaderLibrary::getShader("MLRobotUBO")));
    renderer.addPass(collisionMeshPass);
    renderer.addPass(std::make_shared<scenecore::AxisPass>());
    renderer.addPass(collisionPrimitivePass);
    renderer.addPass(collisionLinePass);

    robot_render::ProjectSceneRenderBridge renderBridge;
    renderBridge.build(scene->simulationRuntime(), sceneGraph);

    AppContext app;
    app.controller = cameracore::CameraFactory::createOrbitController(cameraNode->camera());
    glfwSetWindowUserPointer(window, &app);
    glfwSetCursorPosCallback(window, mouseCallback);
    glfwSetScrollCallback(window, scrollCallback);

    std::cout << "Controls: left drag rotate, right drag pan, wheel zoom, Esc close.\n";
    int renderedFrames = 0;
    const double animationStart = glfwGetTime();
    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(window, GLFW_TRUE);

        const double elapsed = glfwGetTime() - animationStart;
        const double duration = planning.trajectory.duration();
        const double trajectoryTime = duration > 0.0 ? std::fmod(elapsed, duration) : 0.0;
        const robottrajectory::TimedJointPoint sample =
            robottrajectory::RobotTrajectorySampler::evaluate(planning.trajectory, trajectoryTime);
        const motion_planning::StateValidationResult currentValidation = scene->validateState(sample.q);
        renderBridge.sync(scene->simulationRuntime());

        int width = 0;
        int height = 0;
        glfwGetFramebufferSize(window, &width, &height);
        glViewport(0, 0, width, height);
        cameraNode->camera()->setAspect(
            static_cast<float>(width) / static_cast<float>(std::max(1, height)));
        glClearColor(0.05f, 0.06f, 0.08f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        scenecore::DebugDraw& debugDraw = renderer.debug();
        for (std::size_t index = 1; index < toolPath.size(); ++index)
            debugDraw.drawLine(toGlm(toolPath[index - 1]), toGlm(toolPath[index]), glm::vec4(0.0f, 0.85f, 0.9f, 1.0f));
        debugDraw.drawSphere(
            glm::translate(glm::mat4(1.0f), toGlm(startTool)),
            0.055f,
            glm::vec4(0.1f, 1.0f, 0.2f, 1.0f),
            scenecore::DrawType::UsingUnlitShader);
        debugDraw.drawSphere(
            glm::translate(glm::mat4(1.0f), toGlm(goalTool)),
            0.055f,
            glm::vec4(0.1f, 0.3f, 1.0f, 1.0f),
            scenecore::DrawType::UsingUnlitShader);
        const Eigen::Vector3d currentTool = scene->attachmentWorldTransform("420_tool_attachment").translation();
        debugDraw.drawSphere(
            glm::translate(glm::mat4(1.0f), toGlm(currentTool)),
            0.045f,
            currentValidation.valid
                ? glm::vec4(1.0f, 0.85f, 0.1f, 1.0f)
                : glm::vec4(1.0f, 0.0f, 0.0f, 1.0f),
            scenecore::DrawType::UsingUnlitShader);

        const collision::CollisionDebugDrawData collisionDraw =
            scene->collisionRuntime().buildDebugDraw({ options.detectorId }, true);
        robot_render::CollisionRenderBridge::draw(collisionDraw, debugDraw);

        renderer.setCurrentTime(static_cast<float>(elapsed));
        sceneGraph.update();
        renderer.updateSceneUBO(sceneGraph, cameraNode.get());
        renderer.render(sceneGraph);
        glfwSwapBuffers(window);

        ++renderedFrames;
        if (options.maxFrames > 0 && renderedFrames >= options.maxFrames)
            glfwSetWindowShouldClose(window, GLFW_TRUE);
    }

    renderBridge.clear();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
