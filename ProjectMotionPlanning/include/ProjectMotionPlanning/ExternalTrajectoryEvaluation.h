#pragma once

#include <ProjectMotionPlanning/ProjectMotionPlanning.h>
#include <ProjectMotionPlanning/LayeredIkGraph.h>
#include <SimulationProject/ProjectDocument.h>
#include <optional>
#include <ProjectMotionPlanning/CdfQpTrajectoryRepair.h>
#include <ProjectMotionPlanning/AdaptivePostureSearch.h>

namespace motion_planning
{
    enum class ExternalTrajectoryFrame { World, Workpiece };

    // Deterministic planning boundary for external callers. No GUI state is mutated.
    // All translations/mesh scales use metres, all joints use runtime-model radians.
    struct GaRejectedConnection
    {
        std::size_t sourceEdge = 0;
        std::vector<double> fromJoints;
        std::vector<double> toJoints;
    };

    struct ExternalTrajectoryEvaluationRequest
    {
        StoredMotionPlan plan;
        // Independent replay of exact returned joints; never solve a replacement path.
        robottrajectory::JointTrajectory verificationTrajectory;
        // Optional upsert into a private document. sourcePath may point to external STL.
        std::optional<simulation_project::SceneObjectDesc> workpiece;
        std::string workpieceId;
        ExternalTrajectoryFrame frame = ExternalTrajectoryFrame::World;
        std::string tcpLinkName;
        Eigen::Isometry3d linkFromTcp = Eigen::Isometry3d::Identity();
        std::vector<std::string> collisionDetectorIds;
        CartesianMultiIkOptions ik;
        double maxJointDelta = 0.5;
        double validationJointStep = 0.02;
        std::vector<double> maxJointVelocities;
        double minimumClearance = 0.0;
        // Dimensionless Jacobian: linear rows divided by this characteristic length.
        double jacobianLengthScale = 1.0;
        double minimumSingularValue = 1.0e-4;
        double maximumTcpPositionDeviation = 0.001;
        double maximumTcpOrientationDeviation = 0.01;
        std::size_t maxSamplesPerEdge = 10000;
        // Empty retains legacy API. Explicit stages are ik, ga, apf, cdf, validate, downsample.
        std::string pipelineStage;
        AdaptivePostureOptions postureOptions;
        bool includeKinematicAudit = false;
        int gaPointEvaluations = 131072; // established portfolio, then bounded GA rescue
        bool gaGpuEnabled = true; // GPU search merit; CPU exact IK/collision gates
        bool gaGpuIkEnabled = true; // actual serial-chain GPU IK, CPU FK validated
        int gaInitialPostures = 5000;
        int gaGenerations = 8; // 24 nominal offspring/round; enabled radius adds 24/round after unlocking
        bool gaRadiusEvolutionEnabled = false; // opt-in; rounds 3/4/5 unlock +/-30/50/80% only while unsafe
        std::vector<double> gaNominalSprayDistances; // immutable source metres, optional for legacy callers
        int gaFieldEvaluations = 48;
        int gaFeasibilityBatchSize = 256;
        double gaContinuityWeight = 2.;
        double gaPostureWeight = .2;
        // Finite search effort, never an acceptance tolerance. Zero preserves legacy unlimited execution.
        double stageTimeBudgetSeconds = 0;
        int anchorSearchAttempts = 104;
        double anchorSearchSeconds = 0; // zero: no single-point wall-clock deadline
        bool gaEvaluateAllPoints = false; // full initial pool even for already safe controls
        // APF feedback: exact original-control seed; resample only these controls.
        // Other controls remain fixed. Failed joint pairs are search exclusions only.
        std::vector<std::size_t> gaResampleControls;
        std::vector<GaRejectedConnection> gaRejectedConnections;
        robottrajectory::JointTrajectory optimizationSeed;
        bool densifyOptimizationKnots = false;
        // Zero keeps legacy adaptive support. Nonzero targets a balanced node count; safety may require additional nodes.
        std::size_t preferredOutputWaypoints = 0;
        bool optimizeAvoidance = false;
        bool cdfPointOptimization = false;
        ProjectCdfQpRepairOptions cdfOptions = [] { ProjectCdfQpRepairOptions o; o.postSmoothingIterations=2; return o; }();
        std::vector<double> jointWeights;
        std::vector<double> startJoints; // exact shared segment boundary, radians
        bool controlPointsOnly = false;
        // Independent nodes only: no edges, smoothing or whole-path certificate.
        bool independentPointsOnly = false;
        int pointSearchAttempts = 0;
        std::vector<int> pointSearchAttemptLimits;
        int pointSearchSequence = 0;
        int pointSearchPopulationStrategy = 1;
        int pointSearchStartStrategy = 1; // 1 GA, 2 DE, 3 CMA-ES; local refinement follows measured improvement.
        double pointSearchSeconds = 3.0;
        double pointSearchMaxAngleDeg = 30.0;
        // Bound spray-normal tilt independently of rotation about the spray axis.
        // Legacy callers retain the full-orientation cone unless explicitly enabled.
        bool allowSprayAxisRotation = false;
        std::vector<double> pointSearchMinDistances;
        std::vector<double> pointSearchMaxDistances;
        // Rotation from the submitted target to its immutable original reference.
        std::vector<Eigen::Quaterniond> pointSearchReferenceDelta;
        // Fixed surface spray targets in request frame, metres at native boundary.
        std::vector<Eigen::Vector3d> sprayTargets;
        std::vector<double> sprayDistances;
        // Source semantics: 0 spray, 1 boundary, 2 free non-spraying transition.
        std::vector<int> pointTypes;
        std::vector<Eigen::Vector3d> transitionCenters;
        double transitionSearchRadius = 0.3; // metres, anchored to source TCP
        double transitionSearchMaxAngleDeg = 180;
        Eigen::Vector3d sprayAxis = Eigen::Vector3d::UnitZ();
        bool returnCandidates = false;
        std::size_t continuousPathCount = 1;
        int optimizationIterations = 3;
        std::size_t optimizationSeedPaths = 3;
        std::function<void(const std::string&)> progress;
        std::function<bool()> cancelled;
    };

    struct ExternalTrajectoryEvaluationResult
    {
        bool success = false;
        bool cancelled = false;
        // APF geometry/process checks completed, but source timing was rejected.
        // Never an accepted plan: retime these exact joints and independently replay.
        robottrajectory::JointTrajectory retimingCandidate;
        std::vector<ApfConnectionFailure> apfConnectionFailures;
        // Search feedback, never a proof that no other pose/IK/path exists.
        bool retryWorkpiecePose = false;
        std::string kinematicAudit;
        PlanningSceneQueryCounts sceneQueryCounts;
        StoredMotionPlan plan;
        std::vector<MotionPlanningDiagnostic> diagnostics;
        std::vector<std::size_t> candidateCounts;
        std::size_t rejectedStates = 0;
        std::size_t rejectedEdges = 0;
        std::vector<CartesianIkLayer> candidateLayers;
        std::vector<std::vector<std::string>> candidateRejections;
        std::vector<std::vector<double>> candidateDistances;
        std::vector<LayeredIkPath> continuousPaths;
        std::vector<std::string> pointSearchReports;
        std::string velocityTelemetry;
        std::string surfaceMapping;
        // Actual radius fraction unlocked at each original control, not a new nominal radius.
        std::vector<double> gaRadiusFractions;
        std::string cartesianPointsWorld;
    };

    class ProjectExternalTrajectoryEvaluator
    {
    public:
        // Synchronous, potentially expensive. Run on a worker, not the GUI thread.
        // Validates joint-linear interpolation against Cartesian-linear/slerp targets.
        // Collision checks are sampled, not a continuous collision certificate.
        static ExternalTrajectoryEvaluationResult evaluate(
            const simulation_project::ProjectDocument& document,
            const std::filesystem::path& projectBasePath,
            const ExternalTrajectoryEvaluationRequest& request);
    };
}
