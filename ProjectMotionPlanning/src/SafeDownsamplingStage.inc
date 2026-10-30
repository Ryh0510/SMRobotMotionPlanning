                const auto& input=request.verificationTrajectory;
                // Use the submitted knots, not replay's union with source controls.
                // Original motion/surface references remain authoritative at old bends.
                std::vector<double> velocities=request.maxJointVelocities;
                if(velocities.empty())for(const auto& b:scene->jointBounds())
                    velocities.push_back(positive(b.maxVelocity)?b.maxVelocity:request.cdfOptions.fallbackMaxVelocity);
                SafeTrajectoryDownsamplingOptions options;
                options.targetPoints=request.preferredOutputWaypoints;options.cancelled=stop;
                // Preserve explicit source boundaries and process switches.
                for(std::size_t i=0;i<request.pointTypes.size();++i)if(request.pointTypes[i]!=0 ||
                    (i>0 && request.pointTypes[i]!=request.pointTypes[i-1])) {
                    const double time=request.plan.cartesianControlPoints.points[i].time;
                    const auto it=std::lower_bound(input.points.begin(),input.points.end(),time,[](const auto& p,double t){return p.time<t;});
                    if(it!=input.points.end())options.requiredIndices.push_back(std::size_t(it-input.points.begin()));
                    if(it!=input.points.begin())options.requiredIndices.push_back(std::size_t(it-input.points.begin()-1));
                }
                options.progress=[&](std::size_t retained,std::size_t pending) {
                    if(request.progress)request.progress(nlohmann::json{{"type","safe_downsampling"},
                        {"retained",retained},{"pending_edges",pending},{"target",options.targetPoints}}.dump());
                };
                const auto referenceValid=[&](const std::vector<double>& q,double time) {
                    if(!stateValid(q,false))return false;
                    if(spray)return processValid(q,time);
                    // Compare with the ORIGINAL joint-linear motion at the same
                    // clock. Its TCP is generally curved between retained knots;
                    // a Cartesian endpoint chord is not the submitted reference.
                    const auto& controls=input.points;
                    const auto upper=std::upper_bound(controls.begin(),controls.end(),time,[](double t,const auto& p){return t<p.time;});
                    const auto i=upper==controls.begin()?0:std::size_t(upper-controls.begin()-1);
                    const auto j=std::min(i+1,controls.size()-1);
                    const double f=i==j?0.:std::clamp((time-controls[i].time)/(controls[j].time-controls[i].time),0.,1.);
                    auto reference=controls[i].q;
                    for(std::size_t k=0;k<dimension;++k)reference[k]=(1-f)*controls[i].q[k]+f*controls[j].q[k];
                    const auto actual=fk(q),expected=fk(reference);
                    return (actual.translation()-expected.translation()).norm()<=request.maximumTcpPositionDeviation &&
                        Eigen::Quaterniond(expected.linear()).angularDistance(Eigen::Quaterniond(actual.linear()))<=request.maximumTcpOrientationDeviation;
                };
                options.edgeAdmissible=[&](std::size_t from,std::size_t to) {
                    if(stop())return false;
                    const auto& a=input.points[from];const auto& b=input.points[to];
                    const double dt=b.time-a.time;double delta=0;
                    for(std::size_t j=0;j<dimension;++j) {
                        const double d=std::abs(b.q[j]-a.q[j]);
                        if(d>request.maxJointDelta || d>velocities[j]*dt)return false;
                        delta=std::max(delta,d);
                    }
                    const double count=std::max(2.,std::ceil(delta/request.validationJointStep));
                    if(!std::isfinite(count) || count>double(request.maxSamplesPerEdge))return false;
                    // Use replay's exact grid (at least two subdivisions even for
                    // small moves); every worker owns its collision/baffle scene.
                    std::set<double> samples;
                    for(std::size_t s=0;s<=std::size_t(count);++s)samples.insert(a.time+dt*double(s)/count);
                    // Shortcuts must also agree at EVERY skipped reference bend.
                    auto old=std::upper_bound(source.cartesianControlPoints.points.begin(),source.cartesianControlPoints.points.end(),a.time,
                        [](double t,const auto& p){return t<p.time;});
                    for(;old!=source.cartesianControlPoints.points.end() && old->time<b.time;++old)samples.insert(old->time);
                    std::vector<std::vector<double>> values;values.reserve(samples.size());
                    for(double time:samples) {
                        if(stop())return false;
                        const double f=std::clamp((time-a.time)/dt,0.,1.);auto q=a.q;
                        for(std::size_t j=0;j<dimension;++j)q[j]=(1-f)*a.q[j]+f*b.q[j];
                        if(!referenceValid(q,time))return false;
                        values.push_back(std::move(q));
                    }
                    const auto checked=batch().states(values,request.minimumClearance);
                    return checked.size()==values.size() && std::all_of(checked.begin(),checked.end(),
                        [](const auto& state){return state.validation.valid;});
                };
                // Exact source-index keys only, scoped to this immutable scene.
                // Cancellation and exceptions never become safety certificates.
                const auto checkEdge=options.edgeAdmissible;
                std::map<std::pair<std::size_t,std::size_t>,bool> edgeChecks;
                options.edgeAdmissible=[&](std::size_t a,std::size_t b) {
                    if(stop())return false;
                    const auto key=std::make_pair(a,b);const auto found=edgeChecks.find(key);
                    if(found!=edgeChecks.end())return found->second;
                    const bool valid=checkEdge(a,b);
                    if(!stop())edgeChecks.emplace(key,valid);return valid;
                };
                auto sampled=downsampleTrajectorySafely(input,options);
                if(!sampled.success)return fail(sampled.cancelled?"cancelled":"downsampling_unverified",sampled.message);
                auto quality=evaluateTrajectoryQuality(sampled.trajectory,scene->jointBounds(),
                    request.cdfOptions.fallbackMaxVelocity,request.cdfOptions.fallbackMaxAcceleration);
                std::set<std::size_t> dynamicsPins(options.requiredIndices.begin(),options.requiredIndices.end());
                while(quality.valid && quality.timingValid && !quality.positionLimitViolations &&
                    (quality.velocityLimitViolations || quality.accelerationLimitViolations)) {
                    if(stop())return fail("cancelled","Downsampling cancelled.");
                    const auto before=dynamicsPins.size();
                    const auto& points=sampled.trajectory.points;
                    for(std::size_t i=0;i<points.size();++i) {
                        bool invalid=false;
                        for(std::size_t j=0;j<dimension;++j) {
                            if(std::abs(points[i].qdd[j])>quality.jointAccelerationLimits[j]*(1.+1e-8))invalid=true;
                            if(i && std::abs((points[i].q[j]-points[i-1].q[j])/(points[i].time-points[i-1].time))>
                                quality.jointVelocityLimits[j]*(1.+1e-8))invalid=true;
                        }
                        if(!invalid)continue;
                        const auto a=sampled.sourceIndices[i?i-1:0];
                        const auto b=sampled.sourceIndices[std::min(i+1,points.size()-1)];
                        for(auto k=a;k<=b;++k)dynamicsPins.insert(k);
                    }
                    if(dynamicsPins.size()==before)break;
                    options.requiredIndices.assign(dynamicsPins.begin(),dynamicsPins.end());
                    sampled=downsampleTrajectorySafely(input,options);
                    if(!sampled.success)return fail("downsampling_unverified",sampled.message);
                    quality=evaluateTrajectoryQuality(sampled.trajectory,scene->jointBounds(),
                        request.cdfOptions.fallbackMaxVelocity,request.cdfOptions.fallbackMaxAcceleration);
                }
                if(!quality.valid || !quality.timingValid || quality.positionLimitViolations ||
                    quality.velocityLimitViolations || quality.accelerationLimitViolations)
                    return fail("downsampling_dynamics_unverified","Source timing does not meet actual joint limits; no trajectory exported.");
                if(!dynamicsPins.empty())result.diagnostics.push_back({"downsampling_preserved_dynamics",
                    "Necessary source knots retained locally; original surface clock and dynamic limits preserved."});
                if(stop())return fail("cancelled","Downsampling cancelled.");
                result.plan=request.plan;result.plan.trajectory=std::move(sampled.trajectory);
                result.plan.cartesianControlPoints.points.clear();
                for(const auto& q:result.plan.trajectory.points) {
                    robottrajectory::TimedCartesianPoint point;point.time=q.time;point.tcpPose=fk(q.q);
                    result.plan.cartesianControlPoints.points.push_back(point);
                }
                result.success=true;
                result.diagnostics.push_back({"downsampling_sampled_valid",nlohmann::json{
                    {"input_points",input.points.size()},{"target_points",request.preferredOutputWaypoints},
                    {"output_points",result.plan.trajectory.points.size()},{"checked_edges",sampled.checkedEdges},
                    {"validation_joint_step_rad",request.validationJointStep},
                    {"scope","joint-linear motion; sampled collisions, baffles, limits, singularity and reference corridor; target is soft"}}.dump()});
                recordVelocities();return finish();
