// Function-scope adapter from the public, robot-independent search API to this
// isolated real-model scene. The legacy GA block below is deliberately separate.
            if(request.pipelineStage=="posture" && fixed.empty()) {
                using Search=AdaptivePostureSearch;using X=Search::X;
                const double pi=std::acos(-1.);
                // Shared ownership is required: graph is evaluated again after
                // this adapter scope. The callback never captures a local pool.
                auto nodeCosts=std::make_shared<std::map<std::pair<std::size_t,std::vector<double>>,double>>();
                graph.nodeCost=[&,nodeCosts](std::size_t i,const CartesianIkCandidate& c) {
                    const auto key=std::make_pair(i,c.joints);const auto found=nodeCosts->find(key);
                    if(found!=nodeCosts->end())return found->second;
                    const auto distance=scene->nearestDistance(c.joints);
                    // Actual detector distance is a soft robustness preference,
                    // never a substitute for the independent baffle checks.
                    const double d=distance.valid && std::isfinite(distance.distanceMeters)?std::max(0.,distance.distanceMeters):0.;
                    const double value=.01/(d+.001);nodeCosts->emplace(key,value);return value;
                };
                const auto referenceFor=[&](std::size_t i) {
                    auto r=surfaceAt(ik.layers[i].time);
                    if(!spray){r.orientation=Eigen::Quaterniond(source.cartesianControlPoints.points[i].tcpPose.linear());
                        if(!request.pointSearchReferenceDelta.empty())r.orientation=r.orientation*request.pointSearchReferenceDelta[i];
                        r.angle=(isTransition(i)?request.transitionSearchMaxAngleDeg:request.pointSearchMaxAngleDeg)*pi/180;}
                    return r;
                };
                const auto cost=[&](const std::vector<double>& a,const std::vector<double>& b) {
                    double value=0;if(a.size()!=b.size())return value;
                    for(std::size_t j=0;j<a.size();++j)value+=std::pow(a[j]-b[j],2);return value;
                };
                std::vector<std::vector<CartesianIkCandidate>> pools(ik.layers.size()),raw;
                for(const auto& layer:ik.layers)raw.push_back(layer.candidates);
                std::map<std::pair<std::size_t,std::vector<double>>,bool> certificates;
                const auto safe=[&](const CartesianIkCandidate& c,std::size_t i) {
                    if(stop())return false;
                    const auto key=std::make_pair(i,c.joints);const auto found=certificates.find(key);
                    if(found!=certificates.end())return found->second;
                    const bool valid=stateValid(c.joints) && processValid(c.joints,ik.layers[i].time);
                    certificates[key]=valid;return valid;
                };
                const auto add=[&](std::size_t i,const CartesianIkCandidate& c) {
                    if(i==0 && !request.startJoints.empty() && cost(c.joints,request.startJoints)>1e-14)return;
                    for(const auto& old:pools[i])if(cost(old.joints,c.joints)<1e-10)return;
                    pools[i].push_back(c);
                };
                for(std::size_t i=0;i<raw.size();++i)for(const auto& c:raw[i])if(safe(c,i))add(i,c);
                std::vector<double> branchReference;
                if(!pools.front().empty())for(const auto& c:pools.front())if(branchReference.empty() ||
                    Eigen::Map<const Eigen::VectorXd>(c.joints.data(),dimension).squaredNorm()<Eigen::Map<const Eigen::VectorXd>(branchReference.data(),dimension).squaredNorm())branchReference=c.joints;
                const auto refresh=[&]() {
                    for(std::size_t i=0;i<pools.size();++i){ik.layers[i].candidates=pools[i];result.candidateCounts[i]=pools[i].size();
                        result.candidateRejections[i].assign(pools[i].size(),std::string());
                        result.candidateDistances[i].assign(pools[i].size(),std::numeric_limits<double>::quiet_NaN());}
                };
                const auto solve=[&](std::size_t i,const Eigen::Quaterniond& rotation,const std::vector<double>& preferred,
                    bool broad,PostureConstraintFeedback* feedback,std::map<std::string,int>* rejections) {
                    const auto r=referenceFor(i);auto point=source.cartesianControlPoints.points[i];
                    point.tcpPose.linear()=rotation.toRotationMatrix();
                    if(spray && r.constrained)point.tcpPose.translation()=r.anchor-r.distance*point.tcpPose.linear()*request.sprayAxis;
                    auto plan=source;plan.cartesianControlPoints.points={point};auto options=ikOptions;
                    options.cachedCandidates.clear();options.progress={};options.layerReady={};options.maxIterations=80;
                    options.preferredJoints.clear();options.seedCount=broad?16:1;
                    std::vector<std::vector<double>> seeds;
                    if(!preferred.empty())seeds.push_back(preferred);
                    const auto sameBranch=[&](const auto& a,const auto& b){double value=0;for(std::size_t j=0;j<dimension;++j)value+=std::pow(std::remainder(a[j]-b[j],2*pi),2);return value<1e-8;};
                    for(const auto& c:raw[i]){if(std::none_of(seeds.begin(),seeds.end(),[&](const auto& q){return sameBranch(q,c.joints);}))seeds.push_back(c.joints);if(seeds.size()>=4)break;}
                    if(seeds.empty())seeds.push_back(ikOptions.model.seedJoints);
                    if(seeds.front().size()==6){auto flip=seeds.front();flip[3]+=pi;flip[4]=-flip[4];flip[5]+=pi;
                        for(std::size_t j=0;j<dimension;++j){while(flip[j]>ikOptions.upper[j] && flip[j]-2*pi>=ikOptions.lower[j])flip[j]-=2*pi;
                            while(flip[j]<ikOptions.lower[j] && flip[j]+2*pi<=ikOptions.upper[j])flip[j]+=2*pi;flip[j]=std::clamp(flip[j],ikOptions.lower[j],ikOptions.upper[j]);}
                        if(std::none_of(seeds.begin(),seeds.end(),[&](const auto& q){return cost(q,flip)<1e-8;}))seeds.push_back(flip);}
                    std::vector<CartesianIkCandidate> accepted;std::string rejection="no_ik";
                    const auto inspect=[&](const auto& trial) {
                    if(!trial.layers.empty())for(const auto& c:trial.layers.front().candidates) {
                        if(stop())break;
                        const bool valid=stateValid(c.joints) && processValid(c.joints,ik.layers[i].time);
                        const auto reason=stateFailure;
                        certificates[std::make_pair(i,c.joints)]=valid;
                        if(valid){accepted.push_back(c);add(i,c);if(feedback){feedback->feasible=true;feedback->violation=0.;
                            const double value=request.postureOptions.continuityWeight*cost(c.joints,preferred)+
                                request.postureOptions.postureWeight*std::pow(r.orientation.angularDistance(rotation),2);
                            feedback->cost=accepted.size()==1?value:std::min(feedback->cost,value);}}
                        else {rejection=reason.empty()?"process_constraint_violation":reason;
                            if(feedback && !feedback->feasible){double measured=std::numeric_limits<double>::infinity();
                                if(isVirtualBaffleFailure(reason))measured=scene->virtualBaffleSearchViolation(request.minimumClearance);
                                else if(reason=="state_in_collision")measured=measuredPenetration();
                                if(std::isfinite(measured))feedback->violation=std::min(feedback->violation,std::max(1e-12,measured));}}
                    }};
                    if(broad){options.preferredJoints.clear();options.seedCount=16;
                        const auto trial=ProjectTrajectoryInverseKinematics::solveAllCartesianControlPoints(plan,options);inspect(trial);
                        if(!trial.layers.empty())for(const auto& c:trial.layers.front().candidates)if(std::none_of(raw[i].begin(),raw[i].end(),[&](const auto& old){return sameBranch(c.joints,old.joints);}))raw[i].push_back(c);}
                    for(const auto& seed:seeds){if(stop())break;options.preferredJoints={seed};options.model.seedJoints=seed;options.seedCount=1;
                        inspect(ProjectTrajectoryInverseKinematics::solveAllCartesianControlPoints(plan,options));
                        if(!feedback && !accepted.empty())break;}
                    if(rejections && accepted.empty())++(*rejections)[rejection];return accepted;
                };
                std::vector<std::vector<double>> frontierHints;
                const auto searchPoint=[&](std::size_t i,bool refine) {
                    if(i==0 && !request.startJoints.empty())return !pools[i].empty();
                    const auto began=std::chrono::steady_clock::now();const auto r=referenceFor(i);
                    std::vector<X> seeds;std::vector<double> preferred;
                    // Symmetric warm starts from both sides and from distinct IK
                    // branches. These are real relative surface postures, never
                    // transferred TCP positions or fabricated safety anchors.
                    for(std::size_t gap=0;gap<pools.size();++gap)for(int sign:{1,-1}) {
                        const long long index=static_cast<long long>(i)+sign*static_cast<long long>(gap);
                        if(index<0 || index>=static_cast<long long>(pools.size()) || (gap==0 && sign==-1))continue;
                        auto neighbors=pools[std::size_t(index)];
                        if(!branchReference.empty())std::stable_sort(neighbors.begin(),neighbors.end(),[&](const auto& a,const auto& b){return cost(a.joints,branchReference)<cost(b.joints,branchReference);});
                        int count=0;for(const auto& c:neighbors){
                            if(preferred.empty())preferred=c.joints;
                            const auto relative=referenceFor(std::size_t(index)).orientation.conjugate()*Eigen::Quaterniond(fk(c.joints).linear());
                            seeds.push_back(Search::coordinates(relative,request.sprayAxis,r.angle,request.allowSprayAxisRotation));if(++count==3)break;}
                    }
                    if(preferred.empty() && !raw[i].empty())preferred=raw[i].front().joints;
                    if(preferred.empty())preferred=ikOptions.model.seedJoints;
                    if(refine && !frontierHints.empty())preferred=frontierHints.front();
                    // Coarse cone rings are followed by nested Halton coverage.
                    for(int k=0;k<16;++k)for(double phase:request.allowSprayAxisRotation?std::vector<double>{0.,1.}:std::vector<double>{0.})
                        seeds.emplace_back(std::cos(k*pi/8),std::sin(k*pi/8),phase);
                    auto options=request.postureOptions;
                    if(refine)options.pointEvaluations=std::min(options.pointEvaluations,96);
                    int calls=0;std::map<std::string,int> rejected;
                    const auto oracle=[&](const X& x) {
                        PostureConstraintFeedback f;const auto rotation=clampSurfacePosture(r.orientation,
                            r.orientation*Search::rotation(x,request.sprayAxis,r.angle,request.allowSprayAxisRotation),r.angle);
                        const auto accepted=solve(i,rotation,preferred,calls++==0 && raw[i].size()<4,&f,&rejected);
                        if(!f.feasible && std::isfinite(f.violation))f.violation=1.+std::min(1.,f.violation);
                        if(refine && !frontierHints.empty() && !accepted.empty()) {
                            double violation=std::numeric_limits<double>::infinity();
                            for(const auto& c:accepted)for(const auto& q:frontierHints){double excess=0.;
                                for(std::size_t j=0;j<dimension;++j)excess=std::max(excess,std::max(0.,std::abs(c.joints[j]-q[j])-request.maxJointDelta));
                                violation=std::min(violation,excess);}
                            f.feasible=violation<=1e-12;f.violation=violation/(1.+violation);
                        }
                        return f;
                    };
                    const auto expired=[&](){return stop() || std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()>=request.anchorSearchSeconds;};
                    const auto audit=Search::search(options,seeds,request.allowSprayAxisRotation,oracle,expired);
                    const bool solved=!pools[i].empty();
                    const auto record=nlohmann::json{{"strategy","adaptive_posture_graph"},{"point_index",i},{"attempts",audit.evaluations},
                        {"local_evaluations",audit.localEvaluations},{"safe_postures",audit.safePostures},{"safe_candidates_found",pools[i].size()},
                        {"solved",solved},{"stop_reason",audit.stopReason},{"rejections",rejected},{"graph_refinement",refine},
                        {"elapsed_ms",std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count()}};
                    result.pointSearchReports.push_back(record.dump());
                    if(request.progress)request.progress(nlohmann::json{{"type","posture_point"},{"report",record}}.dump());return solved;
                };
                const auto transport=[&](std::size_t station) {
                    const auto starts=pools[station];int count=0;
                    for(const auto& start:starts){if(++count>3)break;
                        const auto relative=referenceFor(station).orientation.conjugate()*Eigen::Quaterniond(fk(start.joints).linear());
                        for(int sign:{-1,1}){auto previous=start.joints;
                            for(long long index=static_cast<long long>(station)+sign;index>=0 && index<static_cast<long long>(pools.size()) && !stop();index+=sign){
                                const auto i=std::size_t(index);if(i==0 && !request.startJoints.empty())break;
                                const auto r=referenceFor(i);const auto rotation=clampSurfacePosture(r.orientation,r.orientation*relative,r.angle);
                                const auto accepted=solve(i,rotation,previous,false,nullptr,nullptr);if(accepted.empty())break;
                                previous=std::min_element(accepted.begin(),accepted.end(),[&](const auto& a,const auto& b){return cost(a.joints,previous)<cost(b.joints,previous);})->joints;
                            }
                        }
                    }
                };
                for(std::size_t reverse=pools.size();reverse>0 && !stop();--reverse){const auto i=reverse-1;
                    if(pools[i].empty()){
                        if(!searchPoint(i,false)){refresh();if(request.returnCandidates)result.candidateLayers=ik.layers;
                            return fail("posture_point_unresolved","Adaptive posture search did not clear source point "+std::to_string(i)+"; APF/CDF must not start");}
                        transport(i);
                    }
                }
                refresh();
                // Graph-driven refinement only expands a disconnected frontier.
                // No edge interiors are queried in this original-point stage.
                auto geometric=graph;geometric.maxJointVelocities.assign(dimension,1e6);
                if(!ProjectLayeredIkGraph::filter(ik,geometric).success && !pools.front().empty() && !pools.back().empty()){
                    // Deterministic smooth-field continuation couples the original
                    // stations before local graph repair. This is a bounded family
                    // of continuation predictors, without a genetic population.
                    auto starts=pools.front();std::stable_sort(starts.begin(),starts.end(),[&](const auto& a,const auto& b){return cost(a.joints,branchReference)<cost(b.joints,branchReference);});
                    std::vector<CartesianIkCandidate> ends;std::vector<Eigen::Quaterniond> rotations;
                    for(const auto& c:pools.back()){const Eigen::Quaterniond rotation(fk(c.joints).linear());
                        if(std::any_of(rotations.begin(),rotations.end(),[&](const auto& q){return q.angularDistance(rotation)<.02;}))continue;
                        ends.push_back(c);rotations.push_back(rotation);if(ends.size()==8)break;}
                    bool connected=false;int fields=0;
                    for(std::size_t s=0;s<std::min<std::size_t>(starts.size(),2) && !connected && !stop();++s){
                        const auto begin=referenceFor(0).orientation.conjugate()*Eigen::Quaterniond(fk(starts[s].joints).linear());
                        for(const auto& end:ends){if(connected || stop())break;
                            const auto finish=referenceFor(pools.size()-1).orientation.conjugate()*Eigen::Quaterniond(fk(end.joints).linear());
                            for(double rampStart:{.15,.35,.55})for(double rampEnd:{.8,1.}){if(connected || stop() || fields>=48)break;++fields;
                                auto previous=starts[s].joints;
                                for(std::size_t i=1;i<pools.size() && !stop();++i){const double fraction=double(i)/std::max<std::size_t>(1,pools.size()-1);
                                    const double t=std::clamp((fraction-rampStart)/(rampEnd-rampStart),0.,1.);const auto r=referenceFor(i);
                                    const auto rotation=clampSurfacePosture(r.orientation,r.orientation*begin.slerp(t,finish),r.angle);
                                    const auto accepted=solve(i,rotation,previous,false,nullptr,nullptr);if(accepted.empty())break;
                                    previous=std::min_element(accepted.begin(),accepted.end(),[&](const auto& a,const auto& b){return cost(a.joints,previous)<cost(b.joints,previous);})->joints;
                                }
                                refresh();connected=ProjectLayeredIkGraph::filter(ik,geometric).success;
                            }
                        }
                    }
                    if(request.progress)request.progress(nlohmann::json{{"type","posture_graph"},{"continuation_fields",fields},{"geometrically_connected",connected}}.dump());
                }
                for(int round=0;round<request.postureOptions.graphRefinementRounds && !stop();++round){
                    if(ProjectLayeredIkGraph::filter(ik,geometric).success)break;
                    std::vector<bool> reachable(pools.front().size(),true);std::size_t frontier=1;
                    for(;frontier<pools.size();++frontier){std::vector<bool> next(pools[frontier].size(),false);
                        for(std::size_t a=0;a<reachable.size();++a)if(reachable[a])for(std::size_t b=0;b<next.size();++b){bool valid=true;
                            for(std::size_t j=0;j<dimension;++j)valid &= std::abs(pools[frontier][b].joints[j]-pools[frontier-1][a].joints[j])<=request.maxJointDelta;
                            if(valid)next[b]=true;}
                        if(std::none_of(next.begin(),next.end(),[](bool x){return x;}))break;reachable=std::move(next);}
                    if(frontier==pools.size())break;
                    frontierHints.clear();for(std::size_t k=0;k<reachable.size();++k)if(reachable[k])frontierHints.push_back(pools[frontier-1][k].joints);
                    if(!branchReference.empty())std::stable_sort(frontierHints.begin(),frontierHints.end(),[&](const auto& a,const auto& b){return cost(a,branchReference)<cost(b,branchReference);});
                    searchPoint(frontier,true);
                    if(!frontierHints.empty())std::stable_sort(pools[frontier].begin(),pools[frontier].end(),[&](const auto& a,const auto& b){
                        double ca=1e300,cb=1e300;for(const auto& q:frontierHints){ca=std::min(ca,cost(a.joints,q));cb=std::min(cb,cost(b.joints,q));}return ca<cb;});
                    frontierHints.clear();transport(frontier);refresh();
                }
                // Prefer diverse branches close to their neighbors. Retain the
                // chosen graph path before pruning so pruning cannot erase it.
                const auto selected=ProjectLayeredIkGraph::filter(ik,geometric);
                for(std::size_t i=0;i<pools.size();++i){auto& pool=pools[i];
                    if(selected.success){const auto chosen=pool[selected.paths.front().selections[i]];
                        std::stable_sort(pool.begin(),pool.end(),[&](const auto& a,const auto& b){return cost(a.joints,chosen.joints)<cost(b.joints,chosen.joints);});}
                    if(pool.size()>std::size_t(request.postureOptions.candidateLimit))pool.resize(request.postureOptions.candidateLimit);
                }
                refresh();
                for(std::size_t i=0;i<pools.size();++i){result.candidateRejections[i].assign(pools[i].size(),std::string());result.candidateDistances[i].clear();
                    for(const auto& c:pools[i]){const auto d=scene->nearestDistance(c.joints);result.candidateDistances[i].push_back(d.valid?d.distanceMeters:std::numeric_limits<double>::quiet_NaN());}}
                result.diagnostics.push_back({"adaptive_posture_point_gate","Original points passed exact real-model IK, fixed-surface mapping, collision, baffle and singularity checks. Layered graph selects branches; interiors remain UNKNOWN until APF."});
            }
