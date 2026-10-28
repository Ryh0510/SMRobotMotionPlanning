#include <fcl/fcl.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
using Json=nlohmann::json;
std::shared_ptr<fcl::BVHModel<fcl::OBBRSSd>> mesh(const char* path) {
    std::ifstream f(path,std::ios::binary);f.seekg(80);uint32_t n;f.read((char*)&n,4);
    std::vector<fcl::Vector3d> v;std::vector<fcl::Triangle> t;
    for(uint32_t i=0;i<n;++i){float a[12];uint16_t attr;f.read((char*)a,48);f.read((char*)&attr,2);
        for(int j=0;j<3;++j)v.emplace_back(a[3+3*j]*.001,a[4+3*j]*.001,a[5+3*j]*.001);
        t.emplace_back(3*i,3*i+1,3*i+2);}
    auto m=std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();m->beginModel();m->addSubModel(v,t);m->endModel();return m;
}
int main(int argc,char** argv){
    if(argc!=4)return 2;std::ifstream f(argv[1]);Json data;f>>data;
    auto gun=mesh(argv[2]),part=mesh(argv[3]);
    if(data.contains("poses")){
        fcl::Transform3d pa=fcl::Transform3d::Identity();
        for(int i=0;i<4;++i)for(int j=0;j<4;++j)pa.matrix()(i,j)=data["part_transform"][i][j];
        size_t index=0;
        for(const auto& pose:data["poses"]){fcl::Transform3d ga=fcl::Transform3d::Identity();
            for(int i=0;i<4;++i)for(int j=0;j<4;++j)ga.matrix()(i,j)=pose[i][j];
            fcl::CollisionRequestd request;request.enable_contact=false;
            fcl::CollisionResultd result;fcl::collide(gun.get(),ga,part.get(),pa,request,result);
            if(!result.isCollision())std::cout<<index<<std::endl;
            ++index;
        }return 0;
    }
    for(const auto& a:data["kinematic_audit"]){
        fcl::Transform3d ga=fcl::Transform3d::Identity(),pa=ga;
        for(const auto& g:a["geometry"])if(g["triangles"]==14346 || g["triangles"]==7082){
            auto& m=g["triangles"]==14346?ga:pa;
            for(int i=0;i<4;++i)for(int j=0;j<4;++j)m.matrix()(i,j)=g["transform"][i][j];}
        fcl::CollisionResultd c;fcl::CollisionRequestd cr;cr.enable_contact=false;cr.num_max_contacts=1;
        fcl::collide(gun.get(),ga,part.get(),pa,cr,c);
        fcl::DistanceResultd d;fcl::DistanceRequestd dr;dr.enable_nearest_points=true;
        if(!c.isCollision())fcl::distance(gun.get(),ga,part.get(),pa,dr,d);
        Json cs=Json::array();for(size_t i=0;i<c.numContacts();++i){auto x=c.getContact(i);cs.push_back({{"a",x.b1},{"b",x.b2}});}
        std::cout<<Json{{"point",a["point_index"]},{"collision",c.isCollision()},{"distance",d.min_distance},{"contacts",cs}}.dump()<<std::endl;
    }
}
