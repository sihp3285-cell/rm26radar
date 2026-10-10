#include <radar27_decision/map_analyzer.hpp>
#include <rm_field/field_geometry.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <chrono>
using Target = radar27_interfaces::msg::WorldTarget;
void check(bool value,const char* message) { if (!value) throw std::runtime_error(message); }
Target target(int team,int role,int id,double x,double y,bool direction=true) {
    Target t; t.valid=true; t.observed=true; t.track_id=id; t.team_id=team;
    t.stable_class_id=role; t.stable_class_conf=1; t.class_id=role;
    t.tracking_state=Target::TRACKING_ACTIVE; t.position_source=Target::POSITION_TRACKED;
    double wx,wz; rm_field::field_to_world(x,y,direction,wx,wz);
    t.world_x=wx; t.world_z=wz; return t;
}
int main(int argc,char** argv) {
    try {
        check(argc==2,"expected regions directory");
        MapAnalyzer real;
        real.loadRegions(argv[1]); auto actual=real.evaluate({});
        check(actual.regions.size()==8,"7 files plus one mirrored gully");
        for (const auto& r:actual.regions) {
            check(r.id!="other_home"&&r.id!="engineer_home","excluded regions");
            check(r.field_x.size()==r.world_x.size(),"geometry published");
        }
        const auto dir=std::filesystem::temp_directory_path()/
            ("radar27-regions-test-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        std::filesystem::create_directory(dir);
        struct Cleanup { std::filesystem::path path; ~Cleanup(){std::filesystem::remove_all(path);} } cleanup{dir};
        std::ofstream(dir/"engineer.yaml") << "blind_zones:\n - polygon: [[5,5],[7,5],[7,7],[5,7]]\n";
        std::ofstream(dir/"middle.yaml") << "regions:\n - polygon: [[5,5],[7,5],[7,7],[5,7]]\n   mirror_centrally: true\n";
        std::ofstream(dir/"engineer_home.yaml") << "deliberately invalid/excluded";
        std::ofstream(dir/"other_home.yaml") << "deliberately invalid/excluded";
        MapAnalyzer a; a.loadRegions(dir.string());
        auto enemy=target(robot_id::RED,robot_id::R2,1,6,6);
        auto own=target(robot_id::BLUE,robot_id::R2,2,6,6);
        auto r=a.evaluate({enemy,own,enemy});
        check(r.our_engineer_on_island&&r.enemy_engineer_on_island,"both engineers simultaneously");
        check(r.regions[0].enemy_count==1,"deduplicate role/track");
        check(r.busiest_region_ids.size()==2,"overlap and tied maxima");
        auto hero=target(robot_id::RED,robot_id::R1,3,4.25,6);
        r=a.evaluate({hero}); check(r.regions[0].enemy_count==1,"exact margin included");
        hero=target(robot_id::RED,robot_id::R1,3,4.24,6);
        check(a.evaluate({hero}).busiest_region_ids.empty(),"beyond margin excluded");
        hero=target(robot_id::RED,robot_id::R1,3,4.4,4.4);
        check(a.evaluate({hero}).busiest_region_ids.empty(),"corner distance not expanded bounding box");
        hero=target(robot_id::RED,robot_id::R1,3,22,9);
        check(a.evaluate({hero}).busiest_region_ids.at(0)=="middle_mirrored","central mirror");
        a.setTeamByFlip(true);
        r=a.evaluate({enemy,own}); check(r.regions[0].enemy_track_ids.at(0)==2,"team switch");
        a.setFieldXFlip(false);
        own=target(robot_id::BLUE,robot_id::R2,2,6,6,false);
        check(a.evaluate({own}).enemy_engineer_on_island,"world axis reversal");
        own.is_dead=true;check(a.evaluate({own}).busiest_region_ids.empty(),"dead excluded");own.is_dead=false;
        own.lost_duration_s=1.01;check(a.evaluate({own}).busiest_region_ids.empty(),"stale prediction excluded");
        own.lost_duration_s=.5;own.tracking_state=Target::TRACKING_PREDICTED;
        check(!a.evaluate({own}).busiest_region_ids.empty(),"recent prediction included");
        own.tracking_state=Target::TRACKING_LOST;check(a.evaluate({own}).busiest_region_ids.empty(),"lost excluded");
        own.tracking_state=Target::TRACKING_ACTIVE;own.world_x=std::numeric_limits<float>::quiet_NaN();
        check(a.evaluate({own}).busiest_region_ids.empty(),"NaN excluded");
        bool rejected=false;try{a.loadRegions(dir.string(),1.1);}catch(const std::exception&){rejected=true;}
        check(rejected,"invalid tolerance rejected");

        // 重叠归属：沟区盖住中心岛(中央区)时，重合处只算沟区，对称实例同样生效。
        const auto dir2=std::filesystem::temp_directory_path()/
            ("radar27-overlap-test-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        std::filesystem::create_directory(dir2);
        struct Cleanup2 { std::filesystem::path path; ~Cleanup2(){std::filesystem::remove_all(path);} } cleanup2{dir2};
        std::ofstream(dir2/"engineer.yaml") << "blind_zones:\n - polygon: [[1,1],[2,1],[2,2],[1,2]]\n";
        std::ofstream(dir2/"middle.yaml") << "regions:\n - polygon: [[4,4],[8,4],[8,8],[4,8]]\n   mirror_centrally: true\n";
        std::ofstream(dir2/"gully.yaml") << "regions:\n - polygon: [[5,5],[7,5],[7,7],[5,7]]\n   mirror_centrally: true\n";
        const auto count_of=[](const radar27_interfaces::msg::MapTactics& m,const std::string& id){
            for (const auto& reg:m.regions) if (reg.id==id) return static_cast<int>(reg.enemy_count);
            return -1;
        };
        MapAnalyzer b; b.loadRegions(dir2.string());
        const auto in_overlap=target(robot_id::RED,robot_id::R1,7,6,6);
        auto counts=b.evaluate({in_overlap});
        check(count_of(counts,"gully")==1,"gully counted before priority rule");
        check(count_of(counts,"middle")==1,"island counted before priority rule");
        check(counts.busiest_region_ids.size()==2,"gully and island tied before rule");
        b.setOverlapPriority({"gully>middle"});
        counts=b.evaluate({in_overlap});
        check(count_of(counts,"gully")==1&&count_of(counts,"middle")==0,"island loses overlap to gully");
        check(counts.busiest_region_ids.size()==1,"overlap attributed to gully only");
        const auto mirrored=target(robot_id::RED,robot_id::R1,8,22,9);
        counts=b.evaluate({mirrored});
        check(count_of(counts,"gully_mirrored")==1&&count_of(counts,"middle_mirrored")==0,
              "mirrored instance also wins");
        check(counts.busiest_region_ids.at(0)=="gully_mirrored","mirrored overlap attributed to gully");
        bool malformed=false;try{b.setOverlapPriority({"gully"});}catch(const std::exception&){malformed=true;}
        check(malformed,"malformed rule rejected");
        bool unknown=false;try{b.loadRegions(dir2.string());}catch(const std::exception&){unknown=true;}
        check(!unknown,"valid rules survive a rejected update");
        MapAnalyzer c; c.setOverlapPriority({"gully>nowhere"});
        bool bad_name=false;try{c.loadRegions(dir2.string());}catch(const std::exception&){bad_name=true;}
        check(bad_name,"unknown region in rule rejected");
        std::cout << "Region analysis: all checks passed\n";
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
