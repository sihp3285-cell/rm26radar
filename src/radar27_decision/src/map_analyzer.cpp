#include <radar27_decision/map_analyzer.hpp>
#include <rm_field/field_geometry.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <map>
#include <set>
#include <stdexcept>

namespace {
using Region = radar27_interfaces::msg::RegionOccupancy;
using Target = radar27_interfaces::msg::WorldTarget;
// Interior OR distance to any boundary segment <= margin, including concave polygons.
bool contains(const Region& r, double x, double y, double margin) {
    bool inside = false;
    for (size_t i=0, j=r.field_x.size()-1; i<r.field_x.size(); j=i++) {
        const double ax=r.field_x[j], ay=r.field_y[j], bx=r.field_x[i], by=r.field_y[i];
        const double dx=bx-ax, dy=by-ay, length2=dx*dx+dy*dy;
        const double t=length2>0 ? std::clamp(((x-ax)*dx+(y-ay)*dy)/length2,0.0,1.0) : 0;
        if (std::hypot(x-ax-t*dx,y-ay-t*dy)<=margin+1e-8) return true;
        if ((ay>y)!=(by>y) && x<(bx-ax)*(y-ay)/(by-ay)+ax) inside=!inside;
    }
    return inside;
}
std::string displayName(const std::string& id) {
    static const std::map<std::string,std::string> names={
        {"home","基地盲区"},{"gully","沟区"},{"engineer","工程岛"},
        {"middle","中央区"},{"left_highland","左侧高地"},
        {"right_highland","右侧高地"},{"flying_slope","飞坡区"}};
    const auto it=names.find(id); return it==names.end()?id:it->second;
}
std::string trim(const std::string& value) {
    const auto first=value.find_first_not_of(" \t");
    if (first==std::string::npos) return {};
    return value.substr(first,value.find_last_not_of(" \t")-first+1);
}
}
void MapAnalyzer::loadRegions(const std::string& directory,double margin,double age) {
    if (!std::isfinite(margin)||margin<0.5||margin>1.0||!std::isfinite(age)||age<0)
        throw std::invalid_argument("invalid region margin/prediction age");
    region_base_.clear();
    std::vector<std::filesystem::path> paths;
    for (const auto& entry:std::filesystem::directory_iterator(directory)) {
        const auto stem=entry.path().stem().string();
        if (entry.is_regular_file()&&entry.path().extension()==".yaml"&&
            stem!="engineer_home"&&stem!="other_home") paths.push_back(entry.path());
    }
    std::sort(paths.begin(),paths.end());
    std::vector<Region> loaded;
    std::set<std::string> ids;
    for (const auto& path:paths) {
        const auto root=YAML::LoadFile(path.string());
        const auto entries=root["regions"]?root["regions"]:root["blind_zones"];
        if (!entries||!entries.IsSequence()||entries.size()==0)
            throw std::runtime_error("missing regions/blind_zones: "+path.string());
        for (size_t n=0;n<entries.size();++n) {
            const auto item=entries[n], points=item["polygon"];
            if (!points||!points.IsSequence()||points.size()<3)
                throw std::runtime_error("invalid polygon: "+path.string());
            Region r; const auto stem=path.stem().string();
            r.id=stem+(entries.size()>1?"_"+std::to_string(n):"");
            r.name=displayName(stem)+(entries.size()>1?" "+std::to_string(n+1):"");
            for (const auto& p:points) {
                if (!p.IsSequence()||p.size()!=2) throw std::runtime_error("invalid vertex: "+path.string());
                const double x=p[0].as<double>(),y=p[1].as<double>();
                if (!std::isfinite(x)||!std::isfinite(y)||x<0||x>28||y<0||y>15)
                    throw std::runtime_error("vertex outside 28x15 field: "+path.string());
                r.field_x.push_back(x); r.field_y.push_back(y);
            }
            double area2=0;
            for (size_t i=0,j=r.field_x.size()-1;i<r.field_x.size();j=i++)
                area2+=r.field_x[j]*r.field_y[i]-r.field_x[i]*r.field_y[j];
            if (std::abs(area2)<1e-6) throw std::runtime_error("degenerate polygon: "+path.string());
            if (!ids.insert(r.id).second) throw std::runtime_error("duplicate region: "+r.id);
            loaded.push_back(r); region_base_.push_back(stem);
            if (item["mirror_centrally"]&&item["mirror_centrally"].as<bool>()) {
                r.id+="_mirrored"; r.name+="（对称）";
                for (auto& x:r.field_x) x=28-x;
                for (auto& y:r.field_y) y=15-y;
                if (!ids.insert(r.id).second) throw std::runtime_error("duplicate region: "+r.id);
                loaded.push_back(r); region_base_.push_back(stem);
            }
        }
    }
    if (loaded.empty()||!ids.count("engineer")) throw std::runtime_error("regions must include engineer.yaml");
    // 规则里写错的区域名不会命中任何区域，必须在装载时就暴露出来。
    for (const auto& [winner, loser] : overlap_priority_)
        for (const auto* base : {&winner, &loser})
            if (std::find(region_base_.begin(),region_base_.end(),*base)==region_base_.end())
                throw std::invalid_argument("overlap rule references unknown region: "+*base);
    regions_=std::move(loaded); margin_m_=margin; max_prediction_age_s_=age;
}
void MapAnalyzer::setOverlapPriority(const std::vector<std::string>& rules) {
    std::vector<std::pair<std::string,std::string>> parsed;
    for (const auto& rule : rules) {
        const auto separator=rule.find('>');
        const std::string winner=trim(rule.substr(0,separator));
        const std::string loser=separator==std::string::npos
            ? std::string{} : trim(rule.substr(separator+1));
        if (winner.empty()||loser.empty()||winner==loser)
            throw std::invalid_argument("invalid region overlap rule: "+rule);
        parsed.emplace_back(winner,loser);
    }
    overlap_priority_=std::move(parsed);
}
radar27_interfaces::msg::MapTactics MapAnalyzer::evaluate(const std::vector<Target>& targets) const {
    radar27_interfaces::msg::MapTactics result;
    result.own_team=own_team_; result.boundary_margin_m=margin_m_; result.regions=regions_;
    std::map<std::pair<int,int>,const Target*> selected;
    for (const auto& t:targets) {
        if (!t.valid||t.is_dead||t.track_id<0||
            (t.team_id!=robot_id::RED&&t.team_id!=robot_id::BLUE)||
            t.stable_class_id<robot_id::R1||t.stable_class_id>robot_id::S||
            !std::isfinite(t.stable_class_conf)||t.stable_class_conf<=0||
            !std::isfinite(t.world_x)||!std::isfinite(t.world_z)||
            (t.tracking_state!=Target::TRACKING_ACTIVE&&t.tracking_state!=Target::TRACKING_PREDICTED)||
            t.position_source==Target::POSITION_INVALID||
            !std::isfinite(t.lost_duration_s)||t.lost_duration_s<0||t.lost_duration_s>max_prediction_age_s_) continue;
        const auto key=std::make_pair(t.team_id,t.stable_class_id);
        const auto it=selected.find(key);
        if (it==selected.end()||(t.observed&&!it->second->observed)||
            (t.observed==it->second->observed&&t.stable_class_conf>it->second->stable_class_conf)) selected[key]=&t;
    }
    // 先把入选目标换算到 field 并判定归属，重叠优先级和分区计数共用这一份结果。
    std::vector<const Target*> chosen;
    std::vector<double> field_x, field_y;
    chosen.reserve(selected.size());
    for (const auto& [identity,ptr]:selected) {
        double x,y; rm_field::world_to_field(ptr->world_x,ptr->world_z,toward_blue_,x,y);
        if (x<0||x>28||y<0||y>15) continue;
        chosen.push_back(ptr); field_x.push_back(x); field_y.push_back(y);
    }
    const size_t target_count=chosen.size(), region_count=result.regions.size();
    std::vector<char> inside(target_count*region_count,0);
    for (size_t ti=0;ti<target_count;++ti)
        for (size_t ri=0;ri<region_count;++ri)
            inside[ti*region_count+ri]=contains(result.regions[ri],field_x[ti],field_y[ti],margin_m_)?1:0;
    unsigned max_count=0;
    for (size_t ri=0;ri<region_count;++ri) {
        auto& r=result.regions[ri];
        for (size_t i=0;i<r.field_x.size();++i) {
            double x,z; rm_field::field_to_world(r.field_x[i],r.field_y[i],toward_blue_,x,z);
            r.world_x.push_back(x); r.world_z.push_back(z);
        }
        const std::string base=ri<region_base_.size()?region_base_[ri]:std::string();
        std::set<std::pair<int,int>> seen_tracks;
        for (size_t ti=0;ti<target_count;++ti) {
            if (!inside[ti*region_count+ri]) continue;
            const auto& t=*chosen[ti];
            // 落在重叠处时让给优先级更高的区域，避免同一目标被两区同时统计。
            bool suppressed=false;
            for (const auto& [winner,loser]:overlap_priority_) {
                if (base!=loser) continue;
                for (size_t rj=0;rj<region_count;++rj)
                    if (region_base_[rj]==winner&&inside[ti*region_count+rj]) {suppressed=true;break;}
                if (suppressed) break;
            }
            if (suppressed) continue;
            if (!seen_tracks.insert({t.team_id,t.track_id}).second) continue;
            if ((r.id=="engineer"||r.id=="engineer_mirrored")&&t.stable_class_id==robot_id::R2) {
                if (t.team_id==own_team_) result.our_engineer_on_island=true;
                else result.enemy_engineer_on_island=true;
            }
            if (t.team_id!=own_team_) {
                r.enemy_class_ids.push_back(t.stable_class_id); r.enemy_track_ids.push_back(t.track_id);
            }
        }
        r.enemy_count=r.enemy_class_ids.size(); max_count=std::max(max_count,r.enemy_count);
    }
    if (max_count) for (const auto& r:result.regions)
        if (r.enemy_count==max_count) result.busiest_region_ids.push_back(r.id);
    return result;
}
