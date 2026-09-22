#include "tensorrt_detect/core/target_fusion.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace radar27_interfaces::msg;
void check(bool ok, const char* text) { if (!ok) throw std::runtime_error(text); }
int main() {
    WorldTargetArray w; w.header.stamp.sec=10; w.targets.resize(10);
    auto& t=w.targets[1]; t.idx=99; t.team_id=1; t.class_id=3; t.track_id=7;
    t.valid=true; t.observed=true; t.position_source=WorldTarget::POSITION_TRACKED;
    t.world_x=1.2; t.world_z=-3.4; t.tracking_confidence=.9; t.last_observed_time.sec=8;
    PriorPredictionArray p; p.header=w.header; p.model_enabled=true;
    p.predictions.resize(1); auto& q=p.predictions[0];
    q.slot_idx=1; q.team_id=1; q.role_class_id=3; q.track_id=7; q.valid=true;
    q.prior_world_x=5; q.prior_world_z=6; q.prior_confidence=.6; q.last_observed_time.sec=8;
    auto get=[&](){return target_fusion::fuse(w,&p,.5);};
    auto out=get(); check(out.targets.size()==10,"fixed slots");
    check(out.targets[1].source==FusedTarget::SOURCE_TRACKED && out.targets[1].world_x==t.world_x,"observed priority/copy");
    t.observed=false; t.position_source=WorldTarget::POSITION_PREDICTED;
    out=get();check(out.targets[1].valid && out.targets[1].source==FusedTarget::SOURCE_PRIOR && out.targets[1].world_x==5,"lost uses prior entry slot not array index");
    check(!target_fusion::fuse(w,nullptr,.5).targets[1].valid,"no CV fallback");
    p.header.stamp.sec=9;p.header.stamp.nanosec=500000000;
    out=get();check(out.targets[1].valid && out.targets[1].source_stamp==p.header.stamp,"age boundary/source timestamp");
    p.header.stamp.nanosec=499999999;check(!get().targets[1].valid,"stale prior");
    p.header.stamp.sec=11;check(!get().targets[1].valid,"future prior");p.header=w.header;
    q.track_id=8;check(!get().targets[1].valid,"wrong identity");q.track_id=7;
    q.team_id=2;check(!get().targets[1].valid,"wrong team");q.team_id=1;
    q.role_class_id=4;check(!get().targets[1].valid,"wrong role");q.role_class_id=3;
    t.last_observed_time.sec=9;check(!get().targets[1].valid,"old anchor after reacquisition");t.last_observed_time.sec=8;
    t.is_dead=true;check(!get().targets[1].valid,"confirmed dead");t.is_dead=false;
    p.model_enabled=false;check(!get().targets[1].valid,"disabled model");p.model_enabled=true;
    q.valid=false;check(!get().targets[1].valid,"rejected prior");q.valid=true;
    q.prior_world_x=std::numeric_limits<float>::quiet_NaN();check(!get().targets[1].valid,"nonfinite");q.prior_world_x=5;
    t.valid=false;t.track_id=-1;check(get().targets[1].valid,"expired tracker slot allows valid persistent prior");
    t.observed=true;check(!get().targets[1].valid,"invalid observation does not resurrect guess");
    w.targets.clear();check(!get().targets[1].valid,"missing slot clears prior");
    std::cout << "Fusion source selection: all checks passed\n";
}
