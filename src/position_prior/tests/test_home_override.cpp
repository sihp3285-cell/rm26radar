#include "position_prior/blind_zone_prior.hpp"
#include "position_prior/prior_gate.hpp"
#include <cmath>
#include <stdexcept>
#include <iostream>
using namespace position_prior;
void check(bool ok) { if (!ok) throw std::runtime_error("home override regression"); }
int main(int argc, char** argv) {
    check(argc == 2);
    const std::string dir = argv[1];
    NavigationMesh mesh;
    mesh.load(dir + "/RB2026_navgrid_v1.json");
    BlindZonePrior blind;
    blind.load({dir + "/home.yaml"}, dir + "/engineer.yaml",
               dir + "/engineer_home.yaml", dir + "/other_home.yaml");
    PriorGateConfig config;
    config.max_guess_distance_m = 4.0;
    PriorGate gate(config);
    gate.set_navigation_mesh(&mesh);
    gate.set_blind_zone_prior(&blind);
    // Interior and entrance outside the polygon: both use the same home rule.
    for (double x : {3.0, 3.6}) {
        for (int horizon : {2, 5, 10}) {
            PriorDistribution distribution;
            distribution.valid = true;
            distribution.horizon_seconds = horizon;
            distribution.local_weight = 1.0;
            distribution.fallback_level = FallbackLevel::LOCAL_ZONE;
            distribution.stay_probability = 0.95;
            PriorCandidate candidate;
            candidate.canonical = {x, 2.4};
            candidate.probability = 1.0;
            distribution.candidates.push_back(candidate);
            const auto result = gate.apply(distribution, {x, 2.4}, {-0.5, 0.0},
                                           horizon, 1.0, "engineer");
            check(result.candidates.size() == 1);
            check(std::abs(result.candidates[0].prior.canonical.x - 2.1736) < 1e-9);
            check(std::abs(result.candidates[0].prior.canonical.y - 0.9343) < 1e-9);
            // This entrance fixture is disconnected in the NavGrid: reject rather than
            // falling back to a historical point. The interior fixture must publish.
            if (x > 3.5) {
                check(result.valid || result.rejection_reason == "no_reachable_candidate");
                continue;
            }
            check(result.valid);
            check(result.stay_anchor_probability_mass == 0.0);
            check(std::abs(result.predicted_canonical.x - 2.1736) < 1e-9);
            check(std::abs(result.predicted_canonical.y - 0.9343) < 1e-9);
        }
    }
    std::cout << "PASS engineer fixed point inside/near home at 2/5/10 seconds\n";
}
