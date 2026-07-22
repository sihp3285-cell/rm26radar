#pragma once

#include "position_prior/prior_types.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace position_prior {

class PositionPriorModel {
public:
    void load(const std::string& model_path, const std::string& expected_sha256 = "");

    bool loaded() const { return loaded_; }
    const std::string& model_path() const { return model_path_; }
    const std::string& model_sha256() const { return model_sha256_; }
    int model_version() const { return model_version_; }
    const std::string& map_id() const { return map_id_; }
    double field_length() const { return field_length_; }
    double field_width() const { return field_width_; }
    const std::vector<int>& horizons_seconds() const { return horizons_seconds_; }

    bool supports_role(const std::string& role) const;
    int zone_index(const Point2d& canonical) const;
    int nearest_horizon(double lost_duration_s) const;

    PriorDistribution query(
        const std::string& role,
        const Point2d& canonical_current,
        int horizon_seconds,
        const std::string& context = "all_phase",
        std::size_t top_k = 0) const;

    static std::string sha256_file(const std::string& path);

public:
    // 暴露仅用于轻量解析辅助；实时调用方使用 query()。
    struct DistributionData {
        std::uint32_t samples = 0;
        double local_weight = 0.0;
        double stay_probability = 0.0;
        double retained_probability_mass = 0.0;
        std::vector<PriorCandidate> candidates;
    };

    struct ContextData {
        DistributionData global;
        std::unordered_map<int, std::unordered_map<int, DistributionData>> zones;
    };

    struct RoleData {
        std::unordered_map<std::string, ContextData> contexts;
    };

private:
    bool valid_canonical(const Point2d& point) const;
    static double normalized_entropy(const std::vector<PriorCandidate>& candidates);

    bool loaded_ = false;
    std::string model_path_;
    std::string model_sha256_;
    int model_version_ = 0;
    std::string map_id_;
    double field_length_ = 0.0;
    double field_width_ = 0.0;
    double zone_size_x_ = 0.0;
    double zone_size_y_ = 0.0;
    int zone_count_x_ = 0;
    int zone_count_y_ = 0;
    double grid_size_x_ = 0.0;
    double grid_size_y_ = 0.0;
    int grid_count_x_ = 0;
    int grid_count_y_ = 0;
    std::vector<int> horizons_seconds_;
    std::string default_context_ = "all_phase";
    std::unordered_map<std::string, RoleData> roles_;
};

}  // namespace position_prior
