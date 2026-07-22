#include "position_prior/position_prior_model.hpp"

#include <openssl/evp.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace position_prior {

namespace {

std::vector<PriorCandidate> parse_candidates(const YAML::Node& node) {
    std::vector<PriorCandidate> result;
    if (!node || !node.IsSequence()) {
        return result;
    }
    result.reserve(node.size());
    for (const auto& item : node) {
        PriorCandidate candidate;
        candidate.grid_index = item["grid_index"].as<int>();
        candidate.canonical.x = item["x"].as<double>();
        candidate.canonical.y = item["y"].as<double>();
        candidate.probability = item["p"].as<double>();
        if (!std::isfinite(candidate.canonical.x) ||
            !std::isfinite(candidate.canonical.y) ||
            !std::isfinite(candidate.probability) ||
            candidate.probability < 0.0) {
            throw std::runtime_error("模型候选点包含无效数值");
        }
        result.push_back(candidate);
    }
    return result;
}

PositionPriorModel::DistributionData parse_distribution(
    const YAML::Node& node,
    bool local) {
    PositionPriorModel::DistributionData data;
    data.samples = node["samples"] ? node["samples"].as<std::uint32_t>() : 0;
    data.local_weight = local && node["local_weight"]
        ? node["local_weight"].as<double>() : 0.0;
    data.stay_probability = local && node["stay_probability"]
        ? node["stay_probability"].as<double>() : 0.0;
    data.retained_probability_mass = node["retained_probability_mass"]
        ? node["retained_probability_mass"].as<double>() : 0.0;
    data.candidates = parse_candidates(node["candidates"]);
    return data;
}

std::string lowercase(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

}  // namespace

std::string PositionPriorModel::sha256_file(const std::string& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("无法打开模型文件计算 SHA-256: " + path);
    }

    EVP_MD_CTX* raw_context = EVP_MD_CTX_new();
    if (!raw_context) {
        throw std::runtime_error("无法创建 SHA-256 context");
    }
    struct ContextGuard {
        EVP_MD_CTX* value;
        ~ContextGuard() { EVP_MD_CTX_free(value); }
    } guard{raw_context};

    if (EVP_DigestInit_ex(raw_context, EVP_sha256(), nullptr) != 1) {
        throw std::runtime_error("SHA-256 初始化失败");
    }

    std::array<char, 1 << 16> buffer{};
    while (input) {
        input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
        const auto count = input.gcount();
        if (count > 0 &&
            EVP_DigestUpdate(raw_context, buffer.data(), static_cast<std::size_t>(count)) != 1) {
            throw std::runtime_error("SHA-256 更新失败");
        }
    }

    std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
    unsigned int digest_length = 0;
    if (EVP_DigestFinal_ex(raw_context, digest.data(), &digest_length) != 1) {
        throw std::runtime_error("SHA-256 结束失败");
    }

    std::ostringstream output;
    output << std::hex << std::setfill('0');
    for (unsigned int i = 0; i < digest_length; ++i) {
        output << std::setw(2) << static_cast<int>(digest[i]);
    }
    return output.str();
}

void PositionPriorModel::load(
    const std::string& model_path,
    const std::string& expected_sha256) {
    loaded_ = false;
    roles_.clear();
    horizons_seconds_.clear();

    model_path_ = model_path;
    model_sha256_ = sha256_file(model_path);
    if (!expected_sha256.empty() &&
        lowercase(model_sha256_) != lowercase(expected_sha256)) {
        throw std::runtime_error(
            "模型 SHA-256 不匹配，expected=" + expected_sha256 +
            " actual=" + model_sha256_);
    }

    const YAML::Node root = YAML::LoadFile(model_path);
    model_version_ = root["metadata"]["model_version"].as<int>();
    map_id_ = root["metadata"]["map_id"].as<std::string>();
    if (model_version_ != 1 || map_id_ != "RMUC2026") {
        throw std::runtime_error("不支持的先验模型版本或地图: version=" +
            std::to_string(model_version_) + " map=" + map_id_);
    }

    const auto geometry = root["geometry"];
    field_length_ = geometry["field_size"][0].as<double>();
    field_width_ = geometry["field_size"][1].as<double>();
    zone_size_x_ = geometry["current_zone_size"][0].as<double>();
    zone_size_y_ = geometry["current_zone_size"][1].as<double>();
    zone_count_x_ = geometry["current_zone_shape"][0].as<int>();
    zone_count_y_ = geometry["current_zone_shape"][1].as<int>();
    grid_size_x_ = geometry["endpoint_grid_size"][0].as<double>();
    grid_size_y_ = geometry["endpoint_grid_size"][1].as<double>();
    grid_count_x_ = geometry["endpoint_grid_shape"][0].as<int>();
    grid_count_y_ = geometry["endpoint_grid_shape"][1].as<int>();

    if (std::abs(field_length_ - 28.0) > 1e-9 ||
        std::abs(field_width_ - 15.0) > 1e-9 ||
        std::abs(zone_size_x_ - 2.0) > 1e-9 ||
        std::abs(zone_size_y_ - 1.5) > 1e-9 ||
        std::abs(grid_size_x_ - 0.5) > 1e-9 ||
        std::abs(grid_size_y_ - 0.5) > 1e-9 ||
        zone_count_x_ != 14 || zone_count_y_ != 10 ||
        grid_count_x_ != 56 || grid_count_y_ != 30 ||
        geometry["canonical_team"].as<std::string>() != "red") {
        throw std::runtime_error("模型 geometry 与 RMUC2026 在线约定不一致");
    }

    for (const auto& value : root["horizons_seconds"]) {
        horizons_seconds_.push_back(value.as<int>());
    }
    std::sort(horizons_seconds_.begin(), horizons_seconds_.end());
    default_context_ = root["default_context"].as<std::string>();
    if (horizons_seconds_ != std::vector<int>{2, 5, 10} ||
        default_context_ != "all_phase") {
        throw std::runtime_error("模型 horizon 或默认 context 与在线 v1 约定不一致");
    }

    const YAML::Node roles = root["roles"];
    for (auto role_it = roles.begin(); role_it != roles.end(); ++role_it) {
        const std::string role_name = role_it->first.as<std::string>();
        RoleData role_data;
        const YAML::Node contexts = role_it->second["contexts"];

        // 当前在线仅加载 all_phase，避免保留无用分阶段树。
        const YAML::Node context_node = contexts[default_context_];
        if (!context_node) {
            continue;
        }
        ContextData context_data;
        context_data.global = parse_distribution(context_node["global"], false);

        const YAML::Node zones = context_node["zones"];
        for (auto zone_it = zones.begin(); zone_it != zones.end(); ++zone_it) {
            const int zone = std::stoi(zone_it->first.as<std::string>());
            for (auto horizon_it = zone_it->second.begin();
                 horizon_it != zone_it->second.end(); ++horizon_it) {
                const std::string key = horizon_it->first.as<std::string>();
                if (key.size() < 2 || key.front() != 'h') {
                    continue;
                }
                const int horizon = std::stoi(key.substr(1));
                context_data.zones[zone][horizon] =
                    parse_distribution(horizon_it->second, true);
            }
        }
        role_data.contexts.emplace(default_context_, std::move(context_data));
        roles_.emplace(role_name, std::move(role_data));
    }

    for (const std::string& required_role :
         {"hero", "engineer", "infantry3", "infantry4", "sentry"}) {
        if (!supports_role(required_role)) {
            throw std::runtime_error("模型缺少在线兵种: " + required_role);
        }
    }
    loaded_ = true;
}

bool PositionPriorModel::supports_role(const std::string& role) const {
    return roles_.find(role) != roles_.end();
}

bool PositionPriorModel::valid_canonical(const Point2d& point) const {
    return std::isfinite(point.x) && std::isfinite(point.y) &&
           point.x >= 0.0 && point.x <= field_length_ &&
           point.y >= 0.0 && point.y <= field_width_;
}

int PositionPriorModel::zone_index(const Point2d& canonical) const {
    if (!loaded_ || !valid_canonical(canonical)) {
        return -1;
    }
    const int ix = std::min(zone_count_x_ - 1,
        std::max(0, static_cast<int>(canonical.x / zone_size_x_)));
    const int iy = std::min(zone_count_y_ - 1,
        std::max(0, static_cast<int>(canonical.y / zone_size_y_)));
    return iy * zone_count_x_ + ix;
}

int PositionPriorModel::nearest_horizon(double lost_duration_s) const {
    if (horizons_seconds_.empty() || !std::isfinite(lost_duration_s)) {
        return 0;
    }
    return *std::min_element(
        horizons_seconds_.begin(), horizons_seconds_.end(),
        [lost_duration_s](int lhs, int rhs) {
            const double lhs_distance = std::abs(lhs - lost_duration_s);
            const double rhs_distance = std::abs(rhs - lost_duration_s);
            return lhs_distance == rhs_distance ? lhs < rhs : lhs_distance < rhs_distance;
        });
}

double PositionPriorModel::normalized_entropy(
    const std::vector<PriorCandidate>& candidates) {
    if (candidates.size() <= 1) {
        return 0.0;
    }
    double total = 0.0;
    for (const auto& candidate : candidates) {
        total += std::max(0.0, candidate.probability);
    }
    if (total <= 0.0) {
        return 1.0;
    }
    double entropy = 0.0;
    for (const auto& candidate : candidates) {
        const double probability = std::max(0.0, candidate.probability) / total;
        if (probability > 0.0) {
            entropy -= probability * std::log(probability);
        }
    }
    return std::clamp(entropy / std::log(static_cast<double>(candidates.size())), 0.0, 1.0);
}

PriorDistribution PositionPriorModel::query(
    const std::string& role,
    const Point2d& canonical_current,
    int horizon_seconds,
    const std::string& context,
    std::size_t top_k) const {
    PriorDistribution result;
    result.role = role;
    result.context = context;
    result.horizon_seconds = horizon_seconds;
    result.zone_index = zone_index(canonical_current);

    if (!loaded_) {
        result.error = "model_not_loaded";
        return result;
    }
    const auto role_it = roles_.find(role);
    if (role_it == roles_.end()) {
        result.error = "unsupported_role";
        return result;
    }
    if (result.zone_index < 0) {
        result.error = "canonical_position_out_of_field";
        return result;
    }
    if (std::find(horizons_seconds_.begin(), horizons_seconds_.end(), horizon_seconds) ==
        horizons_seconds_.end()) {
        result.error = "unsupported_horizon";
        return result;
    }

    auto context_it = role_it->second.contexts.find(context);
    if (context_it == role_it->second.contexts.end()) {
        context_it = role_it->second.contexts.find(default_context_);
        result.context = default_context_;
    }
    if (context_it == role_it->second.contexts.end()) {
        result.error = "missing_context";
        return result;
    }

    const DistributionData* selected = nullptr;
    const auto zone_it = context_it->second.zones.find(result.zone_index);
    if (zone_it != context_it->second.zones.end()) {
        const auto horizon_it = zone_it->second.find(horizon_seconds);
        if (horizon_it != zone_it->second.end() && !horizon_it->second.candidates.empty()) {
            selected = &horizon_it->second;
            result.fallback_level = FallbackLevel::LOCAL_ZONE;
        }
    }
    if (!selected) {
        selected = &context_it->second.global;
        result.fallback_level = FallbackLevel::GLOBAL_ROLE;
    }
    if (selected->candidates.empty()) {
        result.error = "empty_distribution";
        return result;
    }

    result.sample_count = selected->samples;
    result.local_weight = selected->local_weight;
    result.stay_probability = selected->stay_probability;
    result.retained_probability_mass = selected->retained_probability_mass;
    result.candidates = selected->candidates;
    if (top_k > 0 && result.candidates.size() > top_k) {
        result.candidates.resize(top_k);
    }
    result.normalized_entropy = normalized_entropy(result.candidates);
    result.valid = true;
    return result;
}

}  // namespace position_prior
