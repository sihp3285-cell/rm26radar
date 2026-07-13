#include "bot_identity.hpp"

BotIdentity::BotIdentity(const BotIdentityConfig& cfg) {
    configure(cfg);
}

void BotIdentity::configure(const BotIdentityConfig& cfg) {
    maxHistory_ = cfg.maxHistory;
    purgeThreshold_ = cfg.purgeThreshold;
    minHistoryForStable_ = cfg.minHistoryForStable;
    decay_ = cfg.decay;
    numClasses_ = cfg.numClasses;
}

void BotIdentity::update(int class_id, float class_conf, float class_margin) {
    lost_counter_ = 0;
    history_.push_back({class_id, class_conf, class_margin});
    if (history_.size() > static_cast<size_t>(maxHistory_)) {
        history_.pop_front();
    }
}

void BotIdentity::updateRecent(int class_id, float class_conf) {
    recent_history_.push_back({class_id, std::max(class_conf, 0.0f)});
    constexpr size_t RECENT_WINDOW = 5;
    if (recent_history_.size() > RECENT_WINDOW) {
        recent_history_.pop_front();
    }
}

void BotIdentity::markLost() {
    lost_counter_++;
    if (lost_counter_ >= purgeThreshold_) {
        reset();
    }
}

void BotIdentity::reset() {
    history_.clear();
    recent_history_.clear();
    lost_counter_ = 0;
}

bool BotIdentity::empty() const {
    return history_.empty();
}

bool BotIdentity::shouldPurge() const {
    return lost_counter_ >= purgeThreshold_;
}

std::pair<int, float> BotIdentity::getStableClass() const {
    const auto stats = getStats();
    return {stats.class_id, stats.confidence};
}

BotIdentity::Stats BotIdentity::getStats() const {
    Stats stats;
    if (history_.empty()) {
        return stats;
    }

    std::vector<float> scores(numClasses_, 0.0f);
    std::vector<float> margins(numClasses_, 0.0f);
    std::vector<float> class_weights(numClasses_, 0.0f);
    float weight_sum = 0.0f;
    float score_sum = 0.0f;
    int switches = 0;

    int N = static_cast<int>(history_.size());
    for (int i = 0; i < N; ++i) {
        // i=0 是最老的, i=N-1 是最新的
        float weight = std::pow(decay_, N - 1 - i);
        const auto& obs = history_[i];
        if (obs.class_id >= 0 && obs.class_id < numClasses_) {
            const float evidence = obs.class_conf * weight;
            scores[obs.class_id] += evidence;
            margins[obs.class_id] += std::max(obs.class_margin, 0.0f) * evidence;
            class_weights[obs.class_id] += evidence;
            score_sum += evidence;
            if (i > 0 && history_[i - 1].class_id != obs.class_id) {
                switches++;
            }
        }
        weight_sum += weight;
    }

    int best_id = -1;
    float best_score = 0.0f;
    for (int c = 0; c < numClasses_; ++c) {
        if (scores[c] > best_score) {
            best_score = scores[c];
            best_id = c;
        }
    }

    stats.confidence = weight_sum > 0.0f ? best_score / weight_sum : 0.0f;
    stats.margin = best_id >= 0 && class_weights[best_id] > 0.0f
        ? margins[best_id] / class_weights[best_id] : 0.0f;
    stats.switch_rate = history_.size() > 1
        ? static_cast<float>(switches) / static_cast<float>(history_.size() - 1) : 0.0f;
    stats.stability = score_sum > 0.0f ? best_score / score_sum : 0.0f;
    if (history_.size() >= static_cast<size_t>(minHistoryForStable_)) {
        stats.class_id = best_id;
    }
    return stats;
}

float BotIdentity::getRecentConfidence(int class_id) const {
    if (recent_history_.empty()) {
        return 0.0f;
    }

    float confidence = 0.0f;
    for (const auto& observation : recent_history_) {
        if (observation.first == class_id) {
            confidence += observation.second;
        }
    }
    return confidence / static_cast<float>(recent_history_.size());
}
