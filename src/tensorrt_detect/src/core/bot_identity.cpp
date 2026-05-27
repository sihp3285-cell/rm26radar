#include "bot_identity.hpp"
#include <algorithm>

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

void BotIdentity::update(int class_id, float class_conf) {
    lost_counter_ = 0;
    history_.push_back({class_id, class_conf});
    if (history_.size() > static_cast<size_t>(maxHistory_)) {
        history_.pop_front();
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
    lost_counter_ = 0;
}

bool BotIdentity::empty() const {
    return history_.empty();
}

bool BotIdentity::shouldPurge() const {
    return lost_counter_ >= purgeThreshold_;
}

std::pair<int, float> BotIdentity::getStableClass() const {
    if (history_.empty()) {
        return {-1, 0.0f};
    }
    if (history_.size() < static_cast<size_t>(minHistoryForStable_)) {
        return {-1, 0.0f};
    }

    std::vector<float> scores(numClasses_, 0.0f);
    float weight_sum = 0.0f;

    int N = static_cast<int>(history_.size());
    for (int i = 0; i < N; ++i) {
        // i=0 是最老的, i=N-1 是最新的
        float weight = std::pow(decay_, N - 1 - i);
        const auto& obs = history_[i];
        if (obs.class_id >= 0 && obs.class_id < numClasses_) {
            scores[obs.class_id] += obs.class_conf * weight;
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

    float normalized_conf = weight_sum > 0.0f ? best_score / weight_sum : 0.0f;
    return {best_id, normalized_conf};
}
