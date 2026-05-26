#include "bot_identity.hpp"
#include <algorithm>

void BotIdentity::update(int class_id, float class_conf) {
    lost_counter_ = 0;
    history_.push_back({class_id, class_conf});
    if (history_.size() > MAX_HISTORY) {
        history_.pop_front();
    }
}

void BotIdentity::markLost() {
    lost_counter_++;
    if (lost_counter_ >= PURGE_THRESHOLD) {
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
    return lost_counter_ >= PURGE_THRESHOLD;
}

std::pair<int, float> BotIdentity::getStableClass() const {
    if (history_.empty()) {
        return {-1, 0.0f};
    }
    if (history_.size() < MIN_HISTORY_FOR_STABLE) {
        return {-1, 0.0f};
    }

    std::vector<float> scores(NUM_CLASSES, 0.0f);
    float weight_sum = 0.0f;

    int N = static_cast<int>(history_.size());
    for (int i = 0; i < N; ++i) {
        // i=0 是最老的, i=N-1 是最新的
        float weight = std::pow(DECAY, N - 1 - i);
        const auto& obs = history_[i];
        if (obs.class_id >= 0 && obs.class_id < NUM_CLASSES) {
            scores[obs.class_id] += obs.class_conf * weight;
        }
        weight_sum += weight;
    }

    int best_id = -1;
    float best_score = 0.0f;
    for (int c = 0; c < NUM_CLASSES; ++c) {
        if (scores[c] > best_score) {
            best_score = scores[c];
            best_id = c;
        }
    }

    float normalized_conf = weight_sum > 0.0f ? best_score / weight_sum : 0.0f;
    return {best_id, normalized_conf};
}
