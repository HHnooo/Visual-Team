#include "robot.h"

Robot::Robot(uint16_t team, uint16_t id, int type, uint16_t level)
    : team_(team), id_(id), type_(type), level_(level), hp_(0),
      max_hp_(0), heat_(0), max_heat_(0), last_update_time_(0) {
}

Infantry::Infantry(uint16_t team, uint16_t id, uint16_t level)
    : Robot(team, id, 0, level) {
    update_Status_by_level();
    reset();
}

void Infantry::update_Status_by_level() {
    switch (level_) {
        case 1:
            max_hp_ = 100;
            max_heat_ = 100;
            break;
        case 2:
            max_hp_ = 150;
            max_heat_ = 200;
            break;
        case 3:
            max_hp_ = 250;
            max_heat_ = 300;
            break;
        default:
            max_hp_ = 100;
            max_heat_ = 100;
    }
}

void Infantry::reset() {
    hp_ = max_hp_;
    heat_ = 0;
}

void Infantry::update_heat(uint16_t seconds) {
    for (uint16_t i = 0; i < seconds; i++) {
        if (heat_ > 0) heat_--;
        if (heat_ > max_heat_ && hp_ > 0) {
            hp_--;
        }
        if (hp_ == 0) break;
    }
}

void Infantry::update(uint16_t current_time) {
    if (last_update_time_ == 0) {
        last_update_time_ = current_time;
        return;
    }

    if (current_time > last_update_time_) {
        update_heat(current_time - last_update_time_);
        last_update_time_ = current_time;
    }
}

void Infantry::take_damage(uint16_t damage) {
    if (damage >= hp_) {
        hp_ = 0;
    } else {
        hp_ -= damage;
    }
}

void Infantry::add_heat(uint16_t heat) {
    heat_ += heat;
}

bool Infantry::can_upgrade(uint16_t target_level) const {
    return target_level > level_ && target_level >= 1 && target_level <= 3;
}

void Infantry::upgrade(uint16_t target_level) {
    if (can_upgrade(target_level)) {
        level_ = target_level;
        update_Status_by_level();
        hp_ = max_hp_;
    }
}

Engineer::Engineer(uint16_t team, uint16_t id)
    : Robot(team, id, 1, 0) {
    max_hp_ = 300;
    max_heat_ = 0;
    reset();
}

void Engineer::reset() {
    hp_ = max_hp_;
    heat_ = 0;
}

void Engineer::update(uint16_t current_time) {
    last_update_time_ = current_time;
}

void Engineer::take_damage(uint16_t damage) {
    if (damage >= hp_) {
        hp_ = 0;
    } else {
        hp_ -= damage;
    }
}

void Engineer::add_heat(uint16_t heat) {
}

bool Engineer::can_upgrade(uint16_t target_level) const {
    return false;
}

void Engineer::upgrade(uint16_t target_level) {
}
