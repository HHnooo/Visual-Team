#ifndef ROBOT_H
#define ROBOT_H

#include <cstdint>
#include <string>
#include <sstream>
#include <memory>

class Robot {
protected:
    uint16_t max_hp_;
    uint16_t max_heat_;
    uint16_t hp_;
    uint16_t level_;
    uint16_t heat_;
    uint16_t team_;
    uint16_t id_;
    uint16_t type_;
    uint16_t last_update_time_;

public:
    Robot(uint16_t team, uint16_t id, int type, uint16_t level = 1);

    virtual ~Robot() = default;

    virtual void update(uint16_t current_time) = 0;

    virtual void take_damage(uint16_t damage) = 0;

    virtual void add_heat(uint16_t heat) = 0;

    virtual void upgrade(uint16_t target_level) = 0;

    virtual void reset() = 0;

    virtual bool can_upgrade(uint16_t target_level) const = 0;

    uint16_t get_team() const { return team_; }
    uint16_t get_id() const { return id_; }
    int get_type() const { return type_; }
    uint16_t get_level() const { return level_; }
    uint16_t get_hp() const { return hp_; }
    uint16_t get_max_hp() const { return max_hp_; }
    uint16_t get_heat() const { return heat_; }
    uint16_t get_max_heat() const { return max_heat_; }
    bool is_destroyed() const { return hp_ <= 0; }
    uint16_t get_last_update_time() const { return last_update_time_; }
    void set_last_update_time(uint16_t time) { last_update_time_ = time; }
};

class Infantry : public Robot {
public:
    Infantry(uint16_t team, uint16_t id, uint16_t level = 1);

    void update(uint16_t current_time) override;

    void take_damage(uint16_t damage) override;

    void add_heat(uint16_t heat) override;

    bool can_upgrade(uint16_t target_level) const override;

    void upgrade(uint16_t target_level) override;

    void reset() override;

private:
    void update_heat(uint16_t seconds);

    void update_Status_by_level();
};

class Engineer : public Robot {
public:
    Engineer(uint16_t team, uint16_t id);

    void update(uint16_t current_time) override;

    void take_damage(uint16_t damage) override;

    void add_heat(uint16_t heat) override;

    bool can_upgrade(uint16_t target_level) const override;

    void upgrade(uint16_t target_level) override;

    void reset() override;
};

#endif