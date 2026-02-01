#ifndef ROBOT_MANAGER_H
#define ROBOT_MANAGER_H

#include "robot.h"
#include <vector>
#include <algorithm>
#include <memory>

class RobotManager {
protected:
    std::vector<std::shared_ptr<Robot> > alive_robots;
    std::vector<std::shared_ptr<Robot> > destroyed_robots;

public:
    std::vector<std::shared_ptr<Robot> >::iterator find_destroyed_robot(uint16_t target_team, uint16_t target_id);

    std::vector<std::shared_ptr<Robot> >::iterator find_alive_robot(uint16_t target_team, uint16_t target_id);

    void process_input_commands();

    void add_robot(uint16_t team, uint16_t id, uint16_t type);

    void deal_damage(uint16_t team, uint16_t id, int16_t damage);

    void add_heat(int16_t team, uint16_t id, uint16_t heat);

    void upgrade_robot(int16_t team, uint16_t id, uint16_t level);

    void process_single_command(uint16_t time, char cmd, uint16_t param1, uint16_t param2, uint16_t param3);

    void update_all_robots(uint16_t time);

    void output_destroyed_list();
};
#endif