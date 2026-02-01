#include<iostream>
#include"robot_manager.h"

std::vector<std::shared_ptr<Robot> >::iterator
RobotManager::find_destroyed_robot(uint16_t target_team, uint16_t target_id) {
    auto it = std::find_if(destroyed_robots.begin(), destroyed_robots.end(),
                           [target_id,target_team](const std::shared_ptr<Robot> &bot) {
                               return bot && target_id == bot->get_id() && target_team == bot->get_team();
                           });
    return it;
}

std::vector<std::shared_ptr<Robot> >::iterator
RobotManager::find_alive_robot(uint16_t target_team, uint16_t target_id) {
    auto it = std::find_if(alive_robots.begin(), alive_robots.end(),
                           [target_id,target_team](const std::shared_ptr<Robot> &bot) {
                               return bot && target_id == bot->get_id() && target_team == bot->get_team();
                           });
    return it;
}

void RobotManager::output_destroyed_list() {
    for (auto &robot: destroyed_robots) {
        if (robot) {
            std::cout << "D " << robot->get_team() << " " << robot->get_id() << std::endl;
        }
    }
}

//A
void RobotManager::add_robot(uint16_t team, uint16_t id, uint16_t type) {
    auto it = find_destroyed_robot(team, id);

    if (it != destroyed_robots.end()) {
        auto robot_ptr = *it;
        alive_robots.push_back(*it);
        destroyed_robots.erase(it);
        robot_ptr->reset();
    } else {
        std::shared_ptr<Robot> new_robot;
        if (type == 0) {
            new_robot = std::make_shared<Infantry>(team, id, 1);
        } else if (type == 1) {
            new_robot = std::make_shared<Engineer>(team, id);
        }

        if (new_robot) {
            alive_robots.push_back(new_robot);
        }
    }
}

//F
void RobotManager::deal_damage(uint16_t team, uint16_t id, int16_t damage) {
    auto it = find_alive_robot(team, id);
    if (it != alive_robots.end() && *it) {
        (*it)->take_damage(damage);
        if ((*it)->is_destroyed()) {
            destroyed_robots.push_back(*it);
            alive_robots.erase(it);
        }
    }
}

//H
void RobotManager::add_heat(int16_t team, uint16_t id, uint16_t heat) {
    auto it = find_alive_robot(team, id);
    if (it != alive_robots.end() && (*it)->get_type() == 0) {
        (*it)->add_heat(heat);
    }
}

//U
void RobotManager::upgrade_robot(int16_t team, uint16_t id, uint16_t level) {
    auto it = find_alive_robot(team, id);
    if (it != alive_robots.end()) {
        (*it)->upgrade(level);
    }
}

void RobotManager::process_input_commands() {
    int n;
    std::cin >> n;
    for (int i = 0; i < n; i++) {
        uint16_t time, param1, param2, param3;
        char cmd;
        std::cin >> time >> cmd >> param1 >> param2 >> param3;
        process_single_command(time, cmd, param1, param2, param3);
    }
}

void RobotManager::process_single_command(uint16_t time, char cmd, uint16_t param1, uint16_t param2, uint16_t param3) {
    update_all_robots(time);
    switch (cmd) {
        case 'A':
            add_robot(param1, param2, param3);
            break;
        case 'F':
            deal_damage(param1, param2, param3);
            break;
        case 'H':
            add_heat(param1, param2, param3);
            break;
        case 'U':
            upgrade_robot(param1, param2, param3);
            break;
        default:
            break;
    }
}

void RobotManager::update_all_robots(uint16_t time) {
    std::vector<std::shared_ptr<Robot> > newly_destroyed;

    for (auto it = alive_robots.begin(); it != alive_robots.end();) {
        if (*it) {
            (*it)->update(time);
            if ((*it)->is_destroyed()) {
                newly_destroyed.push_back(*it);
                it = alive_robots.erase(it);
                continue;
            }
        }
        ++it;
    }
    destroyed_robots.insert(destroyed_robots.end(),
                            newly_destroyed.begin(),
                            newly_destroyed.end());
}
