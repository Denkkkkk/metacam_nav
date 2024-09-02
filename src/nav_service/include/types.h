#ifndef __NAV_TYPE_H__
#define __NAV_TYPE_H__

#include <json.hpp>
// #include <nlohmann/json.hpp>
#include <vector>

struct Point
{
    float x;
    float y;
    float z;
};

// 导航数据params
struct NavigationModel
{
    int mode;
    std::vector<float> parameters;
    std::vector<Point> points;
};

// define from_json & to_json to be auto used in nlohmann::json::parse
void from_json(const nlohmann::json &j, Point &p)
{
    j.at("x").get_to(p.x);
    j.at("y").get_to(p.y);
    j.at("z").get_to(p.z);
}

/**
 * @brief 非基本格式类型的Point结构体需要定义from_json和to_json
 *
 * @param j
 * @param p
 */
void to_json(nlohmann::json &j, const Point &p)
{
    j = nlohmann::json{{"x", p.x}, {"y", p.y}, {"z", p.z}};
}

void from_json(const nlohmann::json &j, NavigationModel &nm)
{
    // 基本格式int、float、string等不需要定义from_json和to_json
    j.at("mode").get_to(nm.mode);
    j.at("parameters").get_to(nm.parameters);
    j.at("points").get_to(nm.points);
}
void to_json(nlohmann::json &j, const NavigationModel &nm)
{
    j = nlohmann::json{{"mode", nm.mode}, {"parameters", nm.parameters}, {"points", nm.points}};
}

// 导航状态对外通知
struct NavStatus
{
    std::string version;
    bool is_running;
    int target_index;
    std::vector<float> target_pose;
};
// 序列化 NavStatus 到 JSON 字符串
void NavStatus_to_json(const NavStatus &status, std::string &json_str)
{
    nlohmann::json j;
    j["version"] = status.version;
    j["is_running"] = status.is_running;
    j["target_index"] = status.target_index;
    j["target_pose"] = status.target_pose;
    json_str = j.dump(); // 转换为 JSON 字符串
}

void json_to_NavStatus(const std::string &json_str, NavStatus &nav_status)
{
    // 解析json字符串为json
    nlohmann::json j = nlohmann::json::parse(json_str);

    nav_status.version = j["version"].get<std::string>();
    nav_status.is_running = j["is_running"].get<bool>();
    nav_status.target_index = j["target_index"].get<int>();
    nav_status.target_pose = j["target_pose"].get<std::vector<float>>();
}

#endif
