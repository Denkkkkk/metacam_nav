#ifndef __NAV_TYPE_H__
#define __NAV_TYPE_H__

#include <json.hpp>
#include <vector>

struct Point
{
    int x;
    int y;
    int z;
};

struct NavigationModel
{
    int mode;
    std::vector<int> parameters;
    std::vector<Point> points;
};

// define from_json & to_json to be auto used in nlohmann::json::parse
void from_json(const nlohmann::json &j, Point &p)
{
    j.at("x").get_to(p.x);
    j.at("y").get_to(p.y);
    j.at("z").get_to(p.z);
}

void to_json(nlohmann::json &j, const Point &p)
{
    j = nlohmann::json{{"x", p.x}, {"y", p.y}, {"z", p.z}};
}

void from_json(const nlohmann::json &j, NavigationModel &nm)
{
    j.at("mode").get_to(nm.mode);
    j.at("parameters").get_to(nm.parameters);
    j.at("points").get_to(nm.points);
}

void to_json(nlohmann::json &j, const NavigationModel &nm)
{
    j = nlohmann::json{{"mode", nm.mode}, {"parameters", nm.parameters}, {"points", nm.points}};
}

#endif
