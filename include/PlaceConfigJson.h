#ifndef PLACE_CONFIG_JSON_H
#define PLACE_CONFIG_JSON_H

#include "PlaceConfig.h"

#include <nlohmann/json.hpp>

#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>

// Joint 是具名类型，可用宏自动生成 to_json / from_json。
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Joint, j1, j2, j3, j4, j5, j6)

// ArrayConfig 是匿名嵌套结构体，没有独立类型名，不能用上面的宏。
// 必须在 PlaceConfig 这一层手动展开字段。
inline void to_json(nlohmann::json& j, const PlaceConfig& cfg)
{
    j = nlohmann::json{
        {"ArrayConfig", {
            {"layerX", cfg.ArrayConfig.layerX},
            {"layerY", cfg.ArrayConfig.layerY},
            {"obj_length", cfg.ArrayConfig.obj_length},
            {"obj_width", cfg.ArrayConfig.obj_width},
            {"obj_height", cfg.ArrayConfig.obj_height}
        }},
        {"PassJoint", cfg.PassJoint},
        {"PlaceJoint", cfg.PlaceJoint},
        {"WaitJoint", cfg.WaitJoint}
    };
}

inline void from_json(const nlohmann::json& j, PlaceConfig& cfg)
{
    const nlohmann::json& arrayCfg = j.at("ArrayConfig");
    arrayCfg.at("layerX").get_to(cfg.ArrayConfig.layerX);
    arrayCfg.at("layerY").get_to(cfg.ArrayConfig.layerY);
    arrayCfg.at("obj_length").get_to(cfg.ArrayConfig.obj_length);
    arrayCfg.at("obj_width").get_to(cfg.ArrayConfig.obj_width);
    arrayCfg.at("obj_height").get_to(cfg.ArrayConfig.obj_height);

    j.at("PassJoint").get_to(cfg.PassJoint);
    j.at("PlaceJoint").get_to(cfg.PlaceJoint);
    j.at("WaitJoint").get_to(cfg.WaitJoint);
}

inline void SavePlaceConfig(const std::string& filePath, const PlaceConfig& cfg)
{
    std::ofstream ofs(filePath);
    if (!ofs) {
        throw std::runtime_error("Failed to open file for write: " + filePath);
    }

    const nlohmann::json j = cfg;
    ofs << std::setw(4) << j << '\n';
    if (!ofs) {
        throw std::runtime_error("Failed to write JSON: " + filePath);
    }
}

inline PlaceConfig LoadPlaceConfig(const std::string& filePath)
{
    std::ifstream ifs(filePath);
    if (!ifs) {
        throw std::runtime_error("Failed to open file for read: " + filePath);
    }

    nlohmann::json j;
    ifs >> j;
    return j.get<PlaceConfig>();
}

#endif // PLACE_CONFIG_JSON_H
