#include "IkRigConfig.h"

#include <stdexcept>
#include <unordered_set>

void to_json(nlohmann::json& j, const IkRigConfig& config)
{
    j = nlohmann::json{{"targets", config.targetBones}, {"limits", nlohmann::json::array()}};
    for (const JointLimits& limit : config.limits)
    {
        nlohmann::json entry;
        entry["bone"] = limit.bone;
        entry["twistMin"] = limit.twistMinDeg;
        entry["twistMax"] = limit.twistMaxDeg;
        entry["swingCone"] = limit.swingConeDeg;
        j["limits"].push_back(std::move(entry));
    }
}

void from_json(const nlohmann::json& j, IkRigConfig& config)
{
    config = IkRigConfig{};

    if (j.contains("targets"))
    {
        const nlohmann::json& targets = j.at("targets");
        if (!targets.is_array())
            throw std::runtime_error("ikrig: 'targets' must be an array");
        for (const nlohmann::json& entry : targets)
            config.targetBones.push_back(entry.get<std::string>());

        std::unordered_set<std::string> names;
        for (const std::string& name : config.targetBones)
            if (!names.insert(name).second)
                throw std::runtime_error("ikrig: duplicate target bone '" + name + "'");
    }

    if (j.contains("limits"))
    {
        const nlohmann::json& limits = j.at("limits");
        if (!limits.is_array())
            throw std::runtime_error("ikrig: 'limits' must be an array");
        for (const nlohmann::json& entry : limits)
        {
            JointLimits limit;
            limit.bone = entry.at("bone").get<std::string>();
            limit.twistMinDeg = entry.value("twistMin", limit.twistMinDeg);
            limit.twistMaxDeg = entry.value("twistMax", limit.twistMaxDeg);
            limit.swingConeDeg = entry.value("swingCone", limit.swingConeDeg);
            if (limit.twistMinDeg > limit.twistMaxDeg)
                throw std::runtime_error("ikrig: limit for '" + limit.bone + "': twistMin > twistMax");
            if (limit.swingConeDeg < 0.0f || limit.swingConeDeg > 180.0f)
                throw std::runtime_error("ikrig: limit for '" + limit.bone + "': swingCone out of range [0, 180]");
            config.limits.push_back(std::move(limit));
        }

        std::unordered_set<std::string> names;
        for (const JointLimits& limit : config.limits)
            if (!names.insert(limit.bone).second)
                throw std::runtime_error("ikrig: duplicate limits for bone '" + limit.bone + "'");
    }
}

IkRigConfig IkRigConfig::makeDefault()
{
    IkRigConfig config;
    config.targetBones = {"left_hand", "right_hand", "left_foot", "right_foot", "hip"};

    auto add = [&config](std::string bone, float twistMin, float twistMax, float swingCone)
    {
        JointLimits limit;
        limit.bone = std::move(bone);
        limit.twistMinDeg = twistMin;
        limit.twistMaxDeg = twistMax;
        limit.swingConeDeg = swingCone;
        config.limits.push_back(std::move(limit));
    };

    // Knees and elbows: hinges with near-zero twist.
    add("left_lower_leg", -5.0f, 5.0f, 150.0f);
    add("right_lower_leg", -5.0f, 5.0f, 150.0f);
    add("left_lower_arm", -5.0f, 5.0f, 150.0f);
    add("right_lower_arm", -5.0f, 5.0f, 150.0f);
    // Hips and shoulders: wide cones, moderate twist.
    add("left_upper_leg", -30.0f, 45.0f, 120.0f);
    add("right_upper_leg", -45.0f, 30.0f, 120.0f);
    add("left_upper_arm", -90.0f, 90.0f, 170.0f);
    add("right_upper_arm", -90.0f, 90.0f, 170.0f);

    return config;
}
