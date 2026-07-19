#include "IkRigConfig.h"

#include <stdexcept>
#include <unordered_set>

NLOHMANN_JSON_SERIALIZE_ENUM(SolverType,
                             {
                             {SolverType::Anchor, "anchor"},
                             {SolverType::Chain, "chain"},
                             {SolverType::TwoBone, "two_bone"},
                             })

void to_json(nlohmann::json& j, const IkRigConfig& config)
{
	j = nlohmann::json{{"targets", nlohmann::json::array()}, {"limits", nlohmann::json::array()}};
	for (const TargetConfig& target : config.targets)
	{
		nlohmann::json entry;
		entry["bone"] = target.bone;
		entry["solver"] = target.solver;
		j["targets"].push_back(std::move(entry));
	}
	for (const JointLimits& limit : config.limits)
	{
		nlohmann::json entry;
		entry["bone"] = limit.bone;
		entry["twistMin"] = limit.twistMinDeg;
		entry["twistMax"] = limit.twistMaxDeg;
		entry["swingCone"] = limit.swingConeDeg;
		if (limit.pole)
			entry["pole"] = {limit.pole->x, limit.pole->y, limit.pole->z};
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
		{
			if (!entry.is_object())
				throw std::runtime_error("ikrig: each target must be an object {bone, solver}");
			TargetConfig target;
			target.bone = entry.at("bone").get<std::string>();
			target.solver = entry.at("solver").get<SolverType>();
			config.targets.push_back(std::move(target));
		}

		std::unordered_set<std::string> names;
		for (const TargetConfig& target : config.targets)
			if (!names.insert(target.bone).second)
				throw std::runtime_error("ikrig: duplicate target bone '" + target.bone + "'");
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
			if (entry.contains("pole"))
			{
				const nlohmann::json& arr = entry.at("pole");
				if (!arr.is_array() || arr.size() != 3)
					throw std::runtime_error("ikrig: limit for '" + limit.bone + "': pole must be an array of 3 numbers");
				limit.pole = glm::vec3(arr[0].get<float>(), arr[1].get<float>(), arr[2].get<float>());
				if (glm::length(*limit.pole) < 1e-6f)
					throw std::runtime_error("ikrig: limit for '" + limit.bone + "': pole must be non-zero");
			}
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
	config.targets = {
		{"head", SolverType::Anchor},
		{"left_hand", SolverType::TwoBone},
		{"right_hand", SolverType::TwoBone},
		{"left_foot", SolverType::TwoBone},
		{"right_foot", SolverType::TwoBone},
		{"hip", SolverType::Chain},
	};

	auto add = [&config](std::string bone, float twistMin, float twistMax, float swingCone,
	                     std::optional<glm::vec3> pole = std::nullopt)
	{
		JointLimits limit;
		limit.bone = std::move(bone);
		limit.twistMinDeg = twistMin;
		limit.twistMaxDeg = twistMax;
		limit.swingConeDeg = swingCone;
		limit.pole = pole;
		config.limits.push_back(std::move(limit));
	};

	// Poles in the limb socket's frame. The skeleton faces -Z at rest:
	// knees bend forward, elbows point down/back.
	const glm::vec3 kneePole{0.0f, 0.0f, -1.0f};
	const glm::vec3 elbowPole = glm::normalize(glm::vec3{0.0f, -1.0f, 1.0f});

	// Knees and elbows: hinges with near-zero twist, bending toward the pole.
	add("left_lower_leg", -5.0f, 5.0f, 150.0f, kneePole);
	add("right_lower_leg", -5.0f, 5.0f, 150.0f, kneePole);
	add("left_lower_arm", -5.0f, 5.0f, 150.0f, elbowPole);
	add("right_lower_arm", -5.0f, 5.0f, 150.0f, elbowPole);
	// Hips and shoulders: wide cones, moderate twist.
	add("left_upper_leg", -30.0f, 45.0f, 120.0f);
	add("right_upper_leg", -45.0f, 30.0f, 120.0f);
	add("left_upper_arm", -90.0f, 90.0f, 170.0f);
	add("right_upper_arm", -90.0f, 90.0f, 170.0f);

	return config;
}
