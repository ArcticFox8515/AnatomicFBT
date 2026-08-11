#include "IkRigConfig.h"

#include "BoneNames.h"
#include "Error.h"
#include "GlmJson.h"

#include <unordered_set>

NLOHMANN_JSON_SERIALIZE_ENUM(SolverType,
                             {
                             {SolverType::Anchor, "anchor"},
                             {SolverType::Chain, "chain"},
                             {SolverType::TwoBone, "two_bone"},
                             })

NLOHMANN_JSON_SERIALIZE_ENUM(PoleMode,
                             {
                             {PoleMode::Static, "static"},
                             {PoleMode::DynamicFoot, "dynamic_foot"},
                             {PoleMode::DynamicHand, "dynamic_hand"},
                             })

void to_json(nlohmann::json& j, const TargetConfig& target)
{
	j = nlohmann::json{{"bone", target.bone}, {"solver", target.solver}};
}

void from_json(const nlohmann::json& j, TargetConfig& target)
{
	j.at("bone").get_to(target.bone);
	j.at("solver").get_to(target.solver);
}

void to_json(nlohmann::json& j, const JointLimits& limit)
{
	j = nlohmann::json{{"bone", limit.bone},
	                   {"twistMin", limit.twistMinDeg},
	                   {"twistMax", limit.twistMaxDeg},
	                   {"swingCone", limit.swingConeDeg},
	                   {"poleMode", limit.poleMode}};
	if (limit.pole)
		j["pole"] = *limit.pole;
}

void from_json(const nlohmann::json& j, JointLimits& limit)
{
	j.at("bone").get_to(limit.bone);
	limit.twistMinDeg = j.value("twistMin", limit.twistMinDeg);
	limit.twistMaxDeg = j.value("twistMax", limit.twistMaxDeg);
	limit.swingConeDeg = j.value("swingCone", limit.swingConeDeg);
	limit.poleMode = j.value("poleMode", PoleMode::Static);
	if (const auto it = j.find("pole"); it != j.end() && !it->is_null())
		limit.pole = it->get<glm::vec3>();
}

void to_json(nlohmann::json& j, const IkRigConfig& config)
{
	j = nlohmann::json{{"targets", config.targets}, {"limits", config.limits}};
}

void from_json(const nlohmann::json& j, IkRigConfig& config)
{
	config.targets = j.value("targets", std::vector<TargetConfig>{});
	config.limits = j.value("limits", std::vector<JointLimits>{});
	config.validate();
}

void IkRigConfig::validate() const
{
	std::unordered_set<std::string> targetNames;
	for (const TargetConfig& target : targets)
		if (!targetNames.insert(target.bone).second)
			throw Error("ikrig: duplicate target bone '" + target.bone + "'");

	std::unordered_set<std::string> limitNames;
	for (const JointLimits& limit : limits)
	{
		if (!limitNames.insert(limit.bone).second)
			throw Error("ikrig: duplicate limits for bone '" + limit.bone + "'");
		if (limit.pole && glm::length(*limit.pole) < 1e-6f)
			throw Error("ikrig: limit for '" + limit.bone + "': pole must be non-zero");
		if (limit.twistMinDeg > limit.twistMaxDeg)
			throw Error("ikrig: limit for '" + limit.bone + "': twistMin > twistMax");
		if (limit.swingConeDeg < 0.0f || limit.swingConeDeg > 180.0f)
			throw Error("ikrig: limit for '" + limit.bone + "': swingCone out of range [0, 180]");
	}
}

IkRigConfig IkRigConfig::makeDefault()
{
	IkRigConfig config;
	config.targets = {
		{BoneNames::Head, SolverType::Anchor},
		{BoneNames::LeftHand, SolverType::TwoBone},
		{BoneNames::RightHand, SolverType::TwoBone},
		{BoneNames::LeftFoot, SolverType::TwoBone},
		{BoneNames::RightFoot, SolverType::TwoBone},
		{BoneNames::Hips, SolverType::Chain},
	};

	auto add = [&config](std::string bone, float twistMin, float twistMax, float swingCone,
	                     std::optional<glm::vec3> pole = std::nullopt,
	                     PoleMode poleMode = PoleMode::Static)
	{
		JointLimits limit;
		limit.bone = std::move(bone);
		limit.twistMinDeg = twistMin;
		limit.twistMaxDeg = twistMax;
		limit.swingConeDeg = swingCone;
		limit.pole = pole;
		limit.poleMode = poleMode;
		config.limits.push_back(std::move(limit));
	};

	// Poles in the limb socket's frame. The skeleton faces -Z at rest:
	// knees bend forward, elbows point down/back.
	const glm::vec3 kneePole{0.0f, 0.0f, -1.0f};
	const glm::vec3 elbowPole = glm::normalize(glm::vec3{0.0f, -1.0f, 1.0f});

	// Knees and elbows: hinges with near-zero twist. Dynamic bend normals
	// (WP2): the middle-bone pole is derived per-frame from the foot/hand
	// lateral axis via a cross product, perpendicular to the chain aim by
	// construction — no pole||aim degeneracy. The static pole is retained
	// as the singularity guard and the flex-sign reference.
	add(BoneNames::LeftLowerLeg, -5.0f, 5.0f, 150.0f, kneePole, PoleMode::DynamicFoot);
	add(BoneNames::RightLowerLeg, -5.0f, 5.0f, 150.0f, kneePole, PoleMode::DynamicFoot);
	add(BoneNames::LeftLowerArm, -5.0f, 5.0f, 150.0f, elbowPole, PoleMode::DynamicHand);
	add(BoneNames::RightLowerArm, -5.0f, 5.0f, 150.0f, elbowPole, PoleMode::DynamicHand);
	// Hips and shoulders: wide cones, moderate twist.
	add(BoneNames::LeftUpperLeg, -30.0f, 45.0f, 120.0f);
	add(BoneNames::RightUpperLeg, -45.0f, 30.0f, 120.0f);
	add(BoneNames::LeftUpperArm, -90.0f, 90.0f, 170.0f);
	add(BoneNames::RightUpperArm, -90.0f, 90.0f, 170.0f);

	return config;
}
