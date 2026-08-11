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

void to_json(nlohmann::json& j, const ClavicleConfig& clavicle)
{
	j = nlohmann::json{{"elevationWeight", clavicle.elevationWeight},
	                   {"reachWeight", clavicle.reachWeight},
	                   {"reachThreshold", clavicle.reachThreshold},
	                   {"maxAngle", clavicle.maxAngleDeg}};
}

void from_json(const nlohmann::json& j, ClavicleConfig& clavicle)
{
	clavicle.elevationWeight = j.value("elevationWeight", clavicle.elevationWeight);
	clavicle.reachWeight = j.value("reachWeight", clavicle.reachWeight);
	clavicle.reachThreshold = j.value("reachThreshold", clavicle.reachThreshold);
	clavicle.maxAngleDeg = j.value("maxAngle", clavicle.maxAngleDeg);
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
	if (limit.clavicle)
		j["clavicle"] = *limit.clavicle;
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
	if (const auto it = j.find("clavicle"); it != j.end() && !it->is_null())
		limit.clavicle = it->get<ClavicleConfig>();
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
		if (limit.clavicle)
		{
			const ClavicleConfig& clavicle = *limit.clavicle;
			if (clavicle.elevationWeight < 0.0f || clavicle.elevationWeight > 1.0f)
				throw Error("ikrig: limit for '" + limit.bone
					+ "': clavicle elevationWeight out of range [0, 1]");
			if (clavicle.reachWeight < 0.0f || clavicle.reachWeight > 1.0f)
				throw Error("ikrig: limit for '" + limit.bone
					+ "': clavicle reachWeight out of range [0, 1]");
			if (clavicle.reachThreshold < 0.0f || clavicle.reachThreshold > 1.0f)
				throw Error("ikrig: limit for '" + limit.bone
					+ "': clavicle reachThreshold out of range [0, 1]");
			if (clavicle.maxAngleDeg < 0.0f || clavicle.maxAngleDeg > 180.0f)
				throw Error("ikrig: limit for '" + limit.bone
					+ "': clavicle maxAngle out of range [0, 180]");
		}
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
	                     PoleMode poleMode = PoleMode::Static,
	                     std::optional<ClavicleConfig> clavicle = std::nullopt)
	{
		JointLimits limit;
		limit.bone = std::move(bone);
		limit.twistMinDeg = twistMin;
		limit.twistMaxDeg = twistMax;
		limit.swingConeDeg = swingCone;
		limit.pole = pole;
		limit.poleMode = poleMode;
		limit.clavicle = clavicle;
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

	// Clavicles (WP3): the arm sockets follow the hand goal — a fraction of the
	// upward elevation, plus protraction/retraction once the goal is past 80% of
	// the arm's reach. The entries carry no twist/cone limits on purpose: the
	// clavicle stage runs *before* the limb solve and clamps its own output
	// (maxAngle), so a stage-4 clamp here would displace the arm socket after
	// the arm was solved from it and push the hand off its goal.
	const ClavicleConfig clavicle;  // field defaults are the tuned values
	add(BoneNames::LeftShoulder, -180.0f, 180.0f, 180.0f, std::nullopt, PoleMode::Static, clavicle);
	add(BoneNames::RightShoulder, -180.0f, 180.0f, 180.0f, std::nullopt, PoleMode::Static, clavicle);

	return config;
}
