#include "Skeleton.h"

#include "BodyProportions.h"
#include "BoneNames.h"
#include "Error.h"
#include "GlmJson.h"

#include <algorithm>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace
{
struct RawBone
{
	std::string name;
	std::optional<std::string> parent;
	glm::vec3 offset{0.0f, 0.0f, 0.0f};
};

void from_json(const nlohmann::json& j, RawBone& bone)
{
	j.at("name").get_to(bone.name);
	if (const auto it = j.find("parent"); it != j.end() && !it->is_null())
		bone.parent = it->get<std::string>();
	bone.offset = j.value("offset", glm::vec3{0.0f});
}
} // namespace

void to_json(nlohmann::json& j, const Skeleton& skeleton)
{
	j = nlohmann::json{{"bones", nlohmann::json::array()}};
	for (const Joint& joint : skeleton.joints)
	{
		nlohmann::json bone;
		bone["name"] = joint.name;
		if (joint.parentIndex)
			bone["parent"] = skeleton.joints[*joint.parentIndex].name;
		else
			bone["parent"] = nullptr;
		bone["offset"] = joint.restOffset;
		j["bones"].push_back(std::move(bone));
	}
}

void from_json(const nlohmann::json& j, Skeleton& skeleton)
{
	const std::vector<RawBone> raw = j.at("bones").get<std::vector<RawBone>>();

	std::unordered_set<std::string> names;
	for (const RawBone& bone : raw)
		names.insert(bone.name);
	for (const RawBone& bone : raw)
		if (bone.parent && !names.contains(*bone.parent))
			throw Error("skeleton: bone '" + bone.name + "' has unknown parent '" + *bone.parent + "'");

	// Sort parent-before-child, resolving parent names to indices.
	std::vector<Joint> joints;
	joints.reserve(raw.size());
	std::unordered_map<std::string, int> indexOf;
	std::vector<bool> placed(raw.size(), false);
	size_t remaining = raw.size();
	while (remaining > 0)
	{
		bool progress = false;
		for (size_t i = 0; i < raw.size(); ++i)
		{
			if (placed[i])
				continue;

			std::optional<int> parentIndex = std::nullopt;
			if (raw[i].parent)
			{
				const auto it = indexOf.find(*raw[i].parent);
				if (it == indexOf.end())
					continue; // parent not placed yet
				parentIndex = it->second;
			}

			Joint joint;
			joint.name = raw[i].name;
			joint.parentIndex = parentIndex;
			joint.restOffset = raw[i].offset;
			if (!indexOf.emplace(joint.name, static_cast<int>(joints.size())).second)
				throw Error("skeleton: duplicate bone name '" + joint.name + "'");
			joints.push_back(std::move(joint));
			placed[i] = true;
			--remaining;
			progress = true;
		}
		if (!progress)
			throw Error("skeleton: bones contain a cycle");
	}

	Skeleton result;
	result.joints = std::move(joints);
	int rootCount = 0;
	for (const Joint& joint : result.joints)
		if (!joint.parentIndex)
		{
			++rootCount;
			result.rootPosition = joint.restOffset;
		}
	if (rootCount != 1)
		throw Error("skeleton: expected exactly 1 root bone, got " + std::to_string(rootCount));

	skeleton = std::move(result);
}

Skeleton Skeleton::makeDefault(const BodyProportions& proportions)
{
	constexpr float kHeadHeight = 0.15f;
	constexpr float kHandLength = 0.12f;
	constexpr float kFootLength = 0.08f;
	const float hipsY = proportions.upperLegLength + proportions.lowerLegLength;
	const float halfChestToWaist = (proportions.shoulderHeight - proportions.navelHeight) * 0.5f;
	const float waistToHips = proportions.navelHeight - hipsY;
	const float rootY = proportions.shoulderHeight + proportions.neckLength;
	const float halfShoulder = proportions.shoulderWidth * 0.5f;
	const float halfHip = proportions.hipWidth * 0.5f;

	Skeleton skeleton;
	auto add = [&skeleton](std::string name, const std::string& parent, glm::vec3 offset)
	{
		Joint joint;
		joint.name = std::move(name);
		joint.parentIndex = std::nullopt;
		if (!parent.empty())
		{
			for (size_t i = 0; i < skeleton.joints.size(); ++i)
				if (skeleton.joints[i].name == parent)
					joint.parentIndex = static_cast<int>(i);
		}
		joint.restOffset = offset;
		skeleton.joints.push_back(std::move(joint));
	};

	// Head-rooted spine (SlimeVR-style), left side at +X.
	add(BoneNames::Head, "", {0.0f, rootY, 0.0f});
	add(BoneNames::Neck, BoneNames::Head, {0.0f, -kHeadHeight, 0.0f});
	add(BoneNames::Chest, BoneNames::Neck, {0.0f, -proportions.neckLength, 0.0f});
	add(BoneNames::Spine, BoneNames::Chest, {0.0f, -halfChestToWaist, 0.0f});
	add(BoneNames::Waist, BoneNames::Spine, {0.0f, -halfChestToWaist, 0.0f});
	add(BoneNames::Hips, BoneNames::Waist, {0.0f, -waistToHips, 0.0f});

	add(BoneNames::LeftHip, BoneNames::Hips, {halfHip, 0.0f, 0.0f});
	add(BoneNames::LeftUpperLeg, BoneNames::LeftHip, {0.0f, -proportions.upperLegLength, 0.0f});
	add(BoneNames::LeftLowerLeg, BoneNames::LeftUpperLeg, {0.0f, -proportions.lowerLegLength, 0.0f});
	add(BoneNames::LeftFoot, BoneNames::LeftLowerLeg, {0.0f, 0.0f, -kFootLength});

	add(BoneNames::RightHip, BoneNames::Hips, {-halfHip, 0.0f, 0.0f});
	add(BoneNames::RightUpperLeg, BoneNames::RightHip, {0.0f, -proportions.upperLegLength, 0.0f});
	add(BoneNames::RightLowerLeg, BoneNames::RightUpperLeg, {0.0f, -proportions.lowerLegLength, 0.0f});
	add(BoneNames::RightFoot, BoneNames::RightLowerLeg, {0.0f, 0.0f, -kFootLength});

	add(BoneNames::LeftShoulder, BoneNames::Chest, {halfShoulder, 0.0f, 0.0f});
	add(BoneNames::LeftUpperArm, BoneNames::LeftShoulder, {proportions.upperArmLength, 0.0f, 0.0f});
	add(BoneNames::LeftLowerArm, BoneNames::LeftUpperArm, {proportions.lowerArmLength, 0.0f, 0.0f});
	add(BoneNames::LeftHand, BoneNames::LeftLowerArm, {kHandLength, 0.0f, 0.0f});

	add(BoneNames::RightShoulder, BoneNames::Chest, {-halfShoulder, 0.0f, 0.0f});
	add(BoneNames::RightUpperArm, BoneNames::RightShoulder, {-proportions.upperArmLength, 0.0f, 0.0f});
	add(BoneNames::RightLowerArm, BoneNames::RightUpperArm, {-proportions.lowerArmLength, 0.0f, 0.0f});
	add(BoneNames::RightHand, BoneNames::RightLowerArm, {-kHandLength, 0.0f, 0.0f});

	for (const Joint& joint : skeleton.joints)
		if (!joint.parentIndex)
			skeleton.rootPosition = joint.restOffset;

	return skeleton;
}

namespace
{
// Re-roots a skeleton at the named joint: the ancestor chain from the joint up
// to the old root is reversed (each offset along it negated — valid because
// rest rotations are identity), all other joints keep their parent and offset.
// Rest world positions are preserved; the new root's restOffset becomes its
// rest world position. Throws Error if the name is unknown.
Skeleton reroot(const Skeleton& skeleton, const std::string& newRootName)
{
	const std::vector<glm::vec3> restPositions = computeWorldPositions(skeleton);

	int newRoot = -1;
	for (size_t i = 0; i < skeleton.joints.size(); ++i)
		if (skeleton.joints[i].name == newRootName)
			newRoot = static_cast<int>(i);
	if (newRoot < 0)
		throw Error("reroot: no joint named '" + newRootName + "'");

	std::vector<Joint> joints = skeleton.joints;

	// Path from the new root up to the old root.
	std::vector<int> path;
	for (std::optional<int> cur = newRoot; cur; cur = joints[*cur].parentIndex)
		path.push_back(*cur);

	// Reverse the chain: each joint on the path gets its former child as
	// parent, with the former child's offset negated.
	const std::vector<Joint> original = joints;
	for (size_t i = 1; i < path.size(); ++i)
	{
		joints[path[i]].parentIndex = path[i - 1];
		joints[path[i]].restOffset = -original[path[i - 1]].restOffset;
	}
	joints[path[0]].parentIndex = std::nullopt;
	joints[path[0]].restOffset = restPositions[path[0]];

	// Re-sort parent-before-child (edges along the reversed chain now point
	// the other way, so indices must be rebuilt).
	std::vector<std::vector<int>> children(joints.size());
	for (size_t i = 0; i < joints.size(); ++i)
		if (joints[i].parentIndex)
			children[static_cast<size_t>(*joints[i].parentIndex)].push_back(static_cast<int>(i));

	Skeleton result;
	std::vector<int> oldToNew(joints.size(), -1);
	std::vector<int> stack{path[0]};
	while (!stack.empty())
	{
		const int old = stack.back();
		stack.pop_back();
		Joint joint = joints[old];
		if (joint.parentIndex)
			joint.parentIndex = oldToNew[static_cast<size_t>(*joint.parentIndex)];
		oldToNew[static_cast<size_t>(old)] = static_cast<int>(result.joints.size());
		result.joints.push_back(std::move(joint));
		for (const int child : children[static_cast<size_t>(old)])
			stack.push_back(child);
	}
	result.rootPosition = joints[path[0]].restOffset;
	return result;
}
} // namespace

Skeleton Skeleton::makeDefaultHipRooted()
{
	return reroot(makeDefault(BodyProportions()), BoneNames::Hips);
}

WorldTransforms computeWorldTransforms(const Skeleton& skeleton)
{
	WorldTransforms result;
	const size_t count = skeleton.joints.size();
	result.positions.resize(count);
	result.rotations.resize(count);
	for (size_t i = 0; i < count; ++i)
	{
		const Joint& joint = skeleton.joints[i];
		if (joint.parentIndex)
		{
			const size_t parent = static_cast<size_t>(*joint.parentIndex);
			const glm::quat worldRot = result.rotations[parent] * joint.localRot;
			result.rotations[i] = worldRot;
			result.positions[i] = result.positions[parent] + worldRot * joint.restOffset;
		}
		else
		{
			result.rotations[i] = joint.localRot;
			result.positions[i] = skeleton.rootPosition;
		}
	}
	return result;
}

std::vector<glm::vec3> computeWorldPositions(const Skeleton& skeleton)
{
	return computeWorldTransforms(skeleton).positions;
}

std::vector<glm::vec3> computeRestPositions(const Skeleton& skeleton)
{
	std::vector<glm::vec3> positions(skeleton.joints.size());
	for (size_t i = 0; i < skeleton.joints.size(); ++i)
	{
		const Joint& joint = skeleton.joints[i];
		if (joint.parentIndex)
			positions[i] = positions[static_cast<size_t>(*joint.parentIndex)] + joint.restOffset;
		else
			positions[i] = glm::vec3{0.0f, 0.0f, 0.0f};
	}
	return positions;
}

float restHeight(const Skeleton& skeleton)
{
	const std::vector<glm::vec3> rest = computeRestPositions(skeleton);
	int head = -1;
	int leftFoot = -1;
	int rightFoot = -1;
	for (size_t i = 0; i < skeleton.joints.size(); ++i)
	{
		const std::string& name = skeleton.joints[i].name;
		if (name == BoneNames::Head)
			head = static_cast<int>(i);
		else if (name == BoneNames::LeftFoot)
			leftFoot = static_cast<int>(i);
		else if (name == BoneNames::RightFoot)
			rightFoot = static_cast<int>(i);
	}
	if (head < 0 || (leftFoot < 0 && rightFoot < 0))
		return 0.0f;
	const float headY = rest[static_cast<size_t>(head)].y;
	float footY = std::numeric_limits<float>::max();
	if (leftFoot >= 0)
		footY = std::min(footY, rest[static_cast<size_t>(leftFoot)].y);
	if (rightFoot >= 0)
		footY = std::min(footY, rest[static_cast<size_t>(rightFoot)].y);
	const float h = headY - footY;
	return h > 0.0f ? h : 0.0f;
}

void scaleSkeleton(Skeleton& skeleton, float scale)
{
	for (Joint& joint : skeleton.joints)
		joint.restOffset *= scale;
	skeleton.rootPosition *= scale;
}

float matchRestHeight(const Skeleton& src, Skeleton& dst)
{
	const float srcHeight = restHeight(src);
	const float dstHeight = restHeight(dst);
	if (srcHeight <= 0.0f || dstHeight <= 0.0f)
		return 1.0f;
	const float scale = srcHeight / dstHeight;
	scaleSkeleton(dst, scale);
	return scale;
}
