#include "IkRig.h"

#include "IkMath.h"

#include <algorithm>
#include <array>
#include <glm/gtc/constants.hpp>
#include <stdexcept>

namespace
{
// Spine interpolation (SlimeVR-style): swings/curls the neck..hip chain so the
// hip joint lands on the hip target, blending bone orientations from the head
// rotation toward the hip target rotation. Exact in position when the target is
// within spine length and hip rotation equals head rotation; otherwise the hip
// bone orientation wins over exact position (documented interpolation trade-off).
void solveSpine(Skeleton& skeleton, const WorldTransforms& wt, int rootIndex,
                const std::array<int, 5>& chain, const IkTarget& hipTarget)
{
	const glm::quat headRot = wt.rotations[rootIndex];
	const glm::vec3 headPos = wt.positions[rootIndex];

	float len[5];
	float totalLen = 0.0f;
	glm::vec3 netOffset{0.0f};
	for (int i = 0; i < 5; ++i)
	{
		len[i] = glm::length(skeleton.joints[chain[i]].restOffset);
		totalLen += len[i];
		netOffset += skeleton.joints[chain[i]].restOffset;
	}
	if (totalLen < 1e-6f)
		return;

	const glm::vec3 toHip = hipTarget.position - headPos;
	const float dist = glm::length(toHip);
	if (dist < 1e-4f)
		return;

	// Work in the head frame: s0 = rest spine direction, dir = desired direction.
	const glm::quat invHead = glm::inverse(headRot);
	const glm::vec3 s0 = glm::normalize(netOffset);
	const glm::vec3 dir = invHead * (toHip / dist);

	// Curl axis (head frame): bend toward the component of dir perpendicular to
	// the spine; fall back to the hip target's forward, then any perpendicular.
	glm::vec3 bend = dir - s0 * glm::dot(dir, s0);
	if (glm::length(bend) < 1e-3f)
	{
		const glm::vec3 forward = invHead * hipTarget.rotation * glm::vec3(0.0f, 0.0f, -1.0f);
		bend = forward - s0 * glm::dot(forward, s0);
		if (glm::length(bend) < 1e-3f)
			bend = anyPerpendicular(s0);
	}
	bend = glm::normalize(bend);
	const glm::vec3 axis = glm::normalize(glm::cross(s0, bend));

	// Arc model: segment i direction rotates by psi * midW[i] around the axis.
	// Chain end for curl angle psi; |endPos| decreases monotonically with psi.
	float midW[5], endW[5];
	float accum = 0.0f;
	for (int i = 0; i < 5; ++i)
	{
		midW[i] = (accum + 0.5f * len[i]) / totalLen;
		accum += len[i];
		endW[i] = accum / totalLen;
	}
	const auto endPos = [&](float psi)
	{
		glm::vec3 p{0.0f};
		for (int i = 0; i < 5; ++i)
			p += glm::angleAxis(psi * midW[i], axis) * (s0 * len[i]);
		return p;
	};

	// Solve the curl angle so the chain end lands at the target distance.
	float psi = 0.0f;
	if (dist < totalLen)
	{
		float lo = 0.0f, hi = glm::two_pi<float>();
		if (glm::length(endPos(hi)) < dist)
		{
			for (int it = 0; it < 16; ++it)
			{
				const float mid = 0.5f * (lo + hi);
				if (glm::length(endPos(mid)) > dist)
					lo = mid;
				else
					hi = mid;
			}
		}
		psi = 0.5f * (lo + hi);
	}

	// Swing aligns the (possibly curled) chain end with the target direction.
	const glm::quat swing = quatFromTo(glm::normalize(endPos(psi)), dir);
	const glm::quat hipRel = invHead * hipTarget.rotation;  // hip rotation in head frame
	const glm::quat identity(1.0f, 0.0f, 0.0f, 0.0f);

	glm::quat prevWorld = headRot;
	for (int i = 0; i < 5; ++i)
	{
		const glm::quat headFrameRot = swing * glm::angleAxis(psi * midW[i], axis)
			* glm::slerp(identity, hipRel, endW[i]);
		const glm::quat worldRot = glm::normalize(headRot * headFrameRot);
		Joint& joint = skeleton.joints[chain[i]];
		joint.localRot = glm::normalize(glm::inverse(prevWorld) * worldRot);
		prevWorld = worldRot;
	}
}

// Two-bone analytic IK for one limb: chain socket -> j1 -> j2 places j2 on the
// goal implied by the tip joint's target; the tip bone then takes the target
// rotation. The socket joint itself is left untouched (no clavicle/hip-socket
// solving). Pole is expressed in the socket's frame.
void solveTwoBoneLimb(Skeleton& skeleton, const WorldTransforms& wt, int socket,
                      int j1, int j2, int tip, const IkTarget& target, const glm::vec3& poleInSocketFrame)
{
	const float len1 = glm::length(skeleton.joints[j1].restOffset);
	const float len2 = glm::length(skeleton.joints[j2].restOffset);
	if (len1 < 1e-6f || len2 < 1e-6f)
		return;

	const glm::quat socketRot = wt.rotations[socket];
	const glm::vec3 restDir = socketRot * glm::normalize(skeleton.joints[j1].restOffset);
	const glm::vec3 pole = socketRot * poleInSocketFrame;
	const glm::vec3 goal = target.position - target.rotation * skeleton.joints[tip].restOffset;

	const TwoBoneIkResult result =
		solveTwoBoneIk(wt.positions[socket], goal, pole, len1, len2, restDir);

	skeleton.joints[j1].localRot = glm::normalize(glm::inverse(socketRot) * result.rot1);
	skeleton.joints[j2].localRot = glm::normalize(glm::inverse(result.rot1) * result.rot2);
	skeleton.joints[tip].localRot = glm::normalize(glm::inverse(result.rot2) * target.rotation);
}
} // namespace

IkRig::IkRig(Skeleton s, IkRigConfig c)
	: skeleton(std::move(s)), config(std::move(c))
{
	for (size_t i = 0; i < skeleton.joints.size(); ++i)
	{
		jointIndexOf_[skeleton.joints[i].name] = static_cast<int>(i);
		if (!skeleton.joints[i].parentIndex)
			rootIndex_ = static_cast<int>(i);
	}

	for (const std::string& bone : config.targetBones)
	{
		const auto it = jointIndexOf_.find(bone);
		if (it == jointIndexOf_.end())
			throw std::runtime_error("ikrig: target bone '" + bone + "' not in skeleton");
		IkTarget target;
		target.jointIndex = it->second;
		targets.push_back(target);
	}

	for (const JointLimits& limit : config.limits)
		if (!jointIndexOf_.contains(limit.bone))
			throw std::runtime_error("ikrig: limits bone '" + limit.bone + "' not in skeleton");

	resetTargets();
}

void IkRig::solve()
{
	for (Joint& joint : skeleton.joints)
		joint.localRot = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

	// 1. Head: rigidly pinned to the head target.
	if (const IkTarget* headTarget = findTarget("head"))
	{
		skeleton.rootPosition = headTarget->position;
		skeleton.joints[rootIndex_].localRot = headTarget->rotation;
	}

	// 2. Spine: interpolate the chain down to the hip target.
	if (const IkTarget* hipTarget = findTarget("hip"))
	{
		const WorldTransforms wt = computeWorldTransforms(skeleton);
		const std::array<int, 5> chain = {
			findJoint("neck"), findJoint("upper_chest"),
			findJoint("chest"), findJoint("waist"), findJoint("hip")
		};
		if (std::all_of(chain.begin(), chain.end(), [](int i) { return i >= 0; }))
			solveSpine(skeleton, wt, rootIndex_, chain, *hipTarget);
	}

	// 3+4. Legs and arms: two-bone analytic IK off the solved spine pose.
	const WorldTransforms wt = computeWorldTransforms(skeleton);
	const auto limb = [&](const char* socket, const char* j1, const char* j2,
	                      const char* tip, const glm::vec3& pole)
	{
		const IkTarget* target = findTarget(tip);
		if (!target)
			return;
		const int s = findJoint(socket), a = findJoint(j1), b = findJoint(j2), t = findJoint(tip);
		if (s < 0 || a < 0 || b < 0 || t < 0)
			return;
		solveTwoBoneLimb(skeleton, wt, s, a, b, t, *target, pole);
	};

	// Pole directions in the limb socket's frame. The skeleton faces -Z at rest:
	// knees bend forward, elbows point down/back.
	constexpr glm::vec3 kKneePole{0.0f, 0.0f, -1.0f};
	const glm::vec3 kElbowPole = normalize(glm::vec3{0.0f, -1.0f, 1.0f});

	limb("left_hip", "left_upper_leg", "left_lower_leg", "left_foot", kKneePole);
	limb("right_hip", "right_upper_leg", "right_lower_leg", "right_foot", kKneePole);
	limb("left_shoulder", "left_upper_arm", "left_lower_arm", "left_hand", kElbowPole);
	limb("right_shoulder", "right_upper_arm", "right_lower_arm", "right_hand", kElbowPole);

	// 5. Joint limits: swing-twist clamp per configured bone.
	for (const JointLimits& limit : config.limits)
	{
		const int index = findJoint(limit.bone);
		if (index < 0)
			continue;  // validated at construction; guard anyway
		Joint& joint = skeleton.joints[index];
		const float len = glm::length(joint.restOffset);
		if (len < 1e-6f)
			continue;
		joint.localRot = clampSwingTwist(joint.localRot, joint.restOffset / len,
		                                 limit.twistMinDeg, limit.twistMaxDeg, limit.swingConeDeg);
	}
}

void IkRig::resetTargets()
{
	for (Joint& joint : skeleton.joints)
		joint.localRot = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
	for (const Joint& joint : skeleton.joints)
		if (!joint.parentIndex)
			skeleton.rootPosition = joint.restOffset;

	const std::vector<glm::vec3> positions = computeWorldPositions(skeleton);
	for (IkTarget& target : targets)
	{
		target.position = positions[target.jointIndex];
		target.rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
	}
}

const IkTarget* IkRig::findTarget(const std::string& bone) const
{
	for (const IkTarget& target : targets)
		if (skeleton.joints[target.jointIndex].name == bone)
			return &target;
	return nullptr;
}

int IkRig::findJoint(const std::string& bone) const
{
	const auto it = jointIndexOf_.find(bone);
	return it == jointIndexOf_.end() ? -1 : it->second;
}
