#include "IkRig.h"

#include "Error.h"
#include "IkMath.h"

#include <algorithm>
#include <numeric>

namespace
{
// Dynamic bend normal (WP2, PoleMode::DynamicFoot/DynamicHand): the middle-joint
// (knee/elbow) bulge direction, perpendicular to the chain aim *by
// construction*. `aim` is socket->goal in world space; `hingeAxis` is the
// foot/hand medial-lateral axis in world space (`targetRot * (±1,0,0)`). The
// cross product is perpendicular to aim, so there is no pole‖aim degeneracy
// and no fallback — the failure mode of the abandoned blend approach is
// structurally impossible. The lateral axis is the hinge axis itself, so foot
// pitch (a rotation about this axis) does not move it — a shin-mounted
// tracker's pitch cannot contaminate the bend plane; foot yaw/roll (crossed-
// leg splay) move it correctly. `flexSign` (+1/-1, derived once at bind time
// from the static pole) selects the anatomically correct bulge side: knee and
// elbow flex in opposite directions relative to cross(hinge, aim).
// `staticPoleWorld` is the singularity guard only (hinge ‖ aim, anatomically
// impossible for a bent limb).
glm::vec3 dynamicBendNormal(const glm::vec3& aim, const glm::quat& targetRot,
	int sideSign, float flexSign, const glm::vec3& staticPoleWorld)
{
	const glm::vec3 hingeAxis =
		targetRot * glm::vec3(static_cast<float>(sideSign), 0.0f, 0.0f);
	glm::vec3 pole = flexSign * glm::cross(hingeAxis, aim);
	const float len = glm::length(pole);
	if (len < 1e-6f)
		return staticPoleWorld;  // hinge ‖ aim: anatomically impossible for a
		                        // bent limb; the static pole is a stable guard.
	return pole / len;
}
} // namespace

IkRig::IkRig(Skeleton s)
	: skeleton(std::move(s))
{
	for (size_t i = 0; i < skeleton.joints.size(); ++i)
	{
		jointIndexOf_[skeleton.joints[i].name] = static_cast<int>(i);
		if (!skeleton.joints[i].parentIndex)
			rootIndex_ = static_cast<int>(i);
	}
}

void IkRig::loadConfig(IkRigConfig c)
{
	for (const JointLimits& limit : c.limits)
		if (!jointIndexOf_.contains(limit.bone))
			throw Error("ikrig: limits bone '" + limit.bone + "' not in skeleton");

	// Build into locals; the current config/targets are replaced only after
	// every target validated and bound successfully.
	std::vector<SolverBinding> bindings;
	std::vector<IkTarget> newTargets;
	for (const TargetConfig& targetConfig : c.targets)
	{
		const int jointIndex = findJoint(targetConfig.bone);
		if (jointIndex < 0)
			throw Error("ikrig: target bone '" + targetConfig.bone + "' not in skeleton");

		SolverBinding binding;
		binding.solver = targetConfig.solver;
		switch (targetConfig.solver)
		{
		case SolverType::Anchor:
			if (jointIndex != rootIndex_)
				throw Error("ikrig: anchor target '" + targetConfig.bone
					+ "' must be the root joint");
			break;
		case SolverType::Chain:
		{
			if (jointIndex == rootIndex_)
				throw Error("ikrig: chain target '" + targetConfig.bone
					+ "' must not be the root joint");
			// Ancestor path from the target joint up to (excluding) the root.
			for (int i = jointIndex; i != rootIndex_; i = *skeleton.joints[i].parentIndex)
				binding.chain.push_back(i);
			std::reverse(binding.chain.begin(), binding.chain.end());
			break;
		}
		case SolverType::TwoBone:
		{
			// tip -> j2 -> j1 -> socket, walking up the hierarchy.
			int indices[4] = {-1, -1, -1, jointIndex};  // {socket, j1, j2, tip}
			for (int slot = 2; slot >= 0; --slot)
			{
				const std::optional<int>& parent = skeleton.joints[indices[slot + 1]].parentIndex;
				if (!parent)
					throw Error("ikrig: two_bone target '" + targetConfig.bone
						+ "' needs at least 3 ancestors");
				indices[slot] = *parent;
			}
			binding.chain.assign(indices, indices + 4);

			// The middle bone (j2) carries the bend pole in its limits entry.
			const std::string& middleBone = skeleton.joints[indices[2]].name;
			const auto it = std::find_if(c.limits.begin(), c.limits.end(),
				[&middleBone](const JointLimits& limit) { return limit.bone == middleBone; });
			if (it == c.limits.end() || !it->pole)
				throw Error("ikrig: two_bone target '" + targetConfig.bone
					+ "' needs a pole on middle bone '" + middleBone + "' in limits");
			binding.pole = *it->pole;
			binding.poleMode = it->poleMode;
			if (binding.poleMode != PoleMode::Static)
			{
				// Side sign from the socket joint's rest X position: left
				// side at +X (sideSign +1), right side at -X (sideSign -1).
				// The socket is the hip/shoulder, which has a lateral rest
				// offset. Zero (centered socket) degrades to +1.
				const float sx = skeleton.joints[indices[0]].restOffset.x;
				binding.sideSign = (sx < -1e-6f) ? -1 : 1;
				// Flex sign: pick the cross() order that agrees with the
				// proven-correct static pole at rest. Computed once here so
				// no per-frame sign test can flip at the singularity. The
				// arm is straight along the hinge at rest (aim ‖ hinge), so
				// a small downward perturbation gives a reference cross.
				glm::vec3 restAim = skeleton.joints[indices[1]].restOffset;
				const float rLen = glm::length(restAim);
				if (rLen > 1e-6f) restAim /= rLen;
				const glm::vec3 hingeRest(
					static_cast<float>(binding.sideSign), 0.0f, 0.0f);
				glm::vec3 crossRest = glm::cross(hingeRest, restAim);
				if (glm::length(crossRest) < 1e-6f)
				{
					// Limb straight along the hinge at rest: perturb the aim
					// downward (perpendicular to a +X hinge) to get a
					// reference flex direction.
					restAim = glm::normalize(restAim + glm::vec3(0.0f, -0.1f, 0.0f));
					crossRest = glm::cross(hingeRest, restAim);
				}
				binding.flexSign = (glm::dot(crossRest, binding.pole) >= 0.0f) ? 1.0f : -1.0f;
			}

			// WP3: the socket bone (the clavicle) carries the clavicle stage
			// config in its limits entry; absent means the socket stays at rest.
			const std::string& socketBone = skeleton.joints[indices[0]].name;
			const auto socketIt = std::find_if(c.limits.begin(), c.limits.end(),
				[&socketBone](const JointLimits& limit) { return limit.bone == socketBone; });
			if (socketIt != c.limits.end())
				binding.clavicle = socketIt->clavicle;
			break;
		}
		}

		IkTarget target;
		target.jointIndex = jointIndex;
		newTargets.push_back(target);
		bindings.push_back(std::move(binding));
	}

	config = std::move(c);
	bindings_ = std::move(bindings);
	targets = std::move(newTargets);
	resetTargets();
}

void IkRig::solve()
{
	solve(targets);
}

void IkRig::solve(const std::vector<IkTarget>& goals)
{
	if (goals.size() != targets.size())
		throw Error("IkRig::solve: goals size (" + std::to_string(goals.size())
		            + ") does not match targets size (" + std::to_string(targets.size()) + ")");

	for (Joint& joint : skeleton.joints)
		joint.localRot = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

	// 1. Anchors: root rigidly pinned to the goal.
	for (size_t i = 0; i < goals.size(); ++i)
		if (bindings_[i].solver == SolverType::Anchor)
			solveAnchor(skeleton, goals[i].jointIndex, goals[i]);

	// 2. Chains: swing/curl root->joint chains onto their goals.
	{
		const WorldTransforms wt = computeWorldTransforms(skeleton);
		for (size_t i = 0; i < goals.size(); ++i)
			if (bindings_[i].solver == SolverType::Chain)
				solveChain(skeleton, wt, rootIndex_, bindings_[i].chain, goals[i]);
	}

	// 3. Clavicles: procedural socket rotation for two-bone limbs whose socket
	// bone carries a clavicle config (WP3) — the shoulder follows the hand goal
	// instead of staying rigid. Runs before the limb solve, so the limb is
	// solved from the moved socket and still lands exactly on its goal.
	{
		const WorldTransforms wt = computeWorldTransforms(skeleton);
		for (size_t i = 0; i < goals.size(); ++i)
		{
			if (bindings_[i].solver != SolverType::TwoBone || !bindings_[i].clavicle)
				continue;
			const ClavicleConfig& clavicle = *bindings_[i].clavicle;
			solveClavicle(skeleton, wt, bindings_[i].chain, goals[i],
			              clavicle.elevationWeight, clavicle.reachWeight,
			              clavicle.reachThreshold, clavicle.maxAngleDeg);
		}
	}

	// 4. Limbs: two-bone analytic IK off the solved chain pose.
	{
		const WorldTransforms wt = computeWorldTransforms(skeleton);
		for (size_t i = 0; i < goals.size(); ++i)
		{
			if (bindings_[i].solver != SolverType::TwoBone)
				continue;
			const std::vector<int>& chain = bindings_[i].chain;
			const glm::quat socketRot = wt.rotations[chain[0]];
			glm::vec3 poleInSocketFrame = bindings_[i].pole;
			if (bindings_[i].poleMode != PoleMode::Static)
			{
				// WP2: dynamic bend normal, perpendicular to the chain aim by
				// construction. The aim matches solveTwoBone's goal (tip
				// target adjusted by the tip bone's rest offset).
				const glm::vec3 goal = goals[i].position -
					goals[i].rotation * skeleton.joints[chain[3]].restOffset;
				const glm::vec3 aim = goal - wt.positions[chain[0]];
				const glm::vec3 staticPoleWorld = socketRot * bindings_[i].pole;
				const glm::vec3 worldPole = dynamicBendNormal(
					aim, goals[i].rotation, bindings_[i].sideSign,
					bindings_[i].flexSign, staticPoleWorld);
				poleInSocketFrame = glm::inverse(socketRot) * worldPole;
			}
			solveTwoBone(skeleton, wt, chain[0], chain[1], chain[2], chain[3],
			             goals[i], poleInSocketFrame);
		}
	}

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

	// 6. End-effector re-aim: after the clamp, re-aim each tracked end-effector
	// (anchor root, chain end, two-bone tip) at the goal rotation so a mid-bone
	// limit bends the pose but never silently rotates the tracked feature
	// (foot/hand/Hips) away from its target. Policy: tracked rotation wins over
	// limits on end bones; limits constrain the mid-bones only.
	//
	// The skeleton is sorted parent-before-child, so iterating goals by
	// ascending jointIndex is a topological order: an end-effector that is an
	// ancestor of another (the anchor root is an ancestor of everything; the
	// Hips chain end is an ancestor of both leg tips) is re-aimed first. The
	// parent world rotation is recomputed after each write, so no end bone
	// reads a frame an earlier write has just invalidated.
	std::vector<size_t> order(goals.size());
	std::iota(order.begin(), order.end(), 0);
	std::sort(order.begin(), order.end(),
	          [&](size_t a, size_t b) { return goals[a].jointIndex < goals[b].jointIndex; });
	for (size_t idx : order)
	{
		Joint& joint = skeleton.joints[goals[idx].jointIndex];
		if (bindings_[idx].solver == SolverType::Anchor)
		{
			joint.localRot = goals[idx].rotation;  // root has no parent frame
			continue;
		}
		const WorldTransforms wt = computeWorldTransforms(skeleton);
		joint.localRot = glm::normalize(
			glm::inverse(wt.rotations[*joint.parentIndex]) * goals[idx].rotation);
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

int IkRig::findJoint(const std::string& bone) const
{
	const auto it = jointIndexOf_.find(bone);
	return it == jointIndexOf_.end() ? -1 : it->second;
}
