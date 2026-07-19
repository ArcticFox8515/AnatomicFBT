#include "IkRig.h"

#include "Error.h"
#include "IkMath.h"

#include <algorithm>

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

	// 3. Limbs: two-bone analytic IK off the solved chain pose.
	{
		const WorldTransforms wt = computeWorldTransforms(skeleton);
		for (size_t i = 0; i < goals.size(); ++i)
		{
			if (bindings_[i].solver != SolverType::TwoBone)
				continue;
			const std::vector<int>& chain = bindings_[i].chain;
			solveTwoBone(skeleton, wt, chain[0], chain[1], chain[2], chain[3],
			             goals[i], bindings_[i].pole);
		}
	}

	// 4. Joint limits: swing-twist clamp per configured bone.
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

int IkRig::findJoint(const std::string& bone) const
{
	const auto it = jointIndexOf_.find(bone);
	return it == jointIndexOf_.end() ? -1 : it->second;
}
