#pragma once

#include <nlohmann/json.hpp>

// User body measurements (meters, tape-measurable), stored in
// user-proportions.json. Pure value type — no skeleton knowledge; scaling the
// default hierarchy to these proportions is Skeleton::makeDefault()'s job.
//
// Field semantics:
// - shoulderHeight / navelHeight are heights above the floor of the shoulder
//   line (where the arms attach) and the navel (the lower-spine bend point).
// - neckLength spans the base of the skull (Neck joint) down to the shoulder
//   line (Chest joint). The Head root sits one head-height constant above the
//   Neck; calibration aligns the skeleton to the HMD at runtime.
struct BodyProportions
{
    float neckLength = 0.20f;      // Head root -> shoulder line
    float shoulderHeight = 1.50f;  // floor -> shoulder line
    float navelHeight = 1.20f;     // floor -> navel
    float shoulderWidth = 0.40f;   // shoulder joint to shoulder joint
    float hipWidth = 0.20f;        // hip joint to hip joint
    float upperArmLength = 0.28f;  // shoulder -> elbow
    float lowerArmLength = 0.26f;  // elbow -> wrist
    float upperLegLength = 0.45f;  // hip -> knee
    float lowerLegLength = 0.45f;  // knee -> ankle

    static BodyProportions makeDefault();

    // Throws Error on non-positive lengths or inconsistent heights
    // (shoulderHeight must exceed navelHeight, navelHeight must exceed the
    // hip line upperLegLength + lowerLegLength + ankle-height constant).
    void validate() const;
};

void to_json(nlohmann::json& j, const BodyProportions& proportions);
void from_json(const nlohmann::json& j, BodyProportions& proportions);
