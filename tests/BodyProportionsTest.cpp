#include <gtest/gtest.h>
#include "model/BodyProportions.h"
#include "model/Skeleton.h"

namespace
{
int indexOf(const Skeleton& skeleton, const std::string& name)
{
    for (size_t i = 0; i < skeleton.joints.size(); ++i)
        if (skeleton.joints[i].name == name)
            return static_cast<int>(i);
    return -1;
}
} // namespace

TEST(BodyProportionsSerialization, DefaultRoundTrip)
{
    const BodyProportions original = BodyProportions::makeDefault();
    const BodyProportions parsed = nlohmann::json(original).get<BodyProportions>();

    EXPECT_EQ(parsed.neckLength, original.neckLength);
    EXPECT_EQ(parsed.shoulderHeight, original.shoulderHeight);
    EXPECT_EQ(parsed.navelHeight, original.navelHeight);
    EXPECT_EQ(parsed.shoulderWidth, original.shoulderWidth);
    EXPECT_EQ(parsed.hipWidth, original.hipWidth);
    EXPECT_EQ(parsed.upperArmLength, original.upperArmLength);
    EXPECT_EQ(parsed.lowerArmLength, original.lowerArmLength);
    EXPECT_EQ(parsed.upperLegLength, original.upperLegLength);
    EXPECT_EQ(parsed.lowerLegLength, original.lowerLegLength);
}

TEST(BodyProportionsDeserialization, MissingFieldThrows)
{
    const nlohmann::json j = nlohmann::json::parse("{}");
    EXPECT_THROW(j.get<BodyProportions>(), nlohmann::json::exception);
}

TEST(BodyProportionsValidation, RejectsNonPositiveLengths)
{
    BodyProportions p = BodyProportions::makeDefault();
    p.upperLegLength = 0.0f;
    EXPECT_THROW(nlohmann::json(p).get<BodyProportions>(), std::runtime_error);

    p = BodyProportions::makeDefault();
    p.shoulderWidth = -0.4f;
    EXPECT_THROW(nlohmann::json(p).get<BodyProportions>(), std::runtime_error);
}

TEST(BodyProportionsValidation, RejectsNavelBelowHipLine)
{
    // Hip line = upperLegLength + lowerLegLength; the navel must be above it.
    BodyProportions p = BodyProportions::makeDefault();
    p.navelHeight = p.upperLegLength + p.lowerLegLength;
    EXPECT_THROW(nlohmann::json(p).get<BodyProportions>(), std::runtime_error);
}

TEST(BodyProportionsValidation, RejectsShoulderBelowNavel)
{
    BodyProportions p = BodyProportions::makeDefault();
    p.shoulderHeight = p.navelHeight;
    EXPECT_THROW(nlohmann::json(p).get<BodyProportions>(), std::runtime_error);
}

TEST(DefaultSkeletonFromProportions, LandmarkHeightsLandOnTheirJoints)
{
    BodyProportions p = BodyProportions::makeDefault();
    p.shoulderHeight = 1.45f;
    p.navelHeight = 1.10f;
    p.upperLegLength = 0.50f;
    p.lowerLegLength = 0.40f;

    const Skeleton skeleton = Skeleton::makeDefault(p);
    const std::vector<glm::vec3> positions = computeWorldPositions(skeleton);
    const auto at = [&skeleton, &positions](const std::string& name)
    { return positions[static_cast<size_t>(indexOf(skeleton, name))]; };

    // The skeleton has no built-in floor — a standing human measures heights
    // against the ground under their feet, so the skeleton's landmarks are
    // measured against its own ankles.
    const float ankle = at("LeftLowerLeg").y;
    EXPECT_NEAR(at("RightLowerLeg").y, ankle, 1e-6f);

    // Chest (arm attachment) sits shoulderHeight above the ankles, Waist at
    // navelHeight, Hips at the leg span.
    EXPECT_NEAR(at("Chest").y - ankle, p.shoulderHeight, 1e-6f);
    EXPECT_NEAR(at("Waist").y - ankle, p.navelHeight, 1e-6f);
    EXPECT_NEAR(at("Hips").y - ankle, p.upperLegLength + p.lowerLegLength, 1e-6f);
    // The neck spans skull base (Neck) to shoulder line (Chest).
    EXPECT_NEAR(at("Neck").y - at("Chest").y, p.neckLength, 1e-6f);
    // The fictional mid-spine joint splits its span at the midpoint.
    EXPECT_NEAR(at("Spine").y - at("Waist").y, at("Chest").y - at("Spine").y, 1e-6f);
    // The root sits above the neck and seeds rootPosition.
    EXPECT_GT(at("Head").y, at("Neck").y);
    EXPECT_NEAR(skeleton.rootPosition.y, at("Head").y, 1e-6f);
}

TEST(DefaultSkeletonFromProportions, UpperLegLengthRaisesHipLineKneeStaysAnchored)
{
    // Longer upper leg = the hip line rises by the same amount; the knee
    // keeps its height above the ankle (lowerLegLength), the ankle and
    // everything above the hip line don't move at all.
    BodyProportions p = BodyProportions::makeDefault();
    p.upperLegLength += 0.10f;

    const Skeleton baseline = Skeleton::makeDefault(BodyProportions::makeDefault());
    const Skeleton scaled = Skeleton::makeDefault(p);
    const std::vector<glm::vec3> basePos = computeWorldPositions(baseline);
    const std::vector<glm::vec3> scaledPos = computeWorldPositions(scaled);

    ASSERT_EQ(baseline.joints.size(), scaled.joints.size());
    for (size_t i = 0; i < scaled.joints.size(); ++i)
    {
        const std::string& name = scaled.joints[i].name;
        if (name == "Hips" || name == "LeftHip" || name == "RightHip")
            EXPECT_NEAR(scaledPos[i].y, basePos[i].y + 0.10f, 1e-6f) << name;
        else
            EXPECT_EQ(scaledPos[i], basePos[i]) << name;
    }
}

TEST(DefaultSkeletonFromProportions, ShoulderWidthMovesOnlyArms)
{
    BodyProportions p = BodyProportions::makeDefault();
    p.shoulderWidth = 0.50f; // +0.10 -> +0.05 per side

    const Skeleton baseline = Skeleton::makeDefault(BodyProportions::makeDefault());
    const Skeleton scaled = Skeleton::makeDefault(p);
    const std::vector<glm::vec3> basePos = computeWorldPositions(baseline);
    const std::vector<glm::vec3> scaledPos = computeWorldPositions(scaled);

    for (size_t i = 0; i < scaled.joints.size(); ++i)
    {
        const std::string& name = scaled.joints[i].name;
        if (name.find("Shoulder") != std::string::npos || name.find("Arm") != std::string::npos ||
            name.find("Hand") != std::string::npos)
        {
            const float direction = name.find("Left") == 0 ? 1.0f : -1.0f;
            EXPECT_NEAR(scaledPos[i].x, basePos[i].x + direction * 0.05f, 1e-6f) << name;
            EXPECT_NEAR(scaledPos[i].y, basePos[i].y, 1e-6f) << name;
        }
        else
        {
            EXPECT_EQ(scaledPos[i], basePos[i]) << name;
        }
    }
}
