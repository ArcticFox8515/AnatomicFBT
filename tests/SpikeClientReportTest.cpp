// Tests for the throwaway step-1 spike's client-side reporting (doc/driver-plan.md).
//
// spike_client exists to print the *client's* view of a pose so it can be compared,
// line by line, against the two candidate compositions the driver logs. Both halves
// of that are here: the matrix -> pose conversion (the one place where the client
// could silently disagree with the driver's math) and the sampling loop, with
// IVRSystem and Sleep replaced by fakes.

#include "spike/SpikeClientReport.h"
#include "spike/SpikeLog.h"
#include "spike/SpikePoseMath.h"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace
{
constexpr double kTolerance = 1e-6;

// Builds the row-major 3x4 matrix OpenVR would report for this pose, by rotating the
// basis vectors — i.e. from the definition, not from the formula under test.
void fillMatrix(float m[3][4], const spike::Q& rotation, const spike::V3& position)
{
    const spike::V3 axes[3] = {spike::rotate(rotation, {1.0, 0.0, 0.0}),
                               spike::rotate(rotation, {0.0, 1.0, 0.0}),
                               spike::rotate(rotation, {0.0, 0.0, 1.0})};
    const double columns[3][3] = {{axes[0].x, axes[0].y, axes[0].z},
                                  {axes[1].x, axes[1].y, axes[1].z},
                                  {axes[2].x, axes[2].y, axes[2].z}};
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
            m[row][column] = static_cast<float>(columns[column][row]);
    }
    m[0][3] = static_cast<float>(position.x);
    m[1][3] = static_cast<float>(position.y);
    m[2][3] = static_cast<float>(position.z);
}

void expectSameRotation(const spike::Q& actual, const spike::Q& expected)
{
    // q and -q are the same rotation; the conversion is free to pick either.
    const double sign = actual.w * expected.w + actual.x * expected.x + actual.y * expected.y
                                + actual.z * expected.z
                            < 0.0
                        ? -1.0
                        : 1.0;
    EXPECT_NEAR(actual.w * sign, expected.w, kTolerance);
    EXPECT_NEAR(actual.x * sign, expected.x, kTolerance);
    EXPECT_NEAR(actual.y * sign, expected.y, kTolerance);
    EXPECT_NEAR(actual.z * sign, expected.z, kTolerance);
}

TEST(SpikePoseFromMatrix, TranslationIsTheLastColumn)
{
    float m[3][4] = {};
    fillMatrix(m, {}, {1.0, 2.0, 3.0});
    const spike::RigidPose pose = spike::poseFromRowMajor34(m);
    EXPECT_NEAR(pose.pos.x, 1.0, kTolerance);
    EXPECT_NEAR(pose.pos.y, 2.0, kTolerance);
    EXPECT_NEAR(pose.pos.z, 3.0, kTolerance);
    expectSameRotation(pose.rot, {});
}

TEST(SpikePoseFromMatrix, EveryLargestComponentBranchRoundTrips)
{
    // One rotation per branch of the conversion (w, x, y, z largest): a wrong branch is
    // the classic way this formula produces a plausible but wrong quaternion.
    //
    // The 180 degree cases alone are not enough: their matrices are symmetric, so a
    // sign error in the off-diagonal terms cancels. Every dominant-component branch
    // therefore also gets an asymmetric rotation (160 degrees about the same axis).
    const double s = std::sqrt(0.5);
    const double c80 = std::cos(80.0 * 3.14159265358979323846 / 180.0);
    const double s80 = std::sin(80.0 * 3.14159265358979323846 / 180.0);
    const spike::Q rotations[] = {
        {1.0, 0.0, 0.0, 0.0},   // identity      -> w
        {0.0, 1.0, 0.0, 0.0},   // 180 deg  X    -> x
        {0.0, 0.0, 1.0, 0.0},   // 180 deg  Y    -> y
        {0.0, 0.0, 0.0, 1.0},   // 180 deg  Z    -> z
        {c80, s80, 0.0, 0.0},   // 160 deg  X    -> x, asymmetric
        {c80, 0.0, s80, 0.0},   // 160 deg  Y    -> y, asymmetric
        {c80, 0.0, 0.0, s80},   // 160 deg  Z    -> z, asymmetric
        {s, 0.0, s, 0.0},       // 90 deg   Y
        {0.5, 0.5, 0.5, 0.5},   // 120 deg  diagonal
    };

    for (const spike::Q& rotation : rotations)
    {
        float m[3][4] = {};
        fillMatrix(m, rotation, {0.5, -1.5, 2.5});
        const spike::RigidPose pose = spike::poseFromRowMajor34(m);
        expectSameRotation(pose.rot, rotation);

        // The rotation must act the same way, not merely look similar.
        const spike::V3 expected = spike::rotate(rotation, {1.0, 2.0, 3.0});
        const spike::V3 actual = spike::rotate(pose.rot, {1.0, 2.0, 3.0});
        EXPECT_NEAR(actual.x, expected.x, kTolerance);
        EXPECT_NEAR(actual.y, expected.y, kTolerance);
        EXPECT_NEAR(actual.z, expected.z, kTolerance);
    }
}

// ------------------------------------------------------------------ report ----

spike::ClientDeviceSample makeSample()
{
    spike::ClientDeviceSample device;
    device.index = 3;
    device.deviceClass = 3; // vr::TrackedDeviceClass_GenericTracker
    device.serial = "LHR-TESTTRACKER";
    device.poseValid = true;
    device.trackingResult = 200;
    device.raw = {{1.0, 2.0, 3.0}, {}};
    device.standing = {{1.5, 2.0, 3.0}, {}};
    return device;
}

TEST(SpikeClientReport, LinesMatchTheDriverSideFormatSoTheLogsCanBeDiffed)
{
    const std::vector<std::string> lines = formatClientDeviceLines(makeSample());
    ASSERT_EQ(lines.size(), 3u);
    EXPECT_EQ(lines[0], "dev 3 tracker \"LHR-TESTTRACKER\" valid=1 result=200");
    EXPECT_EQ(lines[1], "     raw              " + spike::formatPose(makeSample().raw));
    EXPECT_EQ(lines[2], "     standing         " + spike::formatPose(makeSample().standing));
    // The pose columns must line up with the driver's "A = wFd o local" lines.
    EXPECT_EQ(lines[1].find("pos="), lines[2].find("pos="));
}

TEST(SpikeClientReport, DurationOfZeroMeansUntilInterrupted)
{
    EXPECT_TRUE(spike::clientShouldContinue(0, 0.0));
    EXPECT_TRUE(spike::clientShouldContinue(0, 1e9));
    EXPECT_TRUE(spike::clientShouldContinue(-5, 1e9));
}

TEST(SpikeClientReport, ADurationStopsTheRunWhenItElapses)
{
    EXPECT_TRUE(spike::clientShouldContinue(30, 0.0));
    EXPECT_TRUE(spike::clientShouldContinue(30, 29.999));
    EXPECT_FALSE(spike::clientShouldContinue(30, 30.0));
    EXPECT_FALSE(spike::clientShouldContinue(30, 31.0));
}

class FakeClientPoseSource : public spike::ClientPoseSource
{
public:
    std::vector<spike::ClientDeviceSample> sample() override
    {
        ++samples;
        return devices;
    }

    std::vector<spike::ClientDeviceSample> devices;
    int samples = 0;
};

TEST(SpikeClientReport, SamplingLogsEveryDeviceOncePerPassAndWaitsBetweenPasses)
{
    FakeClientPoseSource source;
    source.devices.push_back(makeSample());
    spike::ClientDeviceSample second = makeSample();
    second.index = 4;
    second.serial = "LHR-OTHER";
    source.devices.push_back(second);

    std::vector<std::string> lines;
    spike::Logger logger;
    logger.setSink([&](const char* message) { lines.emplace_back(message); });

    int waits = 0;
    int passes = 0;
    spike::runClientSampling(
        source, logger, [&] { return passes++ < 2; }, [&] { ++waits; });

    EXPECT_EQ(source.samples, 2);
    EXPECT_EQ(waits, 2);
    ASSERT_EQ(lines.size(), 12u); // 2 passes x 2 devices x 3 lines
    EXPECT_EQ(lines[0], "dev 3 tracker \"LHR-TESTTRACKER\" valid=1 result=200");
    EXPECT_EQ(lines[3], "dev 4 tracker \"LHR-OTHER\" valid=1 result=200");
}

TEST(SpikeClientReport, ASessionWithNoDevicesLogsNothingAndStillStops)
{
    FakeClientPoseSource source;
    spike::Logger logger;
    std::vector<std::string> lines;
    logger.setSink([&](const char* message) { lines.emplace_back(message); });

    int passes = 0;
    spike::runClientSampling(
        source, logger, [&] { return passes++ < 1; }, [] {});

    EXPECT_EQ(source.samples, 1);
    EXPECT_TRUE(lines.empty());
}
} // namespace
