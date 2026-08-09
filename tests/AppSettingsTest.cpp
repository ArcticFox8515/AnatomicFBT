#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

#include "model/AppSettings.h"
#include "model/BoneNames.h"

TEST(AppSettingsDefault, HasEmptyVirtualTrackerSelection)
{
    const AppSettings s = AppSettings::makeDefault();
    EXPECT_TRUE(s.virtualTrackerBones.empty());
}

TEST(AppSettingsSerialization, RoundTripPreservesSelection)
{
    AppSettings original;
    original.virtualTrackerBones = {BoneNames::Neck, BoneNames::Chest, BoneNames::LeftUpperLeg};

    const nlohmann::json j = original;
    const AppSettings parsed = j.get<AppSettings>();

    ASSERT_EQ(parsed.virtualTrackerBones, original.virtualTrackerBones);
}

TEST(AppSettingsSerialization, EmptySelectionRoundTrips)
{
    const AppSettings original = AppSettings::makeDefault();
    const nlohmann::json j = original;
    const AppSettings parsed = j.get<AppSettings>();

    EXPECT_TRUE(parsed.virtualTrackerBones.empty());
}

TEST(AppSettingsSerialization, EmitsExpectedShape)
{
    AppSettings s;
    s.virtualTrackerBones = {std::string("Neck"), std::string("Chest")};
    const nlohmann::json j = s;

    ASSERT_TRUE(j.is_object());
    ASSERT_TRUE(j.contains("virtualTrackers"));
    ASSERT_TRUE(j["virtualTrackers"].is_object());
    ASSERT_TRUE(j["virtualTrackers"].contains("enabled"));
    ASSERT_TRUE(j["virtualTrackers"]["enabled"].is_array());
    ASSERT_EQ(j["virtualTrackers"]["enabled"].size(), 2u);
    EXPECT_EQ(j["virtualTrackers"]["enabled"][0].get<std::string>(), "Neck");
    EXPECT_EQ(j["virtualTrackers"]["enabled"][1].get<std::string>(), "Chest");
}

TEST(AppSettingsDeserialization, MissingBlockYieldsEmptySelection)
{
    // A file written before this feature: no virtualTrackers key. Loads as
    // empty rather than throwing, so the app still starts.
    const nlohmann::json j = nlohmann::json::parse("{}");
    const AppSettings s = j.get<AppSettings>();
    EXPECT_TRUE(s.virtualTrackerBones.empty());
}

TEST(AppSettingsDeserialization, MissingEnabledYieldsEmptySelection)
{
    const nlohmann::json j = nlohmann::json::parse(R"({"virtualTrackers": {}})");
    EXPECT_THROW(j.get<AppSettings>(), nlohmann::json::exception);
}

TEST(AppSettingsDeserialization, NonObjectEnabledThrows)
{
    const nlohmann::json j = nlohmann::json::parse(R"({"virtualTrackers": {"enabled": 5}})");
    EXPECT_THROW(j.get<AppSettings>(), nlohmann::json::exception);
}

TEST(AppSettingsDeserialization, NonObjectVirtualTrackersThrows)
{
    const nlohmann::json j = nlohmann::json::parse(R"({"virtualTrackers": 5})");
    EXPECT_THROW(j.get<AppSettings>(), nlohmann::json::exception);
}

TEST(AppSettingsDeserialization, UnknownTopLevelKeyIgnored)
{
    // A future version adds a key; this version ignores it and still loads
    // the virtual-tracker selection.
    const nlohmann::json j = nlohmann::json::parse(
        R"({"futureSetting": 42, "virtualTrackers": {"enabled": ["Neck"]}})");
    const AppSettings s = j.get<AppSettings>();
    ASSERT_EQ(s.virtualTrackerBones.size(), 1u);
    EXPECT_EQ(s.virtualTrackerBones[0], "Neck");
}

TEST(AppSettingsDeserialization, UnknownNestedKeyIgnored)
{
    const nlohmann::json j = nlohmann::json::parse(
        R"({"virtualTrackers": {"enabled": ["Neck"], "futureNested": true}})");
    const AppSettings s = j.get<AppSettings>();
    ASSERT_EQ(s.virtualTrackerBones.size(), 1u);
    EXPECT_EQ(s.virtualTrackerBones[0], "Neck");
}

TEST(AppSettingsDeserialization, EmptyEnabledArrayYieldsEmptySelection)
{
    const nlohmann::json j = nlohmann::json::parse(R"({"virtualTrackers": {"enabled": []}})");
    const AppSettings s = j.get<AppSettings>();
    EXPECT_TRUE(s.virtualTrackerBones.empty());
}

TEST(AppSettingsDeserialization, NonStringEntryThrows)
{
    const nlohmann::json j = nlohmann::json::parse(R"({"virtualTrackers": {"enabled": [42]}})");
    EXPECT_THROW(j.get<AppSettings>(), nlohmann::json::exception);
}
