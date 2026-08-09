#include "AppSettings.h"

AppSettings AppSettings::makeDefault()
{
    return AppSettings{};
}

void to_json(nlohmann::json& j, const AppSettings& s)
{
    j = nlohmann::json{
        {"virtualTrackers", {{"enabled", s.virtualTrackerBones}}},
    };
}

void from_json(const nlohmann::json& j, AppSettings& s)
{
    s.virtualTrackerBones.clear();
    // A missing block is the empty selection (forward compat with files
    // written before this feature). A present-but-malformed block is an
    // invalid file — nlohmann throws and the caller falls back to default.
    if (j.contains("virtualTrackers"))
        j.at("virtualTrackers").at("enabled").get_to(s.virtualTrackerBones);
}
