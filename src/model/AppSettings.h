#pragma once

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

// App-wide user settings (user-settings.json), shaped so unrelated settings
// can be added later without a schema break. Today holds only the per-bone
// virtual-tracker selection (doc/virtual-trackers-plan.md step 3). Follows the
// load-or-create convention: a missing file is created with the default; an
// invalid file falls back to the default (in memory) with the error logged.
//
// Unknown top-level keys are ignored on load (forward compat: a future version
// that adds a key still loads a file written by this one, and this version
// ignores a key it does not know). A missing `virtualTrackers` block is treated
// as an empty selection rather than an error, so a file written before this
// feature existed still loads.
struct AppSettings
{
    // Bones the user has ticked for virtual-tracker emission (names from the
    // step-1 eligible list). Empty by default — no bone emits on a fresh
    // install. Order is the user's tick order; only set membership matters.
    std::vector<std::string> virtualTrackerBones;

    static AppSettings makeDefault();
};

void to_json(nlohmann::json& j, const AppSettings& settings);
void from_json(const nlohmann::json& j, AppSettings& settings);
