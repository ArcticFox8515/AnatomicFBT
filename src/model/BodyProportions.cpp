#include "BodyProportions.h"

#include "Error.h"

BodyProportions BodyProportions::makeDefault()
{
	return BodyProportions();
}

void BodyProportions::validate() const
{
	struct
	{
		const char* name;
		float value;
	} lengths[] = {
			{"neckLength", neckLength},
			{"shoulderWidth", shoulderWidth},
			{"hipWidth", hipWidth},
			{"upperArmLength", upperArmLength},
			{"lowerArmLength", lowerArmLength},
			{"upperLegLength", upperLegLength},
			{"lowerLegLength", lowerLegLength},
		};
	for (const auto& [name, value] : lengths)
		if (value <= 0.0f)
			throw Error(std::string("body proportions: ") + name + " must be positive");

	// Must match the hip-line derivation in Skeleton::makeDefault.
	const float hipsY = upperLegLength + lowerLegLength;
	if (navelHeight <= hipsY)
		throw Error("body proportions: navelHeight must exceed the hip line "
			"(upperLegLength + lowerLegLength)");
	if (shoulderHeight <= navelHeight)
		throw Error("body proportions: shoulderHeight must exceed navelHeight");
}

void to_json(nlohmann::json& j, const BodyProportions& p)
{
	j = nlohmann::json{
		{"neckLength", p.neckLength},
		{"shoulderHeight", p.shoulderHeight},
		{"navelHeight", p.navelHeight},
		{"shoulderWidth", p.shoulderWidth},
		{"hipWidth", p.hipWidth},
		{"upperArmLength", p.upperArmLength},
		{"lowerArmLength", p.lowerArmLength},
		{"upperLegLength", p.upperLegLength},
		{"lowerLegLength", p.lowerLegLength}
	};
}

void from_json(const nlohmann::json& j, BodyProportions& p)
{
	j.at("neckLength").get_to(p.neckLength);
	j.at("shoulderHeight").get_to(p.shoulderHeight);
	j.at("navelHeight").get_to(p.navelHeight);
	j.at("shoulderWidth").get_to(p.shoulderWidth);
	j.at("hipWidth").get_to(p.hipWidth);
	j.at("upperArmLength").get_to(p.upperArmLength);
	j.at("lowerArmLength").get_to(p.lowerArmLength);
	j.at("upperLegLength").get_to(p.upperLegLength);
	j.at("lowerLegLength").get_to(p.lowerLegLength);
	p.validate();
}
