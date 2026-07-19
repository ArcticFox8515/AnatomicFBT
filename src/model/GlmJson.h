#pragma once

#include <glm/glm.hpp>
#include <nlohmann/json.hpp>

// nlohmann (de)serializers for glm types. Defined in namespace glm so ADL
// finds them for glm::vec3 (nlohmann's adl_serializer relies on ADL).

namespace glm
{
inline void to_json(nlohmann::json& j, const glm::vec3& v)
{
    j = {v.x, v.y, v.z};
}

inline void from_json(const nlohmann::json& j, glm::vec3& v)
{
    j.at(0).get_to(v.x);
    j.at(1).get_to(v.y);
    j.at(2).get_to(v.z);
}
} // namespace glm
