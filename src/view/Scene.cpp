#include "Scene.h"

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

#include "model/IkRig.h"
#include "model/Skeleton.h"

namespace
{
constexpr char kLineVertexShader[] = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
uniform mat4 uViewProj;
void main()
{
    gl_Position = uViewProj * vec4(aPos, 1.0);
}
)";

constexpr char kLineFragmentShader[] = R"(
#version 330 core
uniform vec3 uColor;
out vec4 FragColor;
void main()
{
    FragColor = vec4(uColor, 1.0);
}
)";

constexpr char kMeshVertexShader[] = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
uniform mat4 uModel;
uniform mat4 uViewProj;
out vec3 vNormal;
void main()
{
    vNormal = mat3(uModel) * aNormal;
    gl_Position = uViewProj * uModel * vec4(aPos, 1.0);
}
)";

constexpr char kMeshFragmentShader[] = R"(
#version 330 core
in vec3 vNormal;
uniform vec3 uColor;
uniform vec3 uLightDir;
uniform vec3 uLightColor;
out vec4 FragColor;
void main()
{
    vec3 n = normalize(vNormal);
    float diffuse = max(dot(n, -uLightDir), 0.0);
    vec3 color = uColor * (0.3 + 0.7 * diffuse * uLightColor);
    FragColor = vec4(color, 1.0);
}
)";

unsigned int compileShader(GLenum type, const char* source)
{
    const unsigned int shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    int success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        char log[1024];
        glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
        glDeleteShader(shader);
        throw std::runtime_error(std::string("shader compilation failed: ") + log);
    }
    return shader;
}

unsigned int createProgram(const char* vertexSource, const char* fragmentSource)
{
    const unsigned int vertex = compileShader(GL_VERTEX_SHADER, vertexSource);
    const unsigned int fragment = compileShader(GL_FRAGMENT_SHADER, fragmentSource);

    const unsigned int program = glCreateProgram();
    glAttachShader(program, vertex);
    glAttachShader(program, fragment);
    glLinkProgram(program);
    glDeleteShader(vertex);
    glDeleteShader(fragment);

    int success;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success)
    {
        char log[1024];
        glGetProgramInfoLog(program, sizeof(log), nullptr, log);
        glDeleteProgram(program);
        throw std::runtime_error(std::string("shader link failed: ") + log);
    }
    return program;
}

// Interleaved mesh vertex: position + normal.
struct Vertex
{
    glm::vec3 position;
    glm::vec3 normal;
};

// Unit pyramid with per-face normals (vertices duplicated per face).
// Square base of half-extent 1 in the XZ plane at y=0, apex at (0,1,0).
std::vector<Vertex> buildPyramid()
{
    constexpr glm::vec3 apex(0.0f, 1.0f, 0.0f);
    constexpr glm::vec3 c0(-1.0f, 0.0f, -1.0f), c1(1.0f, 0.0f, -1.0f);
    constexpr glm::vec3 c2(1.0f, 0.0f, 1.0f), c3(-1.0f, 0.0f, 1.0f);
    const glm::vec3 faces[6][3] = {
        {apex, c0, c1}, {apex, c1, c2}, {apex, c2, c3}, {apex, c3, c0},
        {c0, c2, c1}, {c0, c3, c2},
    };

    constexpr glm::vec3 interior(0.0f, 0.25f, 0.0f); // pyramid centroid
    std::vector<Vertex> data;
    constexpr size_t vertexCount = 6 * 3;
    data.reserve(vertexCount);
    for (const auto& face : faces)
    {
        glm::vec3 normal = glm::normalize(glm::cross(face[1] - face[0], face[2] - face[0]));
        const glm::vec3 center = (face[0] + face[1] + face[2]) / 3.0f;
        if (glm::dot(normal, center - interior) < 0.0f)
            normal = -normal;
        for (const glm::vec3& v : face)
            data.push_back({v, normal});
    }
    return data;
}
// Unit octahedron (6 vertices at ±1 on each axis, 8 triangle faces).
std::vector<Vertex> buildOctahedron()
{
    constexpr glm::vec3 px(1.0f, 0.0f, 0.0f), nx(-1.0f, 0.0f, 0.0f);
    constexpr glm::vec3 py(0.0f, 1.0f, 0.0f), ny(0.0f, -1.0f, 0.0f);
    constexpr glm::vec3 pz(0.0f, 0.0f, 1.0f), nz(0.0f, 0.0f, -1.0f);
    const glm::vec3 faces[8][3] = {
        {px, py, pz}, {pz, py, nx}, {nx, py, nz}, {nz, py, px},
        {px, nz, ny}, {nz, nx, ny}, {nx, pz, ny}, {pz, px, ny},
    };

    std::vector<Vertex> data;
    data.reserve(8 * 3);
    for (const auto& face : faces)
    {
        // Face normals all point outward from the origin for a regular octahedron.
        glm::vec3 normal = glm::normalize(face[0] + face[1] + face[2]);
        for (const glm::vec3& v : face)
            data.push_back({v, normal});
    }
    return data;
}
} // namespace

Scene::Scene()
{
    m_lightDir = normalize(m_lightDir);

    m_lineProgram = createProgram(kLineVertexShader, kLineFragmentShader);
    m_meshProgram = createProgram(kMeshVertexShader, kMeshFragmentShader);

    // Static 1m ground grid (XZ plane, 10m extent).
    std::vector<glm::vec3> grid;
    for (int i = -10; i <= 10; ++i)
    {
        const float f = static_cast<float>(i);
        grid.emplace_back(f, 0.0f, -10.0f);
        grid.emplace_back(f, 0.0f, 10.0f);
        grid.emplace_back(-10.0f, 0.0f, f);
        grid.emplace_back(10.0f, 0.0f, f);
    }
    m_gridVertexCount = static_cast<int>(grid.size());
    glGenVertexArrays(1, &m_gridVao);
    glGenBuffers(1, &m_gridVbo);
    glBindVertexArray(m_gridVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_gridVbo);
    glBufferData(GL_ARRAY_BUFFER, grid.size() * sizeof(glm::vec3), grid.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
    glEnableVertexAttribArray(0);

    // Static pyramid mesh for bones.
    const std::vector<Vertex> pyramid = buildPyramid();
    m_boneVertexCount = static_cast<int>(pyramid.size());
    glGenVertexArrays(1, &m_boneVao);
    glGenBuffers(1, &m_boneVbo);
    glBindVertexArray(m_boneVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_boneVbo);
    glBufferData(GL_ARRAY_BUFFER, pyramid.size() * sizeof(Vertex), pyramid.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, position)));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, normal)));
    glEnableVertexAttribArray(1);

    // Static octahedron mesh for IK target markers.
    const std::vector<Vertex> octahedron = buildOctahedron();
    m_markerVertexCount = static_cast<int>(octahedron.size());
    glGenVertexArrays(1, &m_markerVao);
    glGenBuffers(1, &m_markerVbo);
    glBindVertexArray(m_markerVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_markerVbo);
    glBufferData(GL_ARRAY_BUFFER, octahedron.size() * sizeof(Vertex), octahedron.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, position)));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, normal)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
}

Scene::~Scene()
{
    glDeleteProgram(m_lineProgram);
    glDeleteProgram(m_meshProgram);
    glDeleteBuffers(1, &m_markerVbo);
    glDeleteBuffers(1, &m_boneVbo);
    glDeleteBuffers(1, &m_gridVbo);
    glDeleteVertexArrays(1, &m_markerVao);
    glDeleteVertexArrays(1, &m_boneVao);
    glDeleteVertexArrays(1, &m_gridVao);
}

void Scene::update(GLFWwindow* window, bool allowInput)
{
    double x, y;
    glfwGetCursorPos(window, &x, &y);

    const bool pressed = allowInput && glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    if (pressed)
    {
        if (m_dragging)
        {
            constexpr float sensitivity = 0.01f;
            m_yaw += static_cast<float>(x - m_lastCursorX) * sensitivity;
            m_pitch += static_cast<float>(y - m_lastCursorY) * sensitivity;
            constexpr float limit = glm::half_pi<float>() - 0.05f;
            m_pitch = glm::clamp(m_pitch, -limit, limit);
        }
        m_dragging = true;
    }
    else
    {
        m_dragging = false;
    }

    m_lastCursorX = x;
    m_lastCursorY = y;
}

void Scene::beginFrame(int width, int height)
{
    glViewport(0, 0, width, height);
    glClearColor(0.45f, 0.55f, 0.60f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
}

void Scene::setViewport(int x, int y, int width, int height)
{
    glViewport(x, y, width, height);

    const glm::vec3 offset(
        m_distance * std::cos(m_pitch) * std::sin(m_yaw),
        m_distance * std::sin(m_pitch),
        m_distance * std::cos(m_pitch) * std::cos(m_yaw));
    m_view = glm::lookAt(m_target + offset, m_target, glm::vec3(0.0f, 1.0f, 0.0f));
    m_projection = glm::perspective(glm::radians(45.0f),
        static_cast<float>(width) / static_cast<float>(height), 0.01f, 100.0f);
    m_viewProj = m_projection * m_view;

    drawGrid();
}

void Scene::drawGrid()
{
    glUseProgram(m_lineProgram);
    glUniformMatrix4fv(glGetUniformLocation(m_lineProgram, "uViewProj"), 1, GL_FALSE, &m_viewProj[0][0]);
    glUniform3f(glGetUniformLocation(m_lineProgram, "uColor"), 0.35f, 0.37f, 0.40f);
    glBindVertexArray(m_gridVao);
    glDrawArrays(GL_LINES, 0, m_gridVertexCount);
}

void Scene::renderSkeleton(const Skeleton& skeleton)
{
    // One pyramid per bone: base at the parent joint, apex at the joint. The
    // orientation is the joint's live world frame composed with the (constant
    // per bone) rotation that maps the unit pyramid's +Y onto the bone's rest
    // direction — so the rendered roll follows the joint's real world rotation
    // (twist included) instead of a minimal rotation from +Y, which is
    // degenerate near the -Y antipode where every downward bone lives.
    const std::vector<BoneFrame> frames = computeBoneFrames(skeleton);

    glUseProgram(m_meshProgram);
    glUniformMatrix4fv(glGetUniformLocation(m_meshProgram, "uViewProj"), 1, GL_FALSE, &m_viewProj[0][0]);
    glUniform3f(glGetUniformLocation(m_meshProgram, "uColor"), 0.6f, 0.6f, 0.6f);
    glUniform3fv(glGetUniformLocation(m_meshProgram, "uLightDir"), 1, &m_lightDir[0]);
    glUniform3fv(glGetUniformLocation(m_meshProgram, "uLightColor"), 1, &m_lightColor[0]);
    glBindVertexArray(m_boneVao);
    for (const BoneFrame& f : frames)
    {
        const float radius = 0.08f * f.length;
        const glm::mat4 model = translate(glm::mat4(1.0f), f.base)
            * mat4_cast(f.rotation)
            * scale(glm::mat4(1.0f), glm::vec3(radius, f.length, radius));
        glUniformMatrix4fv(glGetUniformLocation(m_meshProgram, "uModel"), 1, GL_FALSE, &model[0][0]);
        glDrawArrays(GL_TRIANGLES, 0, m_boneVertexCount);
    }

    glBindVertexArray(0);
}

void Scene::renderMarkers(const std::vector<Pose>& poses)
{
    glUseProgram(m_meshProgram);
    glUniformMatrix4fv(glGetUniformLocation(m_meshProgram, "uViewProj"), 1, GL_FALSE, &m_viewProj[0][0]);
    glUniform3f(glGetUniformLocation(m_meshProgram, "uColor"), 0.95f, 0.55f, 0.10f);
    glUniform3fv(glGetUniformLocation(m_meshProgram, "uLightDir"), 1, &m_lightDir[0]);
    glUniform3fv(glGetUniformLocation(m_meshProgram, "uLightColor"), 1, &m_lightColor[0]);
    glBindVertexArray(m_markerVao);
    for (const Pose& pose : poses)
    {
        constexpr float markerRadius = 0.04f;
        const glm::mat4 model = translate(glm::mat4(1.0f), pose.position)
            * mat4_cast(pose.rotation)
            * scale(glm::mat4(1.0f), glm::vec3(markerRadius));
        glUniformMatrix4fv(glGetUniformLocation(m_meshProgram, "uModel"), 1, GL_FALSE, &model[0][0]);
        glDrawArrays(GL_TRIANGLES, 0, m_markerVertexCount);
    }

    glBindVertexArray(0);
}

void Scene::renderTargets(const IkRig& rig)
{
    renderMarkers(targetPoses(rig.targets));
}
