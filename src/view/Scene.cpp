#include "Scene.h"

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "model/Skeleton.h"

namespace
{
const char* kLineVertexShader = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
uniform mat4 uViewProj;
void main()
{
    gl_Position = uViewProj * vec4(aPos, 1.0);
}
)";

const char* kLineFragmentShader = R"(
#version 330 core
uniform vec3 uColor;
out vec4 FragColor;
void main()
{
    FragColor = vec4(uColor, 1.0);
}
)";

const char* kMeshVertexShader = R"(
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

const char* kMeshFragmentShader = R"(
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

// Octahedron with per-face normals (vertices duplicated per face).
std::vector<float> buildOctahedron()
{
    const glm::vec3 top(0.0f, 1.0f, 0.0f), bottom(0.0f, -1.0f, 0.0f);
    const glm::vec3 px(1.0f, 0.0f, 0.0f), nx(-1.0f, 0.0f, 0.0f);
    const glm::vec3 pz(0.0f, 0.0f, 1.0f), nz(0.0f, 0.0f, -1.0f);
    const glm::vec3 faces[8][3] = {
        {top, pz, px}, {top, px, nz}, {top, nz, nx}, {top, nx, pz},
        {bottom, px, pz}, {bottom, nx, px}, {bottom, pz, nx}, {bottom, nz, px},
    };

    std::vector<float> data;
    data.reserve(8 * 3 * 6);
    for (const auto& face : faces)
    {
        glm::vec3 normal = glm::normalize(glm::cross(face[1] - face[0], face[2] - face[0]));
        const glm::vec3 center = (face[0] + face[1] + face[2]) / 3.0f;
        if (glm::dot(normal, center) < 0.0f)
            normal = -normal;
        for (const glm::vec3& v : face)
        {
            data.insert(data.end(), {v.x, v.y, v.z, normal.x, normal.y, normal.z});
        }
    }
    return data;
}
} // namespace

Scene::Scene()
{
    m_lightDir = glm::normalize(m_lightDir);

    m_lineProgram = createProgram(kLineVertexShader, kLineFragmentShader);
    m_meshProgram = createProgram(kMeshVertexShader, kMeshFragmentShader);

    // Dynamic bone line buffer.
    glGenVertexArrays(1, &m_boneVao);
    glGenBuffers(1, &m_boneVbo);
    glBindVertexArray(m_boneVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_boneVbo);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
    glEnableVertexAttribArray(0);

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

    // Static octahedron mesh for joints.
    const std::vector<float> octahedron = buildOctahedron();
    m_jointVertexCount = static_cast<int>(octahedron.size() / 6);
    glGenVertexArrays(1, &m_jointVao);
    glGenBuffers(1, &m_jointVbo);
    glBindVertexArray(m_jointVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_jointVbo);
    glBufferData(GL_ARRAY_BUFFER, octahedron.size() * sizeof(float), octahedron.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
}

Scene::~Scene()
{
    glDeleteProgram(m_lineProgram);
    glDeleteProgram(m_meshProgram);
    glDeleteBuffers(1, &m_boneVbo);
    glDeleteBuffers(1, &m_gridVbo);
    glDeleteBuffers(1, &m_jointVbo);
    glDeleteVertexArrays(1, &m_boneVao);
    glDeleteVertexArrays(1, &m_gridVao);
    glDeleteVertexArrays(1, &m_jointVao);
}

void Scene::update(GLFWwindow* window, bool allowInput)
{
    double x, y;
    glfwGetCursorPos(window, &x, &y);

    const bool pressed = allowInput && glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    if (pressed)
    {
        if (m_dragging)
        {
            const float sensitivity = 0.01f;
            m_yaw += static_cast<float>(x - m_lastCursorX) * sensitivity;
            m_pitch += static_cast<float>(y - m_lastCursorY) * sensitivity;
            const float limit = glm::half_pi<float>() - 0.05f;
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

    const glm::vec3 offset(
        m_distance * std::cos(m_pitch) * std::sin(m_yaw),
        m_distance * std::sin(m_pitch),
        m_distance * std::cos(m_pitch) * std::cos(m_yaw));
    const glm::mat4 view = glm::lookAt(m_target + offset, m_target, glm::vec3(0.0f, 1.0f, 0.0f));
    const glm::mat4 projection = glm::perspective(glm::radians(45.0f),
        static_cast<float>(width) / static_cast<float>(height), 0.01f, 100.0f);
    m_viewProj = projection * view;

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
    const std::vector<glm::vec3> positions = computeWorldPositions(skeleton);

    // Bone lines, rebuilt per frame.
    std::vector<glm::vec3> lines;
    for (size_t i = 0; i < skeleton.joints.size(); ++i)
    {
        if (skeleton.joints[i].parentIndex < 0)
            continue;
        lines.push_back(positions[skeleton.joints[i].parentIndex]);
        lines.push_back(positions[i]);
    }
    if (!lines.empty())
    {
        glUseProgram(m_lineProgram);
        glUniformMatrix4fv(glGetUniformLocation(m_lineProgram, "uViewProj"), 1, GL_FALSE, &m_viewProj[0][0]);
        glUniform3f(glGetUniformLocation(m_lineProgram, "uColor"), 0.95f, 0.95f, 0.95f);
        glBindVertexArray(m_boneVao);
        glBindBuffer(GL_ARRAY_BUFFER, m_boneVbo);
        glBufferData(GL_ARRAY_BUFFER, lines.size() * sizeof(glm::vec3), lines.data(), GL_DYNAMIC_DRAW);
        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(lines.size()));
    }

    // Joint octahedra.
    glUseProgram(m_meshProgram);
    glUniformMatrix4fv(glGetUniformLocation(m_meshProgram, "uViewProj"), 1, GL_FALSE, &m_viewProj[0][0]);
    glUniform3f(glGetUniformLocation(m_meshProgram, "uColor"), 0.85f, 0.35f, 0.30f);
    glUniform3fv(glGetUniformLocation(m_meshProgram, "uLightDir"), 1, &m_lightDir[0]);
    glUniform3fv(glGetUniformLocation(m_meshProgram, "uLightColor"), 1, &m_lightColor[0]);
    glBindVertexArray(m_jointVao);
    for (const glm::vec3& position : positions)
    {
        const glm::mat4 model = glm::translate(glm::mat4(1.0f), position) * glm::scale(glm::mat4(1.0f), glm::vec3(0.035f));
        glUniformMatrix4fv(glGetUniformLocation(m_meshProgram, "uModel"), 1, GL_FALSE, &model[0][0]);
        glDrawArrays(GL_TRIANGLES, 0, m_jointVertexCount);
    }

    glBindVertexArray(0);
}
