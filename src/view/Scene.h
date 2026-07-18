#pragma once

#include <glm/glm.hpp>

class Skeleton;
struct GLFWwindow;

// Owns the camera, the light, and everything needed to draw the scene.
class Scene
{
public:
    Scene();
    ~Scene();

    Scene(const Scene&) = delete;
    Scene& operator=(const Scene&) = delete;

    // Orbit camera from mouse input. allowInput should be false while ImGui captures the mouse.
    void update(GLFWwindow* window, bool allowInput);

    // Sets viewport, clears color+depth, computes camera matrices, draws the ground grid.
    void beginFrame(int width, int height);

    // Draws the skeleton: line segments for bones, octahedra at joints.
    void renderSkeleton(const Skeleton& skeleton);

private:
    void drawGrid();

    // Orbit camera: fixed target and distance, only yaw/pitch change.
    glm::vec3 m_target{0.0f, 1.0f, 0.0f};
    float m_distance = 3.5f;
    float m_yaw = 0.0f;
    float m_pitch = 0.3f;
    bool m_dragging = false;
    double m_lastCursorX = 0.0;
    double m_lastCursorY = 0.0;
    glm::mat4 m_viewProj{1.0f};

    // Single directional light.
    glm::vec3 m_lightDir{-0.4f, -1.0f, -0.25f};
    glm::vec3 m_lightColor{1.0f, 1.0f, 1.0f};

    // GL resources.
    unsigned int m_lineProgram = 0;
    unsigned int m_meshProgram = 0;
    unsigned int m_boneVao = 0, m_boneVbo = 0;    // dynamic line segments, rebuilt per frame
    unsigned int m_gridVao = 0, m_gridVbo = 0;    // static 1m ground grid
    unsigned int m_jointVao = 0, m_jointVbo = 0;  // static octahedron mesh
    int m_gridVertexCount = 0;
    int m_jointVertexCount = 0;
};
