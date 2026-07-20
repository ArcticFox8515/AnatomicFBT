#pragma once

#include <glm/glm.hpp>

class Skeleton;
class IkRig;
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

    // Moves the orbit center (yaw/pitch/distance are unchanged). Used to keep
    // the view centered on the user in VR modes.
    void setCameraTarget(const glm::vec3& target) { m_target = target; }

    // Sets the orbit yaw directly (radians around +Y; the camera looks from
    // direction (sin yaw, 0, cos yaw) toward the target). Calibration mode
    // uses this to face the skeleton's front; any later RMB-drag overrides it.
    void setCameraYaw(float yaw) { m_yaw = yaw; }

    // Clears the full framebuffer (color+depth) and enables depth testing.
    void beginFrame(int width, int height);

    // Sets the GL viewport, computes the camera matrices for its aspect ratio,
    // and draws the ground grid. Call once per viewport; afterwards
    // viewMatrix()/projectionMatrix() refer to this viewport.
    void setViewport(int x, int y, int width, int height);

    // Draws the skeleton: one gray pyramid per bone, base at the parent joint, apex at the joint.
    void renderSkeleton(const Skeleton& skeleton);

    // Draws one small marker per IK target at its world position/rotation.
    void renderTargets(const IkRig& rig);

    // Camera matrices from the last beginFrame(); needed by ImGuizmo.
    const glm::mat4& viewMatrix() const { return m_view; }
    const glm::mat4& projectionMatrix() const { return m_projection; }

private:
    void drawGrid();

    // Orbit camera: orbit center + distance, only yaw/pitch change from mouse.
    glm::vec3 m_target{0.0f, 1.0f, 0.0f};
    float m_distance = 3.5f;
    float m_yaw = 0.0f;
    float m_pitch = 0.3f;
    bool m_dragging = false;
    double m_lastCursorX = 0.0;
    double m_lastCursorY = 0.0;
    glm::mat4 m_view{1.0f};
    glm::mat4 m_projection{1.0f};
    glm::mat4 m_viewProj{1.0f};

    // Single directional light.
    glm::vec3 m_lightDir{-0.4f, -1.0f, -0.25f};
    glm::vec3 m_lightColor{1.0f, 1.0f, 1.0f};

    // GL resources.
    unsigned int m_lineProgram = 0;
    unsigned int m_meshProgram = 0;
    unsigned int m_gridVao = 0, m_gridVbo = 0;      // static 1m ground grid
    unsigned int m_boneVao = 0, m_boneVbo = 0;      // static unit pyramid mesh
    unsigned int m_markerVao = 0, m_markerVbo = 0;  // static unit octahedron mesh
    int m_gridVertexCount = 0;
    int m_boneVertexCount = 0;
    int m_markerVertexCount = 0;
};
