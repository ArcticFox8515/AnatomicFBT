#include <cstdlib>
#include <exception>
#include <fstream>
#include <string>
#include <vector>
#include <imgui.h>
#include <ImGuizmo.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include "bindings/imgui_impl_glfw.h"
#include "bindings/imgui_impl_opengl3.h"
#include "model/IkRig.h"
#include "model/IkRigConfig.h"
#include "model/Pose.h"
#include "model/Retarget.h"
#include "model/Skeleton.h"
#include "model/TrackerCalibration.h"
#include "view/Scene.h"
#include "vr/OpenVrTracking.h"

constexpr char kSkeletonPath[] = "user-skeleton.json";
constexpr char kIkRigPath[] = "user-ikrig.json";
constexpr char kAvatarSkeletonPath[] = "user-avatar-skeleton.json";

// Application mode: ManualPose = gizmo-dragged targets (no VR input);
// Calibration = targets mirror raw device poses while the skeleton rests,
// until both triggers freeze device->target offsets; Capture = offsets
// applied every frame, IK solver active.
enum class Mode
{
	ManualPose,
	Calibration,
	Capture
};

// Loads T from a JSON file; creates the file with the default if missing;
// falls back to the default (in memory only) if the file exists but is
// invalid. Any failure is logged with the full exception detail in what().
template <typename T>
static T loadOrCreate(const char* path, T (*makeDefault)())
{
	std::ifstream file(path);
	if (file)
	{
		try
		{
			nlohmann::json j = nlohmann::json::parse(file);
			T value = j.get<T>();

			spdlog::info("Loaded {}", path);
			return value;
		}
		catch (const std::exception& e)
		{
			spdlog::error("Failed to load {}; using defaults: {}", path, e.what());
			return makeDefault();
		}
	}

	T value = makeDefault();
	std::ofstream out(path);
	out << nlohmann::json(value).dump(2) << '\n';
	spdlog::info("Created default {}", path);
	return value;
}

// Draws one ImGuizmo per IK target; ImGuizmo::SetID activates whichever the mouse grabs.
static void manipulateTargets(IkRig& rig, const glm::mat4& view, const glm::mat4& projection,
                              ImGuizmo::OPERATION operation)
{
	for (size_t i = 0; i < rig.targets.size(); ++i)
	{
		IkTarget& target = rig.targets[i];
		ImGuizmo::SetID(static_cast<int>(i));
		glm::mat4 matrix = glm::translate(glm::mat4(1.0f), target.position) * glm::mat4_cast(target.rotation);
		if (ImGuizmo::Manipulate(glm::value_ptr(view), glm::value_ptr(projection),
		                         operation, ImGuizmo::WORLD, glm::value_ptr(matrix)))
		{
			glm::vec3 translation, rotation, scale;
			ImGuizmo::DecomposeMatrixToComponents(glm::value_ptr(matrix),
			                                      glm::value_ptr(translation), glm::value_ptr(rotation), glm::value_ptr(scale));
			target.position = translation;
			target.rotation = glm::normalize(glm::quat(glm::radians(rotation)));
		}
	}
}

// First HMD in a device snapshot list, or nullptr.
static const VrDeviceSnapshot* findHmd(const std::vector<VrDeviceSnapshot>& devices)
{
	for (const VrDeviceSnapshot& device : devices)
		if (device.kind == VrDeviceKind::Hmd)
			return &device;
	return nullptr;
}

static const char* deviceKindName(VrDeviceKind kind)
{
	switch (kind)
	{
	case VrDeviceKind::Hmd:
		return "HMD";
	case VrDeviceKind::Controller:
		return "controller";
	case VrDeviceKind::Tracker:
		return "tracker";
	default:
		return "device";
	}
}

// Packs the snapshot list into (deviceId, pose) pairs — the device id space
// TrackerCalibration binds against.
static std::vector<std::pair<int, Pose>> devicePosePairs(const std::vector<VrDeviceSnapshot>& devices)
{
	std::vector<std::pair<int, Pose>> pairs;
	pairs.reserve(devices.size());
	for (const VrDeviceSnapshot& device : devices)
		pairs.emplace_back(device.deviceIndex, device.pose);
	return pairs;
}

// Calibration mode: the skeleton rests (no IK) with its root (head) aligned
// to the HMD's position and heading; targets mirror the raw poses of the
// devices the proximity assignment picked for them. Fills liveAssignment and
// lastDevices for UI display. Returns true when the user pressed both
// triggers and offsets were captured (caller switches to Capture mode).
static bool updateCalibration(IkRig& rig, OpenVrTracking& vr, TrackerCalibration& calibration,
                              int rootJointIndex, DeviceAssignment& liveAssignment,
                              std::vector<VrDeviceSnapshot>& lastDevices)
{
	lastDevices = vr.pollPoses();
	const std::vector<VrDeviceSnapshot>& devices = lastDevices;

	for (Joint& joint : rig.skeleton.joints)
		joint.localRot = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
	const VrDeviceSnapshot* hmd = findHmd(devices);
	if (hmd)
	{
		rig.skeleton.rootPosition = hmd->pose.position;
		rig.skeleton.joints[rootJointIndex].localRot = yawOnly(hmd->pose.rotation);
	}

	const WorldTransforms wt = computeWorldTransforms(rig.skeleton);
	std::vector<Pose> bonePoses(rig.targets.size());
	std::vector<glm::vec3> bonePositions(rig.targets.size());
	for (size_t i = 0; i < rig.targets.size(); ++i)
	{
		bonePoses[i] = {wt.positions[rig.targets[i].jointIndex], wt.rotations[rig.targets[i].jointIndex]};
		bonePositions[i] = bonePoses[i].position;
	}

	std::vector<glm::vec3> devicePositions;
	devicePositions.reserve(devices.size());
	for (const VrDeviceSnapshot& device : devices)
		devicePositions.push_back(device.pose.position);

	liveAssignment = assignDevicesToTargets(devicePositions, bonePositions);

	// Show raw device poses at the target markers so the user sees what will
	// be captured.
	for (size_t i = 0; i < rig.targets.size(); ++i)
	{
		const int d = liveAssignment.deviceIndex[i];
		if (d < 0)
			continue;
		rig.targets[i].position = devices[d].pose.position;
		rig.targets[i].rotation = devices[d].pose.rotation;
	}

	if (!vr.bothTriggersJustPressed())
		return false;

	// Translate compact snapshot indices to stable OpenVR device indices.
	for (int& d : liveAssignment.deviceIndex)
		if (d >= 0)
			d = devices[d].deviceIndex;
	calibration.calibrate(liveAssignment, devicePosePairs(devices), bonePoses);
	return true;
}

// Capture mode: targets mirror the raw device poses (what gets rendered,
// same as calibration); the solver goals are a copy with the calibrated
// offsets applied. Fills lastDevices for UI display and camera centering.
static void updateCapture(IkRig& rig, OpenVrTracking& vr, const TrackerCalibration& calibration,
                          std::vector<VrDeviceSnapshot>& lastDevices,
                          std::vector<IkTarget>& captureGoals)
{
	lastDevices = vr.pollPoses();
	calibration.applyDevicePoses(devicePosePairs(lastDevices), rig.targets);
	captureGoals = rig.targets;
	calibration.applyOffsets(captureGoals);
}

int WinMain(void* hinst, void* hprev, char* cmdline, int show)
{
	spdlog::set_default_logger(
		spdlog::rotating_logger_mt("tc", "logs/trackingcorrector.log", 5 * 1024 * 1024, 3));
	spdlog::flush_on(spdlog::level::err);

	if (!glfwInit())
	{
		return 1;
	}

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	GLFWwindow* window = glfwCreateWindow(1280, 720, "TrackingCorrector", nullptr, nullptr);
	if (!window)
	{
		glfwTerminate();
		return 1;
	}

	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	if (glewInit() != GLEW_OK)
	{
		glfwTerminate();
		return 1;
	}

	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	const ImGuiIO& io = ImGui::GetIO();
	// Setup Platform/Renderer bindings
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init("#version 330 core");
	// Setup Dear ImGui style
	ImGui::StyleColorsDark();

	const Skeleton skeleton = loadOrCreate<Skeleton>(kSkeletonPath, Skeleton::makeDefault);
	IkRig rig(skeleton);
	try
	{
		rig.loadConfig(loadOrCreate<IkRigConfig>(kIkRigPath, IkRigConfig::makeDefault));
	}
	catch (const std::exception& e)
	{
		spdlog::error("IK rig config invalid; using default IK rig: {}", e.what());
		try
		{
			rig.loadConfig(IkRigConfig::makeDefault());
		}
		catch (const std::exception& e2)
		{
			spdlog::error("Default IK rig does not fit the skeleton; running without IK targets: {}", e2.what());
		}
	}

	// The avatar skeleton the solved pose is retargeted onto (hip-rooted by
	// default, like VRChat/Unity avatars).
	Skeleton avatarSkeleton = loadOrCreate<Skeleton>(kAvatarSkeletonPath, Skeleton::makeDefaultHipRooted);
	const RetargetMap retargetMap = buildRetargetMap(rig.skeleton, avatarSkeleton);
	const std::vector<std::string> unmatched = unmatchedBones(avatarSkeleton, retargetMap);
	if (!unmatched.empty())
	{
		std::string names;
		for (const std::string& name : unmatched)
		{
			if (!names.empty())
				names += ", ";
			names += name;
		}
		spdlog::info("Avatar bones without a source match (kept at rest): {}", names);
	}

	// OpenVR: when SteamVR is running, start straight in calibration mode;
	// otherwise fall back to manual pose (the UI offers a retry).
	OpenVrTracking vr;
	Mode mode = Mode::ManualPose;
	try
	{
		vr.init();
		spdlog::info("OpenVR initialized; starting in calibration mode");
		mode = Mode::Calibration;
	}
	catch (const std::exception& e)
	{
		spdlog::error("OpenVR unavailable; starting in manual pose mode: {}", e.what());
	}

	TrackerCalibration calibration;
	DeviceAssignment liveAssignment;
	std::vector<VrDeviceSnapshot> lastDevices;
	std::vector<IkTarget> captureGoals;  // targets + calibration offsets, Capture-mode solver input

	int rootJointIndex = 0;
	for (size_t i = 0; i < rig.skeleton.joints.size(); ++i)
		if (!rig.skeleton.joints[i].parentIndex)
		{
			rootJointIndex = static_cast<int>(i);
			break;
		}

	// Scene holds GL resources; scope it so it is destroyed before GLFW shutdown.
	{
		Scene scene;
		ImGuizmo::OPERATION gizmoOperation = ImGuizmo::TRANSLATE;

		while (!glfwWindowShouldClose(window))
		{
			int width, height;
			glfwGetFramebufferSize(window, &width, &height);
			if (width <= 0 || height <= 0)
			{
				glfwWaitEventsTimeout(1.0 / 90);
				continue;
			}
			glfwPollEvents();

			// feed inputs to dear imgui, start new frame
			ImGui_ImplOpenGL3_NewFrame();
			ImGui_ImplGlfw_NewFrame();
			ImGui::NewFrame();
			ImGuizmo::BeginFrame();

			const bool gizmoBusy = ImGuizmo::IsUsingAny() || ImGuizmo::IsOver();
			scene.update(window, !io.WantCaptureMouse && !gizmoBusy);
			scene.beginFrame(width, height);

			// VR-driven modes move the targets (calibration also re-poses the
			// resting skeleton); manual pose leaves them to the gizmos.
			if (mode == Mode::Calibration && vr.isInitialized())
			{
				if (updateCalibration(rig, vr, calibration, rootJointIndex, liveAssignment, lastDevices))
				{
					mode = Mode::Capture;
					spdlog::info("Calibration captured; entering capture mode");
				}
			}
			else if (mode == Mode::Capture && vr.isInitialized())
			{
				updateCapture(rig, vr, calibration, lastDevices, captureGoals);
			}

			// In VR modes keep the orbit camera centered on the user's XZ
			// position so the skeleton doesn't walk out of view.
			if (mode == Mode::ManualPose)
			{
				scene.setCameraTarget(glm::vec3(0.0f, 1.0f, 0.0f));
			}
			else if (const VrDeviceSnapshot* hmd = findHmd(lastDevices))
			{
				scene.setCameraTarget(glm::vec3(hmd->pose.position.x, 1.0f, hmd->pose.position.z));
			}

			// Left half: the IK-driven skeleton with its gizmo targets.
			const int leftWidth = width / 2;
			scene.setViewport(0, 0, leftWidth, height);
			ImGuizmo::SetOrthographic(false);
			ImGuizmo::SetRect(0.0f, 0.0f, static_cast<float>(leftWidth), static_cast<float>(height));
			if (mode == Mode::ManualPose)
			{
				manipulateTargets(rig, scene.viewMatrix(), scene.projectionMatrix(), gizmoOperation);
				rig.solve();
			}
			else if (mode == Mode::Capture)
			{
				rig.solve(captureGoals);
			}
			scene.renderSkeleton(rig.skeleton);
			scene.renderTargets(rig);

			// Right half: the avatar skeleton with the pose retargeted onto it.
			retargetPose(rig.skeleton, avatarSkeleton, retargetMap);
			scene.setViewport(leftWidth, 0, width - leftWidth, height);
			scene.renderSkeleton(avatarSkeleton);

			ImGui::Begin("TrackingCorrector");
			ImGui::Text("ImGui initialized. %.1f FPS", io.Framerate);

			ImGui::SeparatorText("Mode");
			const char* modeName = mode == Mode::ManualPose ? "Manual pose"
				: mode == Mode::Calibration           ? "Calibration"
				                                      : "Capture";
			ImGui::Text("Mode: %s", modeName);
			ImGui::Text("SteamVR: %s", vr.isInitialized() ? "connected" : "not connected");
			if (mode != Mode::ManualPose && ImGui::Button("Switch to manual pose"))
				mode = Mode::ManualPose;
			if (mode != Mode::Calibration && ImGui::Button("Switch to calibration"))
			{
				if (!vr.isInitialized())
				{
					try
					{
						vr.init();
						spdlog::info("OpenVR initialized; entering calibration mode");
					}
					catch (const std::exception& e)
					{
						spdlog::error("OpenVR still unavailable: {}", e.what());
					}
				}
				if (vr.isInitialized())
				{
					calibration.clear();
					mode = Mode::Calibration;
				}
			}
			if (mode == Mode::Calibration)
			{
				ImGui::TextWrapped("Stand in T-pose and press both triggers to start capture.");
				for (size_t i = 0; i < rig.targets.size(); ++i)
				{
					const int d = i < liveAssignment.deviceIndex.size() ? liveAssignment.deviceIndex[i] : -1;
					if (d >= 0 && static_cast<size_t>(d) < lastDevices.size())
						ImGui::BulletText("%s <- %s %d", rig.targetName(i).c_str(),
						                  deviceKindName(lastDevices[d].kind), lastDevices[d].deviceIndex);
					else
						ImGui::BulletText("%s <- (no device)", rig.targetName(i).c_str());
				}
			}

			ImGui::SeparatorText("Gizmo mode");
			bool translate = gizmoOperation == ImGuizmo::TRANSLATE;
			if (ImGui::RadioButton("Translate (T)", translate))
				gizmoOperation = ImGuizmo::TRANSLATE;
			ImGui::SameLine();
			if (ImGui::RadioButton("Rotate (R)", !translate))
				gizmoOperation = ImGuizmo::ROTATE;
			if (!io.WantCaptureKeyboard)
			{
				if (ImGui::IsKeyPressed(ImGuiKey_T, false))
					gizmoOperation = ImGuizmo::TRANSLATE;
				if (ImGui::IsKeyPressed(ImGuiKey_R, false))
					gizmoOperation = ImGuizmo::ROTATE;
			}

			ImGui::SeparatorText("IK targets");
			for (size_t i = 0; i < rig.targets.size(); ++i)
			{
				IkTarget& target = rig.targets[i];
				ImGui::PushID(static_cast<int>(i));
				if (ImGui::TreeNode(rig.targetName(i).c_str()))
				{
					ImGui::DragFloat3("Position", &target.position.x, 0.01f);
					glm::vec3 eulerDeg = glm::degrees(glm::eulerAngles(target.rotation));
					if (ImGui::DragFloat3("Rotation (deg)", &eulerDeg.x, 0.5f))
						target.rotation = glm::normalize(glm::quat(glm::radians(eulerDeg)));
					ImGui::TreePop();
				}
				ImGui::PopID();
			}
			if (ImGui::Button("Reset targets"))
				rig.resetTargets();
			ImGui::End();

			// Render dear imgui into screen
			ImGui::Render();
			ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

			glfwSwapBuffers(window);
		}
	}

	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
	glfwTerminate();
	return EXIT_SUCCESS;
}
