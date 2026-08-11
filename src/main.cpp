#include <cfloat>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

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
#include "model/BodyProportions.h"
#include "model/AppSettings.h"
#include "model/FrameTick.h"
#include "model/GripOffsets.h"
#include "model/IkRig.h"
#include "model/IkRigConfig.h"
#include "model/ModeController.h"
#include "model/OpenVrTracking.h"
#include "model/Pose.h"
#include "model/ReplaySession.h"
#include "model/Retarget.h"
#include "model/SessionRecorder.h"
#include "model/Skeleton.h"
#include "model/TrackedDevice.h"
#include "model/TrackerCalibration.h"
#include "model/TrackerCorrection.h"
#include "model/VirtualTrackers.h"
#include "view/Scene.h"
#include "vr/OpenVrInput.h"
#include "link/Log.h"
#include "pipe/Win32Pipe.h"

constexpr char kProportionsPath[] = "user-proportions.json";
constexpr char kIkRigPath[] = "user-ikrig.json";
constexpr char kAvatarSkeletonPath[] = "user-avatar-skeleton.json";
constexpr char kSettingsPath[] = "user-settings.json";

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

// Writes T to a JSON file (pretty). Used to persist user-edited settings
// immediately on change so a tick survives a restart. Failures are logged
// and swallowed — a dropped save is not fatal.
template <typename T>
static void saveToFile(const char* path, const T& value)
{
	std::ofstream out(path);
	out << nlohmann::json(value).dump(2) << '\n';
	if (!out)
		spdlog::error("Failed to save {}", path);
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

	// The IK skeleton is the fixed default hierarchy scaled to the user's
	// measured body proportions (user-proportions.json).
	const Skeleton skeleton = Skeleton::makeDefault(loadOrCreate<BodyProportions>(kProportionsPath, &BodyProportions::makeDefault));
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

	// App settings (per-bone virtual-tracker selection today; shaped for more
	// later). Load-or-create, same convention as the other configs.
	AppSettings settings = loadOrCreate<AppSettings>(kSettingsPath, &AppSettings::makeDefault);
	// Scale the avatar to the reference skeleton's rest height (Head-to-feet
	// Y span). Without this, retargeting a 1.8 m body onto a 1.5 m avatar
	// squashes the pose; with it, overall height is preserved and correction
	// only redistributes proportions. The avatar JSON stays as-authored — the
	// scale is applied in memory, once. Must happen before the retarget and
	// correction maps are built (they are name-based, so scaling first changes
	// nothing about them).
	const float refHeight = restHeight(rig.skeleton);
	const float avatarHeight = restHeight(avatarSkeleton);
	const float avatarScale = matchRestHeight(rig.skeleton, avatarSkeleton);
	if (avatarScale != 1.0f)
		spdlog::info("Avatar height-scaled to match reference: {:.2f} (ref {:.2f} m / avatar {:.2f} m)",
		             avatarScale, refHeight, avatarHeight);
	else if (refHeight <= 0.0f || avatarHeight <= 0.0f)
		spdlog::warn("Avatar height scale skipped: reference ({:.2f}) or avatar ({:.2f}) rest height unusable "
		             "(missing Head or both feet joints)", refHeight, avatarHeight);
	const RetargetMap retargetMap = buildRetargetMap(rig.skeleton, avatarSkeleton);
	// Per-target correction toggle + avatar-joint map. Built once: the rig
	// config is loaded once at startup and the avatar skeleton does not
	// change at runtime. `enabled` is mutated by the ImGui checkboxes below,
	// so this is not const.
	CorrectionMap correctionMap = buildCorrectionMap(rig, avatarSkeleton);
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

	// Poses come from the SteamVR driver over a named pipe (doc/driver-plan.md
	// phase A, step 4); the link logger routes the channel's lines into spdlog.
	// The pipe factory is the only system-dependent piece (a Win32ClientPipe on
	// the driver's pipe name); the clock paces reconnect attempts at 1/s while
	// the driver is absent.
	link::Logger linkLog;
	linkLog.setSink([](const char* message) { spdlog::info("driver link: {}", message); });
	OpenVrTracking vr(linkLog,
	                  [] { return std::make_shared<link::Win32ClientPipe>(link::kDriverPipeName); },
	                  [] { return glfwGetTime(); });

	// Frame-tick event logger (calibration capture, recording start/stop/error).
	// Same pattern as the driver link logger above: the model is log-free, so
	// pollAndUpdate reports through a link::Logger whose sink forwards to spdlog.
	link::Logger frameLog;
	frameLog.setSink([](const char* message) { spdlog::info(message); });

	// The trigger reader is on demand: VR_Init/VR_Shutdown follow Calibration
	// mode (no driver-side input source exists). Held as unique_ptr so the
	// session ends the moment Calibration is left, including the automatic
	// Calibration->Capture transition.
	std::unique_ptr<OpenVrInput> input;

	// Per-controller grip offsets (resolved once per Calibration session by
	// OpenVrInput's render-model query; doc/ik-improvements-plan.md). Must
	// outlive `input`: the automatic Calibration->Capture transition tears the
	// trigger reader down, but the offsets apply to every Capture frame and
	// every replay, so the cache lives here at app scope. `mergeGripOffsets`
	// keeps a controller's entry across a second calibration if it is absent
	// from the fresh query (powered off mid-session).
	std::vector<GripOffset> gripOffsets;

	auto refreshGripOffsets = [&]()
	{
		if (!input || !input->isInitialized())
			return;
		std::vector<GripOffset> fresh;
		fresh.reserve(input->gripOffsets().size());
		for (const OpenVrInput::GripOffsetInfo& info : input->gripOffsets())
		{
			spdlog::info("controller {} render-model=\"{}\" component=\"{}\" grip pos=({:.4f},{:.4f},{:.4f})",
			             info.deviceId, info.renderModelName, info.componentName,
			             info.offset.deviceToGrip.position.x,
			             info.offset.deviceToGrip.position.y,
			             info.offset.deviceToGrip.position.z);
			fresh.push_back(info.offset);
		}
		gripOffsets = mergeGripOffsets(gripOffsets, fresh);
	};

	// OpenVR: when the driver pipe is connected and the trigger session comes
	// up, start straight in calibration mode; otherwise fall back to manual
	// pose (the UI offers a retry).
	Mode initialMode = Mode::ManualPose;
	try
	{
		vr.init();
		input = std::make_unique<OpenVrInput>();
		input->init();
		refreshGripOffsets();
		spdlog::info("OpenVR initialized; starting in calibration mode");
		initialMode = Mode::Calibration;
	}
	catch (const std::exception& e)
	{
		spdlog::error("OpenVR unavailable; starting in manual pose mode: {}", e.what());
		input.reset();
	}

	// The mode state machine (calibration/capture logic) lives in the model
	// layer; this loop only wires VR input in and executes the returned plan.
	ModeController controller(initialMode);

	// Capture sessions are always recorded to kRecordingPath (overwritten per
	// session — copy/rename the file to keep it). The recorder lifecycle
	// lives in the model layer; this only supplies the file stream and logs.
	SessionRecorder recorder([]() -> std::shared_ptr<std::ostream>
	{
		return std::make_shared<std::ofstream>(kRecordingPath, std::ios::binary | std::ios::trunc);
	});

	// Replay-mode state (file list, loaded recording, timeline frame) lives
	// in the model layer; this only binds the UI to it and logs its errors.
	ReplaySession replay;

	auto loadReplayFile = [&](size_t index)
	{
		try
		{
			replay.load(index, controller, rig);
			spdlog::info("Loaded recording {} ({} frames, {:.2f} s)", replay.files()[index],
			             replay.recording().frames.size(), replay.recording().duration());
		}
		catch (const std::exception& e)
		{
			spdlog::error("Failed to load recording {}: {}", replay.files()[index], e.what());
		}
	};

	auto enterReplay = [&]()
	{
		controller.switchToReplay();
		rig.resetTargets();
		try
		{
			replay.scan(".");
		}
		catch (const std::exception& e)
		{
			spdlog::error("{}", e.what());
		}
		if (!replay.files().empty())
			loadReplayFile(0);
	};

	// Scene holds GL resources; scope it so it is destroyed before GLFW shutdown.
	{
		Scene scene;
		ImGuizmo::OPERATION gizmoOperation = ImGuizmo::TRANSLATE;

		while (!glfwWindowShouldClose(window))
		{
			int width, height;
			glfwGetFramebufferSize(window, &width, &height);
			const bool minimized = width <= 0 || height <= 0;
			if (minimized)
				glfwWaitEventsTimeout(1.0 / 90);
			else
				glfwPollEvents();

			// Shared head: runs every frame, minimized or not, so the driver
			// link stays fed (pollPoses pumps the channel, sendOffsets ships
			// overrides). See FrameTick. The model is log-free and owns no VR
			// session, so the two action signals are handled here: reset the
			// trigger reader when Calibration is over; events (capture,
			// recording start/stop/error) flow through the sink into spdlog.
			const UpdateResult tick =
				pollAndUpdate(controller, rig, &replay, recorder, vr, input.get(), glfwGetTime(),
				              gripOffsets, frameLog);
			if (tick.tearDownGestureSource)
				input.reset();

			if (minimized)
			{
				// Goals-only: ManualPose ships no overrides and its Targets
				// solve is gizmo-driven (render-interleaved), so there is
				// nothing to do for it while minimized. The driver link stays
				// fed (overrides + virtual trackers) by retargetAndShip below;
				// VTs ship only in Capture after calibration (step 5), so the
				// minimized path keeps them flowing exactly when the visible
				// path would.
				if (tick.plan.solve == SolveMode::Goals)
				{
					rig.solve(tick.plan.goals);
					retargetAndShip(rig, avatarSkeleton, retargetMap, correctionMap,
					                controller.calibration(), controller.mode(),
					                tick.devices, vr, settings.virtualTrackerBones);
				}
				continue;
			}

			// feed inputs to dear imgui, start new frame
			ImGui_ImplOpenGL3_NewFrame();
			ImGui_ImplGlfw_NewFrame();
			ImGui::NewFrame();
			ImGuizmo::BeginFrame();

			const bool gizmoBusy = ImGuizmo::IsUsingAny() || ImGuizmo::IsOver();
			scene.update(window, !io.WantCaptureMouse && !gizmoBusy);
			scene.beginFrame(width, height);

			const std::vector<TrackedDevice>& devices = tick.devices;
			const FramePlan& plan = tick.plan;
			// Corrected tracker poses for the right viewport + the ImGui
			// readout; computed after retargetPose runs (below). Empty when
			// uncalibrated or no target is enabled.
			RetargetResult retargetResult;

			// In VR modes keep the orbit camera centered on the user's XZ
			// position so the skeleton doesn't walk out of view. Calibration
			// additionally turns the camera to look at the skeleton from the
			// front (its facing = the HMD heading), so tracker assignment is
			// easy to check while T-posing.
			if (controller.mode() == Mode::ManualPose)
			{
				scene.setCameraTarget(glm::vec3(0.0f, 1.0f, 0.0f));
			}
			else if (const TrackedDevice* hmd = findHmd(devices))
			{
				scene.setCameraTarget(glm::vec3(hmd->pose.position.x, 1.0f, hmd->pose.position.z));
				if (controller.mode() == Mode::Calibration)
				{
					const glm::vec3 facing = yawOnly(hmd->pose.rotation) * glm::vec3(0.0f, 0.0f, -1.0f);
					scene.setCameraYaw(std::atan2(facing.x, facing.z));
				}
			}

			// Left half: the IK-driven skeleton with its gizmo targets.
			const int leftWidth = width / 2;
			scene.setViewport(0, 0, leftWidth, height);
			ImGuizmo::SetOrthographic(false);
			ImGuizmo::SetRect(0.0f, 0.0f, static_cast<float>(leftWidth), static_cast<float>(height));
			if (plan.solve == SolveMode::Targets)
			{
				manipulateTargets(rig, scene.viewMatrix(), scene.projectionMatrix(), gizmoOperation);
				rig.solve();
			}
			else if (plan.solve == SolveMode::Goals)
			{
				rig.solve(plan.goals);
			}
			scene.renderSkeleton(rig.skeleton);
			scene.renderTargets(rig);

		// Right half: the avatar skeleton with the pose retargeted onto it,
		// plus the corrected tracker poses re-hung on its bones (see
		// retargetAndShip in FrameTick). Each corrected tracker is placed at
		// the avatar joint's world pose (joint center, no strap offset);
		// controllers keep their raw rotation (aiming must not change) so
		// only their position is corrected. The corrected poses are rendered
		// as markers. Virtual tracker poses come back in the same result,
		// computed from the retargeted avatar, so the wiring is one call —
		// no separate VT path to forget.
	retargetResult = retargetAndShip(rig, avatarSkeleton, retargetMap, correctionMap,
	                            controller.calibration(), controller.mode(),
	                            devices, vr, settings.virtualTrackerBones);
		scene.setViewport(leftWidth, 0, width - leftWidth, height);
		scene.renderSkeleton(avatarSkeleton);
		if (!retargetResult.corrected.empty())
		{
			std::vector<Pose> correctedPoses;
			correctedPoses.reserve(retargetResult.corrected.size());
			for (const CorrectedPose& c : retargetResult.corrected)
				correctedPoses.push_back(c.pose);
			scene.renderMarkers(correctedPoses);
		}
		scene.renderMarkers(retargetResult.virtualTrackers);

			ImGui::Begin("TrackingCorrector");
			ImGui::Text("ImGui initialized. %.1f FPS", io.Framerate);

			ImGui::SeparatorText("Mode");
			const Mode mode = controller.mode();
			const char* modeName = mode == Mode::ManualPose
				                       ? "Manual pose"
				                       : mode == Mode::Calibration
				                       ? "Calibration"
				                       : mode == Mode::Capture
				                       ? "Capture"
				                       : "Replay";
			ImGui::Text("Mode: %s", modeName);
			ImGui::Text("SteamVR: %s", vr.isInitialized() ? "connected" : "not connected");
			if (mode != Mode::ManualPose && ImGui::Button("Switch to manual pose"))
			{
				controller.switchToManual();
				replay.reset();
			}
			if (mode != Mode::Calibration && ImGui::Button("Switch to calibration"))
			{
				if (!vr.isInitialized())
				{
					try
					{
						vr.init();
					}
					catch (const std::exception& e)
					{
						spdlog::error("OpenVR still unavailable: {}", e.what());
					}
				}
				if (vr.isInitialized())
				{
					try
					{
						input = std::make_unique<OpenVrInput>();
						input->init();
						refreshGripOffsets();
					}
					catch (const std::exception& e)
					{
						spdlog::error("OpenVR trigger input unavailable: {}", e.what());
						input.reset();
					}
					// Calibration needs both the driver link (vr) and the
					// trigger session (input); abort the switch if either is
					// missing so the mode stays put.
					if (input && input->isInitialized())
					{
						spdlog::info("OpenVR initialized; entering calibration mode");
						controller.switchToCalibration();
						replay.reset();
					}
				}
			}
			if (mode != Mode::Replay && ImGui::Button("Switch to replay"))
				enterReplay();
			if (mode == Mode::Calibration)
			{
				ImGui::TextWrapped("Stand in T-pose and press both triggers to start capture.");
				const DeviceAssignment& liveAssignment = controller.liveAssignment();
				for (size_t i = 0; i < rig.targets.size(); ++i)
				{
					const int d = i < liveAssignment.deviceIndex.size() ? liveAssignment.deviceIndex[i] : -1;
					if (d >= 0 && static_cast<size_t>(d) < devices.size())
						ImGui::BulletText("%s <- %s %d", rig.targetName(i).c_str(),
						                  deviceKindName(devices[d].kind), devices[d].id);
					else
						ImGui::BulletText("%s <- (no device)", rig.targetName(i).c_str());
				}
			}
		// Tracker correction: grouped on/off + rotation toggle + the
		// raw -> corrected position delta per member target. Only meaningful
		// once calibration froze offsets (Capture/Replay); the markers
		// themselves are drawn in the right viewport next to the avatar.
		if (controller.calibration().isCalibrated())
		{
			ImGui::SeparatorText("Correction");
			ImGui::TextDisabled("Avatar scale: %.2f (ref %.2f m / avatar %.2f m)",
			                    avatarScale, refHeight, avatarHeight);
			const std::vector<std::pair<int, Pose>> devicePoses = devicePosePairs(devices);
			auto findDevice = [&](int id) -> const TrackedDevice*
			{
				for (const TrackedDevice& d : devices)
					if (d.id == id)
						return &d;
				return nullptr;
			};
			auto findDevicePose = [&](int id) -> const Pose*
			{
				for (const auto& [did, pose] : devicePoses)
					if (did == id)
						return &pose;
				return nullptr;
			};
			for (size_t gi = 0; gi < kCorrectionGroupCount; ++gi)
			{
				if (!correctionMap.groupPresent[gi])
					continue;
				const CorrectionGroup group = static_cast<CorrectionGroup>(gi);
				ImGui::PushID(static_cast<int>(gi));
				bool enabled = correctionMap.groupEnabled[gi];
				if (ImGui::Checkbox(correctionGroupName(group), &enabled))
					correctionMap.groupEnabled[gi] = enabled;
				// Rotation toggle: controllers are always rotation-locked
				// (aiming), so the checkbox is disabled unless at least one
				// tracker is bound in the group. For tracker groups it
				// controls whether the avatar joint's rotation is applied or
				// the raw rotation is kept.
				bool groupHasTracker = false;
				for (size_t t = 0; t < correctionMap.targetGroup.size(); ++t)
				{
					if (correctionMap.targetGroup[t] != group)
						continue;
					const std::optional<int> boundId = controller.calibration().boundDevice(t);
					if (!boundId)
						continue;
					const TrackedDevice* d = findDevice(*boundId);
					if (d && d->kind != TrackedDeviceKind::Controller)
						groupHasTracker = true;
				}
				ImGui::SameLine();
				ImGui::BeginDisabled(!groupHasTracker);
				bool rotEnabled = groupHasTracker ? correctionMap.groupRotationEnabled[gi] : false;
				if (ImGui::Checkbox("Rot", &rotEnabled))
					correctionMap.groupRotationEnabled[gi] = rotEnabled;
				ImGui::EndDisabled();
				// Per-target position delta readout (diagnostic).
				for (size_t t = 0; t < correctionMap.targetGroup.size(); ++t)
				{
					if (correctionMap.targetGroup[t] != group || !correctionMap.avatarJoint[t])
						continue;
					ImGui::BulletText("%s", rig.targetName(t).c_str());
					for (const CorrectedPose& c : retargetResult.corrected)
					{
						if (c.targetIndex != t)
							continue;
						if (const Pose* raw = findDevicePose(c.deviceId))
						{
							const glm::vec3 delta = c.pose.position - raw->position;
							ImGui::SameLine();
							ImGui::TextDisabled("(%.1f, %.1f, %.1f) cm",
							                    delta.x * 100.0f, delta.y * 100.0f, delta.z * 100.0f);
						}
						break;
					}
				}
ImGui::PopID();
			}
		}
		// Virtual tracker selection (doc/virtual-trackers-plan.md step 4):
		// the step-1 candidate bones, one checkbox each. Editable only before
		// calibration completes; once calibrated, the selection is fixed until
		// recalibration (switching back to Calibration clears the calibration,
		// re-enabling editing). A tick persists immediately to
		// user-settings.json on every tick. The list is recomputed each frame,
		// so it tracks avatar/rig config changes; bones that leave the
		// eligible set on calibration (the six tracker targets) simply drop
		// out of view.
		{
			const std::vector<std::string> candidateBones =
				eligibleVirtualTrackerBones(rig, avatarSkeleton, controller.calibration());
			if (!candidateBones.empty())
			{
				ImGui::SeparatorText("Virtual trackers");
				const bool locked = controller.calibration().isCalibrated();
				if (locked)
					ImGui::TextDisabled("Locked (calibration complete)");
				ImGui::BeginDisabled(locked);
				bool changed = false;
				for (const std::string& bone : candidateBones)
				{
					bool ticked = std::find(settings.virtualTrackerBones.begin(),
					                        settings.virtualTrackerBones.end(), bone)
					              != settings.virtualTrackerBones.end();
					ImGui::PushID(bone.c_str());
					if (ImGui::Checkbox(bone.c_str(), &ticked))
					{
						if (ticked)
							settings.virtualTrackerBones.push_back(bone);
						else
							settings.virtualTrackerBones.erase(
								std::remove(settings.virtualTrackerBones.begin(),
								            settings.virtualTrackerBones.end(), bone),
								settings.virtualTrackerBones.end());
						changed = true;
					}
					ImGui::PopID();
				}
				if (changed)
					saveToFile(kSettingsPath, settings);
				ImGui::EndDisabled();
			}
		}
		if (mode == Mode::Replay)
			{
				ImGui::SeparatorText("Recordings");
				if (replay.files().empty())
					ImGui::TextWrapped("No %s files in the working directory.", kRecordingFileExtension);
				for (size_t i = 0; i < replay.files().size(); ++i)
				{
					const bool selected = static_cast<int>(i) == replay.loadedIndex();
					if (ImGui::Selectable(replay.files()[i].c_str(), selected) && !selected)
						loadReplayFile(i);
				}
			}

			if (mode == Mode::ManualPose)
			{
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
			}
			ImGui::End();

			// Replay timeline pinned across the bottom of the window. No
			// auto-play: clicking/dragging snaps to the nearest recorded
			// frame, whose devices are fed to the solver next frame.
			if (controller.mode() == Mode::Replay && replay.hasRecording())
			{
				const float timelineHeight = 64.0f;
				ImGui::SetNextWindowPos(ImVec2(0.0f, static_cast<float>(height) - timelineHeight));
				ImGui::SetNextWindowSize(ImVec2(static_cast<float>(width), timelineHeight));
				ImGui::Begin("Timeline", nullptr,
				             ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize
				             | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);
				ImGui::SetNextItemWidth(-FLT_MIN);
				float timelineTime = replay.frameTime();
				if (ImGui::SliderFloat("##timeline", &timelineTime, 0.0f,
				                       replay.recording().duration(), "%.2f s"))
					replay.seek(timelineTime);
				ImGui::Text("Frame %zu / %zu  (%.3f s)", replay.frameIndex() + 1,
				            replay.recording().frames.size(), replay.frameTime());
				ImGui::End();
			}

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
