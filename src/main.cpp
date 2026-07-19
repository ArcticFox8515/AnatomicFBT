#include <cstdlib>
#include <exception>
#include <fstream>
#include <string>
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
#include "model/Skeleton.h"
#include "view/Scene.h"

constexpr char kSkeletonPath[] = "user-skeleton.json";
constexpr char kIkRigPath[] = "user-ikrig.json";

// Loads T from a JSON file; creates the file with the default if missing;
// falls back to the default (in memory only) if the file exists but is
// invalid. Any failure is logged with the full exception detail in what().
template <typename T>
static T loadOrCreate(const char* path)
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
			return T::makeDefault();
		}
	}

	T value = T::makeDefault();
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

	const Skeleton skeleton = loadOrCreate<Skeleton>(kSkeletonPath);
	IkRig rig(skeleton);
	try
	{
		rig.loadConfig(loadOrCreate<IkRigConfig>(kIkRigPath));
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

			ImGuizmo::SetOrthographic(false);
			ImGuizmo::SetRect(0.0f, 0.0f, static_cast<float>(width), static_cast<float>(height));
			manipulateTargets(rig, scene.viewMatrix(), scene.projectionMatrix(), gizmoOperation);
			rig.solve();
			scene.renderSkeleton(rig.skeleton);
			scene.renderTargets(rig);

			ImGui::Begin("TrackingCorrector");
			ImGui::Text("ImGui initialized. %.1f FPS", io.Framerate);

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
