#include <cstdlib>
#include <fstream>
#include <optional>
#include <stdexcept>
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
#include "bindings/imgui_impl_glfw.h"
#include "bindings/imgui_impl_opengl3.h"
#include "model/IkRig.h"
#include "model/IkRigConfig.h"
#include "model/Skeleton.h"
#include "view/Scene.h"

constexpr char kSkeletonPath[] = "user-skeleton.json";
constexpr char kIkRigPath[] = "user-ikrig.json";

// Loads the skeleton from the JSON file; creates the file with the default
// skeleton if missing; falls back to the default skeleton (in memory only)
// if the file exists but is invalid.
static Skeleton loadOrCreateSkeleton(const char* path, std::string& statusMessage)
{
	std::ifstream file(path);
	if (file)
	{
		try
		{
			Skeleton skeleton = nlohmann::json::parse(file).get<Skeleton>();
			statusMessage = std::string("Loaded skeleton from ") + path;
			return skeleton;
		}
		catch (const std::exception& e)
		{
			statusMessage = std::string("Failed to load ") + path + ": " + e.what() + " Using default skeleton.";
			return Skeleton::makeDefault();
		}
	}

	Skeleton skeleton = Skeleton::makeDefault();
	std::ofstream out(path);
	out << nlohmann::json(skeleton).dump(2) << '\n';
	statusMessage = std::string("Created default ") + path;
	return skeleton;
}

// Same load-or-create pattern as the skeleton, for the IK rig config.
static IkRigConfig loadOrCreateIkRigConfig(const char* path, std::string& statusMessage)
{
	std::ifstream file(path);
	if (file)
	{
		try
		{
			IkRigConfig config = nlohmann::json::parse(file).get<IkRigConfig>();
			statusMessage = std::string("Loaded IK rig from ") + path;
			return config;
		}
		catch (const std::exception& e)
		{
			statusMessage = std::string("Failed to load ") + path + ": " + e.what() + " Using default IK rig.";
			return IkRigConfig::makeDefault();
		}
	}

	IkRigConfig config = IkRigConfig::makeDefault();
	std::ofstream out(path);
	out << nlohmann::json(config).dump(2) << '\n';
	statusMessage = std::string("Created default ") + path;
	return config;
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

	std::string statusMessage;
	const Skeleton skeleton = loadOrCreateSkeleton(kSkeletonPath, statusMessage);
	std::string ikStatusMessage;
	IkRigConfig ikConfig = loadOrCreateIkRigConfig(kIkRigPath, ikStatusMessage);

	std::optional<IkRig> rig;
	try
	{
		rig.emplace(skeleton, ikConfig);
	}
	catch (const std::exception& e)
	{
		ikStatusMessage = std::string("IK rig invalid: ") + e.what() + " Using default IK rig.";
		rig.emplace(Skeleton(skeleton), IkRigConfig::makeDefault());
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
			manipulateTargets(*rig, scene.viewMatrix(), scene.projectionMatrix(), gizmoOperation);
			rig->solve();
			scene.renderSkeleton(rig->skeleton);
			scene.renderTargets(*rig);

			ImGui::Begin("TrackingCorrector");
			ImGui::Text("ImGui initialized. %.1f FPS", io.Framerate);
			ImGui::TextUnformatted(statusMessage.c_str());
			ImGui::TextUnformatted(ikStatusMessage.c_str());

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
			for (size_t i = 0; i < rig->targets.size(); ++i)
			{
				IkTarget& target = rig->targets[i];
				ImGui::PushID(static_cast<int>(i));
				if (ImGui::TreeNode(rig->targetName(i).c_str()))
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
				rig->resetTargets();
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
