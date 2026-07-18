#include <cstdlib>
#include <fstream>
#include <string>
#include <imgui.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <nlohmann/json.hpp>
#include "bindings/imgui_impl_glfw.h"
#include "bindings/imgui_impl_opengl3.h"
#include "model/Skeleton.h"
#include "view/Scene.h"

constexpr char kSkeletonPath[] = "user-skeleton.json";

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

	// Scene holds GL resources; scope it so it is destroyed before GLFW shutdown.
	{
		Scene scene;

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

			scene.update(window, !io.WantCaptureMouse);
			scene.beginFrame(width, height);
			scene.renderSkeleton(skeleton);

			ImGui::Begin("TrackingCorrector");
			ImGui::Text("ImGui initialized. %.1f FPS", io.Framerate);
			ImGui::TextUnformatted(statusMessage.c_str());
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
