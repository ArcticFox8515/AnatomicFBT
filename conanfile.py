import os

from conan import ConanFile
from conan.tools.cmake import cmake_layout
from conan.tools.files import copy


class TrackingCorrector(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeDeps", "CMakeToolchain"

    def requirements(self):
        # force=True: imguizmo's recipe pins imgui/1.90.5; we keep the docking
        # variant at the same version so both get one consistent ImGui.
        self.requires("imgui/1.90.5-docking", force=True)
        self.requires("imguizmo/cci.20231114")
        self.requires("glfw/3.4")
        self.requires("glew/2.2.0")
        self.requires("glm/1.0.1")
        self.requires("nlohmann_json/3.11.3")
        self.requires("spdlog/1.15.3")
        self.requires("openvr/1.16.8")
        self.requires("gtest/1.15.0")

    def generate(self):
        # Keep vendored backends in sync with the resolved ImGui version.
        for pattern in ("*glfw*", "*opengl3*"):
            src_dir = os.path.join(self.dependencies["imgui"].package_folder, "res", "bindings")
            dst_dir = os.path.join(self.source_folder, "src", "bindings")
            copy(self, pattern, src_dir, dst_dir)
            for name in os.listdir(dst_dir):
                path = os.path.join(dst_dir, name)
                if os.path.isfile(path):
                    os.utime(path, None)

    def layout(self):
        cmake_layout(self)
