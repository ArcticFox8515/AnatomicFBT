conan install . --output-folder=build --build=missing --settings=build_type=Debug
conan install . --output-folder=build --build=missing --settings=build_type=Release
cd build
cmake .. -G "Visual Studio 17 2022" -DCMAKE_TOOLCHAIN_FILE=build/generators/conan_toolchain.cmake
pause
