#!/bin/bash

# 获取当前脚本所在目录
SCRIPT_DIR=$(dirname "$(realpath "$0")")

# 从当前目录开始，向上查找直到找到 CMakeLists.txt 文件，确定项目根目录
PROJECT_DIR=$SCRIPT_DIR
while [ ! -f "$PROJECT_DIR/CMakeLists.txt" ]; do
  PROJECT_DIR=$(dirname "$PROJECT_DIR")
  # 防止无限循环
  if [ "$PROJECT_DIR" = "/" ]; then
    echo "Error: Could not find CMakeLists.txt in any parent directory."
    exit 1
  fi
done

# 从CMakeLists.txt文件中提取项目名称
CMAKE_PROJECT_NAME=$(grep -E "set\(CMAKE_PROJECT_NAME" "$PROJECT_DIR/CMakeLists.txt" | sed -E 's/set\(CMAKE_PROJECT_NAME[[:space:]]+([^)]+)\)/\1/' | tr -d ' ')
echo "Project root directory: $PROJECT_DIR"
echo "Project name from CMakeLists.txt: $CMAKE_PROJECT_NAME"

# 设置构建类型(默认为Release，可通过参数修改)
BUILD_TYPE="Release"
if [ "$1" == "debug" ] || [ "$1" == "Debug" ]; then
  BUILD_TYPE="Debug"
fi
echo "Build type: $BUILD_TYPE"

# 创建并进入构建目录
BUILD_DIR="$PROJECT_DIR/build"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR" || { echo "Failed to enter build directory"; exit 1; }

# 运行 CMake 配置和编译项目
echo "Configuring project with CMake..."
if command -v ninja &> /dev/null; then
  # 如果有 Ninja，使用 Ninja 生成器
  cmake .. -GNinja -DCMAKE_BUILD_TYPE=$BUILD_TYPE
  echo "Building project with Ninja..."
  ninja -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)
else
  # 否则使用默认生成器
  cmake .. -DCMAKE_BUILD_TYPE=$BUILD_TYPE
  echo "Building project with Make..."
  make -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)
fi

# 检查构建是否成功
if [ $? -ne 0 ]; then
  echo "Build failed!"
  exit 1
fi

# 根据从CMakeLists.txt提取的项目名称生成文件路径
ELF_FILE="$BUILD_DIR/${CMAKE_PROJECT_NAME}.elf"
BIN_FILE="$BUILD_DIR/${CMAKE_PROJECT_NAME}.bin"
HEX_FILE="$BUILD_DIR/${CMAKE_PROJECT_NAME}.hex"

# 检查 ELF 文件是否成功生成
if [ -f "$ELF_FILE" ]; then
  # 将 ELF 文件转换为 BIN 文件和 HEX 文件
  echo "Converting ELF to BIN and HEX formats..."
  arm-none-eabi-objcopy -O binary "$ELF_FILE" "$BIN_FILE"
  arm-none-eabi-objcopy -O ihex "$ELF_FILE" "$HEX_FILE"
  echo "Build completed successfully!"
  echo "Output files:"
  echo "  ELF: $ELF_FILE"
  echo "  BIN: $BIN_FILE"
  echo "  HEX: $HEX_FILE"
else
  echo "Error: ELF file not found. Compilation might have failed."
  exit 1
fi