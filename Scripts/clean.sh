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

echo "Project root directory: $PROJECT_DIR"

# 设置构建目录
BUILD_DIR="$PROJECT_DIR/build"

# 检查构建目录是否存在
if [ -d "$BUILD_DIR" ]; then
  echo "Cleaning build directory: $BUILD_DIR"
  
  # 在Windows Git Bash中使用rm -rf可能会有问题，所以先尝试用原生命令
  if [[ "$OSTYPE" == "msys" || "$OSTYPE" == "cygwin" ]]; then
    # 在Windows环境中使用rmdir命令
    BUILD_DIR_WIN=$(cygpath -w "$BUILD_DIR")
    echo "Running on Windows, converted path: $BUILD_DIR_WIN"
    
    rm -rf "$BUILD_DIR"
  else
    # 在Linux/macOS上直接使用rm -rf
    rm -rf "$BUILD_DIR"
  fi
  
  # 检查清理结果
  if [ ! -d "$BUILD_DIR" ]; then
    echo "Clean completed successfully!"
  else
    echo "Warning: Could not completely remove the build directory."
    echo "Some files may still exist. You might need to manually delete them."
  fi
else
  echo "Build directory does not exist. Nothing to clean."
fi

# 清理其他可能的生成文件
echo "Cleaning other generated files..."

# 清理可能存在的CMake缓存文件
find "$PROJECT_DIR" -name "CMakeCache.txt" -type f -delete
find "$PROJECT_DIR" -name "CMakeFiles" -type d -exec rm -rf {} +
find "$PROJECT_DIR" -name "cmake_install.cmake" -type f -delete
find "$PROJECT_DIR" -name "compile_commands.json" -type f -delete

# 清理编译器生成的中间文件
find "$PROJECT_DIR" -name "*.o" -type f -delete
find "$PROJECT_DIR" -name "*.obj" -type f -delete
find "$PROJECT_DIR" -name "*.elf" -type f -delete
find "$PROJECT_DIR" -name "*.hex" -type f -delete
find "$PROJECT_DIR" -name "*.bin" -type f -delete
find "$PROJECT_DIR" -name "*.map" -type f -delete

echo "===== 清理操作完成 ====="