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

# 设置构建目录和HEX文件路径
BUILD_DIR="$PROJECT_DIR/build"
HEX_FILE="$BUILD_DIR/${CMAKE_PROJECT_NAME}.hex"

# 如果命令行参数提供了HEX文件，则使用命令行参数
if [ -n "$1" ]; then
  HEX_FILE="$1"
fi

# 检查HEX文件是否存在
if [ ! -f "$HEX_FILE" ]; then
  echo "Error: HEX file not found: $HEX_FILE"
  echo "Make sure to build the project before flashing, or provide a valid HEX file path."
  exit 1
fi

echo "Using HEX file: $HEX_FILE"

# 在 Windows Git Bash 中运行时，转换路径格式
if [[ "$OSTYPE" == "msys" || "$OSTYPE" == "cygwin" ]]; then
  # 使用 cygpath 转换路径为 Windows 格式
  HEX_FILE_WIN=$(cygpath -w "$HEX_FILE")
  # 将Windows路径中的反斜杠转换为正斜杠
  HEX_FILE_WIN=${HEX_FILE_WIN//\\/\/}
  echo "Running on Windows Git Bash, converted path: $HEX_FILE_WIN"
  # 使用转换后的 Windows 路径
  openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -c "program \"$HEX_FILE_WIN\" verify reset exit"
else
  # 在 Linux 或 macOS 上使用原始路径
  openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -c "program \"$HEX_FILE\" verify reset exit"
fi