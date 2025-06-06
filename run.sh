#!/bin/bash

# 获取当前脚本所在目录
SCRIPT_DIR=$(dirname "$(realpath "$0")")

# 设置build、flash和clean脚本的路径
BUILD_SCRIPT="$SCRIPT_DIR/Scripts/build.sh"
FLASH_SCRIPT="$SCRIPT_DIR/Scripts/flash.sh"
CLEAN_SCRIPT="$SCRIPT_DIR/Scripts/clean.sh"

# 检查脚本是否存在
if [ ! -f "$BUILD_SCRIPT" ]; then
  echo "Error: Build script not found at $BUILD_SCRIPT"
  exit 1
fi

if [ ! -f "$FLASH_SCRIPT" ]; then
  echo "Error: Flash script not found at $FLASH_SCRIPT"
  exit 1
fi

if [ ! -f "$CLEAN_SCRIPT" ]; then
  echo "Error: Clean script not found at $CLEAN_SCRIPT"
  exit 1
fi

# 确保脚本具有可执行权限
chmod +x "$BUILD_SCRIPT"
chmod +x "$FLASH_SCRIPT"
chmod +x "$CLEAN_SCRIPT"

# 根据参数执行对应功能
if [ "$1" == "build" ]; then
  echo "===== 执行构建操作 ====="
  # 如果有额外参数，传递给build脚本
  if [ -n "$2" ]; then
    "$BUILD_SCRIPT" "$2"
  else
    "$BUILD_SCRIPT"
  fi
elif [ "$1" == "flash" ]; then
  echo "===== 执行烧录操作 ====="
  # 如果有额外参数，传递给flash脚本
  if [ -n "$2" ]; then
    "$FLASH_SCRIPT" "$2"
  else
    "$FLASH_SCRIPT"
  fi
elif [ "$1" == "clean" ]; then
  echo "===== 执行清理操作 ====="
  "$CLEAN_SCRIPT"
else
  # 如果没有参数或参数不是build、flash或clean，则依次执行构建和烧录
  echo "===== 执行构建操作 ====="
  "$BUILD_SCRIPT" "$1"  # 将第一个参数作为构建类型传递
  
  # 检查构建是否成功
  if [ $? -ne 0 ]; then
    echo "构建失败，中止烧录操作。"
    exit 1
  fi
  
  echo "===== 执行烧录操作 ====="
  "$FLASH_SCRIPT"
fi

# 如果执行到这里，表示操作已完成
echo "===== 操作完成 ====="