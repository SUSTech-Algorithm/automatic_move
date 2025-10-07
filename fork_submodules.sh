#!/bin/bash

# Fork submodules script (保留现有更改版本)
# 请在GitHub上手动fork以下仓库后运行此脚本
# 
# 需要fork的仓库：
# 1. https://github.com/Ericsii/FAST_LIO_ROS2
# 2. https://github.com/Livox-SDK/Livox-SDK2  
# 3. https://github.com/Livox-SDK/livox_ros_driver2
# 4. https://github.com/LCAS/livox_laser_simulation_ros2

# 请将YOUR_GITHUB_USERNAME替换为您的实际GitHub用户名
GITHUB_USERNAME="fjienan"

# 检查是否提供了用户名
if [ "$GITHUB_USERNAME" = "YOUR_GITHUB_USERNAME" ]; then
    echo "错误: 请在脚本中将YOUR_GITHUB_USERNAME替换为您的实际GitHub用户名"
    exit 1
fi

echo "保留现有更改并设置fork..."

# 为有更改的子模块设置远程URL并推送
echo "处理FAST_LIO_ROS2子模块..."
cd src/FAST_LIO_ROS2
echo "  - 当前分支: $(git branch --show-current)"
echo "  - 未推送的提交: $(git log --oneline origin/$(git branch --show-current)..HEAD | wc -l)"
git remote add upstream https://github.com/Ericsii/FAST_LIO_ROS2.git 2>/dev/null || echo "  - upstream已存在"
git remote set-url origin https://github.com/$GITHUB_USERNAME/FAST_LIO_ROS2.git
echo "  - 推送到您的fork..."
git push -u origin $(git branch --show-current)
cd ../..

echo "处理livox_laser_simulation_ros2子模块..."  
cd src/livox_laser_simulation_ros2
echo "  - 当前分支: $(git branch --show-current)"
echo "  - 未推送的提交: $(git log --oneline origin/$(git branch --show-current)..HEAD | wc -l)"
git remote add upstream https://github.com/LCAS/livox_laser_simulation_ros2.git 2>/dev/null || echo "  - upstream已存在"
git remote set-url origin https://github.com/$GITHUB_USERNAME/livox_laser_simulation_ros2.git
echo "  - 推送到您的fork..."
git push -u origin $(git branch --show-current)
cd ../..

# 为其他子模块设置远程URL（如果它们存在）
if [ -d "src/Livox-SDK2" ]; then
    echo "处理Livox-SDK2子模块..."
    cd src/Livox-SDK2  
    git remote add upstream https://github.com/Livox-SDK/Livox-SDK2.git 2>/dev/null || echo "  - upstream已存在"
    git remote set-url origin https://github.com/$GITHUB_USERNAME/Livox-SDK2.git
    cd ../..
fi

if [ -d "src/livox_ros_driver2" ]; then
    echo "处理livox_ros_driver2子模块..."
    cd src/livox_ros_driver2
    git remote add upstream https://github.com/Livox-SDK/livox_ros_driver2.git 2>/dev/null || echo "  - upstream已存在"
    git remote set-url origin https://github.com/$GITHUB_USERNAME/livox_ros_driver2.git  
    cd ../..
fi

echo "更新.gitmodules文件..."
# 更新.gitmodules文件
sed -i "s|https://github.com/Ericsii/FAST_LIO_ROS2.git|https://github.com/$GITHUB_USERNAME/FAST_LIO_ROS2.git|g" .gitmodules
sed -i "s|https://github.com/Livox-SDK/Livox-SDK2.git|https://github.com/$GITHUB_USERNAME/Livox-SDK2.git|g" .gitmodules  
sed -i "s|https://github.com/Livox-SDK/livox_ros_driver2.git|https://github.com/$GITHUB_USERNAME/livox_ros_driver2.git|g" .gitmodules
sed -i "s|https://github.com/LCAS/livox_laser_simulation_ros2.git|https://github.com/$GITHUB_USERNAME/livox_laser_simulation_ros2.git|g" .gitmodules

echo "同步.gitmodules更改到git配置..."
git submodule sync --recursive

echo "完成！现在您可以修改子模块并推送到您自己的fork了。"
echo ""
echo "提示："
echo "- 使用 'git remote -v' 在子模块目录中查看远程配置" 
echo "- upstream指向原始仓库，用于拉取更新"
echo "- origin指向您的fork，用于推送更改"
