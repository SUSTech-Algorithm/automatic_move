#!/usr/bin/env python3
"""
Concrete state classes for robot behaviors.
"""

from .state import State

class IdleState(State):
    """Robot is idle, waiting for a command."""

    def enter(self):
        print("Entering Idle state")
        # 在这里添加进入空闲状态时需要执行的代码

    def execute(self):
        print("Executing Idle state")
        # 在这里添加空闲状态的主要逻辑
        # 返回下一个状态，如果要保持在当前状态则返回None

        # 示例：模拟接收开始命令后切换到移动状态
        # 实际应用中，这里可能是检查传感器数据或接收外部命令
        # return MovingState(self.context)
        return None

    def exit(self):
        print("Exiting Idle state")
        # 在这里添加退出空闲状态时需要执行的代码

    def handle_event(self, event):
        if event == "start":
            return MovingState(self.context)
        return None


class MovingState(State):
    """Robot is moving."""

    def enter(self):
        print("Entering Moving state")
        # 在这里添加进入移动状态时需要执行的代码

    def execute(self):
        print("Executing Moving state")
        # 在这里添加移动状态的主要逻辑
        # 返回下一个状态，如果要保持在当前状态则返回None

        # 示例：模拟到达目的地后切换到停止状态
        # 实际应用中，这里可能是检查位置传感器数据
        # return StoppedState(self.context)
        return None

    def exit(self):
        print("Exiting Moving state")
        # 在这里添加退出移动状态时需要执行的代码

    def handle_event(self, event):
        if event == "stop":
            return StoppedState(self.context)
        elif event == "obstacle":
            return AvoidingObstacleState(self.context)
        return None


class StoppedState(State):
    """Robot has stopped."""

    def enter(self):
        print("Entering Stopped state")
        # 在这里添加进入停止状态时需要执行的代码

    def execute(self):
        print("Executing Stopped state")
        # 在这里添加停止状态的主要逻辑
        return None

    def exit(self):
        print("Exiting Stopped state")
        # 在这里添加退出停止状态时需要执行的代码

    def handle_event(self, event):
        if event == "resume":
            return MovingState(self.context)
        return None


class AvoidingObstacleState(State):
    """Robot is avoiding an obstacle."""

    def enter(self):
        print("Entering Obstacle Avoidance state")
        # 在这里添加进入避障状态时需要执行的代码

    def execute(self):
        print("Executing Obstacle Avoidance state")
        # 在这里添加避障状态的主要逻辑
        # 避障完成后可以返回到移动状态
        # return MovingState(self.context)
        return None

    def exit(self):
        print("Exiting Obstacle Avoidance state")
        # 在这里添加退出避障状态时需要执行的代码

    def handle_event(self, event):
        if event == "obstacle_cleared":
            return MovingState(self.context)
        return None