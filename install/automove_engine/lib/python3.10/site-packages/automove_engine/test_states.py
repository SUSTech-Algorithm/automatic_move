#!/usr/bin/env python3
"""
Test script for robot state management system.
"""

import time
import sys
import os

# Add the parent directory to the path so we can import the automove_engine modules
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from automove_engine.state_manager import StateManager
from automove_engine.states import IdleState, MovingState, StoppedState


def test_state_transitions():
    """Test state transitions."""
    print("Testing state management system...")

    # Create state manager
    sm = StateManager()

    # Set initial state
    sm.set_initial_state()

    print(f"Current state: {type(sm.get_current_state()).__name__}")

    # Simulate handling an event to transition to moving state
    print("\nHandling 'start' event...")
    sm.handle_event("start")
    print(f"Current state: {type(sm.get_current_state()).__name__}")

    # Simulate handling an event to transition to stopped state
    print("\nHandling 'stop' event...")
    sm.handle_event("stop")
    print(f"Current state: {type(sm.get_current_state()).__name__}")

    # Simulate handling an event to transition back to moving state
    print("\nHandling 'resume' event...")
    sm.handle_event("resume")
    print(f"Current state: {type(sm.get_current_state()).__name__}")

    print("\nTest completed successfully!")


if __name__ == "__main__":
    test_state_transitions()