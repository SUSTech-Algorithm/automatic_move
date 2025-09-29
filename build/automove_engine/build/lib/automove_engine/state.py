#!/usr/bin/env python3
"""
Base state class for robot state management.
"""

class State:
    """Base class for all robot states."""

    def __init__(self, context=None):
        """
        Initialize the state.

        Args:
            context: The context object that holds shared data between states
        """
        self.context = context

    def enter(self):
        """
        Called when entering this state.
        Override this method in subclasses to perform initialization.
        """
        pass

    def execute(self):
        """
        Execute the main logic of this state.
        Override this method in subclasses to implement state behavior.

        Returns:
            State: The next state to transition to, or None to stay in current state
        """
        return None

    def exit(self):
        """
        Called when exiting this state.
        Override this method in subclasses to perform cleanup.
        """
        pass

    def handle_event(self, event):
        """
        Handle an event that occurred while in this state.
        Override this method in subclasses to handle specific events.

        Args:
            event: The event to handle

        Returns:
            State: The next state to transition to, or None to stay in current state
        """
        return None