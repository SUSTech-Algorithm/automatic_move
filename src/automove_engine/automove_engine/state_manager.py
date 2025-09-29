#!/usr/bin/env python3
"""
State manager for robot state transitions.
"""

from .state import State
from .states import IdleState


class StateManager:
    """Manages robot state transitions."""

    def __init__(self):
        """Initialize the state manager with an initial state."""
        self.current_state = None
        self.context = {}  # Shared data between states

    def set_initial_state(self, state=None):
        """
        Set the initial state of the robot.

        Args:
            state (State): The initial state. If None, defaults to IdleState.
        """
        if state is None:
            state = IdleState(self.context)

        self.transition_to(state)

    def transition_to(self, new_state):
        """
        Transition to a new state.

        Args:
            new_state (State): The state to transition to.
        """
        if self.current_state:
            self.current_state.exit()

        self.current_state = new_state
        self.current_state.context = self.context
        self.current_state.enter()

    def update(self):
        """
        Update the current state.
        This should be called regularly to execute state logic.
        """
        if self.current_state:
            next_state = self.current_state.execute()
            if next_state and next_state != self.current_state:
                self.transition_to(next_state)

    def handle_event(self, event):
        """
        Handle an external event that may cause a state transition.

        Args:
            event: The event to handle.
        """
        if self.current_state:
            next_state = self.current_state.handle_event(event)
            if next_state and next_state != self.current_state:
                self.transition_to(next_state)

    def get_current_state(self):
        """
        Get the current state.

        Returns:
            State: The current state.
        """
        return self.current_state


# Example usage:
if __name__ == "__main__":
    # Create state manager
    sm = StateManager()

    # Set initial state
    sm.set_initial_state()

    # Main loop
    try:
        while True:
            sm.update()
            # In a real application, you would add a sleep here
            # to control the update rate
    except KeyboardInterrupt:
        print("Exiting...")