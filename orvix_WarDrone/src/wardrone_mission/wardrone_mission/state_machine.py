"""Generic finite state machine engine.

Provides a reusable state machine with:
- Typed states and events (enums)
- Guard conditions on transitions
- Enter/exit callbacks per state
- Transition actions
- Global transitions (from any state)
- History recording for debugging/logging
"""

import time
from dataclasses import dataclass, field
from typing import Dict, Callable, Optional, List, Tuple, Any
from enum import Enum


@dataclass
class Transition:
    """A single state transition."""
    target_state: Any
    guard: Optional[Callable[[], bool]] = None
    action: Optional[Callable[[], None]] = None


@dataclass
class TransitionRecord:
    """Record of a state transition for history/debugging."""
    timestamp: float
    from_state: Any
    event: Any
    to_state: Any
    reason: str = ""


class StateMachine:
    """Generic finite state machine.

    Usage:
        sm = StateMachine(initial_state=MissionState.IDLE)
        sm.add_transition(MissionState.IDLE, MissionEvent.CMD_START, MissionState.PREFLIGHT)
        sm.add_global_transition(MissionEvent.SAFETY_CRITICAL, MissionState.EMERGENCY)
        sm.on_enter(MissionState.TAKEOFF, my_callback)

        # Fire events
        sm.handle_event(MissionEvent.CMD_START)
    """

    def __init__(self, initial_state):
        self._state = initial_state
        self._initial_state = initial_state
        self._transitions: Dict[tuple, Transition] = {}
        self._on_enter: Dict[Any, Callable] = {}
        self._on_exit: Dict[Any, Callable] = {}
        self._on_transition: Optional[Callable[[TransitionRecord], None]] = None
        self._history: List[TransitionRecord] = []
        self._terminal_states: set = set()

    @property
    def state(self):
        return self._state

    @property
    def history(self) -> List[TransitionRecord]:
        return self._history

    @property
    def is_terminal(self) -> bool:
        return self._state in self._terminal_states

    def set_terminal_states(self, *states):
        """Mark states as terminal (no transitions out except reset)."""
        self._terminal_states = set(states)

    def add_transition(self, from_state, event, to_state,
                       guard: Optional[Callable[[], bool]] = None,
                       action: Optional[Callable[[], None]] = None):
        """Add a transition from a specific state on an event."""
        self._transitions[(from_state, event)] = Transition(to_state, guard, action)

    def add_global_transition(self, event, to_state,
                               guard: Optional[Callable[[], bool]] = None,
                               action: Optional[Callable[[], None]] = None,
                               exclude_states: Optional[set] = None):
        """Add a transition from any state on an event.

        Args:
            event: The triggering event.
            to_state: Target state.
            guard: Optional guard condition.
            action: Optional action on transition.
            exclude_states: States to exclude from this global transition.
        """
        exclude = exclude_states or set()
        # Get all possible states from the enum type of the initial state
        state_enum = type(self._initial_state)
        for state in state_enum:
            if state not in exclude and state != to_state:
                self.add_transition(state, event, to_state, guard, action)

    def on_enter(self, state, callback: Callable):
        """Register a callback to be called when entering a state."""
        self._on_enter[state] = callback

    def on_exit(self, state, callback: Callable):
        """Register a callback to be called when exiting a state."""
        self._on_exit[state] = callback

    def on_any_transition(self, callback: Callable[[TransitionRecord], None]):
        """Register a callback for any transition (useful for logging)."""
        self._on_transition = callback

    def handle_event(self, event, reason: str = "") -> bool:
        """Process an event. Returns True if a transition occurred.

        Args:
            event: The event to process.
            reason: Optional human-readable reason for the event.

        Returns:
            True if a valid transition was found and executed.
        """
        if self.is_terminal:
            return False

        key = (self._state, event)
        if key not in self._transitions:
            return False

        transition = self._transitions[key]

        # Check guard condition
        if transition.guard is not None and not transition.guard():
            return False

        old_state = self._state

        # Exit callback
        if old_state in self._on_exit:
            self._on_exit[old_state]()

        # Transition action
        if transition.action is not None:
            transition.action()

        # State change
        self._state = transition.target_state

        # Record
        record = TransitionRecord(
            timestamp=time.time(),
            from_state=old_state,
            event=event,
            to_state=self._state,
            reason=reason,
        )
        self._history.append(record)

        # Notify
        if self._on_transition is not None:
            self._on_transition(record)

        # Enter callback
        if self._state in self._on_enter:
            self._on_enter[self._state]()

        return True

    def reset(self):
        """Reset the state machine to its initial state."""
        self._state = self._initial_state
        self._history.clear()

    def get_valid_events(self) -> list:
        """Return list of events valid in the current state."""
        events = []
        for (state, event), _ in self._transitions.items():
            if state == self._state:
                events.append(event)
        return events
