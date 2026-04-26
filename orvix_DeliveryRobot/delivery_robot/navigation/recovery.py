"""Recovery: detects 'stuck' conditions and escalates them to ERROR.

Without recovery, several states are silently terminal-but-not-marked:
- APPROACHING_CROSSING with `light_state=unknown` forever (perception broken,
  semáforo apagado/dañado, robot mal orientado).
- WAITING_AT_CROSSING for many minutes (semáforo broken, never turns green).
- STOPPED_FOR_OBSTACLE for too long (something parked in the path).

The `RecoveryMonitor` watches the stream of `NavigationDecision`s, tracks
how long we've been in each state, and when a configurable threshold is
crossed it transforms the decision into either a warning (state unchanged
but `recovery_warning` populated) or an escalation (state -> ERROR).

GPS dropout is handled separately in `hardware/robot.py` because it
happens before the orchestrator runs (no pose, no decision).
"""
from dataclasses import dataclass, field
from typing import Dict, Optional

from .decision import NavigationDecision
from .states import NavigationAction, NavigationState


@dataclass
class RecoveryPolicy:
    """All thresholds in seconds. None disables that escalation."""
    obstacle_warn_after_s: float = 60.0          # Log a warning, keep waiting.
    obstacle_escalate_after_s: float = 300.0     # 5 min → ERROR.
    waiting_red_warn_after_s: float = 90.0
    waiting_red_escalate_after_s: float = 240.0  # 4 min red → likely broken.
    approaching_unknown_warn_after_s: float = 15.0
    approaching_unknown_escalate_after_s: float = 45.0


@dataclass
class _StateMemory:
    state: Optional[NavigationState] = None
    entered_at_s: float = 0.0
    warned: bool = False


@dataclass
class RecoveryMonitor:
    """Stateful tracker. Use one instance per orchestrator (per mission)."""
    policy: RecoveryPolicy = field(default_factory=RecoveryPolicy)
    _memory: _StateMemory = field(default_factory=_StateMemory)

    def review(
        self,
        decision: NavigationDecision,
        now_s: float,
    ) -> NavigationDecision:
        """Examine the decision; possibly transform it (warning or ERROR)."""
        if decision.state != self._memory.state:
            self._memory = _StateMemory(state=decision.state, entered_at_s=now_s)
            return decision

        elapsed = now_s - self._memory.entered_at_s
        warn_threshold, escalate_threshold = self._thresholds_for(decision)
        if escalate_threshold is not None and elapsed >= escalate_threshold:
            return self._escalate(decision, elapsed)
        if warn_threshold is not None and elapsed >= warn_threshold and not self._memory.warned:
            self._memory.warned = True
            return self._warn(decision, elapsed)
        return decision

    def _thresholds_for(self, decision: NavigationDecision):
        s = decision.state
        if s == NavigationState.STOPPED_FOR_OBSTACLE:
            return self.policy.obstacle_warn_after_s, self.policy.obstacle_escalate_after_s
        if s == NavigationState.WAITING_AT_CROSSING:
            return self.policy.waiting_red_warn_after_s, self.policy.waiting_red_escalate_after_s
        if s == NavigationState.APPROACHING_CROSSING:
            return (
                self.policy.approaching_unknown_warn_after_s,
                self.policy.approaching_unknown_escalate_after_s,
            )
        return None, None

    def _warn(self, decision: NavigationDecision, elapsed: float) -> NavigationDecision:
        decision.recovery_warning = (
            f"Stuck in {decision.state.value} for {elapsed:.0f}s — supervisor attention recommended"
        )
        return decision

    def _escalate(self, decision: NavigationDecision, elapsed: float) -> NavigationDecision:
        decision.state = NavigationState.ERROR
        decision.action = NavigationAction.STOP
        decision.reason = (
            f"Recovery timeout — stuck in {self._memory.state.value} for {elapsed:.0f}s. "
            f"Original reason: {decision.reason}"
        )
        decision.recovery_warning = "Escalated to ERROR by RecoveryMonitor"
        return decision

    def reset(self) -> None:
        self._memory = _StateMemory()
