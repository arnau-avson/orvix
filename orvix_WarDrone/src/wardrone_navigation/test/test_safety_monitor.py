"""Tests for safety_monitor_node module.

These tests exercise the safety monitoring logic (battery thresholds, link loss,
GPS quality) as pure Python -- no ROS 2 runtime required.
"""

import pytest


class TestSafetyMonitorImport:
    def test_import(self):
        from wardrone_navigation.safety_monitor_node import SafetyMonitorNode
        assert SafetyMonitorNode is not None


# ---------------------------------------------------------------------------
# Battery threshold logic
#
# SafetyMonitorNode uses these defaults:
#   battery_warning_pct  = 30.0
#   battery_critical_pct = 15.0
#
# _monitor_tick fires when the drone is armed AND in air:
#   - battery_pct <= critical  AND not critical_sent  -> CRITICAL_BATTERY
#   - battery_pct <= warning   AND not warning_sent   -> LOW_BATTERY
# ---------------------------------------------------------------------------

class TestBatteryThresholdLogic:
    """Test the battery comparison logic used by the safety monitor.

    We replicate the exact conditional pattern from _monitor_tick so we
    can verify threshold behaviour without instantiating a ROS 2 Node.
    """

    BATTERY_WARNING_PCT = 30.0
    BATTERY_CRITICAL_PCT = 15.0

    @staticmethod
    def _evaluate_battery(battery_pct, warning_threshold, critical_threshold,
                          warning_sent=False, critical_sent=False):
        """Replicate the battery check logic from _monitor_tick.

        Returns a tuple (event, new_warning_sent, new_critical_sent).
        event is one of: 'CRITICAL_BATTERY', 'LOW_BATTERY', or None.
        """
        event = None
        if battery_pct <= critical_threshold and not critical_sent:
            event = 'CRITICAL_BATTERY'
            critical_sent = True
        elif battery_pct <= warning_threshold and not warning_sent:
            event = 'LOW_BATTERY'
            warning_sent = True
        return event, warning_sent, critical_sent

    # -- Normal battery levels (above warning) --

    def test_battery_100_no_event(self):
        event, _, _ = self._evaluate_battery(
            100.0, self.BATTERY_WARNING_PCT, self.BATTERY_CRITICAL_PCT)
        assert event is None

    def test_battery_50_no_event(self):
        event, _, _ = self._evaluate_battery(
            50.0, self.BATTERY_WARNING_PCT, self.BATTERY_CRITICAL_PCT)
        assert event is None

    def test_battery_just_above_warning(self):
        event, _, _ = self._evaluate_battery(
            30.1, self.BATTERY_WARNING_PCT, self.BATTERY_CRITICAL_PCT)
        assert event is None

    # -- Warning threshold --

    def test_battery_at_warning_triggers_low_battery(self):
        event, ws, cs = self._evaluate_battery(
            30.0, self.BATTERY_WARNING_PCT, self.BATTERY_CRITICAL_PCT)
        assert event == 'LOW_BATTERY'
        assert ws is True
        assert cs is False

    def test_battery_below_warning_triggers_low_battery(self):
        event, ws, _ = self._evaluate_battery(
            25.0, self.BATTERY_WARNING_PCT, self.BATTERY_CRITICAL_PCT)
        assert event == 'LOW_BATTERY'
        assert ws is True

    def test_battery_warning_not_sent_twice(self):
        """Once warning_sent is True, repeated checks must not re-fire."""
        event, _, _ = self._evaluate_battery(
            25.0, self.BATTERY_WARNING_PCT, self.BATTERY_CRITICAL_PCT,
            warning_sent=True)
        assert event is None

    # -- Critical threshold --

    def test_battery_at_critical_triggers_critical(self):
        event, ws, cs = self._evaluate_battery(
            15.0, self.BATTERY_WARNING_PCT, self.BATTERY_CRITICAL_PCT)
        assert event == 'CRITICAL_BATTERY'
        assert cs is True

    def test_battery_below_critical_triggers_critical(self):
        event, _, cs = self._evaluate_battery(
            5.0, self.BATTERY_WARNING_PCT, self.BATTERY_CRITICAL_PCT)
        assert event == 'CRITICAL_BATTERY'
        assert cs is True

    def test_battery_zero_triggers_critical(self):
        event, _, cs = self._evaluate_battery(
            0.0, self.BATTERY_WARNING_PCT, self.BATTERY_CRITICAL_PCT)
        assert event == 'CRITICAL_BATTERY'
        assert cs is True

    def test_battery_critical_not_sent_twice(self):
        """Once critical_sent is True, repeated checks must not re-fire."""
        event, _, _ = self._evaluate_battery(
            5.0, self.BATTERY_WARNING_PCT, self.BATTERY_CRITICAL_PCT,
            warning_sent=True, critical_sent=True)
        assert event is None

    def test_critical_takes_priority_over_warning(self):
        """When battery is at critical AND warning has not been sent,
        critical must fire (it is checked first)."""
        event, _, cs = self._evaluate_battery(
            10.0, self.BATTERY_WARNING_PCT, self.BATTERY_CRITICAL_PCT,
            warning_sent=False, critical_sent=False)
        assert event == 'CRITICAL_BATTERY'
        assert cs is True

    # -- Just above critical, below warning --

    def test_battery_just_above_critical_triggers_warning(self):
        """Battery at 15.1 is above critical (15) but below warning (30)."""
        event, ws, cs = self._evaluate_battery(
            15.1, self.BATTERY_WARNING_PCT, self.BATTERY_CRITICAL_PCT)
        assert event == 'LOW_BATTERY'
        assert ws is True
        assert cs is False

    # -- Sequence simulation --

    def test_battery_drain_sequence(self):
        """Simulate a full battery drain: 100 -> 30 -> 15 -> 5."""
        w_sent = False
        c_sent = False

        # 100% - no event
        event, w_sent, c_sent = self._evaluate_battery(
            100.0, self.BATTERY_WARNING_PCT, self.BATTERY_CRITICAL_PCT,
            w_sent, c_sent)
        assert event is None

        # 30% - warning fires
        event, w_sent, c_sent = self._evaluate_battery(
            30.0, self.BATTERY_WARNING_PCT, self.BATTERY_CRITICAL_PCT,
            w_sent, c_sent)
        assert event == 'LOW_BATTERY'

        # 30% again - no repeat
        event, w_sent, c_sent = self._evaluate_battery(
            30.0, self.BATTERY_WARNING_PCT, self.BATTERY_CRITICAL_PCT,
            w_sent, c_sent)
        assert event is None

        # 15% - critical fires
        event, w_sent, c_sent = self._evaluate_battery(
            15.0, self.BATTERY_WARNING_PCT, self.BATTERY_CRITICAL_PCT,
            w_sent, c_sent)
        assert event == 'CRITICAL_BATTERY'

        # 5% - critical already sent, no repeat
        event, w_sent, c_sent = self._evaluate_battery(
            5.0, self.BATTERY_WARNING_PCT, self.BATTERY_CRITICAL_PCT,
            w_sent, c_sent)
        assert event is None


class TestLinkLossLogic:
    """Test the telemetry link loss detection logic from _monitor_tick.

    Logic: if elapsed > telem_timeout and not link_lost_sent -> LINK_LOST
    """

    TELEM_TIMEOUT = 3.0

    @staticmethod
    def _evaluate_link(elapsed_s, timeout_s, link_lost_sent=False):
        """Replicate the link loss check from _monitor_tick.

        Returns (event, new_link_lost_sent).
        """
        event = None
        if elapsed_s > timeout_s and not link_lost_sent:
            event = 'LINK_LOST'
            link_lost_sent = True
        elif elapsed_s <= timeout_s:
            link_lost_sent = False
        return event, link_lost_sent

    def test_no_loss_when_recent(self):
        event, _ = self._evaluate_link(0.5, self.TELEM_TIMEOUT)
        assert event is None

    def test_no_loss_at_boundary(self):
        event, _ = self._evaluate_link(3.0, self.TELEM_TIMEOUT)
        assert event is None

    def test_link_lost_after_timeout(self):
        event, sent = self._evaluate_link(3.1, self.TELEM_TIMEOUT)
        assert event == 'LINK_LOST'
        assert sent is True

    def test_link_lost_not_repeated(self):
        event, _ = self._evaluate_link(5.0, self.TELEM_TIMEOUT, link_lost_sent=True)
        assert event is None

    def test_link_recovery_resets_flag(self):
        """When link recovers (elapsed <= timeout), the flag must be reset."""
        _, sent = self._evaluate_link(1.0, self.TELEM_TIMEOUT, link_lost_sent=True)
        assert sent is False


class TestGPSQualityLogic:
    """Test the GPS quality degradation logic from _monitor_tick.

    Logic: if gps_sats < gps_min_sats and gps_fix < 3 and not gps_warning_sent
           -> GPS_DEGRADED
    """

    GPS_MIN_SATS = 6

    @staticmethod
    def _evaluate_gps(gps_sats, gps_fix, gps_min_sats, gps_warning_sent=False):
        """Replicate the GPS check from _monitor_tick.

        Returns (event, new_gps_warning_sent).
        """
        event = None
        if gps_sats < gps_min_sats and gps_fix < 3 and not gps_warning_sent:
            event = 'GPS_DEGRADED'
            gps_warning_sent = True
        elif gps_sats >= gps_min_sats:
            gps_warning_sent = False
        return event, gps_warning_sent

    def test_good_gps_no_event(self):
        event, _ = self._evaluate_gps(10, 3, self.GPS_MIN_SATS)
        assert event is None

    def test_low_sats_good_fix_no_event(self):
        """Low satellites but good fix type (>= 3) should not fire (fix check fails)."""
        event, _ = self._evaluate_gps(4, 3, self.GPS_MIN_SATS)
        assert event is None

    def test_good_sats_bad_fix_no_event(self):
        """Enough satellites even with bad fix should not fire (sat check fails)."""
        event, _ = self._evaluate_gps(8, 2, self.GPS_MIN_SATS)
        assert event is None

    def test_low_sats_and_bad_fix_triggers(self):
        event, sent = self._evaluate_gps(3, 2, self.GPS_MIN_SATS)
        assert event == 'GPS_DEGRADED'
        assert sent is True

    def test_gps_warning_not_repeated(self):
        event, _ = self._evaluate_gps(3, 2, self.GPS_MIN_SATS, gps_warning_sent=True)
        assert event is None

    def test_gps_recovery_resets_flag(self):
        """When satellites recover, the flag must be reset."""
        _, sent = self._evaluate_gps(8, 3, self.GPS_MIN_SATS, gps_warning_sent=True)
        assert sent is False


class TestGroundResetLogic:
    """Verify that safety flags reset when the drone is not armed or not in air.

    _monitor_tick returns early and resets all flags when not armed/in-air.
    """

    @staticmethod
    def _should_reset(is_armed, is_in_air):
        """Replicate the guard at the top of _monitor_tick."""
        return not is_armed or not is_in_air

    def test_disarmed_on_ground_resets(self):
        assert self._should_reset(False, False) is True

    def test_armed_on_ground_resets(self):
        assert self._should_reset(True, False) is True

    def test_disarmed_in_air_resets(self):
        assert self._should_reset(False, True) is True

    def test_armed_in_air_does_not_reset(self):
        assert self._should_reset(True, True) is False
