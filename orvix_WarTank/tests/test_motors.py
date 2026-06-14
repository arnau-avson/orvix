"""Motor controller tests."""
from delivery_robot.hardware.motors import MockMotorController
from delivery_robot.navigation import NavigationAction


class TestMockMotorController:
    def test_records_history(self):
        m = MockMotorController()
        m.execute(NavigationAction.GO)
        m.execute(NavigationAction.WAIT, target_heading_deg=45.0)
        m.execute(NavigationAction.STOP)
        assert m.history == [
            ("go", None),
            ("wait", 45.0),
            ("stop", None),
        ]

    def test_emergency_stop(self):
        m = MockMotorController()
        m.execute(NavigationAction.GO)
        m.emergency_stop()
        assert m.history[-1] == ("estop", None)

    def test_last_action_tracked(self):
        m = MockMotorController()
        assert m.last_action is None
        m.execute(NavigationAction.GO)
        assert m.last_action == NavigationAction.GO
        m.execute(NavigationAction.STOP)
        assert m.last_action == NavigationAction.STOP

    def test_context_manager(self):
        with MockMotorController() as m:
            m.execute(NavigationAction.GO)
            assert m.last_action == NavigationAction.GO
