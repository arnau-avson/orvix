"""Multi-frame fusion tests."""
from delivery_robot.fusion.temporal import TemporalStateVoter


class TestTemporalStateVoter:
    def test_empty_window_returns_fallback(self):
        v = TemporalStateVoter(window_size=5, min_agreement=3)
        assert v.fused() == "unknown"
        assert v.fused(fallback="x") == "x"

    def test_below_min_agreement_returns_fallback(self):
        v = TemporalStateVoter(window_size=5, min_agreement=3)
        v.observe("red")
        v.observe("green")
        assert v.fused() == "unknown"

    def test_majority_wins(self):
        v = TemporalStateVoter(window_size=5, min_agreement=3)
        for s in ["red", "red", "red", "green", "yellow"]:
            v.observe(s)
        assert v.fused() == "red"

    def test_window_evicts_old(self):
        v = TemporalStateVoter(window_size=3, min_agreement=2)
        for s in ["red", "red", "red"]:
            v.observe(s)
        assert v.fused() == "red"
        # Push 2 greens; window now [red, green, green]
        v.observe("green")
        v.observe("green")
        assert v.fused() == "green"

    def test_reset_clears(self):
        v = TemporalStateVoter(window_size=3, min_agreement=2)
        v.observe("red")
        v.observe("red")
        assert v.fused() == "red"
        v.reset()
        assert v.fused() == "unknown"

    def test_invalid_min_agreement(self):
        import pytest
        with pytest.raises(ValueError):
            TemporalStateVoter(window_size=3, min_agreement=5)

    def test_filled_property(self):
        v = TemporalStateVoter(window_size=3, min_agreement=2)
        assert not v.filled
        v.observe("a")
        v.observe("b")
        assert not v.filled
        v.observe("c")
        assert v.filled

    def test_latest(self):
        v = TemporalStateVoter(window_size=3, min_agreement=2)
        assert v.latest is None
        v.observe("red")
        assert v.latest == "red"
        v.observe("green")
        assert v.latest == "green"
