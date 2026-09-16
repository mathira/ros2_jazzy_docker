"""The monitor must show mission progress, not only the raw grid fraction."""

import pytest

from turtleboot3_autonomous_nav.training_monitor import format_status


def test_status_line_leads_with_reachable_coverage():
    """Reachable coverage is the number that says how much is left to do."""
    line = format_status((7.0, 120.0, 0.855, 1030.4, 0.9418, 0.0, 0.964), 4.3)
    assert 'ep    7' in line
    assert 'alcanzable  96.4%' in line
    assert 'grid  85.500%' in line
    assert 'eps 0.942' in line


def test_a_metrics_vector_without_reachable_coverage_still_prints():
    """An older trainer publishing six values must not break the monitor."""
    assert 'n/a' in format_status((1.0, 1.0, 0.0, 0.0, 1.0, 0.0), 0.0)


def test_status_line_requires_the_core_metrics():
    with pytest.raises(ValueError):
        format_status((1.0, 2.0, 3.0), 1.0)
