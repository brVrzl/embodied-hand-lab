from __future__ import annotations

import numpy as np

from tools.serve_realsense_viewer import _make_preview_panel, _viewer_filter_config


def test_viewer_filter_profiles_keep_global_hole_filling_disabled() -> None:
    spatial = _viewer_filter_config("spatial")
    static = _viewer_filter_config("static")

    assert spatial["spatial"]["enabled"] is True
    assert spatial["temporal"]["enabled"] is False
    assert static["temporal"]["enabled"] is True
    assert static["hole_filling"]["enabled"] is False


def test_preview_uses_fixed_range_and_renders_invalid_depth_black() -> None:
    rgb = np.zeros((2, 3, 3), dtype=np.uint8)
    depth = np.array([[0.0, 0.3, 0.9], [1.5, 2.0, np.nan]], dtype=np.float32)

    panel = _make_preview_panel(rgb, depth, depth_min_m=0.3, depth_max_m=1.5)
    depth_panel = panel[:, 3:]

    np.testing.assert_array_equal(depth_panel[0, 0], [0, 0, 0])
    np.testing.assert_array_equal(depth_panel[1, 1], [0, 0, 0])
    np.testing.assert_array_equal(depth_panel[1, 2], [0, 0, 0])
