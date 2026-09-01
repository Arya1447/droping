from geofence import check_geofence, point_in_polygon

# A simple square roughly 100m x 100m around (-6.900, 107.600), matching
# the equirectangular scale assumptions used elsewhere in this project.
SQUARE = [
    (-6.8995, 107.5995),
    (-6.8995, 107.6005),
    (-6.9005, 107.6005),
    (-6.9005, 107.5995),
]


def test_point_inside_polygon():
    assert point_in_polygon(-6.9000, 107.6000, SQUARE) is True


def test_point_outside_polygon():
    assert point_in_polygon(-6.9100, 107.6000, SQUARE) is False


def test_point_in_polygon_degenerate_polygon_is_false():
    assert point_in_polygon(-6.9000, 107.6000, [(-6.9, 107.6), (-6.9, 107.61)]) is False


def test_check_geofence_unconfigured_is_fail_closed():
    status = check_geofence(-6.9, 107.6, None)
    assert status.inside is False
    assert status.polygon_configured is False
    assert status.source == "UNKNOWN"


def test_check_geofence_missing_position_is_fail_closed():
    status = check_geofence(None, None, SQUARE)
    assert status.inside is False
    assert status.polygon_configured is True


def test_check_geofence_inside():
    status = check_geofence(-6.9000, 107.6000, SQUARE)
    assert status.inside is True
    assert status.source == "CONFIGURED"


def test_check_geofence_outside():
    status = check_geofence(-6.9500, 107.6000, SQUARE)
    assert status.inside is False
    assert status.source == "CONFIGURED"
