from xarm6_gello_teleop.motion_profiles import motion_profile


def test_safe_and_responsive_profiles_have_distinct_speed_limits() -> None:
    safe = motion_profile("safe")
    responsive = motion_profile("responsive")

    assert safe.max_delta_rad == 0.004
    assert safe.max_velocity_rad_s == 0.20
    assert responsive.max_delta_rad == 0.005
    assert responsive.max_velocity_rad_s == 0.25
