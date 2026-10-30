from src.pulse_odometry import PulseOdometry


def test_deg_to_pulse_home_is_stop_all_motors():
    po = PulseOdometry("config/arm.json")
    for m in range(1, 9):
        c = po.calibration()[m]
        home = c["home_deg"]
        stop = c["pulse_stop"]
        assert po.deg_to_pulse(m, home) == stop


def test_pulse_to_deg_stop_is_home_within_tolerance():
    po = PulseOdometry("config/arm.json")
    for m in range(1, 9):
        c = po.calibration()[m]
        home = c["home_deg"]
        stop = c["pulse_stop"]
        got = po.pulse_to_deg(m, stop)
        assert abs(got - home) <= 0.6


def test_roundtrip_deg_pulse_deg_within_tolerance():
    po = PulseOdometry("config/arm.json")
    for m in range(1, 9):
        c = po.calibration()[m]
        home = c["home_deg"]
        # test home +/- 40
        for delta in [-40.0, -20.0, 0.0, 20.0, 40.0]:
            deg_in = home + delta
            # clamp to joint range for safety
            deg_in_clamped = po.clamp(m, deg_in)
            p = po.deg_to_pulse(m, deg_in_clamped)
            deg_out = po.pulse_to_deg(m, p)
            assert abs(deg_out - deg_in_clamped) <= 1.5


def test_shoulder_coupling():
    po = PulseOdometry("config/arm.json", shoulder_coupling=1)
    thetas = [0, 90, 0, 0, 0, 50]
    motors = po.thetas_to_motors(thetas)
    assert motors[6] == motors[5]
    po2 = PulseOdometry("config/arm.json", shoulder_coupling=-1)
    motors2 = po2.thetas_to_motors(thetas)
    assert motors2[6] == 180 - motors2[5]


def test_thetas_to_motors_example():
    po = PulseOdometry("config/arm.json")
    motors = po.thetas_to_motors([0, 90, 0, 0, 0, 50])
    assert motors == {1: 50, 2: 0, 3: 90, 4: 90, 5: 90, 6: 90, 7: 0, 8: 90}


def test_clamping_at_joint_limits():
    po = PulseOdometry("config/arm.json")
    # motor 5 has limits [15, 165] in config
    assert po.clamp(5, 0) == 15
    assert po.clamp(5, 200) == 165
    assert po.clamp(1, -10) == 0
    assert po.clamp(1, 150) == 100
