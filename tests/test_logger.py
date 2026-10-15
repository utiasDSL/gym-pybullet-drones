from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pytest

from gym_pybullet_drones.utils.Logger import Logger

plt.switch_backend("Agg")


@pytest.fixture(
    params=[(100, 7, 0), (240, 23, 0), (48, 5, 0), (100, 1, 0), (100, 8, 0), (30, 7, 1)]
)
def logger(request, tmp_path):
    frequency, samples, duration = request.param
    result = Logger(frequency, str(tmp_path), num_drones=2, duration_sec=duration)
    for step in range(samples):
        for drone in range(2):
            # Export/plot retain their existing uniform time grid, not supplied timestamps.
            result.log(drone, 5.0 + step / frequency, np.arange(20) + step + drone)
    return result


def test_csv_time_axis_matches_stored_samples(logger):
    logger.save_as_csv()
    expected_time = np.arange(logger.timestamps.shape[1]) / logger.LOGGING_FREQ_HZ
    files = list(Path(logger.OUTPUT_FOLDER).glob("save-flight-*/*.csv"))
    assert len(files) == 46
    for path in files:
        values = np.loadtxt(path, delimiter=",", ndmin=2)
        assert values.shape == (len(expected_time), 2)
        np.testing.assert_allclose(values[:, 0], expected_time)
        if path.name in ("x0.csv", "x1.csv"):
            drone = int(path.stem[-1])
            np.testing.assert_array_equal(values[:, 1], logger.states[drone, 0])


def test_plot_time_axis_matches_stored_samples(logger, monkeypatch):
    monkeypatch.setattr(plt, "show", lambda: None)
    try:
        logger.plot()
        figure = plt.gcf()
        expected_time = np.arange(logger.timestamps.shape[1]) / logger.LOGGING_FREQ_HZ
        assert len(figure.axes) == 20
        for axes in figure.axes:
            assert axes.lines
            for line in axes.lines:
                np.testing.assert_allclose(line.get_xdata(), expected_time)
        np.testing.assert_array_equal(
            figure.axes[0].lines[0].get_ydata(), logger.states[0, 0]
        )
    finally:
        plt.close("all")
