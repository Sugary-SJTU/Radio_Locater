"""根运行入口的程序实际运行时间测试。"""

import run as run_entrypoint


def test_run_reports_wall_clock_elapsed_time(monkeypatch, capsys) -> None:
    ticks = iter((10.0, 12.345))
    called: list[bool] = []
    monkeypatch.setattr(run_entrypoint.time, "perf_counter", lambda: next(ticks))
    monkeypatch.setattr(run_entrypoint, "main", lambda: called.append(True))

    run_entrypoint.run()

    assert called == [True]
    assert "程序实际运行时间：2.35 s" in capsys.readouterr().out


def test_run_reports_elapsed_time_when_command_fails(monkeypatch, capsys) -> None:
    ticks = iter((20.0, 20.5))
    monkeypatch.setattr(run_entrypoint.time, "perf_counter", lambda: next(ticks))

    def fail() -> None:
        raise RuntimeError("test failure")

    monkeypatch.setattr(run_entrypoint, "main", fail)
    try:
        run_entrypoint.run()
    except RuntimeError:
        pass
    else:
        raise AssertionError("run() should preserve command failures")

    assert "程序实际运行时间：0.50 s" in capsys.readouterr().out
