"""本地问题 3、4 模拟器的场景、物理规则、计时与幂等测试。"""

from radio_locator.local_simulator import (
    InterferenceSource,
    ProtocolError,
    SimulatorEngine,
    SimulatorScenario,
    decode_request_json,
)


def _base(request_id: str) -> dict[str, object]:
    """构造测试用公共字段。"""

    return {"arena_id": "default", "robot_id": "demo", "request_id": request_id}


def _action(
    request_id: str,
    x_m: float,
    y_m: float,
    channel: int,
) -> dict[str, object]:
    """构造测试用 measure/clear 请求体。"""

    return {
        **_base(request_id),
        "position": {"x": x_m, "y": y_m},
        "channel": channel,
    }


def test_seed_reproduces_sources_and_problem_types() -> None:
    """同题号同种子必须产生相同真值，且题 3 全向、题 4 两类兼有。"""

    first = SimulatorScenario.generate(3, 2026)
    second = SimulatorScenario.generate(3, 2026)
    mixed = SimulatorScenario.generate(4, 2026)
    assert 10 <= len(first.sources) <= 16
    assert first.sources == second.sources
    assert len({source.channel for source in first.sources}) == len(first.sources)
    assert all(source.source_type == "omnidirectional" for source in first.sources)
    assert {source.source_type for source in mixed.sources} == {
        "omnidirectional",
        "directional",
    }


def test_attachment_timing_sequence_and_clear_channel_semantics() -> None:
    """附件示例的移动、切换频道和清除计时应得到 0/105/111/194/199。"""

    engine = SimulatorEngine(
        SimulatorScenario.generate(3, 2026),
        "demo",
        timestamp_ms=lambda: 1_760_000_000_000,
    )
    assert engine.process("/enter", _base("enter-1"))[1]["virtual_time_s"] == 0
    assert engine.process("/measure", _action("measure-1", 300, 400, 1))[1][
        "virtual_time_s"
    ] == 105
    assert engine.process("/measure", _action("measure-2", 300, 400, 2))[1][
        "virtual_time_s"
    ] == 111
    clear_response = engine.process(
        "/clear", _action("clear-1", 300, 0, 3)
    )[1]
    assert clear_response["clear_result"] == "no_target_in_range"
    assert clear_response["virtual_time_s"] == 194
    # /clear 的 channel=3 不改变测向机仍处于频道 2 的状态。
    assert engine.process("/measure", _action("measure-3", 300, 0, 2))[1][
        "virtual_time_s"
    ] == 199


def test_idempotent_retry_does_not_repeat_action() -> None:
    """相同 request_id 和动作返回首次响应，改动内容则返回 HTTP 409。"""

    engine = SimulatorEngine(
        SimulatorScenario.generate(3, 9),
        "demo",
        timestamp_ms=lambda: 123,
    )
    engine.process("/enter", _base("enter"))
    request = _action("measure", 30, 40, 1)
    first = engine.process("/measure", request)
    second = engine.process("/measure", request)
    conflict = engine.process("/measure", _action("measure", 31, 40, 1))
    assert first == second
    assert engine.virtual_time_s == 15
    assert conflict[0] == 409
    assert conflict[1]["accepted"] is False


def test_unknown_fields_are_business_rejection_and_not_id_occupying() -> None:
    """未知字段返回 200/accepted=false，修正后可复用同一 request_id。"""

    engine = SimulatorEngine(SimulatorScenario.generate(3, 1), "demo")
    invalid = {**_base("enter"), "typo": True}
    status, response = engine.process("/enter", invalid)
    assert status == 200 and response["accepted"] is False
    status, response = engine.process("/enter", _base("enter"))
    assert status == 200 and response["accepted"] is True


def test_real_timeout_rejects_new_but_not_idempotent_requests() -> None:
    """现实时间耗尽后拒绝新动作，但已经接受动作的幂等重试仍可读取原响应。"""

    clock = [0.0]
    engine = SimulatorEngine(
        SimulatorScenario.generate(3, 2),
        "demo",
        monotonic_s=lambda: clock[0],
    )
    enter_request = _base("enter")
    first_enter = engine.process("/enter", enter_request)
    clock[0] = 1_200.0
    assert engine.process("/enter", enter_request) == first_enter
    status, response = engine.process(
        "/measure", _action("late-measure", 0, 0, 1)
    )
    assert status == 200 and response["accepted"] is False


def test_json_decoder_rejects_duplicate_keys_bom_and_non_object() -> None:
    """附件禁止重复键、UTF-8 BOM 和非对象顶层 JSON。"""

    invalid_bodies = (
        b'{"request_id":"a","request_id":"b"}',
        b"\xef\xbb\xbf{}",
        b"[]",
    )
    for body in invalid_bodies:
        try:
            decode_request_json(body)
        except ProtocolError:
            continue
        raise AssertionError(f"invalid JSON body was accepted: {body!r}")


def test_directional_coverage_fixed_error_and_clear_rules() -> None:
    """定向覆盖影响测量但不影响清除，同地点不同 ID 的示向误差保持固定。"""

    source = InterferenceSource(
        channel=7,
        x_m=100.0,
        y_m=0.0,
        reception_radius_m=1_200.0,
        source_type="directional",
        direction_deg=180.0,
    )
    engine = SimulatorEngine(SimulatorScenario(4, 88, [source]), "demo")
    engine.process("/enter", _base("enter"))
    first = engine.process("/measure", _action("m1", 0, 0, 7))[1]
    repeated = engine.process("/measure", _action("m2", 0, 0, 7))[1]
    assert first["measure_result"] == repeated["measure_result"] == "direction"
    assert first["svd_deg"] == repeated["svd_deg"]
    outside = engine.process("/measure", _action("m3", 200, 0, 7))[1]
    assert outside["measure_result"] == "no_signal"
    at_source = engine.process("/measure", _action("m4", 100, 0, 7))[1]
    assert at_source["measure_result"] == "near"
    cleared = engine.process("/clear", _action("c1", 100, 0, 7))[1]
    assert cleared["clear_result"] == "success"
    repeated_clear = engine.process("/clear", _action("c2", 100, 0, 7))[1]
    assert repeated_clear["clear_result"] == "no_target_in_range"
