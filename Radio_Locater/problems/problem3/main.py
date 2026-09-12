"""问题 3 的模拟器运行入口。

运行前需要先启动模拟器对应的问题 3 测试模块，并设置 ``CUMCM_ROBOT_ID``。
程序按 ``/enter -> 串行动作 -> /exit`` 生命周期执行一次测试，把每个动作写入
JSONL 明文日志，并把汇总指标追加写入正式测试表。
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

from config.simulator import (
    CLEAR_SUCCESS,
    MEASURE_DIRECTION,
    MEASURE_NEAR,
    MEASURE_NO_SIGNAL,
)
from problems.problem3.config import (
    ACTION_LOG,
    AVAILABLE_CHANNELS,
    FORMAL_TEST_TABLE,
    ROBOT_ID,
    START_CHANNEL,
    START_POSITION,
    TARGET_CLEARANCE_RADIUS_M,
)
from problems.problem3.model import (
    ClearTask,
    DetectedSource,
    build_multi_task,
    build_single_task,
    choose_second_point,
    clear_point_from_measurements,
    discovery_centers,
    localization_circle,
    OPPORTUNISTIC_CLEAR_DETOUR_M,
    order_clear_tasks,
    should_add_second_bearing,
)
from radio_locator.client import SimulatorClient


class _ActionLogger:
    """按行追加 JSON 明文动作日志，避免程序中途退出丢失已执行动作。"""

    def __init__(self, path: Path) -> None:
        self.path = path
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._sequence = 0
        self._file = self.path.open("a", encoding="utf-8")

    def record(
        self,
        action: str,
        position: tuple[float, float] | None,
        channel: int | None,
        response: dict[str, Any],
    ) -> None:
        self._sequence += 1
        entry = {
            "sequence": self._sequence,
            "action": action,
            "position": position,
            "channel": channel,
            "response": response,
        }
        self._file.write(json.dumps(entry, ensure_ascii=False) + "\n")
        self._file.flush()

    def close(self) -> None:
        self._file.close()


def _measure(
    client: SimulatorClient,
    logger: _ActionLogger,
    position: tuple[float, float],
    channel: int,
    current_position: tuple[float, float],
    current_channel: int,
    virtual_time: float,
) -> tuple[dict[str, Any], tuple[float, float], int, float]:
    """执行一次 /measure 并返回响应、新位置、新频道和新虚拟时刻。"""

    response = client.measure(position, channel)
    logger.record("/measure", position, channel, response)
    return (
        response,
        position,
        channel,
        float(response.get("virtual_time_s", virtual_time)),
    )


def _clear(
    client: SimulatorClient,
    logger: _ActionLogger,
    position: tuple[float, float],
    channel: int,
    current_position: tuple[float, float],
    virtual_time: float,
) -> tuple[dict[str, Any], tuple[float, float], float]:
    """执行一次 /clear；该动作不改变测向机频道。"""

    response = client.clear(position, channel)
    logger.record("/clear", position, channel, response)
    return response, position, float(response.get("virtual_time_s", virtual_time))


def _clear_near_source(
    client: SimulatorClient,
    logger: _ActionLogger,
    channel: int,
    center: tuple[float, float],
    current_position: tuple[float, float],
    virtual_time: float,
) -> tuple[bool, tuple[float, float], float]:
    """发现阶段遇到 ``near`` 时，在 5 m 邻域内尝试直接清除。"""

    offsets = [
        (0.0, 0.0),
        (12.0, 0.0),
        (-12.0, 0.0),
        (0.0, 12.0),
        (0.0, -12.0),
    ]
    for offset_x, offset_y in offsets:
        position = (center[0] + offset_x, center[1] + offset_y)
        response, current_position, virtual_time = _clear(
            client,
            logger,
            position,
            channel,
            current_position,
            virtual_time,
        )
        if response.get("clear_result") == CLEAR_SUCCESS:
            return True, current_position, virtual_time
    return False, current_position, virtual_time


def _try_clear_at_estimate(
    client: SimulatorClient,
    logger: _ActionLogger,
    channel: int,
    estimate: tuple[float, float],
    stations: list[tuple[float, float]],
    bearings: list[float],
    current_position: tuple[float, float],
    current_channel: int,
    virtual_time: float,
) -> tuple[bool, tuple[float, float], int, float]:
    """尝试在交会估计点清除；失败则就地补测并再次精修清除。"""

    response, current_position, virtual_time = _clear(
        client,
        logger,
        estimate,
        channel,
        current_position,
        virtual_time,
    )
    if response.get("clear_result") == CLEAR_SUCCESS:
        return True, current_position, current_channel, virtual_time

    response, current_position, current_channel, virtual_time = _measure(
        client,
        logger,
        estimate,
        channel,
        current_position,
        current_channel,
        virtual_time,
    )
    if response.get("measure_result") == MEASURE_NEAR:
        response, current_position, virtual_time = _clear(
            client,
            logger,
            estimate,
            channel,
            current_position,
            virtual_time,
        )
        return (
            response.get("clear_result") == CLEAR_SUCCESS,
            current_position,
            current_channel,
            virtual_time,
        )
    if response.get("measure_result") == MEASURE_DIRECTION:
        stations.append(estimate)
        bearings.append(float(response["svd_deg"]))
        refined = clear_point_from_measurements(stations, bearings)
        response, current_position, virtual_time = _clear(
            client,
            logger,
            refined,
            channel,
            current_position,
            virtual_time,
        )
        return (
            response.get("clear_result") == CLEAR_SUCCESS,
            current_position,
            current_channel,
            virtual_time,
        )
    return False, current_position, current_channel, virtual_time


def _localize_and_clear(
    client: SimulatorClient,
    logger: _ActionLogger,
    task: ClearTask,
    current_position: tuple[float, float],
    current_channel: int,
    virtual_time: float,
) -> tuple[bool, tuple[float, float], int, float]:
    """对一个待清除任务完成定位和清除。"""

    channel = task.channel
    second = choose_second_point(task, current_position)

    if second is None:
        # 已有至少两条示向度，直接用最小二乘交会点清除。
        stations = list(task.stations)
        bearings = list(task.bearings)
        estimate = clear_point_from_measurements(stations, bearings)
        return _try_clear_at_estimate(
            client,
            logger,
            channel,
            estimate,
            stations,
            bearings,
            current_position,
            current_channel,
            virtual_time,
        )

    response, current_position, current_channel, virtual_time = _measure(
        client,
        logger,
        second,
        channel,
        current_position,
        current_channel,
        virtual_time,
    )
    result = response.get("measure_result")
    if result == MEASURE_NEAR:
        response, current_position, virtual_time = _clear(
            client,
            logger,
            second,
            channel,
            current_position,
            virtual_time,
        )
        return (
            response.get("clear_result") == CLEAR_SUCCESS,
            current_position,
            current_channel,
            virtual_time,
        )

    if result == MEASURE_NO_SIGNAL:
        other = (
            task.second_right
            if second == task.second_left
            else task.second_left
        )
        assert other is not None
        response, current_position, current_channel, virtual_time = _measure(
            client,
            logger,
            other,
            channel,
            current_position,
            current_channel,
            virtual_time,
        )
        result = response.get("measure_result")
        if result == MEASURE_NEAR:
            response, current_position, virtual_time = _clear(
                client,
                logger,
                other,
                channel,
                current_position,
                virtual_time,
            )
            return (
                response.get("clear_result") == CLEAR_SUCCESS,
                current_position,
                current_channel,
                virtual_time,
            )
        if result != MEASURE_DIRECTION:
            stations = [task.stations[0]]
            bearings = [task.bearings[0]]
            return _try_clear_at_estimate(
                client,
                logger,
                channel,
                task.clear_point,
                stations,
                bearings,
                current_position,
                current_channel,
                virtual_time,
            )
        second = other

    stations = [task.stations[0], second]
    bearings = [task.bearings[0], float(response["svd_deg"])]
    estimate = clear_point_from_measurements(stations, bearings)
    return _try_clear_at_estimate(
        client,
        logger,
        channel,
        estimate,
        stations,
        bearings,
        current_position,
        current_channel,
        virtual_time,
    )


def run_test(
    client: SimulatorClient,
    action_log: Path = ACTION_LOG,
) -> dict[str, Any]:
    """运行一次问题 3 测试，返回统计指标。"""

    if not client.robot_id:
        raise ValueError("CUMCM_ROBOT_ID 未设置，无法进入模拟器")

    logger = _ActionLogger(action_log)
    real_start = time.perf_counter()
    try:
        enter_response = client.enter()
        logger.record("/enter", None, None, enter_response)
        virtual_time = float(enter_response.get("virtual_time_s", 0.0))
        current_position = START_POSITION
        current_channel = START_CHANNEL
        unfound = set(AVAILABLE_CHANNELS)
        measurements: dict[int, list[DetectedSource]] = {}
        extra_attempted: set[int] = set()
        cleared: set[int] = set()

        discovery_centers_list = discovery_centers()
        for center_index, center in enumerate(discovery_centers_list):
            pending = set(unfound)
            while pending:
                channel = min(pending, key=lambda value: abs(value - current_channel))
                pending.remove(channel)
                response, current_position, current_channel, virtual_time = _measure(
                    client,
                    logger,
                    center,
                    channel,
                    current_position,
                    current_channel,
                    virtual_time,
                )
                result = response.get("measure_result")
                if result == MEASURE_DIRECTION:
                    measurements[channel] = [
                        DetectedSource(
                            channel=channel,
                            station=center,
                            bearing_deg=float(response["svd_deg"]),
                        )
                    ]
                    unfound.remove(channel)
                elif result == MEASURE_NEAR:
                    cleared_now, current_position, virtual_time = _clear_near_source(
                        client,
                        logger,
                        channel,
                        center,
                        current_position,
                        virtual_time,
                    )
                    if cleared_now:
                        unfound.remove(channel)
                        cleared.add(channel)
                    else:
                        unfound.remove(channel)

            # 当前点同时承担“顺手补测”的职责：对已有单示向度源，如果该点能
            # 形成良好交会，就补测一次，降低第二阶段绕路成本。
            for channel, history in list(measurements.items()):
                if (
                    channel in extra_attempted
                    or channel in cleared
                    or len(history) != 1
                ):
                    continue
                if not should_add_second_bearing(history[0], center):
                    continue
                extra_attempted.add(channel)
                response, current_position, current_channel, virtual_time = _measure(
                    client,
                    logger,
                    center,
                    channel,
                    current_position,
                    current_channel,
                    virtual_time,
                )
                if response.get("measure_result") == MEASURE_DIRECTION:
                    history.append(
                        DetectedSource(
                            channel=channel,
                            station=center,
                            bearing_deg=float(response["svd_deg"]),
                        )
                    )
                elif response.get("measure_result") == MEASURE_NEAR:
                    cleared_now, current_position, virtual_time = _clear_near_source(
                        client,
                        logger,
                        channel,
                        center,
                        current_position,
                        virtual_time,
                    )
                    if cleared_now:
                        cleared.add(channel)

            # 改进3：已定位且顺路的源，在扫描阶段立即清除。
            next_center = (
                discovery_centers_list[center_index + 1]
                if center_index + 1 < len(discovery_centers_list)
                else None
            )
            if next_center is not None:
                for channel, history in list(measurements.items()):
                    if channel in cleared or len(history) < 2:
                        continue
                    stations = [item.station for item in history]
                    bearings = [item.bearing_deg for item in history]
                    circle = localization_circle(stations, bearings)
                    if circle is None or circle[1] > TARGET_CLEARANCE_RADIUS_M:
                        continue
                    clear_point, _ = circle
                    base_distance = (
                        (current_position[0] - next_center[0]) ** 2
                        + (current_position[1] - next_center[1]) ** 2
                    ) ** 0.5
                    detour = (
                        (current_position[0] - clear_point[0]) ** 2
                        + (current_position[1] - clear_point[1]) ** 2
                    ) ** 0.5
                    detour += (
                        (clear_point[0] - next_center[0]) ** 2
                        + (clear_point[1] - next_center[1]) ** 2
                    ) ** 0.5
                    detour -= base_distance
                    if detour > OPPORTUNISTIC_CLEAR_DETOUR_M:
                        continue
                    cleared_now, current_position, current_channel, virtual_time = (
                        _localize_and_clear(
                            client,
                            logger,
                            build_multi_task(channel, history),
                            current_position,
                            current_channel,
                            virtual_time,
                        )
                    )
                    if cleared_now:
                        cleared.add(channel)

        tasks: list[ClearTask] = []
        for channel in sorted(measurements):
            if channel in cleared:
                continue
            history = measurements[channel]
            if len(history) >= 2:
                tasks.append(build_multi_task(channel, history))
            else:
                tasks.append(build_single_task(history[0]))

        for task in order_clear_tasks(tasks, current_position):
            cleared_now, current_position, current_channel, virtual_time = (
                _localize_and_clear(
                    client,
                    logger,
                    task,
                    current_position,
                    current_channel,
                    virtual_time,
                )
            )
            if cleared_now:
                cleared.add(task.channel)

        exit_response = client.exit()
        logger.record("/exit", None, None, exit_response)
        virtual_time = float(exit_response.get("virtual_time_s", virtual_time))
    finally:
        logger.close()

    real_duration = time.perf_counter() - real_start
    cleared_count = len(cleared)
    average_time = (
        virtual_time / cleared_count if cleared_count > 0 else float("nan")
    )
    return {
        "cleared_count": cleared_count,
        "average_time_s": average_time,
        "virtual_time_s": virtual_time,
        "real_duration_s": real_duration,
    }


def _append_formal_row(
    table_path: Path,
    case_code: str,
    result: dict[str, Any],
) -> None:
    """把一次测试的汇总结果追加写入正式测试表。"""

    try:
        from openpyxl import Workbook, load_workbook
    except ImportError as exc:  # pragma: no cover - 环境缺依赖时只跳过表格
        print(f"未安装 openpyxl，跳过表格写入：{exc}")
        return

    headers = ["测试案例编码", "清除干扰源个数", "平均定位清除时间", "程序运行时间"]
    table_path.parent.mkdir(parents=True, exist_ok=True)
    if table_path.exists():
        workbook = load_workbook(table_path)
        worksheet = workbook.active
    else:
        workbook = Workbook()
        worksheet = workbook.active
        worksheet.append(headers)

    average_time = result["average_time_s"]
    if average_time != average_time:  # NaN 检查，避免写入非法数值。
        average_time = None
    else:
        average_time = round(float(average_time), 6)
    worksheet.append(
        [
            case_code,
            result["cleared_count"],
            average_time,
            round(float(result["real_duration_s"]), 6),
        ]
    )
    workbook.save(table_path)


def run_problem3(case_code: str | None = None) -> dict[str, Any]:
    """创建客户端、运行测试并保存汇总。"""

    client = SimulatorClient(robot_id=ROBOT_ID)
    result = run_test(client)
    _append_formal_row(FORMAL_TEST_TABLE, case_code or "未填写", result)
    return result


def main(case_code: str | None = None) -> None:
    """命令行入口；打印测试结果。"""

    result = run_problem3(case_code)
    print(
        "问题 3 测试完成："
        f"清除 {result['cleared_count']} 个，"
        f"平均定位清除时间 {result['average_time_s']:.3f} s，"
        f"虚拟总时间 {result['virtual_time_s']:.3f} s，"
        f"程序运行时间 {result['real_duration_s']:.3f} s"
    )


if __name__ == "__main__":
    main()
