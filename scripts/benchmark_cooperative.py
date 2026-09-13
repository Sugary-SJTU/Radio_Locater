import sys,json
from pathlib import Path
from dataclasses import asdict, replace
from benchmark_problem3 import EngineClient,ROOT
from problems.problem3.config import Problem3Settings
from problems.problem3.shared import Problem3State,Problem3Executor,JsonlRunLogger
from problems.problem3.strategies import CooperativeBearingTourStrategy, IntegratedBearingTourStrategy
variant=sys.argv[1]; seeds=list(map(int,sys.argv[2:])) or [1,2,2026,5,12,19]
results=[]
for seed in seeds:
    settings=Problem3Settings()
    if variant=='integrated7': settings=replace(settings,tour_polygon_sides=7,tour_polygon_radius_m=1000.0,tour_scan_origin=False)
    state=Problem3State(settings); client=EngineClient(seed)
    logger=JsonlRunLogger(ROOT/f'res/logs/problem3/{variant}_{seed}.jsonl')
    try:
        executor=Problem3Executor(client,state,logger,variant); executor.enter()
        (IntegratedBearingTourStrategy(settings) if variant.startswith('integrated') else CooperativeBearingTourStrategy(settings,joint=variant=='joint')).run(executor)
    finally: logger.close()
    result=dict(seed=seed,variant=variant,source_count=len(client.engine.scenario.sources),time_s=state.virtual_time_s,resolved=state.all_resolved(),**asdict(state.counters))
    results.append(result); print(json.dumps(result),flush=True)
(ROOT/f'res/tables/problem3/{variant}_comparison.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
