"""同种子配对半径/频道顺序消融；真值仅在模拟器与事后统计中使用。"""
import json,time
from dataclasses import asdict,replace
from benchmark_problem3 import ROOT,EngineClient
from problems.problem3.config import Problem3Settings
from problems.problem3.shared import Problem3State,Problem3Executor
from problems.problem3.strategies import IntegratedBearingTourStrategy
class NullLogger:
    def write(self,record): pass
results=[]
output=ROOT/'res/tables/problem3/radius_channel_ablation.json'
for radius,order in [(1200,'legacy'),(1200,'alternating'),(1175,'legacy'),(1175,'alternating'),(1150,'legacy'),(1150,'alternating'),(1130,'legacy'),(1130,'alternating')]:
    started=time.monotonic()
    for seed in range(60):
        settings=replace(Problem3Settings(),tour_endgame_mode="legacy",tour_polygon_radius_m=float(radius),tour_channel_order=order)
        state=Problem3State(settings);client=EngineClient(seed)
        try:
            executor=Problem3Executor(client,state,NullLogger(),'integrated_bearing_tour');executor.enter()
            IntegratedBearingTourStrategy(settings).run(executor)
            error=None
        except Exception as exc:
            error=f'{type(exc).__name__}: {exc}'
        results.append(dict(radius=radius,order=order,seed=seed,split='development' if seed<30 else 'validation',source_count=len(client.engine.scenario.sources),time_s=state.virtual_time_s,resolved=state.all_resolved(),error=error,**asdict(state.counters)))
    output.write_text(json.dumps(results,indent=2),encoding='utf-8')
    sample=results[-60:]
    print(radius,order,'mean',round(sum(r['time_s'] for r in sample)/60,2),'resolved',sum(r['resolved'] for r in sample),'wall',round(time.monotonic()-started,1),flush=True)
