"""末段精确路线与低置信度共享补测配对试验。"""
import json,time,sys
from dataclasses import asdict,replace
from benchmark_problem3 import ROOT,EngineClient
from problems.problem3.config import Problem3Settings
from problems.problem3.shared import Problem3State,Problem3Executor
from problems.problem3.strategies import IntegratedBearingTourStrategy
class NullLogger:
 def write(self,record):pass
start_seed=int(sys.argv[1]) if len(sys.argv)>1 else 0
results=[]
for mode in ['legacy','exact','probe']:
 start=time.monotonic()
 for seed in range(start_seed,start_seed+60):
  settings=replace(Problem3Settings(),tour_endgame_mode=mode)
  state=Problem3State(settings);client=EngineClient(seed)
  try:
   executor=Problem3Executor(client,state,NullLogger(),mode);executor.enter()
   IntegratedBearingTourStrategy(settings).run(executor);error=None
  except Exception as exc:error=f'{type(exc).__name__}: {exc}'
  results.append(dict(mode=mode,seed=seed,source_count=len(client.engine.scenario.sources),time_s=state.virtual_time_s,resolved=state.all_resolved(),error=error,**asdict(state.counters)))
 (ROOT/f'res/tables/problem3/endgame_comparison_{start_seed}.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
 sample=results[-60:];print(mode,'mean',sum(r['time_s'] for r in sample)/60,'resolved',sum(r['resolved'] for r in sample),'wall',time.monotonic()-start,flush=True)
