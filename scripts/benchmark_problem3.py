import sys, json, importlib.util
from pathlib import Path
from dataclasses import asdict
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'src')]
from radio_locator.local_simulator import SimulatorEngine, SimulatorScenario
from radio_locator.client import ActionExchange
from problems.problem3.config import Problem3Settings
from problems.problem3.shared import Problem3State, Problem3Executor, JsonlRunLogger
from problems.problem3.strategies import RobustPolygonRollingStrategy
class EngineClient:
    def __init__(self, seed):
        self.engine=SimulatorEngine(SimulatorScenario.generate(3,seed),'benchmark',monotonic_s=lambda:0)
        self.counter=0
    def check_connection(self): pass
    def call(self,path,position=None,channel=None):
        self.counter+=1
        payload=dict(arena_id='default',robot_id='benchmark',request_id=str(self.counter))
        if position is not None: payload.update(position=dict(x=position[0],y=position[1]),channel=channel)
        status,response=self.engine.process(path,payload)
        assert status==200 and response['accepted'], response
        return ActionExchange(path,payload,status,response)
    def enter(self): return self.call('/enter')
    def measure(self,position,channel): return self.call('/measure',position,channel)
    def clear(self,position,channel): return self.call('/clear',position,channel)
def run(seed,baseline=False):
    strategy=RobustPolygonRollingStrategy
    if baseline:
        spec=importlib.util.spec_from_file_location('baseline_strategy',ROOT/'res/tables/problem3/strategies_before_time_optimization.py')
        module=importlib.util.module_from_spec(spec); sys.modules[spec.name]=module; spec.loader.exec_module(module)
        strategy=module.RobustPolygonRollingStrategy
    settings=Problem3Settings()
    state=Problem3State(settings); client=EngineClient(seed)
    label='before' if baseline else 'after'
    logger=JsonlRunLogger(ROOT/f'res/logs/problem3/optimization_{label}_{seed}.jsonl')
    try:
        executor=Problem3Executor(client,state,logger,strategy.name); executor.enter(); strategy(settings).run(executor)
    finally: logger.close()
    result=dict(seed=seed,variant=label,source_count=len(client.engine.scenario.sources),time_s=state.virtual_time_s,resolved=state.all_resolved(),**asdict(state.counters))
    print(json.dumps(result),flush=True)
    return result
if __name__=='__main__':
    baseline='--baseline' in sys.argv
    seeds=[int(s) for s in sys.argv[1:] if not s.startswith('--')] or [1,2,2026]
    results=[run(s,baseline) for s in seeds]
    (ROOT/f'res/tables/problem3/optimization_{"before" if baseline else "after"}.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
