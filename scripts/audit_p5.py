import sys,json,math
from pathlib import Path
ROOT=Path('Radio_Locater').resolve();sys.path[:0]=[str(ROOT),str(ROOT/'src')]
from problems.problem3.config import Problem3Settings
from problems.problem3.shared import Problem3State
from problems.problem3.coverage import optimize_remaining_route,held_karp_open_path,route_length
rows=[json.loads(x) for x in (ROOT/'res/logs/problem3/integrated_bearing_tour_seed1_run1_20260912_141046_254292.jsonl').read_text(encoding='utf-8').splitlines()]
state=Problem3State(Problem3Settings());P5=(-575.,-995.9292143521043);P6=(575.,-995.9292143521044)
for i,row in enumerate(rows):
 state.virtual_time_s=row['virtual_time_s'];state.position=tuple(row['position'].values());state.current_channel=row['current_channel']
 if row['action_type']=='measure':
  ch=row['target_channels'][0];r=row['raw_response'];kind=r['measure_result']
  if kind=='direction':state.apply_direction(ch,state.position,r['svd_deg'])
  elif kind=='no_signal':state.apply_no_signal(ch,state.position)
  elif kind=='near':state.apply_near(ch,state.position)
 elif row['action_type']=='clear':state.apply_clear(row['target_channels'][0],row['raw_response']['clear_result']=='success')
 if math.dist(state.position,P5)<.01 and i+1<len(rows) and math.dist(tuple(rows[i+1]['position'].values()),P5)>.01:
  print('P5 end',i,state.virtual_time_s)
  alltargets=[]
  for c,t in state.tracks.items():
   if t.status not in ['cleared','absent']:
    print(c,t.status,'n',len(t.measurements),'radius',None if t.clear_circle is None else t.clear_circle.radius,'center',None if t.clear_circle is None else t.clear_circle.center.tolist())
    if t.clear_circle is not None:alltargets.append((c,tuple(t.clear_circle.center)))
  targets=[(c,p) for c,p in alltargets if state.tracks[c].clear_circle.radius<=150]
  for label,ts in [('eligible',targets),('all_detected',alltargets)]:
   points=[P6]+[p for c,p in ts];names=['P6']+[str(c) for c,p in ts]
   route=optimize_remaining_route(state.position,points);order=held_karp_open_path(state.position,points)
   print(label,'heuristic',[names[points.index(p)] for p in route],route_length(tuple(route),state.position),'exact',[names[k] for k in order],route_length(tuple(points[k] for k in order),state.position))
  break
