from pathlib import Path
import json,hashlib,math
ROOT=Path(__file__).resolve().parents[1]
log=ROOT/'res/logs/problem3/integrated_bearing_tour_seed1_run1_20260912_141046_254292.jsonl'
summary=ROOT/'res/tables/problem3/integrated_bearing_tour_seed1_run1_20260912_141046_254292.json'
rows=[json.loads(x) for x in log.read_text(encoding='utf-8-sig').splitlines() if x.strip()]
s=json.loads(summary.read_text(encoding='utf-8'))
events=[]; prev=0.; pos=[0.,0.]
for row in rows:
 if row['action_type'] not in ('measure','clear'):continue
 p=[row['position']['x'],row['position']['y']];r=row['raw_response'];b=row['time_breakdown_s']
 assert abs(row['virtual_time_s']-prev-b['total'])<1e-4
 assert abs(math.dist(pos,p)/5-b['movement'])<1e-4
 events.append(dict(t=row['virtual_time_s'],start=prev,p=p,fromPos=pos,kind=row['action_type'],ch=row['target_channels'][0],current=row['current_channel'],result=r.get('measure_result',r.get('clear_result')),bearing=r.get('svd_deg'),move=b['movement'],switch=b['channel_switch'],op=b['measure_or_clear'],status=row['channel_status'],reason=row['reason']))
 prev=row['virtual_time_s'];pos=p
assert len(events)==s['measure_count']+s['clear_success_count']+s['clear_failure_count']
assert abs(prev-s['total_virtual_time_s'])<1e-4
payload=dict(events=events,total=prev,points=s['coverage_plan']['points'],radius=s['settings']['tour_polygon_radius_m'],distance=s['total_movement_distance_m'],counts=[s['measure_count'],s['switch_count'],s['clear_success_count'],s['clear_failure_count']],logName=log.name,sha256=hashlib.sha256(log.read_bytes()).hexdigest(),case='CW3X-R6UG-PUKS-2GNQ',start=rows[0]['raw_response']['real_timestamp_ms'],end=rows[-1]['raw_response']['real_timestamp_ms'],exportTime='2026-09-12T06:10:49.168Z')
html=(ROOT/'scripts/replay_template.html').read_text(encoding='utf-8').replace('__REPLAY_DATA__',json.dumps(payload,ensure_ascii=False).replace('<','\\u003c'))
out=ROOT/'res/replays/问题三_实际演练回放_CW3X.html';out.write_text(html,encoding='utf-8')
print(json.dumps(dict(output=str(out),events=len(events),time=prev,counts=payload['counts'],sha256=payload['sha256']),ensure_ascii=False))
