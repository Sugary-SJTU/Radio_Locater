"""按包导出时间匹配明文日志，重建已知状态并输出诊断；不解密官方载荷。"""
import sys,json,math,collections,os
from pathlib import Path
from datetime import datetime
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'src')]
from problems.problem3.config import Problem3Settings
from problems.problem3.shared import Problem3State
logs=[]
for p in (ROOT/'res/logs/problem3').glob('*20260912_*.jsonl'):
 try:
  rows=[json.loads(x) for x in p.read_text(encoding='utf-8-sig').splitlines() if x.strip()]
  if rows:logs.append((p,rows))
 except (ValueError,IndexError):pass
reports=[]
official_log_dir=Path(os.environ.get('CUMCM_JLOG_DIR', ROOT/'res/logs/official'))
for file in official_log_dir.glob('*.jlog'):
 raw=file.read_bytes();header,_=json.JSONDecoder().raw_decode(raw[raw.index(b'{'):].decode('utf-8',errors='replace'))
 created=datetime.fromisoformat(header['created_at_utc'].replace('Z','+00:00')).timestamp()*1000
 same=[(abs(r[-1]['raw_response'].get('real_timestamp_ms',0)-created),p,r) for p,r in logs if r[0]['raw_request'].get('robot_id')==header['team_no']]
 gap,p,rows=min(same,key=lambda x:x[0]);assert gap<1000
 summary=json.loads((ROOT/'res/tables/problem3'/p.with_suffix('.json').name).read_text(encoding='utf-8'))
 settings=Problem3Settings(**{k:tuple(v) if isinstance(v,list) else v for k,v in summary['settings'].items()});state=Problem3State(settings)
 counts=collections.Counter();times=collections.Counter();measures=collections.defaultdict(list);zero_gain=[];redundant=[];legs=[];probes=[];clears=[];snapshots=[];opportunities=[];clear_times={r["target_channels"][0]:r["virtual_time_s"] for r in rows if r["action_type"]=="clear" and r["raw_response"].get("clear_result")=="success"}
 plan=summary['coverage_plan']['points'];first_seen={};prev=(0.,0.)
 for i,r in enumerate(rows):
  pos=(r['position']['x'],r['position']['y']);ch=r['target_channels'][0] if r['target_channels'] else None;resp=r['raw_response'];kind=r['action_type'];before=state.tracks[ch] if ch else None
  radius=before.clear_circle.radius if before and before.clear_circle else None
  priorstatus=before.status if before else None
  if ch and before.clear_circle and priorstatus in ('detected','localized'):
   if math.dist(pos,before.clear_circle.center)+radius<=19.8+1e-9 and kind=='measure':redundant.append(dict(i=i,t=r['virtual_time_s'],ch=ch,radius=radius,result=resp.get('measure_result')))
  for k,v in r['time_breakdown_s'].items():times[k]+=v
  if r['time_breakdown_s']['movement']>0:legs.append(dict(i=i,t=r['virtual_time_s'],ch=ch,kind=kind,seconds=r['time_breakdown_s']['movement'],start=prev,end=pos,reason=r['reason']))
  if r['time_breakdown_s']['movement'] > 0:
   for other,tr in state.tracks.items():
    if tr.status in ('detected','localized') and tr.clear_circle and tr.clear_circle.radius<=19.8 and clear_times.get(other,0)>r['virtual_time_s']+60:
     c=tuple(float(x) for x in tr.clear_circle.center)
     extra=math.dist(prev,c)+math.dist(c,pos)-math.dist(prev,pos)
     if extra<=100:opportunities.append(dict(ch=other,at_start=state.virtual_time_s,leg_end=r['virtual_time_s'],extra_m=extra,center=c,radius=tr.clear_circle.radius,actual_clear=clear_times[other]))
  state.position=pos;state.virtual_time_s=r['virtual_time_s'];state.current_channel=r['current_channel']
  if kind=='measure':
   result=resp['measure_result'];counts[result]+=1;counts['measure']+=1
   if result=='direction':state.apply_direction(ch,pos,resp['svd_deg'])
   elif result=='no_signal':state.apply_no_signal(ch,pos)
   else:state.apply_near(ch,pos)
   after=state.tracks[ch];newradius=after.clear_circle.radius if after.clear_circle else None
   if priorstatus=='unknown':counts['unknown_measure']+=1
   else:counts['known_measure']+=1
   if result=='no_signal' and priorstatus!='unknown':counts['known_no_signal']+=1
   if radius is not None and newradius is not None and result=='direction' and abs(radius-newradius)<1e-5:zero_gain.append(dict(i=i,ch=ch,t=r['virtual_time_s'],radius=radius,reason=r['reason']))
   measures[ch].append(dict(i=i,t=r['virtual_time_s'],result=result,old_radius=radius,new_radius=newradius,priorstatus=priorstatus,reason=r['reason']))
   if '末段' in r['reason']:probes.append(dict(i=i,ch=ch,t=r['virtual_time_s'],point=pos,result=result,old_radius=radius,new_radius=newradius,reason=r['reason']))
  elif kind=='clear':
   success=resp['clear_result']=='success';state.apply_clear(ch,success);counts['clear_success' if success else 'clear_failure']+=1
   clears.append(dict(ch=ch,t=r['virtual_time_s'],pos=pos,result=resp['clear_result']))
  for k,point in enumerate(plan):
   if math.dist(pos,point)<.01:first_seen.setdefault(k,r['virtual_time_s'])
  if i+1<len(rows) and any(math.dist(pos,q)<.01 for q in plan) and math.dist(pos,tuple(rows[i+1]['position'].values()))>.01:
   snapshots.append(dict(t=r['virtual_time_s'],point=pos,tracks={c:dict(status=t.status,n=len(t.measurements),radius=t.clear_circle.radius if t.clear_circle else None) for c,t in state.tracks.items() if t.status not in ('cleared','absent')}))
  prev=pos
 delays=[]
 for c,tr in state.tracks.items():
  if tr.cleared_time_s is not None:delays.append(dict(ch=c,discovered=tr.first_detected_time_s,localized=tr.localized_time_s,cleared=tr.cleared_time_s,wait_after_localized=tr.cleared_time_s-tr.localized_time_s if tr.localized_time_s is not None else None))
 last_scan=max((r['virtual_time_s'] for r in rows if any(math.dist(tuple(r['position'].values()),q)<.01 for q in plan)),default=0)
 reports.append(dict(case=header['case_code'],jlog=str(file),log=str(p),gap_ms=gap,settings=summary['settings'],total=state.virtual_time_s,distance=summary['total_movement_distance_m'],source_count=counts['clear_success'],time=dict(times),counts=dict(counts),last_coverage_scan=last_scan,postcoverage_time=state.virtual_time_s-last_scan,zero_gain=zero_gain,redundant=redundant,probes=probes,legs=sorted(legs,key=lambda x:-x['seconds']),delays=sorted(delays,key=lambda x:-(x['wait_after_localized'] or 0)),measures=dict(measures),clears=clears,snapshots=snapshots,opportunities=opportunities))
reportpath=ROOT/'res/tables/problem3/four_practice_audit.json';reportpath.write_text(json.dumps(reports,ensure_ascii=False,indent=2),encoding='utf-8')
for r in reports:
 print(json.dumps({k:r[k] for k in ['case','gap_ms','total','distance','source_count','time','counts','last_coverage_scan','postcoverage_time']},ensure_ascii=False))
 print('mode',r['settings'].get('tour_endgame_mode','legacy'),'zero_gain',len(r['zero_gain']),'redundant',len(r['redundant']),'probes',len(r['probes']))
 print('longest',r['legs'][:3]);print('waits',r['delays'][:3])
