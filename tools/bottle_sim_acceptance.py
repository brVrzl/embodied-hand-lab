#!/usr/bin/env python3
"""Environment-only target replay and slow-release expert; never policy evidence."""
import argparse,json
import cv2
from bottle_sim_rollout import Stages
from pathlib import Path
import numpy as np
import pyarrow.parquet as pq
from embodied_bottle_sim.scene import BottleScene

def main():
 p=argparse.ArgumentParser();p.add_argument('--dataset',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--video',type=Path,help='Optional directory for labeled environment-only videos');args=p.parse_args()
 a=np.array(pq.read_table(args.dataset/'data/chunk-000/episode_000000.parquet')['action'].to_pylist())
 hold=a[550].copy();hold[6:]=a[500,6:];release=hold.copy();release[6:]=[0,0,0,0,0,.55];retreat=a[-1].copy();retreat[6:]=release[6:]
 expert=list(a[:551])+[hold]*90+[(1-t)*hold+t*release for t in np.linspace(0,1,180)]+[release]*90+[(1-t)*release+t*retreat for t in np.linspace(0,1,180)]+[retreat]*90
 output={}
 for name,seq in [('replay',a),('expert_slowrelease',expert)]:
  s=BottleScene();stages=Stages(s.snapshot());rows=[];video=None
  if args.video:
   args.video.mkdir(parents=True,exist_ok=True);video=cv2.VideoWriter(str(args.video/f'environment_{name}.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),15,(640,480))
   if not video.isOpened():raise RuntimeError('VideoWriter failed')
  for i,act in enumerate(seq):
   snapshot=s.step(act);rows.append(snapshot);stages.update(snapshot)
   if video is not None and i%2==0:
    frame=s.render()[...,::-1].copy();cv2.putText(frame,f'ENVIRONMENT {name} ep0 t={snapshot["time"]:.1f}s',(8,24),0,.5,(0,0,255),1);video.write(frame)
  if video is not None:video.release()
  tail=rows[-60:]
  stable=all(r['on_box'] and r['hand_contacts']==0 and np.linalg.norm(r['bottle_velocity'][:3])<.02 and np.linalg.norm(r['bottle_velocity'][3:])<.15 for r in tail)
  output[name]={'classification':'environment/expert validation; excluded from policy success','steps':len(rows),'duration_s':rows[-1]['time'],'lift_height_m':max(r['bottle_pos'][2] for r in rows)-s.config['bottle_center'][2],'hand_contact_frames':sum(r['hand_contacts']>0 for r in rows),'box_contact_frames':sum(r['on_box'] for r in rows),'stable_released_final_2s':stable,'stage_times':stages.events,'full_task_success':stable and stages.events['final_success'] is not None,'final':rows[-1],'arm_max_tracking_error_rad':np.max(np.abs(np.array([r['state'][:6] for r in rows])-np.asarray(seq)[:,:6]),axis=0).tolist()};s.close()
 args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(output,indent=2));print(json.dumps(output,indent=2))
if __name__=='__main__':main()
