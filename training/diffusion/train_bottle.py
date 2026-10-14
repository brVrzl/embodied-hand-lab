"""Offline official LeRobot DiffusionPolicy on the locked physical-bottle split.

Run in the existing LeRobot container with source assets mounted read-only.
No robot imports or device connections. Outputs must be in a new directory.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import random
import time

import numpy as np
import torch
from torch.utils.data import DataLoader
from lerobot.configs.types import FeatureType, PolicyFeature
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.diffusion.configuration_diffusion import DiffusionConfig
from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy
from lerobot.policies.factory import make_pre_post_processors


def seed_all(seed):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--mode', choices=['prepare', 'smoke', 'train'], required=True)
    args = p.parse_args()
    cfg = json.loads(args.config.read_text())
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / 'frozen_config.json').write_text(json.dumps(cfg, indent=2))
    seed_all(cfg['seed'])
    torch.set_num_threads(8)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False
    root = Path(cfg['source_root'])
    provenance = json.loads((root / 'meta/embodied_lab_provenance.json').read_text())
    assert provenance['normalization_stats_scope'] == 'train logical segments only'
    train = [e for e in provenance['episode_map'] if e['split'] == 'train']
    val = [e for e in provenance['episode_map'] if e['split'] == 'val']
    assert len(train) == 37 and sum(e['length'] for e in train) == 23802
    assert len(val) == 15 and sum(e['length'] for e in val) == 9309
    assert not ({e['source_episode_index'] for e in train} & {e['source_episode_index'] for e in val})
    images = ['observation.images.workspace', 'observation.images.wrist']
    delta = {k: [-1/30, 0.] for k in images + ['observation.state']}
    delta['action'] = [i/30 for i in range(-1, 15)]
    dataset = LeRobotDataset('local/physical_bottle_v4_nominal52_act', root=root,
        episodes=[e['lerobot_episode_index'] for e in train], delta_timestamps=delta,
        video_backend='pyav', return_uint8=False)
    loader = DataLoader(dataset, batch_size=cfg['batch_size'], shuffle=True,
        num_workers=0 if args.mode != 'train' else cfg['num_workers'], drop_last=True,
        persistent_workers=args.mode == 'train' and cfg['num_workers'] > 0)
    batch = next(iter(loader))
    report = {'mode': args.mode, 'seed': cfg['seed'], 'train_episodes': len(train),
        'train_frames': len(dataset), 'validation_episodes': len(val),
        'shapes': {k:list(v.shape) for k,v in batch.items() if isinstance(v,torch.Tensor)},
        'action_offsets': list(range(-1,15)), 'observation_offsets': [-1,0],
        'inference_action_slice': [1,9], 'padding': 'LeRobot endpoint repetition with action_is_pad loss mask',
        'normalization_scope': provenance['normalization_stats_scope']}
    def write_report():
        (args.output / 'report.json').write_text(json.dumps(report,indent=2)+'\n')
    write_report()
    if args.mode == 'prepare':
        print(json.dumps(report),flush=True)
        return
    device = 'cuda'
    assert torch.cuda.is_available()
    pc = DiffusionConfig(device=device, push_to_hub=False,
        input_features={**{k:PolicyFeature(type=FeatureType.VISUAL,shape=tuple(cfg['image_shape'])) for k in images},
            'observation.state': PolicyFeature(type=FeatureType.STATE,shape=(12,))},
        output_features={'action':PolicyFeature(type=FeatureType.ACTION,shape=(12,))},
        n_obs_steps=cfg['n_obs_steps'], horizon=cfg['horizon'],n_action_steps=cfg['n_action_steps'],
        down_dims=tuple(cfg['down_dims']),use_group_norm=cfg['use_group_norm'],
        num_train_timesteps=cfg['num_train_timesteps'],num_inference_steps=cfg['num_inference_steps'],
        noise_scheduler_type=cfg['noise_scheduler_type'],do_mask_loss_for_padding=True,
        crop_shape=None,crop_ratio=1.0,optimizer_lr=cfg['learning_rate'])
    stats = json.loads((root / 'meta/stats.json').read_text())
    stats = {k:{n:torch.tensor(x) for n,x in v.items()} for k,v in stats.items()}
    for k in images:
        stats[k]['mean']=torch.tensor([.485,.456,.406]).reshape(3,1,1)
        stats[k]['std']=torch.tensor([.229,.224,.225]).reshape(3,1,1)
    pre,post=make_pre_post_processors(pc,dataset_stats=stats)
    model=DiffusionPolicy(pc).to(device)
    optimizer=torch.optim.AdamW(model.parameters(), lr=cfg['learning_rate'],betas=(.95,.999),weight_decay=1e-6)
    def lr_scale(step):
        if step < cfg['warmup_steps']: return (step+1)/cfg['warmup_steps']
        return .5*(1+math.cos(math.pi*(step-cfg['warmup_steps'])/(cfg['steps']-cfg['warmup_steps'])))
    scheduler=torch.optim.lr_scheduler.LambdaLR(optimizer,lr_scale) if args.mode=='train' else None
    steps=40 if args.mode=='smoke' else cfg['steps']
    iterator=iter(loader)
    fixed=pre(batch) if args.mode=='smoke' else None
    model.train()
    start=time.monotonic()
    losses=[]
    with (args.output/'metrics.jsonl').open('w') as log:
        for step in range(1,steps+1):
            if fixed is not None:
                b=fixed
                torch.manual_seed(cfg['seed']+1)  # fixed diffusion noise for one-batch fit diagnostic
            else:
                try: b=next(iterator)
                except StopIteration:
                    iterator=iter(loader); b=next(iterator)
                b=pre(b)
            optimizer.zero_grad(set_to_none=True)
            loss,_=model(b)
            if not torch.isfinite(loss): raise FloatingPointError(f'nonfinite loss step {step}')
            loss.backward()
            norm=torch.nn.utils.clip_grad_norm_(model.parameters(),10.)
            if not torch.isfinite(norm): raise FloatingPointError(f'nonfinite gradient step {step}')
            optimizer.step()
            if scheduler:scheduler.step()
            value=float(loss.detach());losses.append(value)
            if step==1 or step%20==0 or step==steps:
                torch.cuda.synchronize()
                row={'step':step,'loss':value,'gradient_norm':float(norm),'seconds':time.monotonic()-start,
                    'sample_visits':step*cfg['batch_size'],'peak_cuda_bytes':torch.cuda.max_memory_allocated()}
                log.write(json.dumps(row)+'\n');log.flush();print(json.dumps(row),flush=True)
    if args.mode == 'train':
        validation = LeRobotDataset('local/physical_bottle_v4_nominal52_act', root=root,
            episodes=[e['lerobot_episode_index'] for e in val], delta_timestamps=delta,
            video_backend='pyav', return_uint8=False)
        indices=[]
        offset=0
        for episode in val:
            indices.extend((offset+np.linspace(0,episode['length']-1,8,dtype=int)).tolist())
            offset+=episode['length']
        val_loader=DataLoader(validation,batch_size=cfg['batch_size'],sampler=indices,num_workers=0)
        model.eval()
        validation_losses=[]
        with torch.no_grad():
            torch.manual_seed(cfg['seed']+3)
            for vb in val_loader:
                loss,_=model(pre(vb));validation_losses.append(float(loss))
        report['validation_probe']={'windows':len(indices),'seed':cfg['seed']+3,
            'mean_noise_mse':float(np.mean(validation_losses)),
            'selection_used':False,'protocol':'8 evenly spaced windows per validation episode'}
    checkpoint=args.output/'pretrained_model'
    model.save_pretrained(checkpoint);pre.save_pretrained(checkpoint);post.save_pretrained(checkpoint)
    model.eval();model.reset()
    probe=pre(batch)
    with torch.no_grad():
        torch.manual_seed(cfg['seed']+2); original=model.predict_action_chunk(probe)
    original_weights={k:v.cpu().clone() for k,v in model.state_dict().items()}
    del model
    torch.cuda.empty_cache()
    loaded=DiffusionPolicy.from_pretrained(checkpoint).to(device).eval()
    with torch.no_grad():
        torch.manual_seed(cfg['seed']+2); restored=loaded.predict_action_chunk(probe)
    for k,v in loaded.state_dict().items():
        torch.testing.assert_close(original_weights[k],v.cpu(),rtol=0,atol=0)
    torch.testing.assert_close(original,restored,rtol=1e-4,atol=1e-3)
    reload_max_abs=float((original-restored).abs().max())
    native=post(restored)
    assert tuple(native.shape)==(cfg['batch_size'],8,12) and torch.isfinite(native).all()
    report.update(optimizer_steps=steps,sample_visits=steps*cfg['batch_size'],elapsed_seconds=time.monotonic()-start,
        initial_loss=losses[0],final_loss=losses[-1],save_load_weights_exact=True,reload_inference_max_abs=reload_max_abs,inference_shape=list(native.shape),
        peak_cuda_bytes=torch.cuda.max_memory_allocated(),checkpoint=str(checkpoint),
        complete_normal_training=args.mode=='train',parameters=sum(p.numel() for p in loaded.parameters()))
    if args.mode=='smoke':
        report['single_batch_loss_decreased']=losses[-1]<losses[0]
        assert report['single_batch_loss_decreased']
    write_report();print(json.dumps(report),flush=True)


if __name__=='__main__':main()
