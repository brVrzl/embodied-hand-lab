#!/usr/bin/env python3
"""Offline bottle policies: private Unix socket, native RGB/state in, raw actions out."""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import pickle
import socket
import struct
import subprocess
import tempfile
import time
import numpy as np


def send(conn, value):
    def encode(item):
        if isinstance(item, np.ndarray):
            return {'__array__': item.tobytes(), 'dtype': str(item.dtype), 'shape': item.shape}
        if isinstance(item, dict):
            return {k: encode(v) for k, v in item.items()}
        return item
    blob = pickle.dumps(encode(value), protocol=5)
    conn.sendall(struct.pack('!Q', len(blob)) + blob)


def receive(conn):
    def exact(n):
        result = bytearray()
        while len(result) < n:
            chunk = conn.recv(n - len(result))
            if not chunk:
                raise EOFError('policy socket closed')
            result.extend(chunk)
        return result
    length = struct.unpack('!Q', exact(8))[0]
    if length > 16 * 1024 * 1024:
        raise ValueError('request exceeds 16 MiB')
    def decode(item):
        if isinstance(item, dict) and '__array__' in item:
            return np.frombuffer(item['__array__'], dtype=item['dtype']).reshape(item['shape']).copy()
        if isinstance(item, dict):
            return {k: decode(v) for k, v in item.items()}
        return item
    return decode(pickle.loads(exact(length)))


class BottlePolicyClient:
    """One offline container per policy; only its own container is stopped."""
    def __init__(self, model, source_root, worktree, log_dir, checkpoint=None):
        self.model = model
        self.source = Path(source_root).resolve()
        self.worktree = Path(worktree).resolve()
        self.log_dir = Path(log_dir).resolve()
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.runtime = Path(tempfile.mkdtemp(prefix='bottle-policy-'))
        self.socket_path = self.runtime / 'policy.sock'
        self.container_name = f'bottle-sim-{model}-{os.getpid()}-{time.time_ns()}'
        self.conn = None
        self.proc = None
        suffix = ('pi05_rh56/checkpoints/pi05_rh56/weekend/4500' if model == 'pi05'
                  else 'physical_bottle_v4_nominal52/act_strong_run/checkpoints/100000/pretrained_model')
        self.checkpoint = (Path(checkpoint).resolve() if checkpoint is not None else
                           self.worktree / 'outputs/diffusion_bottle/train_seed20261014/pretrained_model'
                           if model == 'dp' else self.source / 'outputs/training' / suffix)
        self.metadata = {'model': model, 'checkpoint': str(self.checkpoint), 'image_input': 'uint8 RGB HWC 480x640', 'native_action_dim': 12}

    def __enter__(self):
        image = 'jaka-openpi:thor-cuda13' if self.model == 'pi05' else 'jaka-lerobot-dev:snapshot-before-raw-mount'
        paths = '/workspace/src:/workspace/training/pi05:/workspace/training/pi05/scripts:/app/src'
        if self.model in ('act', 'dp'):
            paths += ':/lerobot/src'
        if self.model == 'dp':
            paths = '/workspace/outputs/diffusion_bottle/deps:' + paths
        cmd = ['docker', 'run', '--rm', '--name', self.container_name, '--runtime=nvidia', '--network=none', '--ipc=host',
               '-v', f'{self.source}:{self.source}:ro', '-v', f'{self.worktree}:/workspace:ro',
               '-v', f'{self.runtime}:/runtime:rw', '-v', f'{self.source}/third_party/openpi:/app:ro',
               '-v', f'{self.source}/third_party/lerobot:/lerobot:ro',
               '-v', f'{self.source}/outputs/training/pi05_rh56/openpi_cache:/root/.cache/openpi:ro',
               '-e', f'PYTHONPATH={paths}', '-e', 'PYTHONUNBUFFERED=1',
               '-e', 'XLA_PYTHON_CLIENT_PREALLOCATE=false', '-e', 'OMP_NUM_THREADS=4',
               '-w', '/workspace', image, '/.venv/bin/python' if self.model == 'pi05' else 'python',
               'tools/bottle_policy_worker.py', '--serve', '--model', self.model,
               '--checkpoint', str(self.checkpoint).replace(str(self.worktree), '/workspace'),
               '--socket', '/runtime/policy.sock']
        self.log = (self.log_dir / f'{self.model}_worker.log').open('a')
        self.proc = subprocess.Popen(cmd, stdout=self.log, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + 600
        while not self.socket_path.exists():
            if self.proc.poll() is not None or time.monotonic() > deadline:
                self.close()
                raise RuntimeError(f'{self.model} worker startup failed; see {self.log_dir}')
            time.sleep(0.5)
        self.conn = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.conn.settimeout(600)
        self.conn.connect(str(self.socket_path))
        return self

    def request(self, value):
        send(self.conn, value)
        result = receive(self.conn)
        if 'error' in result:
            raise RuntimeError(result['error'])
        return result

    def reset(self, seed=0):
        return self.request({'command': 'reset', 'seed': int(seed)})

    def infer(self, state, workspace, wrist):
        return self.request({'state': np.asarray(state, np.float32), 'workspace': workspace, 'wrist': wrist})

    def infer_history(self, previous_obs, current_obs):
        if self.model != 'dp':
            raise ValueError('explicit two-tick history is only used by DP')
        previous_obs = current_obs if previous_obs is None else previous_obs
        history = {key: np.stack([previous_obs[key], current_obs[key]])
                   for key in ('state', 'workspace', 'wrist')}
        history['state'] = history['state'].astype(np.float32)
        return self.request({'history': history})

    def close(self):
        if self.conn is not None:
            try:
                self.request({'command': 'stop'})
            except (OSError, EOFError, RuntimeError):
                pass
            self.conn.close()
            self.conn = None
        if self.proc is not None:
            try:
                self.proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                subprocess.run(['docker', 'stop', '--time', '5', self.container_name], capture_output=True)
                self.proc.wait(timeout=15)
            self.log.close()
            self.proc = None
        for path in self.runtime.iterdir():
            path.unlink()
        self.runtime.rmdir()

    def __exit__(self, *exc):
        self.close()


def load_pi(checkpoint):
    import dataclasses
    import jax
    from openpi.policies.policy_config import create_trained_policy
    from openpi.transforms import Group
    from openpi_config import Rh56DataConfig, Rh56Inputs, TASK_PROMPT, make_config
    from openpi_runner import _patch_flax_state_map_for_thor, _patch_orbax_restore_for_thor
    _patch_flax_state_map_for_thor()
    _patch_orbax_restore_for_thor()

    class InferenceInputs:
        def __call__(self, data):
            # Reuse exact trained image/mask/state transform. The training-only
            # target satisfies its required input then is removed before the model.
            result = Rh56Inputs()({**data, 'actions': np.zeros((16, 12), np.float32)})
            del result['actions']
            return result

    class InferenceData(Rh56DataConfig):
        def create(self, assets_dirs, model_config):
            data = super().create(assets_dirs, model_config)
            return dataclasses.replace(data, data_transforms=Group(inputs=[InferenceInputs()], outputs=data.data_transforms.outputs))

    config = make_config(experiment_root=checkpoint.parents[3], exp_name='simulation', num_train_steps=1, batch_size=1)
    config = dataclasses.replace(config, data=InferenceData())
    policy = create_trained_policy(config, checkpoint, default_prompt=TASK_PROMPT)

    def infer(req):
        observation = {'images': {k: req[k] for k in ('workspace', 'wrist')}, 'state': req['state']}
        return np.asarray(policy.infer(observation)['actions'], dtype=np.float32)

    def reset(seed):
        policy._rng = jax.random.key(seed)
    return infer, reset


def load_act(checkpoint):
    import cv2
    import torch
    from lerobot.configs import PreTrainedConfig
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.policies.factory import make_pre_post_processors
    config = PreTrainedConfig.from_pretrained(checkpoint)
    config.pretrained_backbone_weights = None
    config.device = 'cuda'
    policy = ACTPolicy.from_pretrained(checkpoint, config=config).to('cuda').eval()
    pre, post = make_pre_post_processors(policy_cfg=config, pretrained_path=str(checkpoint), preprocessor_overrides={'device_processor': {'device': 'cuda'}})
    def infer(req):
        batch = {'observation.state': torch.from_numpy(req['state'])}
        for key in ('workspace', 'wrist'):
            rgb = cv2.resize(req[key], (320, 240), interpolation=cv2.INTER_AREA)
            batch[f'observation.images.{key}'] = torch.from_numpy(np.ascontiguousarray(rgb.transpose(2, 0, 1), dtype=np.float32) / 255.)
        with torch.inference_mode():
            return post(policy.predict_action_chunk(pre(batch))).detach().cpu().numpy()[0].astype(np.float32)
    def reset(seed):
        torch.manual_seed(seed)
        policy.reset()
        pre.reset()
        post.reset()
    return infer, reset



def load_dp(checkpoint, device='cuda'):
    import cv2
    import torch
    from lerobot.configs import PreTrainedConfig
    from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy
    from lerobot.policies.factory import make_pre_post_processors
    config = PreTrainedConfig.from_pretrained(checkpoint)
    config.device = device
    config.pretrained_backbone_weights = None  # Complete weights are in the local checkpoint.
    if (config.n_obs_steps, config.horizon, config.n_action_steps) != (2, 16, 8):
        raise ValueError('DP checkpoint must declare n_obs_steps=2, horizon=16, n_action_steps=8')
    policy = DiffusionPolicy.from_pretrained(checkpoint, config=config).to(device).eval()
    pre, post = make_pre_post_processors(
        policy_cfg=config, pretrained_path=str(checkpoint),
        preprocessor_overrides={'device_processor': {'device': device}})

    def infer(req):
        history = req['history']
        # Explicit B=1,T=2: automatic batch processor only adds a dimension
        # for single observations. Do not let T be interpreted as batch size.
        batch = {'observation.state': torch.from_numpy(history['state'])[None]}
        for key in ('workspace', 'wrist'):
            rgb = np.stack([cv2.resize(frame, (320, 240), interpolation=cv2.INTER_AREA)
                            for frame in history[key]])
            image = np.ascontiguousarray(rgb.transpose(0, 3, 1, 2), dtype=np.float32) / 255.
            batch[f'observation.images.{key}'] = torch.from_numpy(image)[None]
        with torch.inference_mode():
            # Official generate_actions already slices horizon[1:9]. Do not
            # offset again, and never populate select_action's history queue.
            return post(policy.predict_action_chunk(pre(batch))).detach().cpu().numpy()[0].astype(np.float32)

    def reset(seed):
        torch.manual_seed(seed)
        policy.reset()
        pre.reset()
        post.reset()
    return infer, reset

def serve(args):
    infer, reset = {'pi05': load_pi, 'act': load_act, 'dp': load_dp}[args.model](args.checkpoint)
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(args.socket))
    os.chmod(args.socket, 0o666)
    server.listen(1)
    conn, _ = server.accept()
    with server, conn:
        while True:
            req = receive(conn)
            command = req.get('command')
            if command == 'stop':
                send(conn, {'stopped': True})
                break
            try:
                if command == 'reset':
                    reset(req['seed'])
                    send(conn, {'reset': True, 'seed': req['seed']})
                    continue
                obs = req['history'] if args.model == 'dp' else req
                prefix = (2,) if args.model == 'dp' else ()
                for key in ('workspace', 'wrist'):
                    if obs[key].shape != prefix + (480, 640, 3) or obs[key].dtype != np.uint8:
                        raise ValueError(f'{key} must be uint8 RGB, shape {prefix + (480,640,3)}')
                if obs['state'].shape != prefix + (12,) or not np.isfinite(obs['state']).all():
                    raise ValueError('state must be finite native12 with declared history')
                started = time.perf_counter()
                actions = infer(req)
                elapsed = 1000 * (time.perf_counter() - started)
                if actions.ndim != 2 or actions.shape[1] != 12 or len(actions) < 8 or not np.isfinite(actions).all():
                    raise ValueError(f'invalid raw native chunk {actions.shape}')
                send(conn, {'prediction': actions, 'latency_ms': elapsed})
            except Exception as exc:
                send(conn, {'error': f'{type(exc).__name__}: {exc}'})


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serve', action='store_true', required=True)
    parser.add_argument('--model', choices=['act', 'pi05', 'dp'], required=True)
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--socket', type=Path, required=True)
    serve(parser.parse_args())
