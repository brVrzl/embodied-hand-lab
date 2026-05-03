from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, TensorDataset


CANONICAL_HAND_ORDER = ["index", "middle", "ring", "pinky", "thumb_close", "thumb_lateral"]

RH56_THUMB_LATERAL_ANCHORS = np.asarray(
    [
        [0.00, 0.00, 0.00, 0.00, 0.00, 0.00],
        [0.00, 0.00, 0.00, 0.00, 0.00, 1.00],
        [0.00, 0.00, 0.12, 0.15, 0.40, 1.00],
        [0.10, 0.10, 0.55, 0.60, 0.68, 1.00],
        [0.75, 0.75, 0.80, 0.80, 0.55, 0.65],
    ],
    dtype=np.float32,
)


def _iter_npz(paths: list[Path]) -> list[Path]:
    files: list[Path] = []
    for path in paths:
        if path.is_dir():
            files.extend(sorted(path.glob("*.npz")))
        else:
            files.append(path)
    return files


def _load_sequences(paths: list[Path], source: str) -> tuple[np.ndarray, list[dict[str, Any]]]:
    arrays: list[np.ndarray] = []
    sources: list[dict[str, Any]] = []
    for path in _iter_npz(paths):
        data = np.load(path, allow_pickle=True)
        if source not in data:
            continue
        arr = np.asarray(data[source], dtype=np.float32)
        if arr.ndim != 2 or arr.shape[1] != 6:
            raise RuntimeError(f"{path}: expected {source} shape [N,6], got {arr.shape}")
        arr = arr[np.isfinite(arr).all(axis=1)]
        raw_min = arr.min(axis=0)
        raw_max = arr.max(axis=0)
        if float(raw_min.min()) < -0.05 or float(raw_max.max()) > 1.05:
            sources.append(
                {
                    "path": str(path),
                    "source": source,
                    "frames": int(arr.shape[0]),
                    "skipped": True,
                    "skip_reason": "out_of_normalized_range",
                    "min": raw_min.round(6).tolist(),
                    "max": raw_max.round(6).tolist(),
                }
            )
            continue
        arr = np.clip(arr, 0.0, 1.0)
        arrays.append(arr)
        sources.append(
            {
                "path": str(path),
                "source": source,
                "frames": int(arr.shape[0]),
                "skipped": False,
                "min": arr.min(axis=0).round(6).tolist(),
                "max": arr.max(axis=0).round(6).tolist(),
            }
        )
    if not arrays:
        raise RuntimeError(f"No valid {source} arrays found in {paths}")
    return np.concatenate(arrays, axis=0), sources


def _sample_rows(x: np.ndarray, max_samples: int, seed: int) -> np.ndarray:
    if x.shape[0] <= max_samples:
        return x
    rng = np.random.default_rng(seed)
    idx = rng.choice(x.shape[0], size=max_samples, replace=False)
    return x[idx]


class ResidualVQVAE(nn.Module):
    def __init__(self, latent_dim: int, k1: int, k2: int) -> None:
        super().__init__()
        self.encoder = nn.Sequential(
            nn.Linear(6, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, latent_dim),
        )
        self.codebook1 = nn.Embedding(k1, latent_dim)
        self.codebook2 = nn.Embedding(k2, latent_dim)
        self.decoder = nn.Sequential(
            nn.Linear(latent_dim, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, 6),
            nn.Sigmoid(),
        )
        nn.init.uniform_(self.codebook1.weight, -0.5, 0.5)
        nn.init.uniform_(self.codebook2.weight, -0.1, 0.1)

    @staticmethod
    def _nearest(z: torch.Tensor, codebook: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        dist = torch.sum((z[:, None, :] - codebook[None, :, :]) ** 2, dim=-1)
        idx = torch.argmin(dist, dim=1)
        return codebook[idx], idx

    def encode_quantize(self, x: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        z = self.encoder(x)
        q1, idx1 = self._nearest(z, self.codebook1.weight)
        residual = z - q1.detach()
        q2, idx2 = self._nearest(residual, self.codebook2.weight)
        q = q1 + q2
        q_st = z + (q - z).detach()
        return z, q, q_st, torch.stack([idx1, idx2], dim=1)

    def forward(self, x: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        z, q, q_st, indices = self.encode_quantize(x)
        recon = self.decoder(q_st)
        return recon, z, q, q_st, indices

    def decode_all_codes(self) -> tuple[torch.Tensor, torch.Tensor]:
        q_values: list[torch.Tensor] = []
        pairs: list[list[int]] = []
        for idx1 in range(self.codebook1.num_embeddings):
            for idx2 in range(self.codebook2.num_embeddings):
                q_values.append(self.codebook1.weight[idx1] + self.codebook2.weight[idx2])
                pairs.append([idx1, idx2])
        q = torch.stack(q_values, dim=0)
        return self.decoder(q), torch.as_tensor(pairs, dtype=torch.long, device=q.device)


def _weighted_mse(pred: torch.Tensor, target: torch.Tensor, weights: torch.Tensor) -> torch.Tensor:
    return torch.mean(((pred - target) * weights) ** 2)


def _sort_centroids(centroids: np.ndarray, pairs: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    score = centroids[:, :5].mean(axis=1)
    order = np.lexsort((centroids[:, 5], score))
    return centroids[order], pairs[order]


def train(args: argparse.Namespace) -> dict[str, Any]:
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    x, sources = _load_sequences([Path(path) for path in args.input], args.source)
    raw_stats = {
        "frames": int(x.shape[0]),
        "min": x.min(axis=0).round(6).tolist(),
        "max": x.max(axis=0).round(6).tolist(),
        "std": x.std(axis=0).round(6).tolist(),
    }
    sampled = _sample_rows(x, args.max_samples, args.seed)
    if args.thumb_lateral_anchors:
        anchors = np.repeat(RH56_THUMB_LATERAL_ANCHORS, args.anchor_repeat, axis=0)
        train_x = np.concatenate([sampled, anchors], axis=0)
    else:
        train_x = sampled

    device = torch.device(args.device)
    model = ResidualVQVAE(latent_dim=args.latent_dim, k1=args.k1, k2=args.k2).to(device)
    weights = torch.as_tensor(
        [1.0, 1.0, 1.0, 1.0, args.thumb_close_weight, args.thumb_lateral_weight],
        dtype=torch.float32,
        device=device,
    )
    loader = DataLoader(
        TensorDataset(torch.from_numpy(train_x.astype(np.float32))),
        batch_size=args.batch_size,
        shuffle=True,
        drop_last=False,
    )
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=args.weight_decay)

    losses: list[dict[str, float]] = []
    model.train()
    for epoch in range(args.epochs):
        total_loss = 0.0
        total_recon = 0.0
        total_vq = 0.0
        total_count = 0
        for (batch,) in loader:
            batch = batch.to(device)
            recon, z, q, _, _ = model(batch)
            recon_loss = _weighted_mse(recon, batch, weights)
            codebook_loss = torch.mean((q - z.detach()) ** 2)
            commit_loss = torch.mean((z - q.detach()) ** 2)
            vq_loss = codebook_loss + args.commitment_beta * commit_loss
            loss = recon_loss + vq_loss
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            n = int(batch.shape[0])
            total_loss += float(loss.item()) * n
            total_recon += float(recon_loss.item()) * n
            total_vq += float(vq_loss.item()) * n
            total_count += n
        if epoch == 0 or (epoch + 1) % args.log_every == 0 or epoch + 1 == args.epochs:
            losses.append(
                {
                    "epoch": epoch + 1,
                    "loss": total_loss / total_count,
                    "recon_loss": total_recon / total_count,
                    "vq_loss": total_vq / total_count,
                }
            )

    model.eval()
    with torch.no_grad():
        centroids_t, pairs_t = model.decode_all_codes()
        centroids = centroids_t.detach().cpu().numpy().astype(np.float32)
        pairs = pairs_t.detach().cpu().numpy().astype(np.int64)
        centroids, pairs = _sort_centroids(centroids, pairs)

        sample_tensor = torch.from_numpy(sampled.astype(np.float32)).to(device)
        _, _, _, _, assigned_pairs_t = model(sample_tensor)
        assigned_pairs = assigned_pairs_t.detach().cpu().numpy()
    pair_to_sorted_index = {tuple(pair.tolist()): idx for idx, pair in enumerate(pairs)}
    assigned_sorted = np.asarray([pair_to_sorted_index.get(tuple(pair.tolist()), -1) for pair in assigned_pairs], dtype=np.int64)
    counts = np.bincount(assigned_sorted[assigned_sorted >= 0], minlength=args.k1 * args.k2)
    occupancy = counts / max(1, counts.sum())

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    torch.save(
        {
            "model_state_dict": model.state_dict(),
            "latent_dim": args.latent_dim,
            "k1": args.k1,
            "k2": args.k2,
            "canonical_hand_order": CANONICAL_HAND_ORDER,
        },
        output.with_suffix(".pt"),
    )
    np.savez_compressed(
        output,
        centroids=centroids,
        code_pairs=pairs,
        sampled_code_counts=counts.astype(np.int64),
        sampled_code_occupancy=occupancy.astype(np.float32),
        canonical_hand_order=np.asarray(CANONICAL_HAND_ORDER, dtype=object),
    )
    manifest = {
        "schema_version": "rh56_hand_codebook_v0.2",
        "method": "dqrise_style_two_layer_residual_vqvae",
        "output": str(output),
        "model": str(output.with_suffix(".pt")),
        "k1": int(args.k1),
        "k2": int(args.k2),
        "k": int(args.k1 * args.k2),
        "latent_dim": int(args.latent_dim),
        "source": args.source,
        "canonical_hand_order": CANONICAL_HAND_ORDER,
        "raw_stats": raw_stats,
        "sampled_frames": int(sampled.shape[0]),
        "train_frames_with_anchors": int(train_x.shape[0]),
        "thumb_lateral_anchors": bool(args.thumb_lateral_anchors),
        "anchor_repeat": int(args.anchor_repeat) if args.thumb_lateral_anchors else 0,
        "thumb_close_weight": float(args.thumb_close_weight),
        "thumb_lateral_weight": float(args.thumb_lateral_weight),
        "losses": losses,
        "sources": sources,
        "sampled_code_counts": counts.astype(int).tolist(),
        "sampled_code_occupancy": occupancy.round(6).tolist(),
        "code_pairs": pairs.astype(int).tolist(),
        "centroids": centroids.round(6).tolist(),
    }
    output.with_suffix(".json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description="Train a DQ-RISE-style two-layer residual VQ-VAE hand codebook for RH56.")
    parser.add_argument("--input", action="append", required=True, help="NPZ file or directory containing extracted hand sequences.")
    parser.add_argument("--source", choices=["hand_state", "hand_cmd"], default="hand_state")
    parser.add_argument("--output", default="data/models/rh56_hand_codebook_dqrise_rvqvae_unitree_state_k16.npz")
    parser.add_argument("--k1", type=int, default=4)
    parser.add_argument("--k2", type=int, default=4)
    parser.add_argument("--latent-dim", type=int, default=6)
    parser.add_argument("--epochs", type=int, default=80)
    parser.add_argument("--batch-size", type=int, default=2048)
    parser.add_argument("--lr", type=float, default=3e-4)
    parser.add_argument("--weight-decay", type=float, default=1e-5)
    parser.add_argument("--commitment-beta", type=float, default=0.25)
    parser.add_argument("--thumb-close-weight", type=float, default=1.25)
    parser.add_argument("--thumb-lateral-weight", type=float, default=8.0)
    parser.add_argument("--thumb-lateral-anchors", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--anchor-repeat", type=int, default=4000)
    parser.add_argument("--max-samples", type=int, default=200000)
    parser.add_argument("--seed", type=int, default=11)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--log-every", type=int, default=10)
    args = parser.parse_args()
    result = train(args)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
