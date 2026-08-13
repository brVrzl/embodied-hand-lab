# Broad research scout: low-data real-robot manipulation

Date: 2026-08-13
Thread: C (parallel exploratory research; offline only)
Branch/worktree: `research/broad-exploration` in
`/home/thor/projects/embodied_lab_broad_research`; the initial survey was based
on `origin/dev` at `c50a1c5`, and the continuation worktree was fast-forwarded
to local `dev` at `0dc8bf989e610c616517691a79097b516c5775dc`
Scope: literature, official-code due diligence, dataset compatibility, and
low-cost experiment ranking. No robot, hand, camera, or controller connection
was made. No training sweep or physical rollout was run.

## Executive decision

The strongest near-term paper need not be force-centered. The current evidence
instead supports two promising robot-learning questions:

1. **Before a valid rollout exists:** how should representation, temporal
   context, view/state regularization, and demonstration sampling be allocated
   in the 25-demonstration regime? The cleanest first falsification is the
   already-supported ACT scratch-versus-ImageNet initialization comparison,
   followed by episode-balanced sampling and a short-history/view-dropout
   factorial.
2. **After Thread A establishes a valid rollout:** under a fixed human-time
   budget, do short corrections and recovery segments improve robustness more
   than additional nominal full demonstrations? This has greater ICRA upside
   than a policy-backbone comparison because it addresses covariate shift and
   robot-data allocation with a reusable learning insight.

SmolVLA and Diffusion Policy are valuable **baselines**, not paper ideas by
themselves. DP3/RISE-style 3D policies are scientifically attractive but cannot
be trained honestly from the existing demonstrations: the maintained source
contract explicitly declares that depth was not recorded. A 3D study therefore
requires new synchronized RGB-D collection and calibration, rather than a
conversion script.

The conservative recommendation is:

- **Do now, offline:** representation initialization, sampling audit, short
  history/view robustness, retrieval negative control, and dataset-size curves.
- **Audit now but do not scale yet:** Diffusion Policy and SmolVLA one-batch /
  checkpoint-compatibility paths.
- **Do after a valid baseline rollout:** equal-operator-time correction versus
  nominal-data study, and then deployment/chunk-fusion comparisons.
- **Defer:** true 3D, object-foundation-model pipelines, large VLAs, and a full
  hierarchy until a cheap audit supplies a reason to pay their integration and
  data costs.

## How this survey was conducted

The survey emphasizes 2024--2026 work, while retaining older papers when they
define a useful low-cost control. Evidence came from original papers, official
project pages, official repositories, and the exact LeRobot checkout used by
this project. Repositories were cloned into a temporary directory for read-only
inspection; no external repository was modified. Missing facts are marked
**UNKNOWN**, rather than reconstructed from neighboring papers or unofficial
summaries.

The maintained LeRobot checkout is official commit
[`f66e5128ecb2456e8c54a63d15404fa59c16aebc`](https://github.com/huggingface/lerobot/tree/f66e5128ecb2456e8c54a63d15404fa59c16aebc)
(reported as 0.6.2 by this repository). The upstream project is Apache-2.0.
Important compatibility claims below were checked against that checkout, not
only against current online documentation.

### Cost and verdict rubric

- Engineering: **E0** analysis only; **E1** configuration change; **E2** small
  repository-owned module; **E3** moderate module; **E4** major integration;
  **E5** effectively a new project.
- Evidence/data: **D0** existing-data analysis; **D1** retrain existing data;
  **D2** a few targeted rollouts; **D3** a few new demonstrations; **D4**
  moderate new collection; **D5** major dataset/hardware requirement.
- Implementation distance: **DROP-IN**, **SMALL ADAPTATION**, **MODERATE PORT**,
  **MAJOR PORT**, or **UNREALISTIC**.
- Verdict vocabulary used in this living document: **DO NOW**, **NEXT**,
  **AUDIT FIRST**, **BACKLOG**, **STRETCH**, and **REJECT**. `DO NOW` means an
  offline probe, not physical authorization.
- ICRA positioning: **ROBOTICS-HEAVY**, **BALANCED**, **ROBOT-LEARNING**, or
  **GENERIC-AI**. The preferred categories are BALANCED and ROBOT-LEARNING.

## Repository and data compatibility audit

### What the automated 25-trajectory materialization contains

The v2 materialization reports 25 automatically split logical trajectories,
20,744 aligned samples, and 690.7 seconds at approximately 30 Hz. Its earlier
description as 25 *clean* trajectories is superseded by the human audit in the
continuation below: the operator now identifies approximately 16 nominal
segments after correcting missed splits in source episodes 99 and 102. The
following remains a description of the stored schema, not a quality claim.
Each sample has:

- workspace RGB and wrist RGB;
- 12-D state: six measured JAKA joints plus six measured RH56 actuator
  positions;
- 12-D absolute/native action: six accepted JAKA joint targets plus six RH56
  targets;
- an optional, separate 6-D raw RH56 actuator-load vector.

See
[`research_log/physical_bottle_training_dataset_v2.md`](physical_bottle_training_dataset_v2.md)
and
[`docs/data/DATASET_SCHEMA.md`](../docs/data/DATASET_SCHEMA.md).
The action chunks are confined to logical episodes and repeat-last padding is
masked. Thus policies that assume Cartesian delta actions need a scientifically
explicit action adapter; changing the target semantics would no longer be a
controlled architecture comparison.

The demonstrations range from 557 to 1,487 rows (18.5--49.5 s). Uniform frame
sampling therefore gives the longest trajectory **2.67 times** the training
mass of the shortest. This is not a loader bug, but it is an uncontrolled choice
about whether frames or demonstrations define the empirical objective. The
existing report already calls out this issue in
[`research_log/physical_bottle_v2_training_val4.md`](physical_bottle_v2_training_val4.md).

The four held-out source episodes are 89, 98, 114, and 116. They were selected
as relatively good demonstrations, so validation loss on this split is useful
for detecting pipeline/configuration failures but is not an unbiased success or
generalization estimate. Any new comparison must keep episode-level splits,
normalization from training rows only, and multiple seeds. Demonstration-size
curves should use nested **episode** subsets, never random frames.

### Depth availability: negative finding

True depth cannot be recovered from the maintained recordings. The physical
capture schema states `depth_recorded=false`; TCP and Quest packets are also not
stored in the core table. The staging configuration had depth capture and
alignment disabled, and no RealSense bag/depth stream was found in the audited
source tree. RGB monocular depth estimation would produce pseudo-depth, not the
metric synchronized point cloud assumed by DP3 or RISE, and must be named and
tested as a separate method.

Consequences:

- existing-data DP3/RISE reproduction: **not possible**;
- an RGB-only 3D proxy study: possible but scientifically weaker and still E3;
- true DP3-like baseline: E3--E4/D4, requiring synchronized depth, intrinsics,
  workspace cropping, camera-to-robot calibration, and a new data split;
- future collection should record aligned raw depth if storage/timing checks can
  be satisfied, but this survey does not authorize collection.

### Current ACT is a particularly clean pretraining experiment

The maintained ACT configuration uses `resnet18`, one observation step, a
16-step prediction/execution chunk, no temporal ensemble, disabled image
transforms, and `pretrained_backbone_weights: null`. The pinned LeRobot ACT
default already supports `ResNet18_Weights.IMAGENET1K_V1`; changing only this
field is therefore an E1 controlled intervention. In contrast, ACT 0.6.2 rejects
`n_obs_steps != 1`, so an ACT history experiment needs a small adapter rather
than a truthful configuration-only claim.

Current LeRobot ACT processes both cameras with a shared ResNet and concatenates
their spatial tokens. It does not, in the inspected implementation, add an
explicit learned camera-identity token. This makes the following cheap
ablation order more defensible than immediately adding cross-view attention:

1. workspace only, wrist only, both views;
2. both views plus train-time camera dropout;
3. camera-identity embeddings;
4. separate encoders;
5. only then, cross-view attention.

### Native policy support in the pinned ecosystem

| Policy | Exact inspected defaults relevant here | Data compatibility | Distance | Initial disposition |
| --- | --- | --- | --- | --- |
| ACT | 1 observation; chunk/action steps 100 upstream; ImageNet ResNet-18 default; mean/std state/action; optional temporal ensemble | Already maintained with local 16-step absolute joint chunks | DROP-IN | Strong baseline; change one factor at a time |
| Diffusion Policy | 2 observations; horizon 64; execute 32; ImageNet ResNet-18; min/max state/action; native multiple cameras; DDPM/100 train steps | Existing LeRobot schema is compatible; deployment/action timing still needs an adapter audit | SMALL ADAPTATION | Highest-priority alternative policy baseline |
| VQ-BeT | 5 observations; five-action chunks; ImageNet ResNet-18; separate VQ-VAE stage | Schema compatible, but two-stage training and history add complexity | MODERATE PORT | Secondary multimodal/autoregressive baseline |
| SmolVLA | 1 observation; 50-action chunk; 32-D max state/action; mean/std; frozen vision encoder and action-expert-only defaults; 10 flow steps | 12-D state/action and two images fit; prompt/task metadata and action cadence require audit | SMALL ADAPTATION | One-batch/checkpoint audit now; no sweep yet |
| pi0 | 1 observation; 50-action chunk; 32-D max state/action; 10 flow steps | Shapes fit; pretrained embodiment/action normalization and compute do not | MAJOR PORT | Stretch baseline, not a 25-demo first move |
| pi0-FAST | 50-action autoregressive token chunk; 32-D max state/action; official FAST tokenizer | Shapes fit; tokenizer/statistics and large-model adaptation are substantial | MAJOR PORT | Reject as primary direction |
| RTC | Inference-time overlap/inpainting for supported flow policies | Not applicable to ACT; usable after a valid SmolVLA/pi0-family deployment | MODERATE PORT | Later deployment module |

## Literature map

The useful literature clusters into six connected questions:

```text
small real dataset
├── reduce representation burden
│   ├── ImageNet / R3M / MVP / VC-1 / DINOv2 / Theia
│   └── frozen vs partial vs full fine-tuning
├── use each recorded sample better
│   ├── episode/phase balance, nested demo scaling, semantic augmentation
│   ├── two-view dropout/fusion and short history
│   └── explicit retrieval or prototypes
├── choose a stronger conditional action model
│   ├── ACT, Diffusion Policy, VQ-BeT / BeT, ARP
│   └── SmolVLA, pi0, pi0-FAST, OpenVLA-OFT
├── improve closed-loop execution
│   ├── temporal ensembling / receding horizon
│   └── asynchronous inference / RTC / real-time iteration
├── spend new human time where BC fails
│   ├── DAgger / IWR / Sirius / ThriftyDAgger
│   └── short correction and recovery segments
└── add spatial structure
    ├── DP3 / RISE / EquiBot (true point cloud)
    ├── VIOLA / RoboTAP / object crops and points
    └── hybrid free-space + local interaction policies
```

The main cross-cutting warning is causal confusion: state, camera identity,
trajectory time, and manually inferred phase can be excellent shortcuts on a
fixed dataset while failing under deployment shift. Every robustness proposal
below therefore specifies an observable deployment input and a held-out shift,
not just lower training loss.

## Serious candidate papers and projects

This catalog records implementation facts rather than treating an abstract as
a reproduction plan. “Weights” means usable pretrained or task checkpoints
were identified in the official release; it does not mean they match JAKA or
RH56 action semantics.

### Policy and action-generation families

| Paper / year / venue | Official project and repository | Inputs, outputs, horizons, normalization | Real evidence, demonstrations, compute | Code / license / weights | Compatibility and verdict |
| --- | --- | --- | --- | --- | --- |
| **ACT / ALOHA**, 2023, RSS ([paper](https://arxiv.org/abs/2304.13705), [project](https://tonyzhaozh.github.io/aloha/)) | [Official repository](https://github.com/tonyzhao/act) | Multi-RGB + joints to continuous joint-action chunks; current LeRobot port uses one observation, mean/std state/action, spatial ResNet tokens, optional overlapping temporal ensemble. | ALOHA real bimanual robot; 50 demonstrations/task in the paper. Original fine-tuned pretrained ResNet. Current local ACT inference was already measured separately; that is not a physical success claim. | Code/checkpoints available; MIT. | **DROP-IN, E0--E1/D1.** Existing baseline. Pretraining and consumer strategy are controlled changes, not new methods. |
| **Diffusion Policy**, 2023, RSS ([paper](https://arxiv.org/abs/2303.04137), [project](https://diffusion-policy.cs.columbia.edu/)) | [Official repository](https://github.com/real-stanford/diffusion_policy) | Image dictionary `(B,To,H,W,3)` plus low-dimensional inputs to `(B,Ta,Da)` actions. Official normalization is checkpointed. Pinned LeRobot: `To=2`, horizon 64, execute 32, ImageNet ResNet-18, 32-keypoint spatial softmax, min/max state/action, DDPM with 100 train/inference steps by default. | Real UR5/UR5e with two RealSense D415 cameras; real demonstration count and exact GPU spec **UNKNOWN** in inspected official sources. | Original MIT; LeRobot port Apache-2.0; code, configs, logs, task checkpoints available; PyTorch, Hydra, zarr, `diffusers`. | **SMALL ADAPTATION, E1--E2/D1.** Closest strong policy alternative. Match observation/execution horizons before attributing gains to diffusion. |
| **DP3**, 2024, RSS ([paper](https://arxiv.org/abs/2403.03954), [project](https://3d-diffusion-policy.github.io/)) | [Official repository](https://github.com/YanjieZe/3D-Diffusion-Policy) | `(T,N,6)` XYZRGB point cloud, RGB, depth, agent position to relative end-effector + relative dexterous-hand actions in real release. Simple recipe: 1,024 cropped/FPS points, obs 2, horizon 16, execute 8, DDIM 100 train/10 inference. | 72 simulation tasks, often 10 demos; four real Franka+Allegro+L515 tasks, 40 demos/task. Repository reports about 10 GB and 3 h on A40; Simple-DP3 1--2 h and ~25 FPS. It warns D435 point-cloud quality may be insufficient. | MIT; code/configs and example real data available; matching pretrained JAKA checkpoint **UNKNOWN**. | **MAJOR PORT, E3--E4/D4.** Existing data has no depth and action semantics differ. Stretch only after new calibrated RGB-D data. |
| **RISE**, 2024, IROS ([paper](https://arxiv.org/abs/2404.12281), [project](https://rise-policy.github.io/)) | [Official repository](https://github.com/rise-policy/RISE) | Single-view point cloud through sparse 3D encoder + transformer + diffusion action head; released workflow expects processed point-cloud trajectories. | Six real manipulation tasks; 50 demonstrations/task; Flexiv arm, AG95 gripper, RealSense. Training/inference compute **UNKNOWN**. | Code available; CC BY-NC-SA 4.0; weights/checkpoint coverage **UNKNOWN**. | **MAJOR PORT, E4/D4.** No maintained depth and noncommercial/share-alike license. Reference for 3D hypothesis, not reuse priority. |
| **EquiBot**, 2024, CoRL ([paper/project](https://equi-bot.github.io/)) | [Official repository](https://github.com/yjy0625/equibot) | SIM(3)-equivariant point-cloud diffusion policy; point clouds + robot state to continuous actions, with explicit geometric equivariance. Exact target action adapter/normalization for JAKA **UNKNOWN**. | Real results on six tasks/variations; project reports roughly five minutes of demonstrations per task. Exact episode count and compute **UNKNOWN**. | Code available; license/checkpoint status **UNKNOWN** in this audit. | **MAJOR PORT, E4/D4.** Strong generalization motivation but cannot use current RGB-only data. |
| **Consistency Policy**, 2024, RSS ([paper](https://arxiv.org/abs/2405.07503), [project](https://consistency-policy.github.io/)) | [Official repository](https://github.com/Aaditya-Prasad/Consistency-Policy) | Distills a trained Diffusion Policy teacher into one/few-step trajectory generation; example real wrapper uses two 84×84 cameras, obs 2, action 8. Teacher, warm start, and distillation are prerequisites. | Six simulation and two real tasks; project reports about 10× sampling speedup. Real demo counts and target checkpoints **UNKNOWN**. | MIT; code/configs available. | **MODERATE PORT, E3/D1.** Defer until ordinary diffusion works and measured latency is a bottleneck. |
| **RTI-DP**, 2025, IROS ([project/paper](https://rti-dp.github.io/)) | [Official repository](https://github.com/RTI-DP/rti-dp) | Training-free real-time iteration warm-starts denoising from the prior predicted chunk; special handling for discrete grippers. Forks original Diffusion Policy formats. | Official inspected results: PushT, BlockPush, RoboMimic; real-robot result **UNKNOWN**. | MIT; code and a Hugging Face model available. | **MODERATE PORT, E2--E3/D2.** Only after valid Diffusion Policy deployment. |
| **One-Step Diffusion Policy**, 2025, ICML ([paper](https://proceedings.mlr.press/v267/wang25ba.html), [project](https://research.nvidia.com/labs/dir/onedp/)) | Official runnable repository **UNKNOWN** | Distills Diffusion Policy to one-step inference; preserves the teacher’s observation/action interface. Detailed target data adapter **UNKNOWN**. | Six simulation and four Franka real tasks; reported 1.5 to 62 Hz. Exact demos **UNKNOWN**; paper reports 2--10% extra teacher-training cost. | PMLR states code will be available; license/checkpoints **UNKNOWN**. | **MAJOR PORT, E4. REJECT now** until an official reusable release and a latency need both exist. |
| **SmolVLA**, 2025, arXiv ([paper](https://arxiv.org/abs/2506.01844), [docs](https://github.com/huggingface/lerobot/blob/v0.6.1/docs/source/smolvla.mdx)) | [LeRobot](https://github.com/huggingface/lerobot), [base weights](https://huggingface.co/lerobot/smolvla_base) | 450M SmolVLM2 plus flow action expert; multiple images, current state and task text to continuous chunk. Defaults: obs 1, chunk/execute 50, max state/action 32, 512² padded images, 10 flow steps, mean/std, frozen vision and expert-only training. | Community pretraining: 22.9K episodes / 10.6M frames per paper table. Real SO100/SO101 evaluations. Official guide recommends ~50 target episodes and explicitly reports a similar 25-episode set performed badly; example 20K steps, batch 64, ~4 h on one A100. | Apache-2.0 code; released base weights. | **SMALL ADAPTATION, E1--E2/D1. AUDIT FIRST.** Shape/schema fits, but evidence predicts data starvation. Run load/one-batch/overfit checks only before allocating a full run. |
| **π0**, 2025, RSS ([paper](https://arxiv.org/abs/2410.24164), [project](https://www.physicalintelligence.company/blog/pi0)) | [Official OpenPI](https://github.com/Physical-Intelligence/openpi) and pinned LeRobot port | Normally three masked 224² image slots, 32-D padded state, prompt to 50×32 continuous flow chunk; 10 inference steps. Supports z-score or quantile stats and optional relative actions. | Base pretraining spans 10K+ robot hours; exact task demo counts **UNKNOWN**. Official estimates: >8 GB inference, >22.5 GB LoRA, >70 GB full fine-tune. | Apache-2.0; base/embodiment checkpoints. | LeRobot **SMALL ADAPTATION**, native OpenPI **MODERATE PORT**; E2--E3/D1. Shapes fit, but action/statistics/third-image masks need explicit audit. Not a primary 25-demo direction. |
| **π0-FAST / FAST**, 2025, arXiv ([paper](https://arxiv.org/abs/2501.09747), [project](https://www.physicalintelligence.company/research/fast)) | [Official OpenPI](https://github.com/Physical-Intelligence/openpi); LeRobot FAST tokenizer/port | Normalizes actions, DCTs each dimension, sparsifies/rounds coefficients, orders low frequencies first, then BPE tokenizes. General tokenizer trained on 1M+ action sequences; LeRobot default chunk 50, 32-D maximum, 256 tokens, autoregressive decoding. | Reports up to 5× faster VLA training. Public LeRobot LIBERO reproduction used 8 H100s, batch 256, 40K further steps; target real demo count **UNKNOWN**. | Apache-2.0; tokenizer and base weights available. | **MODERATE PORT, E2--E3/D1.** Single fixed instruction gives little language leverage; audit only. |
| **RTC**, 2025, NeurIPS ([paper](https://arxiv.org/abs/2506.07339), [project](https://www.pi.website/research/real_time_chunking)) | [Official simulation repository](https://github.com/Physical-Intelligence/real-time-chunking-kinetix), [LeRobot docs](https://huggingface.co/docs/lerobot/rtc) | Inference-time overlap/inpainting for flow policies: committed actions are frozen while the remaining chunk is regenerated asynchronously. It is not an ACT method. | Six real bimanual tasks in the paper; official simulation reproduction uses million-transition datasets and ~60 GiB released assets. Exact real demo counts **UNKNOWN**. | Kinetix code MIT; LeRobot integration Apache-2.0. | **MAJOR/MODERATE PORT, E3--E4/D2.** Later execution module after a flow policy and valid rollout, not a current training idea. |
| **Octo**, 2024, RSS ([paper/project](https://octo-models.github.io/)) | [Official repository](https://github.com/octo-models/octo) | 27M/93M JAX transformer diffusion policies pretrained on 800K trajectories; multiple RGB, proprioception and text/goal image; history 2, action chunk 4; RLDS and normal/bounds normalization. | Six real platforms. Fine-tuning used roughly 100 trajectories/domain, 50K steps, ~5 h on one 24 GB A5000; inference reported 13/17 it/s for 93M/27M on 4090. | MIT; code and weights available. | **MODERATE PORT, E3/D1.** RLDS/JAX conversion and 25-demo mismatch. Useful evidence that extra wrist views may hurt and should be ablated. |
| **BAKU**, 2024, NeurIPS ([paper](https://arxiv.org/abs/2406.07539), [project](https://baku-robot.github.io/)) | [Official repository](https://github.com/siddhanthaldar/BAKU) | Modular multi-view/proprio/language transformer trunk with deterministic, GMM, BeT, diffusion or VQ-BeT heads; configurable history/temporal aggregation. Real xArm example: four 128² views, chunk 10 at 10 Hz. | 30 real xArm tasks, mean 17 demos/task, and five long tasks, mean 19 demos/task. Compute **UNKNOWN**. | MIT; code; general pretrained target checkpoint **UNKNOWN**. | **MODERATE PORT, E3/D1.** Its result comes from cross-task sharing, absent in this one-task dataset. Design reference, not first baseline. |
| **VQ-BeT**, 2024, ICML Spotlight ([paper](https://arxiv.org/abs/2403.03181), [project](https://sjlee.cc/vq-bet)) | [Official repository](https://github.com/jayLEE0301/vq_bet_official); LeRobot port | Residual VQ action chunks + transformer + offsets. Pinned defaults: obs 5, three prediction tokens, chunk 5, min/max, ImageNet ResNet-18, 84 random crop, separate 20K VQ pretraining. Inspected LeRobot validator permits exactly one image. | Physical result and demonstration count **UNKNOWN** in inspected sources. | Original MIT; LeRobot Apache-2.0. | **MODERATE PORT, E2--E3/D1.** A one-camera run is not a matched dual-view baseline; patching and codebook training are not first-order priorities. |
| **Behavior Transformer**, 2022, NeurIPS ([paper](https://proceedings.neurips.cc/paper_files/paper/2022/hash/90d17e882adbdda42349db6f50123817-Abstract-Conference.html), [project](https://notmahi.github.io/bet)) | [Official repository](https://github.com/notmahi/bet) | History-conditioned transformer predicts k-means action bin plus continuous residual. Released environments/formats cover Franka Kitchen, block pushing, and CARLA. | Physical real-robot result and demo count **UNKNOWN**. | MIT; code; older environment dependencies; weights **UNKNOWN**. | **MODERATE PORT, E3/D1. REJECT initially.** Local futures must first be shown multimodal. |
| **ARP / Chunking Causal Transformer**, 2024, arXiv ([paper](https://arxiv.org/abs/2410.03132)) | [Official repository](https://github.com/mlzxy/arp) | Autoregressively emits heterogeneous continuous/discrete action groups, with configurable chunk size per action type. Normalization and direct LeRobot data adapter for this embodiment **UNKNOWN**. | Push-T, ALOHA and RLBench evaluation; physical evidence/demo count and compute **UNKNOWN**. | Code and pretrained models reported; license **UNKNOWN**. | **MAJOR PORT, E3--E4/D1.** Interesting action-factorization reference, but no cheap advantage over maintained ACT/DP. |
| **OpenVLA-OFT**, 2025, arXiv ([paper](https://arxiv.org/abs/2502.19645), [project](https://openvla-oft.github.io/)) | [Official repository](https://github.com/moojink/openvla-oft) | OpenVLA fine-tuning recipe: parallel decoding, continuous action chunks, L1 objective and multiple images; not a LeRobot-native data path. | LIBERO and real ALOHA. Inference ~16--18 GB; training 1--8 GPUs with 27--80 GB each depending on mode. Target demonstration counts **UNKNOWN**. | Code license should be read from repo before reuse; base OpenVLA code MIT but model inherits Llama-2 terms; weights available. | **MAJOR PORT, E4/D1.** Strong VLA baseline at 7B, but disproportionate for one fixed task and 25 demos. |

### Pretrained visual representations

| Representation / year / venue | Official source | Input/interface and pretraining | Robot evidence / compute | Code / license / weights | Fit and verdict |
| --- | --- | --- | --- | --- | --- |
| **ImageNet ResNet-18** | [TorchVision model card](https://docs.pytorch.org/vision/main/models/generated/torchvision.models.resnet18) | 11.7M CNN, standard ImageNet-1K V1 weights; ACT preserves layer-4 spatial features rather than using only a class vector. | Generic vision pretraining; no claim that ImageNet alone solves robot control. Extra training compute is zero; normal ACT inference cost. | TorchVision BSD code; weights available; pretrained-data terms remain the user’s responsibility. | **DROP-IN, E1/D1. DO NOW.** Fully trainable at a lower backbone LR first; freezing/partial unfreeze are E2. |
| **DINOv2**, 2023, arXiv | [Paper](https://arxiv.org/abs/2304.07193), [official repository](https://github.com/facebookresearch/dinov2) | Self-supervised LVD-142M; ViT-S/B/L/g 21M/86M/300M/1.1B, patch 14; exposes dense patch and global tokens. | Broad downstream evidence; direct real-robot demo count **not applicable/UNKNOWN**. ViT-S is the relevant compact probe. | Apache-2.0 standard code/weights. | **MODERATE PORT, E2--E3/D1.** Best second-wave dense-token encoder if ImageNet helps; do not reduce it to CLS only. |
| **R3M**, 2022, CoRL | [Paper](https://proceedings.mlr.press/v205/nair23a.html), [official repository](https://github.com/facebookresearch/r3m) | ResNet-18/34/50 trained on Ego4D human video with temporal/language objectives; 224² input; standard interface is globally pooled. | Real Franka evaluations included approximately 20 demos/task; training recipe reports 1.5M representation steps. | MIT; weights; repository archived/read-only. | **SMALL ADAPTATION, E2/D1.** Easy ResNet comparator, but pooled interface and maintenance make it lower priority than ImageNet/DINO. |
| **MVP / Real-World MVP**, 2022, CoRL Oral | [Project](https://tetexiao.com/projects/mvp), [real-robot paper](https://proceedings.mlr.press/v205/radosavovic23a.html), [repository](https://github.com/ir413/mvp) | MAE-pretrained ViT-S/B/L on 0.7M/4.5M egocentric + Internet images; official control recipe freezes the encoder. | Real-robot gains over CLIP/ImageNet/scratch reported; exact per-task demo counts/compute vary and are **UNKNOWN** here. | Weights available; no top-level license found in official repo, so reuse license **UNKNOWN**. | **MODERATE PORT, E2--E3/D1.** Scientifically relevant, but do not copy without license clarification. |
| **VC-1**, 2023, arXiv / CortexBench | [Project](https://eai-vc.github.io/), [official repository](https://github.com/facebookresearch/eai-vc), [model](https://huggingface.co/facebook/vc1-base) | ViT-B/16 MAE on 5.62M frames / 4,000+ h from seven egocentric sources plus ImageNet; 224², 768-D representation. | Manipulation/navigation benchmark; exact target-task real demos and inference compute **UNKNOWN**. No single representation won universally. | Predominantly CC BY-NC 4.0; weights available. | **MODERATE PORT, E2--E3/D1.** Noncommercial license and global interface reduce reuse value. |
| **Theia**, 2024, CoRL | [Paper](https://arxiv.org/abs/2407.20179), [project](https://theia.theaiinstitute.com/), [repository](https://github.com/rai-opensource/theia) | Distills CLIP, DINOv2 and ViT teachers; 10M--200M models. Official study found spatial transformer tokens consistently stronger than CLS for control and naive feature concatenation worse than individual encoders. | Real BC with RGB + joints, 5 Hz, chunks 5/10, with task demo counts approximately 48/63/101/50 (paper prose/table has an acknowledged ordering ambiguity). Frozen models failed on some long tasks while fine-tuning helped. | Custom AI Institute noncommercial research license; weights. | **MODERATE PORT, E2--E3/D1.** Excellent design evidence; second-wave due license and adapter cost. |
| **CLIP**, 2021, ICML | [Paper](https://arxiv.org/abs/2103.00020), [official repository](https://github.com/openai/CLIP) | Image-text pretraining on 400M pairs; released ResNet/ViT global and patch features. | Generic vision-language evidence, no relevant target demo count; fixed task language provides no variation. | MIT; weights. | **MODERATE PORT, E2--E3/D1.** Low priority; spatial precision is a concern. |
| **SigLIP / SigLIP 2**, 2023/2025, ICCV/arXiv | [SigLIP](https://arxiv.org/abs/2303.15343), [SigLIP 2](https://arxiv.org/abs/2502.14786), [official code](https://github.com/google-research/big_vision) | Sigmoid image-text objective; B/16-224 ~86M/768-D. SigLIP 2 emphasizes localization/dense features and variable-resolution variants. | Direct target robot evidence and demo count **UNKNOWN**; already appears inside SmolVLA/π0-family models. | Apache-2.0 standard code/weights; model-card terms must still be checked. | **MODERATE PORT, E2--E3/D1.** Watch, but standalone swap duplicates a cleaner DINO test. |
| **MAE**, 2022, CVPR | [Paper](https://arxiv.org/abs/2111.06377), [official repository](https://github.com/facebookresearch/mae) | ViT-B/L/H masked image reconstruction, ImageNet, patch 16 at 224². | No direct robot result in foundational paper. Old `timm` dependency. | CC BY-NC 4.0; weights; repository archived. | **MODERATE PORT, E2--E3. REJECT initially** in favor of robot-tested descendants or DINOv2. |
| **SpawnNet**, 2024, ICRA | [Paper/project](https://xingyu-lin.github.io/spawnnet/) and official code link | Fuses frozen multi-layer pretrained features with a separately learned CNN through adapters; real setup used third-person + wrist RGB-D, four-frame stack, and deliberately omitted proprioception after observing overfit. Output was 6-DoF delta end-effector action + gripper at 5 Hz. | xArm7 real category-generalization tasks; exact demonstrations/compute **UNKNOWN** in this audit. | Code available; license/weights **UNKNOWN**. | **MAJOR PORT, E3/D1--D4.** Highly relevant motivation for dense features/history/proprio ablations, but its Cartesian interface is not drop-in. |

### Why the first encoder comparison stays deliberately small

The scientifically clean order is:

1. scratch ResNet-18;
2. ImageNet ResNet-18, fully trainable with a lower backbone LR;
3. frozen ImageNet backbone with a trainable projection/head;
4. partial unfreeze of layer 4;
5. frozen DINOv2-S **spatial tokens** with a shallow adapter;
6. only if step 5 survives, unfreeze final DINO blocks.

This order tests initialization, capacity to adapt, and representation interface
without changing several axes simultaneously. It also respects Theia’s two
negative findings: global CLS tokens can be poor control features, and frozen
encoders are not universally better in long-horizon/domain-specific control.

### Multi-view, 3D, and object-centric perception

| Paper / year / venue | Reusable idea and official implementation facts | Evidence / cost | License / compatibility verdict |
| --- | --- | --- | --- |
| **iDP3**, 2025, IROS ([paper](https://arxiv.org/abs/2410.10803), [repository](https://github.com/YanjieZe/Improved-3D-Diffusion-Policy)) | Egocentric camera-frame XYZ cloud without base calibration/segmentation; 4,096 points; pyramid PointNet; **joint target** actions. Released config: 32-D state, 25-D action, obs 2, horizon 16, execute 15, 50 train/10 DDIM steps; state/cloud identity and action limit normalization. | Fourier GR1 + L515; pick/place settings use 1--2 demonstrations containing 20 rounds; showcases 10 demos × 10 rollouts. RTX 4090, ~30 min in reported setting; ~15 Hz onboard. Fine-tuned R3M DP was stronger in-domain, iDP3 much stronger OOD. | MIT; data/checkpoints for three tasks. **E4/D4, STRETCH.** Joint targets are a better semantic fit than DP3, but current depth is absent and authors discourage D435 quality. |
| **CAGE**, 2025, ICRA ([paper](https://arxiv.org/abs/2410.14974), [project](https://cage-policy.github.io/), [repository](https://github.com/cage-policy/CAGE)) | Workspace+wrist RGB history 4; frozen DINOv2-L + LoRA; view/time tokens compressed by causal Perceiver; diffusion predicts relative 20-step EE chunks, executes 8 or ensembles 12. Modest coherent perspective/crop augmentation improved held-out camera setting. | Flexiv+AG95, two D435; 50/40/40 demos on three tasks. ~280 ms asynchronous inference at 10 Hz on RTX 3090. Training 4×A100 80 GB, 500 epochs; repo notes ~60 GB/GPU at batch 16. | CC BY-NC-SA 4.0. **E4/D1, REJECT full model.** Reuse only the cheap augmentation/history hypotheses; CAGE does not prove generic cross-attention beats concat. |
| **VIOLA**, 2022, CoRL ([paper](https://proceedings.mlr.press/v205/zhu23a.html), [project](https://ut-austin-rpl.github.io/VIOLA/), [repository](https://github.com/UT-Austin-RPL/VIOLA)) | Workspace top-K pretrained region proposals (K=15 real), ROI features + box coordinates + global workspace + wrist RGB + proprio, 10-step history, transformer/GMM at 20 Hz. Includes color jitter, pixel shift and small random erasing. | Franka, three real tasks, 50 demos/task. Full dependency stack includes robomimic/robosuite/Detectron2/Detic. | MIT; datasets/checkpoints. **E3/D1, BACKLOG.** First test fixed/oracle crops; detector integration is justified only if the upper bound is positive. |
| **KALM**, 2025, ICRA ([paper](https://arxiv.org/abs/2410.23254), [project](https://kalm-il.github.io/), [repository](https://github.com/FANG-Xiaolin/KALM)) | GPT-4o proposes task region; SAM/SAM2, DINO+FeatUp and FPFH establish sparse correspondences; eight keypoints condition diffusion of 48 EE poses. Needs organized RGB-D, intrinsics/extrinsics, EE/world and joint trajectories. | Franka Research 3 + wrist D435i, three tasks, 10 demos/task; object/view/instance generalization. 200K iterations; compute **UNKNOWN**. | MIT; example data; trained task checkpoints **UNKNOWN**. **E4/D3--D4, STRETCH.** Counterexample that sparse D435i keypoints can work, but current labels/data do not fit. |
| **DemoGen**, 2025, RSS ([paper](https://arxiv.org/abs/2502.16932), [project](https://demo-generation.github.io/), [repository](https://github.com/TEA-Lab/DemoGen)) | Segments contact skills and free-space motion from one 3D demo; rigidly transforms skill segments, replans free space, edits point clouds/robot geometry, then emits zarr demonstrations. | Eight real tasks, one source demo/task, 530 evaluations; 2,214 generated trajectories/147K pairs in 22 s excluding slower rendering. Needs object segmentation, FK/planning and trusted 3D geometry. | MIT. **E4/D3--D4, REJECT now.** High-upside only after synchronized depth and geometry exist. |
| **EquiBot**, 2024, CoRL ([project](https://equi-bot.github.io/)) | Object-segmented 1,024-point cloud + 13-D proprio to SIM(3)-equivariant PointNet++ diffusion and 7-D EE velocity/gripper; obs 2, horizon 16, execute 8. | Kinova Gen3 + ZED2; six real tasks, 15 demos/task (~5 min), 3 Hz. Complete real perception uses Grounded-SAM, DEVA and a proprietary stereo model. | MIT repository; no generic weights. **E4/D4, REJECT.** Joint actions are not naturally SIM(3)-equivariant, creating a central mismatch. |
| **3D Diffuser Actor**, 2024, CoRL ([paper](https://arxiv.org/abs/2402.10885), [project](https://3d-diffuser-actor.github.io/), [repository](https://github.com/nickgkan/3d_diffuser_actor)) | Single/multi-view RGB-D + language + proprioception to a 3D feature field and diffused EE-pose trajectory. Requires calibrated views and Cartesian actions. | RLBench/CALVIN checkpoints; exact real-world demo count **UNKNOWN**; CUDA/DGL/flash-attention stack. | MIT; code/checkpoints. **E5/D4, REJECT.** Too many representation, action, language and dependency changes. |
| **Multi-View Masked World Models**, 2023, ICML ([paper](https://arxiv.org/abs/2302.02408), [project](https://sites.google.com/view/mv-mwm), [repository](https://github.com/younggyoseo/MV-MWM)) | Masks complete viewpoints during representation pretraining, then learns a world model; official TensorFlow/RLBench implementation. It establishes a motivation for whole-view masking, not for a particular two-view BC architecture. | Sim-to-real/viewpoint robustness; demo counts and compute **UNKNOWN**. Official repo warns released code may not reproduce the paper exactly. | No explicit license found: **UNKNOWN**. **E5/D4, REJECT architecture; cleanly reimplement view masking only.** |
| **Seeing from Hands**, 2022, ICLR Oral ([paper/project](https://sites.google.com/view/seeing-from-hands), [paper](https://arxiv.org/abs/2203.12677)) | Wrist view improves efficiency/OOD when hand-centric observability suffices; workspace view can be necessary yet overfit. Uses a variational bottleneck on third-person branch and simple concatenation. | Franka real grasping, 360 demos, 100² RGB, current frames + EE/gripper state. | Code/license **UNKNOWN**. **E2/D1 concept.** Strong reason to test asymmetric view roles; not evidence that its bottleneck will work with 25 long trajectories. |
| **RoboTAP**, 2024, ICRA ([project](https://robotap.github.io/), [repository](https://github.com/google-deepmind/tapnet)) | Tracks arbitrary visual points and uses point geometry for visual servo/control. A sparse object/hand track can be computed offline before committing to a detector or full point-cloud policy. | Real manipulation with very few demonstrations reported; exact action mapping/compute for JAKA **UNKNOWN**. | TapNet code license must be verified per release; weights available. **E3/D1--D3, BACKLOG.** Absolute joint-action adapter and occlusion handling are nontrivial. |
| **SAM 2**, 2024 ([paper/project](https://ai.meta.com/sam2/), [repository](https://github.com/facebookresearch/sam2)) | Promptable image/video segmentation; can cache bottle/box masks/crops for an **offline upper-bound** study. It does not provide task identity or a reliable prompt source at deployment. | No robot-policy demonstrations; segmentation compute/model size depend on checkpoint. | Apache-2.0 code/checkpoints. **E1 offline / E3 online.** Use only after fixed/oracle crop gains justify it. |
| **GreenAug**, 2024, arXiv ([project](https://greenaug.github.io/), [paper](https://arxiv.org/abs/2407.07868)) | Chroma-key green-screen backgrounds, then texture replacement. | 800+ demos over 8 real tasks and 8.2K evaluations. Existing demos lack a green screen. | Code linked; site CC BY-SA 4.0, code license must be checked separately. **D4, REJECT for current data.** Useful only as future collection-design evidence. |
| **GenAug**, 2023, arXiv ([project](https://genaug.github.io/)) | Generatively changes backgrounds, objects/textures/classes in RGB-D while respecting a task data-generation pipeline. | Tabletop examples from 10 demonstrations; reported ~40% real generalization improvement. Requires depth/calibration and generation validation. | Code linked; exact code/weight license **UNKNOWN**. **E4/D1--D4, REJECT initially.** Generated semantic errors could corrupt action labels. |
| **RoboEngine**, 2025, IROS ([paper](https://arxiv.org/abs/2503.18738), [project](https://roboengine.github.io/)) | Robo-SAM + object masks + background diffusion for plug-and-play scene augmentation. | Six new-scene evaluations; RoboSeg has 3,800 images from 35+ robot datasets. Release page promised data/weights; exact compute/license **UNKNOWN**. | Code/data links exist; license **UNKNOWN** in this audit. **E4/D1, BACKLOG.** Simple augmentation should fail first before adding a generative pipeline. |

### Low-data, history, retrieval, intervention, and execution

| Paper / year / venue | Reusable component and due diligence | Fit / cost | License / verdict |
| --- | --- | --- | --- |
| **robomimic BC-RNN**, 2021, CoRL ([paper](https://arxiv.org/abs/2108.03298), [project](https://robomimic.github.io/), [repository](https://github.com/ARISE-Initiative/robomimic)) | Configurable recurrent BC over image and low-dimensional observation sequences. No universal history length should be copied. | A two-frame or short-window derived LeRobot view is E2/D1; full framework port is unnecessary. | MIT. **DO NOW as history baseline.** |
| **Copycat Agents**, 2020, NeurIPS ([paper](https://proceedings.neurips.cc/paper/2020/hash/1b113258af3968aaf3969ca67e744ff8-Abstract.html)) | Demonstrates history can let BC infer and repeat previous expert actions rather than use causal scene information. | Add shuffled-history and previous-action-predictability controls; E1/D0--D1. | Method-level diagnostic; code reuse not needed. **Mandatory guardrail.** |
| **Towards Balanced Behavior Cloning**, 2025, Autonomous Robots ([paper](https://link.springer.com/article/10.1007/s10514-025-10237-0)) | Formalizes policy bias toward overrepresented behaviors; compares reweighting/meta-gradient balance on CALVIN RGB+wrist+state. It does **not** directly establish equal-episode sampling for a single task. | Start with frame/episode/phase occupancy and simple weights, E1--E2/D0--D1; skip meta-gradient initially. | Official code/license **UNKNOWN**. **DO simple audit.** |
| **GAP**, 2026, ICLR ([paper](https://arxiv.org/abs/2602.12032), [repository](https://github.com/GeWu-Lab/GAP)) | Finds proprioception can dominate learning during motion-transition phases; uses change-point/learned phase probabilities to reduce proprioceptive gradients. PyTorch 2.1 + customized `ruptures`. | Strong hypothesis fit. Offline change-point audit and groupwise state dropout E1--E2/D1; faithful phase-gradient method E3. | No explicit repo license: do not copy. **TOP-PRIORITY ANALYSIS.** |
| **NADA / proprioception shift**, 2025, arXiv ([paper](https://arxiv.org/abs/2506.23944), [project](https://proprioception-shift.github.io/)) | Measures expert/rollout state shift with time-conditioned Wasserstein distance, optimizes groupwise Gaussian noise, retrains. Reports Franka pick-place/locker; cheap masking is a baseline. | Full method needs rollout traces E3/D2. Current state has no velocity, so its velocity findings cannot be assumed. Run calibrated dropout/noise baselines only. | Code “coming soon”; license **UNKNOWN**. **NEXT after baseline.** |
| **VINN**, 2021, RSS ([paper](https://arxiv.org/abs/2112.01511), [project](https://jyopari.github.io/VINN/), [repository](https://github.com/jyopari/VINN)) | Learns/uses visual embedding, retrieves nearest demo frames, executes locally weighted actions. Explicit memory is transparent for 20,744 frames. | Clean 1-NN mathematical reimplementation, leave-one-episode-out; add normalized state/phase only after visual baseline. E2/D0--D1. Never start physical control with averaged absolute joints. | No top-level license found: do not copy. **DO NOW offline.** |
| **MT3**, 2025, Science Robotics ([paper](https://arxiv.org/abs/2511.10110), [project](https://www.robot-learning.uk/learning-1000-tasks), [repository](https://github.com/kamil-dreczkowski/learning_thousand_tasks)) | Decomposes alignment and interaction and retrieves both. Release expects 720×1280 RGB-D, binary object mask, intrinsics, SE(3) bottleneck pose and `T×7` EE twists at 30 Hz. | 3,450 controlled + 2,200 large-scale real rollouts; order-of-magnitude few-demo advantage under 10 demos/task. Full port E4/D4; simplified phase/retrieval hypothesis E2--E3/D1. | MIT. **Use as scientific motivation, not drop-in.** |
| **DemInf**, 2025, RSS ([paper](https://arxiv.org/abs/2502.08623), [project](https://jhejna.github.io/demonstration-info), [repository](https://github.com/jhejna/demonstration-information)) | kNN mutual-information estimate in learned state/action embeddings for demonstration curation; release uses JAX/Flax/RLDS and notes relative actions work best. | With 25 absolute-action episodes, estimates may be unstable. Do simple coverage/duration/phase audits first; E3/D0--D1. | MIT. **BACKLOG.** |
| **DAgger**, 2011, AISTATS ([paper](https://proceedings.mlr.press/v15/ross11a.html)) | Aggregates expert labels on learner-induced states to control sequential covariate shift. | Full oracle query assumption is impractical; human takeover is safer. Foundational citation, not literal reproduction. | Algorithmic reference. |
| **IWR**, 2020, arXiv ([paper](https://arxiv.org/abs/2012.06733), [project](https://sites.google.com/stanford.edu/iwr)) | Human takes over near bottlenecks; intervention/non-intervention samples are balanced 50/50. Outperformed an equivalent amount of full-demo data on threading/coffee. | Direct equal-human-time correction hypothesis; E2/D3, only after reliable rollout/intervention capture. | Reusable code/license **UNKNOWN**. **TOP post-rollout direction.** |
| **Sirius**, 2023, RSS ([paper](https://arxiv.org/abs/2211.08416), [project](https://ut-austin-rpl.github.io/sirius), [repository](https://github.com/UT-Austin-RPL/sirius)) | Trust-weighted BC and intervention memory management; released robomimic/HDF5 collection/training workflow. | Real contact-rich tasks; project reports 27% hardware success gain, 2× faster convergence and 85% memory reduction relative to its baselines. E2--E3/D3 adapter. | MIT. **TOP post-rollout implementation reference.** |
| **ThriftyDAgger**, 2021, CoRL ([paper](https://arxiv.org/abs/2109.08273)) | Robot-gated novelty + risk criteria solicit interventions under a desired supervisor budget. | Physical cable routing and user study. Automated risk gating is E3/D3 and premature; its budget framing is useful now. | Official reusable code/license **UNKNOWN**. **BACKLOG; use budget principle.** |
| **Diffusion Meets DAgger**, 2024, RSS ([paper](https://www.roboticsproceedings.org/rss20/p048.html)) | Synthesizes eye-in-hand OOD states instead of collecting them. Reports 80% pushing with 8 demos versus 20% BC, plus stacking/pouring/shirt tasks. | Faithful reproduction needs a generative state/action relabeling pipeline; E4/D1, and label validity is difficult for joint-space dual-view data. | Code/license **UNKNOWN**. **REJECT now; relevant reference for covariate-shift augmentation.** |
| **CCIL**, 2024, ICLR ([project/paper](https://personalrobotics.github.io/CCIL/), [repository](https://github.com/personalrobotics/CCIL)) | Learns locally Lipschitz state dynamics and synthesizes corrective labels; released interface consumes `T×o` observations and `T×a` actions. | 12-D numeric shapes superficially fit, but bottle/box/contact state is missing; synthesized state-only corrections may be physically false. E3/D1. | No explicit repo license. **ANALYSIS ONLY.** |
| **HIL-SERL**, 2024, arXiv ([paper](https://arxiv.org/abs/2410.21845), [project](https://hil-serl.github.io/), [repository](https://github.com/rail-berkeley/hil-serl)) | Reward classifier + demo buffer + online SAC + asynchronous actor/learner and interventions, tied to JAX/CUDA and Franka impedance infrastructure. | Real tasks with 100-trial evaluations; E5/D4--D5 for this system. | Apache-2.0. **REJECT now.** This is an online-RL/hardware project, not a small correction adapter. |
| **MILES**, 2024, CoRL ([paper](https://arxiv.org/abs/2410.19693), [project](https://www.robot-learning.uk/miles)) | One wrist-camera demonstration plus autonomous perturb-and-return collection with reachability/environment checks; paper uses around 10 perturbations per waypoint. | New autonomous motion/data/safety project, E5/D4. | Code linked; exact license **UNKNOWN**. **REJECT now.** |
| **ACT temporal ensembling**, 2023, RSS | Exponentially fuses overlapping predictions; original coefficient 0.01 and current LeRobot supports it when executing one action per query. | E1/D2 and valid rollout required. Must compare identical checkpoints/consumers after Thread A. | ACT MIT / LeRobot Apache-2.0. **NEXT, rollout-owned.** |
| **LeRobot asynchronous inference**, current official framework ([docs](https://huggingface.co/docs/lerobot/async), [design](https://huggingface.co/blog/async-robot-inference)) | Policy server, streamed observations, local action queue, replace/weighted overlap; parameters include actions/chunk and queue threshold. | Current project is pinned older, so E3/D2 runtime/safety port, overlapping Thread A. | Apache-2.0. **WAIT.** |

## Candidate research directions: broad screen

These are hypotheses, not a menu to combine indiscriminately. A direction
advances only if its cheapest falsification passes. Offline action loss is used
to kill ideas, not to claim physical task success.

| ID | Direction and testable hypothesis | Existing components | Cheapest falsification | E / D | Immediate status |
| --- | --- | --- | --- | --- | --- |
| R1 | **Visual pretraining sample efficiency.** ImageNet initialization should matter most at 5--15 demos if it reduces representation-learning burden. | ACT + TorchVision ResNet-18. | Same episode split, seeds, update budget and head; scratch vs ImageNet at nested 5 and 25-demo subsets first. | E1 / D1 | DO NOW |
| R2 | **Spatial foundation features, not pooled semantics.** Dense DINOv2 patches may preserve bottle/box geometry better than scratch CNN or CLS embeddings. | DINOv2-S + ACT token decoder; Theia findings. | Cache frozen spatial features; shallow episode-level action/phase probes. Reject if gains vanish under episode split or only occur with global/time leakage. | E2 / D1 | NEXT if R1 is positive |
| R3 | **Matched ACT versus Diffusion Policy.** Generative action modeling helps only if local expert futures are genuinely multimodal. | Pinned LeRobot ACT and Diffusion. | Quantify per-phase conditional action variance; smoke/overfit a DP with matched pretraining and ~16-step consumer. | E1--E2 / D1 | AUDIT FIRST |
| R4 | **Small VLA data boundary.** Robot/VLM pretraining may or may not outweigh embodiment shift at 25 demos. | SmolVLA base + LeRobot. | Model load, schema/stat check, one forward/backward, tiny-subset overfit, memory/latency record; stop before a sweep if any fail. | E1--E2 / D1 | AUDIT FIRST |
| R5 | **Short causal history.** `[t-1,t]` should add motion information without long-context overfit. | Diffusion Policy obs=2 or small ACT history adapter; BC-RNN precedent. | Current, two-frame, shuffled-previous-frame and state-history-only controls; matched parameters/updates. | E2 / D1 | DO NOW |
| R6 | **Phase-dependent modality shortcut.** Proprioception may dominate around motion transitions, where vision should matter most. | GAP/NADA insights + groupwise state dropout + observable history. | Derive change points from hand/action velocity, manually audit all 25, measure phase-wise gradients/occlusion sensitivity, then run state-group dropout controls. | E1--E3 / D0--D1 | TOP-PRIORITY ANALYSIS |
| R7 | **Episode/phase-balanced learning.** Current frame-uniform objective may overrepresent long/slow phases. | Weighted sampler; Balanced BC motivation. | Plot actual sampler mass, unique-frame exposure and phase occupancy; frame-uniform vs episode-uniform vs phase-uniform with matched optimizer steps. | E0--E2 / D0--D1 | DO NOW as enabling baseline |
| R8 | **View specialization and robustness.** Workspace provides global goal context; wrist provides local interaction; whole-view dropout may prevent brittle co-adaptation. | Current two views/shared ACT encoder; Seeing-from-Hands, CAGE, Octo. | Workspace-only, wrist-only, shared-both, separate-both; then modest whole-view dropout held constant over history. Evaluate normal and missing-view validation. | E1--E2 / D1 | DO NOW |
| R9 | **Object-centric upper bound.** Bottle/box localization may reduce small-data background burden, but crops may remove hand and approach context. | Fixed ROI/manual boxes; later SAM2/detector. | Full frame vs fixed task ROI vs oracle crop-plus-context on episode-held-out data. If oracle does not help, stop detector work. | E1--E2 / D0--D1 | DO NOW audit |
| R10 | **Semantics-preserving RGB augmentation.** Small coherent geometric/photometric shifts should improve camera/lighting robustness without changing action meaning. | LeRobot image transforms; CAGE/VIOLA evidence. | Mild crop/translation/color grid, transform fixed across temporal window; reject variants that reduce in-distribution fit without held-out shift benefit. | E1 / D1 | DO NOW |
| R11 | **Explicit memory baseline.** With 20,744 frames, nearest expert context can be competitive and exposes dataset coverage gaps. | Clean VINN-style 1-NN + pretrained embeddings. | Leave one episode out; image-only, image+state and phase-filtered retrieval; report phase consistency, first/chunk error and continuity. | E2 / D0--D1 | DO NOW offline |
| R12 | **Phase-conditioned or phase-regularized policy.** Separating approach/grasp/lift/transport/place/release may ease long-horizon BC only if phase is causally observable. | Change points, auxiliary phase head, phase balance. | Use phase labels for stratified loss/analysis first; compare oracle-label upper bound with a causal image/state predictor. Never feed demo time or oracle phase at rollout. | E2--E3 / D1 | AUDIT FIRST |
| R13 | **Equal-time targeted corrections.** Short failure-state corrections should outperform more nominal demonstrations under a fixed operator budget. | IWR/Sirius + current BC policy. | Protocol design now; after valid rollout compare equal operator minutes and labeled transitions across full demos, correction segments and balanced corrections. | E2--E3 / D3 | TOP post-rollout direction |
| R14 | **Recovery data allocation.** Five to ten deliberately sampled recovery clips may cover failure states better than nominal data. | DAgger/IWR/Sirius; failure taxonomy. | From validated rollout logs, predeclare failure classes and estimate their frequency/coverage; do not collect until classes and safe takeover/reset are operationally defined. | E2--E3 / D3 | NEXT after R13 pilot |
| R15 | **Execution-induced distribution shift.** The chunk consumer can matter as much as the checkpoint. | Fixed chunks, receding horizon, ACT temporal ensemble, later async/RTC. | Offline replay seam, prediction-age, action-replacement and deadline metrics; physical comparison only after the identical checkpoint is valid. | E1--E3 / D2 | WAIT for Thread A |
| R16 | **Hybrid learned/classical phases.** Deterministic close/release or grasp-completion logic may remove low-variance subproblems from BC. | Phase analysis + bounded state machine + learned motion. | Measure within-phase action variance and cross-demo alignment. If close/release is nearly deterministic, compare phase-specific offline predictors before controller integration. | E1--E2 / D1--D2 | BACKLOG |
| R17 | **True 3D representation for spatial generalization.** Metric geometry may help held-out bottle/box/view positions even if it does not improve in-domain success. | iDP3/Simple-DP3; RealSense depth; joint-action decoder. | Current-data gate already fails. Future short RGB-D sensor-quality audit, then matched RGB, RGB-D and XYZ encoders with the same action head/horizons. | E3--E4 / D4 | STRETCH |
| R18 | **Auxiliary dynamics/progress supervision.** Predicting next state, future feature or observable phase may regularize the representation. | Existing sequential labels; shallow auxiliary heads. | Linear probes first. Add one head only if its target is predictable under episode split and correlated with a failure-relevant phase, not merely time. | E2 / D1 | BACKLOG |
| R19 | **Demo subset/coverage selection.** Coverage may matter more than count, but learned quality ranking can overfit 25 samples. | Nested subsets; simple position/action/phase coverage; later DemInf. | Several coverage-stratified and random 5/10/15/21-demo subsets; multiple draws/seeds; no frame subsets. | E0--E1 / D1 | DO NOW with R1 |
| R20 | **Hybrid alignment/interaction retrieval.** Retrieve global alignment context and local interaction separately instead of asking one parametric policy to learn both. | MT3 decomposition + VINN memory + phase boundaries. | Offline phase-conditioned retrieval and continuity metrics. Only add a learned residual if 1-NN retrieves correct phase but misses precise actions. | E2--E3 / D0--D1 | NEXT if R11 succeeds |

### Augmentations that preserve this task’s semantics

Good first candidates are small translation/random-resized crop, mild
brightness/contrast/color jitter, small perspective after label inspection,
feature dropout, and whole-camera dropout. For temporal inputs, sample one
geometric transform per camera **per observation window**; independent
frame-to-frame transforms invent motion. Keep a camera-drop mask constant over
the window, and never drop both views simultaneously.

Do not begin with horizontal flips, arbitrary 90-degree rotations, strong
perspective warps, temporal shuffling, or image compositing that moves the
bottle without transforming its action labels. State noise must be calibrated
from measured synchronization/noise or train-deployment residuals, grouped by
arm/hand semantics. Arbitrary action noise is not augmentation of an expert
label and is excluded. Monocular predicted depth must be described as
pseudo-depth, not as a DP3 reproduction.

### Multi-view decision rule

There is no controlled general result that cross-view attention beats a matched
concatenation baseline for this setting. Start with separate encoders because a
fixed workspace camera and moving wrist camera have different statistics; keep
shared weights as an ablation because current ACT already uses them. Add camera
identity and missing-view masks before cross-attention. Cross-attention advances
only if concatenation is clearly limited under occlusion/camera shift and if it
beats a parameter-matched MLP.

For future RGB-D work, the least risky first combination is workspace point
cloud + wrist RGB. Merging two clouds sounds simple but wrist hand-eye/FK,
timestamp alignment, close-range holes and occlusion make it an E3 module.

## Top-10 idea matrix

The top ten include baseline/enabling experiments as well as paper directions.
“Our contribution” is intentionally narrower than the full combination.

| ID | Direction | Source paper(s) | Reusable component | What we would change | Our possible contribution | Engineering cost E0-E5 | Data cost D0-D5 | Can test now? | Requires valid rollout? | Expected paper value | Novelty risk | Implementation risk | License status | ICRA positioning | Verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| I1 | Pretrained spatial vision in low-demo ACT | ACT; ImageNet; DINOv2; Theia | Spatial pretrained backbone/tokens | Hold ACT/data/head fixed; vary scratch, trainable/frozen/partial ImageNet, then DINO spatial tokens | Evidence about which representation interface and adaptation regime improves **physical sample efficiency and spatial generalization**; not the swap itself | E1 initially, E2--E3 DINO | D1 | Yes | No for kill test; yes for paper | HIGH if tied to scaling + OOD; LOW as one swap | HIGH | LOW initially | ACT MIT; LeRobot/DINOv2 Apache-2.0; TorchVision BSD | ROBOT-LEARNING | DO NOW |
| I2 | Phase-dependent vision–proprioception robustness | GAP; NADA; Causal Confusion; BC-RNN | Change-point audit, group state dropout/noise, short history | Measure and regulate modality reliance around grasp/place transitions without oracle phase at inference | A causal phase-confidence or transition-aware regularizer that generalizes beyond JAKA; simple dropout alone is existing | E1--E3 | D0--D2 | Yes, offline | Physical claim yes | HIGH | MEDIUM | MEDIUM | GAP/NADA code license UNKNOWN; implement cleanly from paper | ROBOT-LEARNING | DO NOW analysis / NEXT method |
| I3 | Equal-human-time targeted corrections | DAgger; IWR; Sirius; ThriftyDAgger | Intervention capture labels and balanced/trust-weighted sampling | Compare additional full demos with short failure-state corrections under equal minutes and transitions | A general data-allocation result for low-data real manipulation; potentially a simple correction sampler/protocol | E2--E3 | D3 | Protocol only | Yes | HIGH | MEDIUM | MEDIUM | Sirius MIT; IWR/Thrifty code UNKNOWN; clean adapter | BALANCED | NEXT after valid rollout |
| I4 | View specialization + controlled view dropout | Seeing-from-Hands; CAGE; Octo; MV-MWM | Two-view ablations, shared/separate encoders, whole-view mask | Identify workspace/wrist roles and train for missing/shifted view robustness before adding attention | General insight about asymmetric camera observability in low-data long-horizon BC; camera dropout itself is not novel | E1--E2 | D1--D2 | Yes | Physical paper claim yes | MEDIUM--HIGH | HIGH | LOW--MEDIUM | Current LeRobot Apache-2.0; external methods only attributed | ROBOT-LEARNING | DO NOW |
| I5 | Explicit retrieval and phase-continuous memory | VINN; MT3 | Frozen embedding kNN and alignment/interaction decomposition | Leave-episode-out, phase-filtered, continuity-constrained retrieval of joint-action chunks; residual only if justified | A safe/continuous chunk retrieval principle or learned-versus-retrieved phase finding; not generic kNN | E2--E3 | D0--D2 | Yes | Only for physical claim | MEDIUM--HIGH | MEDIUM--HIGH | MEDIUM | MT3 MIT; VINN no license, so clean math reimplementation | BALANCED / ROBOT-LEARNING | DO NOW baseline |
| I6 | Episode/phase-balanced objective + demo scaling | Balanced BC; standard weighted sampling | Episode-uniform sampler, phase occupancy audit, nested subsets | Compare frame-, episode- and phase-uniform exposure with matched steps and unique frames; 5/10/15/21 demos | A measurement/enabler; publishable only inside a larger hypothesis about data allocation | E0--E2 | D0--D1 | Yes | No for kill test | LOW alone / MEDIUM as mechanism | HIGH | LOW | Repository-owned; paper code license UNKNOWN and not needed | ROBOT-LEARNING | DO NOW |
| I7 | Matched Diffusion Policy baseline | Diffusion Policy | Pinned native implementation, two-observation history, receding horizon | Match cameras, pretraining, split, action semantics and ~16-step execution to ACT | Controlled evidence on whether action-distribution modeling helps a low-modal 25-demo task; baseline, not architecture novelty | E1--E2 | D1--D2 | Yes offline | Yes for success | MEDIUM as evidence | HIGH | MEDIUM | Original MIT; LeRobot Apache-2.0 | BALANCED | AUDIT FIRST |
| I8 | SmolVLA low-data/embodiment boundary | SmolVLA | 450M pretrained VLA + flow expert | Explicitly map two images, task, 12-D absolute action, stats and 16-step consumer; first only load/overfit audit | A negative/positive scaling-boundary result if evaluated across demo counts/tasks; “SmolVLA on RH56” is not novelty | E1--E2 | D1--D2 | Yes audit | Yes for paper | MEDIUM | HIGH | MEDIUM--HIGH | Apache-2.0 code/base model; verify model card | ROBOT-LEARNING | AUDIT FIRST |
| I9 | Execution-induced distribution shift | ACT temporal ensemble; Diffusion receding horizon; LeRobot async; RTC | Overlap fusion, prediction-age/queue metrics | Same checkpoint, controlled consumers, latency/seam/response measurements | Insight connecting chunk consumer to closed-loop shift across policies; not a runtime port | E1--E3 | D0--D2 | Replay yes | Yes | MEDIUM--HIGH | MEDIUM--HIGH | HIGH due safety/timing | MIT/Apache-2.0 for relevant code | BALANCED / ROBOTICS-HEAVY | WAIT for Thread A |
| I10 | True 3D under equal action decoder | DP3; iDP3; RISE; KALM | Metric point cloud or sparse keypoints | Future synchronized RGB-D; compare RGB/RGB-D/XYZ with matched joint decoder, horizons and splits | When 3D helps low-demo joint-space manipulation under position/view shift; not a DP3 port | E3--E4 | D4 | No | Yes | HIGH stretch | MEDIUM | HIGH | Prefer iDP3/DP3 MIT; avoid RISE NC-SA reuse | BALANCED / ROBOT-LEARNING | STRETCH |

### Independent paper-value assessment

| ID | Novelty potential | Scientific clarity | Expected effect size | Reproducibility | Physical relevance | Reviewer defensibility | Implementation risk | New-data dependence | Rollout-bug dependence | Generality beyond robot | Algorithmic strength | Embodiment-specific strength | Incremental likelihood | Likely reviewer criticism |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| I1 | MEDIUM | HIGH | MEDIUM--HIGH | HIGH | HIGH | MEDIUM | LOW--MEDIUM | LOW | MEDIUM | HIGH | LOW--MEDIUM unless new adapter/analysis | MEDIUM | HIGH | “Known encoder swap; one task; pretraining confounds capacity.” |
| I2 | HIGH | HIGH | MEDIUM | MEDIUM | HIGH | HIGH if causal | MEDIUM | LOW--MEDIUM | MEDIUM | HIGH | HIGH if phase-conditioned regularizer is new | MEDIUM | MEDIUM | “Phase is a hidden clock; noise creates impossible inputs; force thread overlap.” |
| I3 | MEDIUM--HIGH | HIGH | HIGH | MEDIUM | HIGH | HIGH | MEDIUM | HIGH | HIGH | HIGH | MEDIUM | MEDIUM | MEDIUM | “Corrections had more useful labels/easier starts; operator-time accounting unfair.” |
| I4 | MEDIUM | HIGH | MEDIUM | HIGH | HIGH | MEDIUM | LOW--MEDIUM | LOW | MEDIUM | HIGH | LOW--MEDIUM | LOW | HIGH | “Camera dropout/concat are standard; missing-view test is artificial.” |
| I5 | MEDIUM | MEDIUM--HIGH | UNKNOWN--MEDIUM | HIGH offline | MEDIUM--HIGH | MEDIUM | MEDIUM | LOW | MEDIUM | MEDIUM--HIGH | MEDIUM if continuity mechanism is real | LOW | MEDIUM--HIGH | “Memory just memorizes a fixed scene; absolute-action neighbors are unsafe.” |
| I6 | LOW | HIGH | LOW--MEDIUM | HIGH | MEDIUM | LOW alone | LOW | LOW | LOW | HIGH | LOW | LOW | HIGH | “Sampler hygiene, not a contribution; oversamples short/poor demos.” |
| I7 | LOW | HIGH | UNKNOWN--MEDIUM | HIGH | HIGH | LOW alone | MEDIUM | LOW | HIGH | MEDIUM | LOW | LOW | HIGH | “Policy bakeoff with unmatched horizons/compute; no learning insight.” |
| I8 | LOW--MEDIUM | MEDIUM | UNKNOWN | MEDIUM | HIGH | LOW--MEDIUM | MEDIUM--HIGH | LOW--MEDIUM | HIGH | HIGH | LOW | LOW | HIGH | “One instruction does not test a VLA; 25 demos below recommended regime.” |
| I9 | MEDIUM | HIGH | MEDIUM--HIGH | MEDIUM | HIGH | MEDIUM | HIGH | LOW | HIGH | HIGH | MEDIUM if formalized | MEDIUM | MEDIUM--HIGH | “Engineering tuning; consumer comparisons alter effective feedback rate.” |
| I10 | MEDIUM--HIGH | HIGH | MEDIUM under OOD | MEDIUM | HIGH | HIGH if matched | HIGH | HIGH | HIGH | HIGH | MEDIUM | MEDIUM | MEDIUM | “Representation/action/horizon confounds; sensor quality; one tabletop task.” |

---

# Continuation update — 2026-08-13

The material above is retained as the initial Thread-C survey. This update
records what changed after local `dev` commit
`0dc8bf989e610c616517691a79097b516c5775dc` and the operator's manual
demonstration audit. Where the two sections conflict, this continuation is the
current decision record.

## 1. CONTINUATION STATUS

### COMPLETED

- Recovered the existing isolated worktree and branch. The shared `dev`
  worktree was not modified; `research/broad-exploration` was fast-forwarded
  only to local `dev` at `0dc8bf9`.
- Audited repository data contracts, policy interfaces, camera roles, depth
  availability, pinned LeRobot support, and more than 30 serious papers and
  official projects. The original policy, vision, 3D, retrieval, intervention,
  and execution matrices above remain useful.
- Established that maintained demonstrations contain two RGB streams and
  12-D state/action, while synchronized metric depth was not recorded. A true
  current-data DP3 reproduction is therefore closed.
- Ranked the initial R1--R20 directions and I1--I10 ideas, including explicit
  licenses and implementation distances.
- Inspected the new ACT failure report and supporting offline evidence rather
  than treating the physical symptom as a runtime-freshness problem.

### PARTIAL

- The broad report existed as an uncommitted 399-line living document. This
  continuation completes its missing decision sections; it still intentionally
  contains no physical result.
- Thread A added an ACT ImageNet-pretrained comparison config, but no clean
  nominal16 scratch/pretrained comparison has been trained. The old val4 data
  cannot support that causal claim.
- Phase, quality, retrieval, and history probes have designs and literature
  support, but Thread C has not implemented or run them.
- SmolVLA and Diffusion Policy are interface-audited. Neither has been promoted
  to a physical baseline, and no large training run was launched.

### NOT STARTED

- No Thread-C model training, sampler implementation, history adapter,
  retrieval baseline, action-representation conversion, or robot evaluation.
- No new demonstration, correction, recovery, RGB-D, or force collection.
- No paper modification, external-repository modification, or merge. Before
  this continuation, Thread C had made no commit and the survey was untracked;
  this report is intended to be the first isolated, documentation-only commit.

### BLOCKED

- Final clean-data comparisons must wait for Thread A's canonical corrected
  nominal16 dataset view and source-level split. Thread C will not reproduce or
  compete with that segmentation.
- Correction-versus-full-demonstration experiments require a valid rollout,
  a separately reviewed intervention protocol, and explicit authorization for
  physical operation. None is granted by this report.
- Physical claims for any policy remain blocked on Thread A's valid baseline
  and controlled rollouts. Offline validation may kill ideas, not establish
  task success.

### SUPERSEDED BY NEW EVIDENCE

- “25 clean expert trajectories” is superseded. There are 25 automatically
  materialized trajectories, but approximately 16 operator-audited nominal
  segments after correcting source episode boundaries.
- The old val4 0/56 transition-anticipation statistic is a valuable mechanism
  clue, not final clean-data evidence, because the training/validation material
  was mixed quality and validation source 89 is not in the operator's nominal
  list.
- “Try a larger policy first” is downgraded. A larger decoder sees the same
  inconsistent demonstrations and may learn the same persistence shortcut.
- Current-data DP3 is superseded by the negative depth audit. It is not merely
  waiting for an adapter.

The first unfinished high-value item is therefore a controlled, canonical-data
test of **demonstration quality and rare critical transitions**, before broad
architecture replacement.

## 2. NEW EVIDENCE FROM ACT FAILURE

Thread A's diagnosis in
[`physical_bottle_act_baseline_diagnosis_20260813.md`](physical_bottle_act_baseline_diagnosis_20260813.md)
establishes the following on the **old mixed-quality dataset/checkpoint**:

- runtime freshness was restored and was no longer the primary failure;
- consuming two actions per chunk did not explain failure;
- later positions in the predicted chunk did not hide a grasp command;
- on 56 recorded approach-to-closure transitions whose future closure entered
  chunk positions 2--15, the model anticipated 0;
- when the recorded observation was already in a grasped state, it predicted
  closure almost all the time;
- predicted RH56 chunks had only about `0.000123--0.000160` mean peak-to-peak
  range, indicating a near-static target sequence;
- an older five-demonstration checkpoint anticipated 13/70 measured cases, so
  the symptom is not evidence of a universal adapter or controller defect.

The most economical interpretation is a possible closed-loop fixed point:
the model waits for grasp-state evidence that its own action must create. That
is consistent with several mechanisms but proves none of them.

| Candidate mechanism | What the evidence supports | What remains unknown | Cheapest discriminating test |
| --- | --- | --- | --- |
| Rare-event / phase imbalance | Grasp initiation occupies few frames relative to persistent open/closed phases; recent correction work independently reports long phases dominating datasets. | Event frequency in canonical nominal16 and whether weighting alone changes anticipation. | Derive event windows from commanded hand-target changes; compare frame-uniform and event-balanced training with identical updates. |
| Mixed-quality contradictory supervision | Human audit identifies missing approaches, stalls and collisions in many sources. | Whether nominal16 alone restores anticipatory chunks, or whether quantity loss offsets quality gain. | Mixed corrected view versus nominal16, plus matched-count random mixed subsets. |
| Single-frame temporal aliasing | The same pose/image may occur just before “stay open” and “begin close”; current ACT sees one observation. | Whether two recent frames make the boundary causally separable rather than merely revealing trajectory time. | One frame versus two-frame history, plus shuffled-previous-frame and previous-action predictability controls. |
| Absolute-action persistence shortcut | Long static intervals make copying the current absolute hand target a low-loss predictor. | Whether chunk-relative targets improve rare transitions for RH56; modern action-space evidence usually keeps grippers absolute. | Change only the hand target to a chunk-relative-to-query representation, reconstruct absolute targets before deployment, and measure event/non-event errors. |
| Multimodal future at a boundary | A single observation might admit both wait and close futures. | Whether local action futures are actually multimodal after conditioning on short history and phase. | Cluster normalized future hand chunks inside nearest observation neighborhoods before porting a generative model. |
| Vision suppressed by proprioception | GAP reports underuse of vision around motion transitions. | Whether current joint state creates that shortcut; no gradient/occlusion evidence exists yet. | Phase-stratified image/state occlusion and gradient sensitivity; try state-group dropout only if the audit is positive. |
| Chunk execution | Execution affects closed-loop response. | It cannot explain why the offline chunk itself is nearly static and contains no future grasp. | Leave with Thread A until a valid checkpoint; compare consumers only with the same checkpoint. |

This changes the evaluation contract. Aggregate action MSE is insufficient.
Every clean-data training comparison should also report:

1. event frequency and sampler exposure;
2. first-action grasp/release transition precision and recall;
3. future-chunk transition recall by horizon position;
4. per-channel chunk peak-to-peak dynamic range;
5. non-event hand error and arm error, to detect destructive oversampling;
6. episode/source-level validation, never random-frame validation.

The rare-transition hypothesis is broader than grasping only if it is tested on
at least two event types (for example grasp and release, or contact-to-lift) and
does not require an oracle phase token at deployment.

## 3. NEW EVIDENCE FROM HUMAN DATA AUDIT

The operator identifies these nominal full source episodes:

`67, 70, 81, 87, 88, 95, 96, 97, 98, 108, 109, 116, 117`.

Source episode 99 contains two nominal demonstrations that the automatic
splitter failed to separate. Source episode 102 also contains two
demonstrations: the first is non-nominal and the second nominal. The corrected
total is therefore approximately 16 nominal trajectory segments. Other source
episodes can contain missing approach, stalls, bottle collision/knock-down, or
other non-nominal behavior.

Consequences for scientific use:

| Dataset use | Allowed interpretation | Required guardrail |
| --- | --- | --- |
| Canonical nominal16 | Expert-policy baseline and final clean training comparisons. | Use Thread A's canonical view; split by source, so two segments from one source never cross train/validation. |
| Corrected all-data view | Mixed-quality imitation and quality-scoring research. | Preserve quality labels and segment provenance; never call it expert-only. |
| Excluded/non-nominal segments | Failure taxonomy, automatic filtering, transition detection, or future recovery/correction research. | Do not silently mix into nominal BC or relabel failure as expert action. |
| Old v2/val4 results | Historical diagnosis and pipeline evidence. | Label every result “old mixed-quality”; do not use as final method comparison. |

The high-value question is not the unsurprising statement that human curation
helps. It is whether a small, reproducible signal can identify **which segments
or temporal regions are useful for downstream policy behavior**. The four
conceptual baselines must be kept distinct:

- **A: naive mixed-quality BC** — corrected boundaries, every segment equally
  treated as expert;
- **B: human-curated nominal BC** — the essential upper-quality baseline;
- **C: automatic selection** — a held-out scorer or deterministic segment rule,
  evaluated against human labels and downstream policy impact;
- **D: quality weighting** — retains uncertain data but changes its influence;
- **E: targeted replacement/correction** — collects support specifically near
  learner failure states, only after a valid rollout.

A fair quality-versus-count result needs both a natural comparison (all mixed
data versus nominal16) and a matched-budget control (nominal16 versus multiple
random or coverage-matched 16-segment subsets from the corrected mixed pool).
Otherwise quality, quantity, segmentation, and optimizer exposure are
confounded.

## 4. UPDATED LITERATURE MAP

The continuation search focused only on gaps exposed by the new evidence. It
did not repeat the policy/vision/3D catalog above.

### Demonstration quality and critical-transition work

| Work | Exact reusable idea | Due diligence | Compatibility and decision |
| --- | --- | --- | --- |
| **S2I: Towards Effective Utilization of Mixed-Quality Demonstrations via Segment-Level Selection and Optimization**, ICRA 2025 ([paper](https://arxiv.org/abs/2409.19917), [project](https://tonyfang.net/s2i/), [code](https://github.com/junxix/s2i)) | Segment at gripper changes / near-zero robot velocity; learn a contrastive segment representation from three expert references; select/weight similar segments; optionally optimize and relabel trajectories. | Demonstrated with BC-RNN, ACT, Diffusion Policy and RISE in simulation and on three Flexiv+AG95 real tasks with two D435s. Real studies use 50-demo mixed sets with different expert/suboptimal proportions; exact collection accounting in the paper is not fully clear. Unified observation/action spec and training compute are **UNKNOWN** because downstream policies differ. Code is MIT; example configs/data exist; generic pretrained weights are **UNKNOWN**. | Three clean reference demos and segment-level selection fit conceptually. Full optimization/relabeling is E3--E4 and action-space specific. **Clean selection audit E2/D0--D1; do not reproduce full S2I first.** |
| **GAP: When Would Vision-Proprioception Policies Fail in Robotic Manipulation?**, ICLR 2026 ([paper](https://arxiv.org/abs/2602.12032), [project](https://gewu-lab.github.io/GAP/), [code](https://github.com/GeWu-Lab/GAP)) | Detect motion-consistent phases from end-effector/gripper changes; learn continuous transition probability from proprioceptive differences; attenuate proprioceptive gradients during transitions. | Five-observation history, ResNet-18 + temporal transformer, proprio MLP, action sequence 9. Simulation uses 100/500 demos; real xArm/Robotiq and Cobot Magic experiments use 50 demos/task and 20 rollouts. One RTX 3090; reported examples span roughly 2--8 h, exact real time **UNKNOWN**. No explicit repository license found. | Strong mechanism match, not proof of our cause. Current joint/action differences can propose events, but copying code is prohibited absent a license. Audit E0--E1; faithful method E3. **UPGRADE as scientific reference.** |
| **Towards Balanced Behavior Cloning from Imbalanced Datasets**, Autonomous Robots 2026 ([paper](https://link.springer.com/article/10.1007/s10514-025-10237-0), [preprint](https://arxiv.org/abs/2508.06319)) | Shows equal per-sample weighting biases BC toward frequent subpolicies; evaluates fixed and learned subpolicy weights. | Controlled simulation tasks with known behavior groups; no directly matching 16-demo real-robot result. Exact code/license and compute are **UNKNOWN**. | Supports a phase/event-balanced control, not the claim that equal phase weights are optimal. Simple reweighting E1--E2/D1; meta-gradient version is unnecessary now. |
| **UVD: Universal Visual Decomposer**, ICRA 2024 ([paper](https://arxiv.org/abs/2310.08581), [project](https://zcczhang.github.io/UVD/), [code](https://github.com/zcczhang/UVD)) | Recursively propose long-horizon phase boundaries from extrema in distances between frozen visual embeddings (VIP/R3M/LIV/CLIP/VC-1/DINOv2). | RGB-video-only decomposition; real Franka multistage manipulation. Exact real demonstration count and compute are **UNKNOWN**. Code is MIT; pretrained representation weights are external. | E1/D0 offline phase proposal. S2I reports visual decomposition can over-segment noisy suboptimal demos, so compare it against hand/action change points and operator labels; never trust it automatically. |
| **DataMIL: Selecting Data for Robot Imitation Learning with Datamodels**, arXiv 2025, revised 2026 ([paper](https://arxiv.org/abs/2505.09603), [project](https://robin-lab.cs.utexas.edu/datamodels4imitation/), [code](https://github.com/UT-Austin-RobIn/datamil)) | Approximate each temporal cluster's policy-dependent effect on a target validation objective, then select and co-train source/target data. | More than 60 simulated/real tasks and OXE-source transfer to four real target tasks, including a new Tiago embodiment. JAX/TensorFlow/RLDS/MDS stack, optional Octo, four-GPU experiments and five seeds. Code is MIT; task checkpoints are not directly reusable here. | Scientifically stronger than hand heuristics but statistically and computationally disproportionate to 16 same-task trajectories. **E4/D1, KILL implementation; retain influence-based evaluation idea only.** |
| **DemInf**, RSS 2025 ([paper](https://arxiv.org/abs/2502.08623), [project](https://jhejna.github.io/demonstration-info), [code](https://github.com/jhejna/demonstration-information)) | Score demonstrations with a kNN mutual-information estimator in learned state/action embeddings. | JAX/Flax/RLDS release; relative actions worked best in its experiments. Exact estimator stability at 16 trajectories is not established. MIT. | E3/D0--D1. Use only after simple human-label, coverage, duration and event-occupancy baselines; **DOWNGRADE as immediate implementation.** |

### Action representation, temporal structure, and corrections

| Work | Exact reusable idea | Due diligence | Compatibility and decision |
| --- | --- | --- | --- |
| **Demystifying Action Space Design for Robotic Manipulation Policies**, 2026 (the official project labels it ICML 2026; an OpenReview PDF is also labeled ICLR 2026, so venue metadata is inconsistent) ([paper](https://arxiv.org/abs/2602.23408), [project](https://cathyf9600.github.io/empirical/), [code](https://github.com/CathyF9600/DemystifyActionSpace)) | A **chunk-wise relative** target subtracts the query-time state from every chunk step; unlike sequential deltas, reconstruction does not accumulate error across the chunk. The study finds chunk-wise relative actions and shorter execution horizons strong in tested settings. | More than 2,000 demonstrations, 13,000 real rollouts and 500 models across four tasks; main real settings use 250 demos/task with 100/250/500 scaling. Joint and task-space actions are compared. Exact training compute is **UNKNOWN**. No top-level code license was found: do not copy. | Far larger than nominal16 and therefore motivation, not a prediction. Current LeRobot guidance normally keeps grippers absolute, making RH56-relative an open controlled test. **E2/D1; test hand-only chunk-relative, never sequential delta first.** |
| **LeRobot action representations**, current official documentation ([docs](https://huggingface.co/docs/lerobot/action_representations)) | Defines absolute, query-state-relative, and sequential-delta representations; relative targets are converted before normalization. Supports excluding specified joints, especially grippers. | Documentation reflects current upstream and may not exist in pinned 0.6.2; an adapter must be repository-owned and checkpointed with its stats. Apache-2.0 framework. | Use as semantics reference, not as evidence of benefit. Arm remains unchanged in the first probe; predicted RH56 relative chunks are reconstructed to legal absolute targets before any deployment. |
| **AWE: Waypoint-Based Imitation Learning for Robotic Manipulation**, CoRL 2023 ([paper](https://proceedings.mlr.press/v229/shi23b.html), [project](https://lucys0.github.io/awe/), [code](https://github.com/lucys0/awe)) | Dynamic programming finds the minimum position-control waypoints whose interpolation stays within a reconstruction tolerance, reducing effective decision horizon. | Proprioceptive joint or EE waypoints, including gripper width; demonstrated with ACT and Diffusion Policy. Simulation commonly uses 50 demos/task; RoboMimic 30--200. Real ALOHA uses four RGB views and joint actions at 50 Hz; exact real demo count and compute are **UNKNOWN**. No repository license found. | The current absolute joint targets fit well. First do E1/D0 compression and transition-retention audit. Policy port is E3/D1 and only justified if compression is large without deleting grasp/release events. |
| **HYDRA: Hybrid Robot Actions for Imitation Learning**, CoRL 2023 ([paper](https://arxiv.org/abs/2306.17237), [project](https://sites.google.com/view/hydra-il-2023)) | Use sparse waypoint actions in free space and dense one-step delta actions in contact/dexterous phases, with a learned mode classifier. | Three simulation and four real tasks; released description uses EE/gripper observations/actions, human mode labels, a waypoint controller, RNN dense head and MLP sparse head. Exact real demo counts, compute, official code and license are **UNKNOWN**. | Useful coarse-free-space/fine-transition hypothesis; full port E3--E4 because joint actions and a safe waypoint controller differ. **Audit action compressibility first.** |
| **Set-Supervised Diffusion Policy (SDP)**, RSS 2026 ([paper](https://arxiv.org/abs/2606.01865), [project](https://set-supervised-diffusion-policy.github.io/), [code](https://github.com/ZhaotingLi/Set_Supervised_DP)) | Pair the robot's rejected action chunk with the human's corrective chunk, define a set of desired chunks, and train diffusion to sample inside that set rather than imitating one correction exactly. | Real observations are two RGB cameras + EE pose; observation horizon 2, action horizon 16, execute 8 at 10 Hz. Insert-T uses 2-D position; round-table uses absolute world-frame EE pose + gripper. Real studies use 50 demonstrations + 40 intervention episodes (Insert-T), and Demo30 versus Demo30+Corrections versus Demo60 (round-table). Demo30 and each continuation train 12 h on an A40. MIT code; real ROS1/Franka adapters and HDF5 correction examples; no JAKA checkpoint. | Strongest new evidence for corrections: with roughly equal data, corrections concentrate on bottleneck stages and outperform 30 extra full demos. Full set-supervised diffusion is E4; an equal-operator-time ACT/DP correction protocol is E2--E3/D3 after valid rollout. **UPGRADE post-rollout direction.** |
| **PF-DAG: Primary-Fine Decoupling for Action Generation**, ICLR 2026 ([paper](https://arxiv.org/abs/2602.21684), [project](https://xiaohanlei.github.io/projects/PF-DAG/), [code](https://github.com/XiaohanLei/PF-DAG)) | VQ-VAE action-chunk modes plus a mode-conditioned continuous flow decoder explicitly factor coarse action mode and fine trajectory. | Simulation and real tactile dexterous experiments; xArm7, L515 and XHand/Quest. Exact target-compatible action interface, real demo count and compute are **UNKNOWN**. Repository is MIT but marked early access and not fully deployed. | Could address multimodal boundary actions, but current data has not shown local multimodality. **E4/D1, KILL until a multimodality audit is positive.** |
| **SARM: Stage-Aware Reward Modeling**, ICLR 2026 ([paper](https://arxiv.org/abs/2509.25358), [code](https://github.com/xdofai/opensarm), [LeRobot docs](https://huggingface.co/docs/lerobot/sarm)) | Stage classifier + within-stage progress regressor from RGB/joints; use learned reward to filter or reweight demonstrations. | Frozen CLIP and natural-language subtask annotations; long-sequence T-shirt-folding data on a much larger scale than 16 demonstrations. Exact general checkpoint, compute, and repository license are **UNKNOWN** in this audit. Current pinned LeRobot does not include the newer support. | Progress supervision is relevant, but a learned reward model and manual language stages are disproportionate. **E4/D3, KILL full port; use simple causal progress probes only.** |
| **The Pitfalls of Imitation Learning when Actions are Continuous**, COLT 2025 ([paper](https://proceedings.mlr.press/v291/simchowitz25a.html)) | Establishes that small expert-distribution error for smooth deterministic Markov policies need not imply small closed-loop error; stochastic/non-Markov policies and broader expert support can help. | Theory plus illustrative experiments, not a matching real-robot recipe; reusable code/license are **UNKNOWN/not required**. | Supports evaluating closed-loop support and history/generative baselines, but does not specifically prove a grasp-transition failure. **Conceptual reference only.** |

The updated causal map is:

```text
mixed demonstrations + long persistent phases
                    |
          corrected source segmentation
                    |
       +------------+-------------+
       |                          |
quality/utility question     critical-event question
       |                          |
human nominal seed          event proposals from action/state
selection / weighting       exposure + identifiability audit
       |                          |
       +------------+-------------+
                    |
          matched clean BC baseline
                    |
       +------------+-------------+
       |            |             |
pretrained vision  short history  chunk-relative hand target
       |            |             |
       +------------+-------------+
                    |
       diffusion only if futures are multimodal
                    |
 targeted corrections only after a valid rollout
```

This is a sequence of falsifications, not a request to train the combined
diagram as one model.

## 5. UPDATED IDEA MATRIX

This table evolves I1--I10 rather than resetting them. New candidates start at
I11. “Novelty risk HIGH” means a high risk of being perceived as incremental.
“Can test now?” assumes offline access only; rows requiring nominal16 say so
explicitly. All physical work remains unauthorized in this thread.

| Rank / ID | Direction | Status versus initial matrix and why | Existing components | What we would change / what could be ours | E / D | Can test now? / valid rollout? | Time to first falsification | Expected effect | Paper value | Scientific generality | ICRA robot-learning fit | Novelty risk | Implementation risk | Physical-robot dependence | Force | New demos | License status | Verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| **1 / I11** | Critical-transition learning | **NEW, UPGRADE to #1.** The offline 0/56 anticipation and near-static chunks identify a concrete failure, while GAP/Balanced BC supply independent mechanisms. | GAP change points/transition probabilities; balanced BC; ACT metrics; short history. | First add self-supervised event proposals and event-aware metrics/sampling. A publishable contribution would isolate event **rarity**, **observability**, and **action representation**, then introduce only the smallest mechanism needed; event sampling alone is borrowed/enabling. | E0--E2 / D0--D1 | After canonical nominal16; no rollout for kill test, yes for final claim | VERY SHORT | HIGH if imbalance is causal; otherwise diagnostic | HIGH if shown across events/tasks | HIGH | HIGH | MEDIUM | LOW--MEDIUM | LOW offline / MEDIUM final | NONE | NONE initially | Clean-room implementation; GAP repo has no explicit license; weighting is repository-owned | **DO NOW** after nominal16 |
| **2 / I12** | Quality-over-count and segment utility | **NEW, UPGRADE to #2.** Human audit changes the effective expert set from 25 records to about 16 nominal segments. | S2I segment selection; DemInf; DataMIL; human labels. | Compare mixed, human-curated, automatic selection and weighting. Our possible contribution is a low-sample, event-aware segment-utility signal that predicts downstream policy impact—not “we removed bad demos.” | E0--E2 / D0--D1 | After Thread A publishes corrected views; no rollout for kill test | VERY SHORT | HIGH | HIGH only with automatic/generalizable criterion | HIGH | HIGH | HIGH | LOW--MEDIUM | LOW offline / MEDIUM final | NONE | NONE | S2I/DemInf/DataMIL MIT; scorer can be clean repository code | **DO NOW**; curation alone is baseline |
| **3 / I1** | Clean pretrained spatial vision + ACT | **KEEP near top.** The E1 control already exists, but it must be rerun scratch versus ImageNet on the same nominal16, not dirty versus clean. | ACT; TorchVision ImageNet ResNet-18; later DINOv2 spatial tokens. | Change initialization only. Our possible contribution requires sample-efficiency and spatial/OOD evidence or an event-specific representation insight; the backbone swap is not novel. | E1 initially / D1 | After nominal16; no rollout for offline kill, yes for paper | SHORT | MEDIUM | MEDIUM; LOW as one swap | MEDIUM--HIGH | MEDIUM--HIGH | HIGH | LOW | LOW offline / MEDIUM final | NONE | NONE | ACT MIT; LeRobot/DINOv2 Apache-2.0; TorchVision BSD | **DO NOW** after data control |
| **4 / I3** | Equal-human-time targeted corrections | **UPGRADE.** SDP gives direct 2026 real evidence that corrections concentrate at bottleneck stages and can beat roughly equal full-demo data. | DAgger, IWR, Sirius, SDP; existing rollout/capture pipeline once valid. | Compare equal operator minutes, frames and transition labels for full demos versus targeted corrections. Ours could be event-directed correction allocation for rare skill transitions; a plain SDP/IWR port is not novel. | E2--E3 / D3 | Protocol now; experiment requires valid rollout | MEDIUM | HIGH | HIGH | HIGH | HIGH | MEDIUM | MEDIUM | HIGH | NONE | SMALL | Sirius/SDP MIT; IWR license UNKNOWN | **NEXT** after valid rollout and authorization |
| **5 / I7** | Stronger-policy falsification ladder: Diffusion Policy first | **UPGRADE as required baseline, not paper idea.** Its two-observation history, pretrained vision and generative head could address ambiguity, but also confound three axes. | Pinned LeRobot Diffusion Policy; original MIT implementation. | First match cameras, clean split, normalization, action horizon, execute horizon and compute. Only claim decoder value after history/pretraining controls. Our contribution is the controlled boundary, not diffusion. | E1--E2 / D1--D2 | Schema audit now; clean training after nominal16; rollout for success | SHORT--MEDIUM | UNKNOWN--MEDIUM | MEDIUM as evidence / LOW alone | MEDIUM | MEDIUM | HIGH | MEDIUM | MEDIUM | NONE | NONE | Original MIT; LeRobot Apache-2.0 | **AUDIT/NEXT**, after simple ACT tests |
| **6 / I13** | Chunk-relative RH56 action target | **NEW.** Modern large-scale evidence favors chunk-wise relative actions, and current absolute hand targets may reward persistence. | ICML 2026 action-space study; LeRobot action semantics. | Keep arm absolute; subtract current measured RH56 position from every future hand target at query time; normalize relative data; reconstruct absolute targets before the existing adapter. Ours could be interaction between rare events and action coordinates in small-data underactuated control, not relative actions themselves. | E2 / D1 | After nominal16; no rollout for kill test | SHORT | UNKNOWN--MEDIUM | MEDIUM if mechanistic | HIGH | HIGH | HIGH | LOW--MEDIUM | LOW offline / MEDIUM final | NONE | NONE | LeRobot Apache-2.0; external comparison repo license UNKNOWN, so clean implementation | **NEXT** after event audit; reject sequential delta |
| **7 / I6** | Episode/phase/event-balanced objective | **UPGRADE as enabler, ABSORB under I11.** The new failure is phase-local and human audit changes phase occupancy. | Weighted sampling; Balanced BC motivation. | Frame-uniform versus episode-uniform versus event-window-balanced exposure with matched optimizer steps and unique-frame accounting. This is not standalone novelty. | E0--E2 / D0--D1 | After canonical view; no rollout for kill test | VERY SHORT | MEDIUM--HIGH | LOW alone / HIGH inside I11 | HIGH | MEDIUM--HIGH | HIGH | LOW | LOW | NONE | NONE | Repository-owned; no external code needed | **DO NOW** within I11 |
| **8 / I2** | Phase-dependent vision/proprioception robustness | **KEEP but DOWNGRADE behind simpler causes.** GAP strengthens the hypothesis, yet no local gradient/occlusion evidence shows proprioception is the culprit. | GAP; NADA; state/image masking and phase-stratified sensitivity. | Measure modality use at event windows, then try grouped state dropout or transition-conditioned gradient scaling. Our contribution requires causal cross-embodiment evidence; dropout alone is standard. | E0--E3 / D0--D2 | Audit after nominal16; physical shift later | SHORT | UNKNOWN--MEDIUM | HIGH if causal | HIGH | HIGH | MEDIUM | MEDIUM | LOW offline / MEDIUM final | OPTIONAL diagnostic only | NONE initially | GAP/NADA licenses UNKNOWN; clean implementation only | **AUDIT FIRST** |
| **9 / I5** | Retrieval / expert-continuation memory | **KEEP, slight DOWNGRADE.** Sixteen nominal trajectories make explicit memory attractive, but quality/transition causes are more directly evidenced. | VINN mathematics; MT3 alignment/interaction split; frozen visual features. | Leave-source-out nearest frame/segment and retrieve an expert continuation with phase/continuity gates. Ours needs a safe continuity or coverage insight; kNN itself is known. | E2--E3 / D0--D2 | Yes after nominal view; rollout only for physical claim | SHORT | UNKNOWN--MEDIUM | MEDIUM | MEDIUM | MEDIUM--HIGH | MEDIUM--HIGH | MEDIUM | LOW offline / MEDIUM final | NONE | NONE | MT3 MIT; VINN no license—clean implementation only | **NEXT negative control** |
| **10 / I14** | Waypoint/hybrid temporal abstraction | **NEW.** AWE/HYDRA directly question whether every 30 Hz target should be learned equally. | AWE waypoint extraction; HYDRA sparse/dense modes. | First quantify reconstruction compression and whether grasp/release survive. A possible contribution is automatically allocating dense prediction only around learned critical events in joint-space manipulation; full state-machine control is not the idea. | E1 audit, E3 method / D0--D2 | Offline audit now after nominal view; final requires rollout | VERY SHORT audit / MEDIUM method | UNKNOWN | MEDIUM--HIGH if compression is causal | HIGH | HIGH | MEDIUM | MEDIUM--HIGH | LOW audit / HIGH method | NONE | NONE initially | AWE/HYDRA code/license UNKNOWN; clean-room math only | **AUDIT FIRST / HOLD port** |
| **11 / I4** | View specialization + view dropout | **DOWNGRADE.** Still cheap, but new evidence points first to labels/events rather than camera fusion. | Current two cameras; Seeing-from-Hands; CAGE. | Matched workspace/wrist/both/shared/separate tests, then coherent whole-view dropout. Any contribution needs a partial-observability/generalization result, not camera masking alone. | E1--E2 / D1--D2 | After nominal16; rollout for OOD claim | SHORT | MEDIUM | MEDIUM | MEDIUM--HIGH | MEDIUM | HIGH | LOW--MEDIUM | MEDIUM | NONE | NONE | Local/LeRobot Apache-2.0; external ideas attributed | **NEXT**, not top three |
| **12 / I8** | SmolVLA low-data boundary | **DOWNGRADE to compatibility-only.** Official guidance already reports poor performance around 25 episodes and recommends about 50; nominal16 is even smaller. | LeRobot SmolVLA base/action expert. | Load, map schema, one-batch and tiny-overfit audit only. A meaningful result needs scaling or transfer across tasks, not “SmolVLA on RH56.” | E1--E2 / D1--D3 | Audit now; useful evaluation needs rollout | VERY SHORT audit / MEDIUM train | LOW--UNKNOWN | LOW--MEDIUM | MEDIUM--HIGH | MEDIUM | HIGH | MEDIUM--HIGH | MEDIUM--HIGH | NONE | MODERATE if scaling | Apache-2.0 code/base; verify model card | **AUDIT ONLY** |
| **13 / I9** | Chunk-consumer / execution distribution shift | **KEEP, HOLD.** It remains important but cannot explain static offline chunks and overlaps Thread A. | ACT temporal ensemble; receding horizon; async; RTC for flow policies. | Same checkpoint under controlled consumers with prediction-age/seam/effective-feedback metrics. Ours requires cross-policy closed-loop insight, not runtime tuning. | E1--E3 / D0--D2 | Replay only; valid rollout required for decision | SHORT replay / MEDIUM physical | MEDIUM | MEDIUM--HIGH | HIGH | MEDIUM--HIGH | MEDIUM | HIGH due control timing | HIGH | NONE | NONE | MIT/Apache-2.0 components | **HOLD — Thread A ownership** |
| **14 / I10** | True 3D / DP3-like representation | **KILL for current data.** No synchronized metric depth exists in maintained demonstrations; pseudo-depth is not reproduction. | DP3/iDP3/KALM concepts; future RGB-D. | Only a future matched RGB/RGB-D/XYZ study with new calibrated data. Our contribution would need spatial generalization under matched decoder/horizon. | E3--E4 / D4 | No | LONG | MEDIUM under OOD | HIGH stretch | HIGH | HIGH | MEDIUM | HIGH | HIGH | NONE | LARGE | Prefer DP3/iDP3 MIT; avoid NC-SA code reuse | **KILL NOW / future stretch** |

Three interactions are scientifically motivated, but each must be assembled only
after its components pass independent controls:

1. **Pretrained vision + event-balanced sampling.** Existing components:
   ImageNet/ACT and weighted sampling. Complementarity: one reduces visual
   representation burden while the other prevents rare event supervision from
   disappearing. Possible contribution: show these are orthogonal factors and
   identify which matters by demo count. Cheapest falsification: a matched
   `2×2` offline design after each single-factor run; do not start with the
   factorial.
2. **Short history + chunk-relative RH56 actions.** Existing components:
   recurrent/history BC and query-relative actions. Complementarity: history
   may identify *when* to transition, while relative targets make the required
   change explicit. Possible contribution: a causal decomposition of
   transition observability versus target-coordinate bias. Cheapest
   falsification: run the two main effects separately; add the combination only
   if both improve event recall without degrading non-event/arm error.
3. **Human nominal seed + automatic segment utility.** Existing components:
   S2I/DemInf-style selection. Complementarity: 16 human labels can supervise
   or calibrate a small scorer while mixed segments provide hard negatives.
   Possible contribution: an event-aware scorer that predicts held-out policy
   impact. Cheapest falsification: leave-source-out quality classification and
   rank correlation with retraining ablations; if duration/simple heuristics
   match it, stop.

## 6. TOP 5 DIRECTIONS

### 1. Rare critical-transition learning without deployment-time phase labels

This is the strongest current robot-learning direction. It addresses a concrete
physical failure—failure to initiate a task-changing action—through a general
learning question: how should chunked BC learn low-frequency events embedded in
long persistent phases? Start with event-aware measurement and sampling, then
use short history or relative targets only if the corresponding ambiguity is
observed. Force dependence: **NONE**.

### 2. Demonstration quality versus count in small physical imitation

The human audit creates a rare controlled resource: mixed-quality segments and
an operator-vetted nominal seed from the same collection process. The useful
paper question is whether automatic segment utility can recover the human
quality advantage or identify downstream-useful exceptions. Human curation by
itself is essential engineering, not sufficient novelty. This is the primarily
**data / imitation-learning** candidate. Force dependence: **NONE**.

### 3. Clean pretrained spatial perception under 16-demo supervision

Run scratch and ImageNet ACT on the identical nominal16 view before paying for
DINOv2. This is the lowest-cost test of whether policy failure is partly a
representation-learning burden. It becomes a paper direction only with a
sample-efficiency curve, spatial/object generalization, or a link to transition
recognition. Force dependence: **NONE**.

### 4. Targeted corrections versus more full demonstrations

IWR/Sirius and especially SDP support the proposition that correction data
naturally covers learner bottlenecks that long nominal trajectories
underrepresent. The experiment should compare equal operator time, frames and
failure-stage coverage after a valid rollout. It has high paper upside but is
not a current offline action. Force dependence: **NONE**.

### 5. Stronger-policy boundary: matched Diffusion Policy, then SmolVLA audit

Diffusion Policy is the nearest credible modern baseline in the pinned
ecosystem and plausibly helps only if futures are genuinely multimodal or its
two-frame/pretrained interface matters. Separate those factors before claiming
diffusion value. SmolVLA receives only a load/batch/overfit audit because its
own guidance predicts data starvation below roughly 50 episodes. This top-five
item explicitly tests whether a stronger pretrained/generalist policy is worth
further investment; it is not an architecture-novelty bet. Force dependence:
**NONE**.

The strongest current paper story is therefore **not primarily about force**.
Force remains an optional complementary signal owned by Thread B; the present
evidence more directly supports data quality and critical-event learning.

## 7. TOP 3 FAST FALSIFICATION EXPERIMENTS

These are “72-hour style” scopes, not wall-clock promises. They require no new
demonstrations and no robot motion. All use source-disjoint splits, identical
optimizer-step budgets, multiple fixed seeds, training-only normalization and
the transition metrics defined in Section 2.

### F1 — Quality/count causal ladder

**Controlled variable:** training-set quality, with a separate count control.

- Train the same scratch ACT configuration on Thread A's canonical nominal16
  and corrected mixed-quality view.
- Separately compare nominal16 against several source-stratified, matched-count
  16-segment subsets drawn from the mixed pool. Keep validation nominal and
  source-disjoint.
- Report aggregate/phase loss, event recall, chunk dynamic range, and variance
  across subsets/seeds.

**Decision:** if nominal16 consistently improves transition metrics despite
fewer frames, upgrade quality-aware selection. If matched-count mixed subsets
match nominal16, the manual quality narrative weakens. If every condition fails
similarly, prioritize representation/observability rather than adding a scorer.

**Cost:** E0--E1/D1; time to first falsification **VERY SHORT** once the
canonical view exists. This does not authorize Thread C to create that view.

### F2 — Uniform versus critical-event-balanced sampling

**Controlled variable:** sampling weights only.

- Propose grasp/release event windows from changes in the recorded RH56 target;
  freeze the threshold using training sources and manually spot-check it.
- Train frame-uniform and event-balanced samplers on the same nominal16 rows,
  initialization family, updates and seed set. Account for unique-frame
  exposure and do not simply duplicate a tiny window without reporting it.
- Preserve the ordinary non-event objective and report arm/non-event regression
  so a gain cannot come from forgetting the rest of the task.

**Decision:** if event recall and chunk dynamics improve without broad
degradation, critical-transition learning becomes the leading method track. If
the same frames remain unlearnable, test short history before more weighting.

**Cost:** E1--E2/D1; time **VERY SHORT--SHORT**.

### F3 — Scratch versus ImageNet on canonical nominal16

**Controlled variable:** ResNet-18 initialization/backbone learning rate only.

- Use the existing scratch and strong-pretrained ACT design but point both at
  the exact same nominal16 view and split. Do not compare the old dirty scratch
  checkpoint to a new clean pretrained one.
- Match head, chunk length, batch, update budget, transforms and evaluation.
- In addition to action metrics, train a frozen linear event probe on held-out
  spatial features or measure event-window image occlusion sensitivity.

**Decision:** a robust gain upgrades pretrained spatial representations and
justifies a DINOv2-S spatial-token adapter. No gain kills the expensive encoder
tour and moves short history/action coordinates forward.

**Cost:** E1/D1; time **SHORT**. The current val4 pretrained config is a useful
template, not the final controlled run.

The immediate next falsification after these three is one-frame versus
two-frame history with a shuffled-previous-frame control. Hand-only
chunk-relative targets follow if absolute-target persistence remains visible.

## 8. DIRECTIONS DOWNGRADED/KILLED

| Direction | Updated decision | Reason |
| --- | --- | --- |
| Dirty25 as an expert baseline | **KILL** | Human audit shows non-nominal behavior and missed episode boundaries. Use only as explicitly mixed-quality data. |
| Human curation as the paper contribution | **KILL alone** | Necessary baseline, but the result “bad demonstrations hurt” lacks an algorithmic/general insight. |
| Train a larger/VLA policy before fixing data controls | **DOWNGRADE** | Larger models see the same contradictory labels and rare-event frequency; they do not create missing supervision. |
| SmolVLA full fine-tuning now | **DOWNGRADE to audit** | Nominal16 is below the release's own roughly 50-episode guidance; task language has almost no variation. |
| π0 / π0-FAST / OpenVLA-OFT | **HOLD/KILL now** | Compute, embodiment statistics and adapters are disproportionate; no evidence a large language-conditioned model addresses the measured transition issue. |
| Full S2I | **DOWNGRADE to simple selection audit** | Trajectory optimization, action relabeling and policy-specific representations are E3--E4; human labels already permit cheaper controls. |
| Full GAP | **HOLD after modality audit** | Transition failure does not yet establish proprioceptive shortcut. Repository has no explicit reuse license. |
| DataMIL / SARM / PF-DAG | **KILL now** | Heavy data/compute or unestablished multimodality; no small-sample advantage over direct controls. |
| Sequential delta actions | **KILL as first action test** | Reconstruction accumulates error. Test query-time chunk-relative hand targets while keeping arm coordinates fixed. |
| Full AWE/HYDRA port | **HOLD after E1 compression audit** | Potentially strong long-horizon structure, but waypoint reconstruction must preserve critical hand events and a safe controller would be new work. |
| Complex multi-view attention/object foundation stack | **DOWNGRADE** | No matched evidence it beats simple concat/crops; new local evidence points to quality/events first. |
| Current-data DP3/RISE/KALM | **KILL** | Maintained demonstrations have no synchronized metric depth; pseudo-depth would not be faithful reproduction. |
| Monocular predicted depth called DP3 | **KILL** | It changes the input semantics and cannot recover missing metric geometry. |
| Chunk fusion/async as explanation for 0/56 | **KILL as cause; HOLD as later deployment study** | Static offline chunks lack a hidden closure command. Thread A owns controlled consumers. |
| Force-primary Thread-C narrative | **DOWNGRADE, not disproven** | Thread B owns force. Current Thread-C evidence supports a general data/transition problem that requires no force. |

## 9. POSSIBLE ICRA PAPER NARRATIVES

### Narrative A — Learning rare critical transitions in long-horizon imitation

**PROBLEM**
Frame-uniform action-chunk BC can achieve low loss by modeling persistent
actions while missing rare transitions that determine physical task success.

**ROBOT-LEARNING INSIGHT**
Long-horizon performance may be governed by both the exposure and causal
observability of a small set of action-changing events, so aggregate action
error is a poor learning target and evaluation measure.

**METHOD**
Detect candidate events from training-only action/state change points; measure
event-conditioned chunk behavior; introduce event-aware sampling/loss. Add the
smallest of short history, transition-aware modality regulation, or
chunk-relative hand targets only when a diagnostic identifies that mechanism.
No oracle phase label is required at deployment.

**WHAT IS BORROWED FROM PRIOR WORK**
ACT action chunking; Balanced BC's imbalance formulation; GAP's motion-transition
analysis; standard history and relative-action representations.

**WHAT COULD ACTUALLY BE OUR CONTRIBUTION**
A general formulation and metric suite for critical-transition underlearning,
a self-supervised event-aware objective, and causal evidence separating rare
exposure from temporal aliasing and target-coordinate persistence on physical
joint-space manipulation.

**MINIMUM EXPERIMENTS**
Clean nominal baseline; frame/episode/event-balanced controls; single/two-frame
history and shuffled-history control; absolute/chunk-relative hand target;
grasp and release (or contact-to-lift) events; physical success and event-stage
success; at least one second task/event family or a public offline benchmark;
one cheap spatial/object-position generalization axis.

**MAIN REVIEWER RISK**
The event rule may look hand-engineered or task-specific; one bottle task is
insufficient; oversampling could simply repeat labels; retrospective event
definitions could leak oracle information. Predeclare thresholds on training
sources and validate transfer across event types/tasks.

### Narrative B — When does demonstration quality beat quantity in small-data robot imitation?

**PROBLEM**
Small physical datasets mix nominal and stalled/colliding/incomplete behavior,
yet treating every recorded frame as expert supervision can be worse than using
fewer demonstrations.

**ROBOT-LEARNING INSIGHT**
Demonstration utility is segment- and phase-dependent: a short critical segment
may matter more than a long nominal-looking phase, so trajectory count and frame
count are poor data-value proxies.

**METHOD**
Use a tiny human nominal seed to calibrate simple segment quality/coverage/event
features; select or softly weight corrected mixed data; keep human-curated,
duration, random, coverage and existing S2I/DemInf-inspired baselines.

**WHAT IS BORROWED FROM PRIOR WORK**
S2I segment selection, DemInf information scoring, DataMIL's downstream-impact
view, and standard robust/weighted BC.

**WHAT COULD ACTUALLY BE OUR CONTRIBUTION**
A statistically lightweight event-aware utility estimator for tens—not
thousands—of physical demonstrations, plus evidence that it predicts policy
impact rather than merely matching subjective labels.

**MINIMUM EXPERIMENTS**
Corrected mixed and nominal views; source-disjoint quality labels; matched-count
random/coverage baselines; leave-source-out scorer evaluation; ACT and at least
one different policy head; physical success/stage results; sensitivity to the
number of clean reference segments; ideally a second task or public mixed-quality
dataset.

**MAIN REVIEWER RISK**
“Cleaning bad data helps” is obvious; 16 labels are too small; human quality is
subjective; scoring may be circular or simply detect episode duration; results
may not transfer beyond one operator/task.

### Narrative C — Spend human time at policy bottlenecks, not on more full demonstrations

**PROBLEM**
Full demonstrations devote most operator time and frames to already-solved
phases, while learner-induced critical failures remain out of distribution.

**ROBOT-LEARNING INSIGHT**
Data collection should optimize bottleneck-state coverage under a human-time
budget, not demonstration count.

**METHOD**
After a valid baseline, collect either full nominal demonstrations or short
targeted corrections under equal operator minutes; record policy attempts,
intervention boundaries and stage coverage; train a matched BC baseline with
balanced intervention sampling. Set-supervised diffusion is a later comparison,
not required for the first hypothesis.

**WHAT IS BORROWED FROM PRIOR WORK**
DAgger's on-policy aggregation, IWR balancing, Sirius intervention weighting,
and SDP's positive/negative action-chunk supervision.

**WHAT COULD ACTUALLY BE OUR CONTRIBUTION**
Event-directed correction allocation for rare phase transitions under
underactuated joint-space control, with a strict equal-time/equal-frame study
and failure-stage accounting.

**MINIMUM EXPERIMENTS**
Valid baseline and safe takeover protocol; predeclared failure taxonomy; equal
operator time for full demos/corrections; at least two collection rounds;
policy attempt plus correction logging; clean held-out initial conditions;
stage and full-task success; operator-time and frame efficiency; multiple seeds
or independently initialized policies.

**MAIN REVIEWER RISK**
Corrections start closer to success and are policy-specific; operator-time
accounting may be unfair; physical safety/takeover latency can confound data;
the method may be an IWR/SDP reproduction without a new allocation principle.

### Narrative D — Representation or policy capacity: what is actually worth pretraining at 16 demos?

**PROBLEM**
With very little physical data, poor results are often answered by a larger
encoder or VLA without isolating perception, temporal context and action-head
capacity.

**ROBOT-LEARNING INSIGHT**
The right pretrained interface may matter more than model scale: spatial visual
features can reduce representation burden, while generalist action priors may
fail under unseen joint/hand semantics.

**METHOD**
A matched ladder: scratch/ImageNet ACT, then DINOv2 spatial tokens if justified;
Diffusion Policy with matched history/pretraining controls; SmolVLA only as a
low-data transfer boundary. Evaluate nested demonstration counts and spatial or
object-instance shifts.

**WHAT IS BORROWED FROM PRIOR WORK**
ACT, Diffusion Policy, SmolVLA, ImageNet/DINOv2 and Theia's spatial-token
findings.

**WHAT COULD ACTUALLY BE OUR CONTRIBUTION**
A controlled account of which pretrained component transfers to small-data,
multi-view, joint-space underactuated manipulation, possibly an efficient
spatial-token adapter. A model bakeoff or encoder swap alone is not a
contribution.

**MINIMUM EXPERIMENTS**
Nested clean demonstration counts; matched optimizer/parameter controls;
scratch/frozen/partial/full adaptation; event-conditioned and full-task metrics;
held-out bottle/box position plus one unseen appearance/object axis; ACT and one
modern policy; memory/latency and multiple seeds.

**MAIN REVIEWER RISK**
Incremental benchmark on one robot/task; pretrained data or parameter count is
confounded; VLA language is unused; success differences may come from history,
horizon or normalization rather than pretraining.

Current ordering is A, B, C, then D. Narrative A has the best balance of a
specific physical problem and a reusable learning insight. Narrative C may
overtake it if a valid rollout reveals concentrated, repeatable failure states.

## 10. NEXT IMPLEMENTATION ACTIONS

1. **Wait for and consume Thread A's canonical nominal16 view.** Verify source
   IDs, segment provenance, split grouping and row counts read-only. Do not add
   a second splitter or edit raw/master data.
2. **E0 transition/quality audit.** Add one repository-owned offline analyzer
   only if it can consume the canonical manifest without duplicating it. It
   should report per-source duration, action-change points, event-window
   occupancy, phase/frame sampler mass, transition futures, and human-quality
   labels; no model training or raw writes.
3. **Run F1 and F2 with a small fixed seed/update budget.** Retain only durable
   code: extend the current sampler/data interface rather than introducing a
   parallel training stack. Treat all old mixed-data outputs as diagnostic.
4. **Run F3 using a nominal16 equivalent of the existing ImageNet config.** If
   ImageNet does not improve held-out event/full metrics, do not port DINOv2.
5. **Implement a two-frame derived observation adapter (E2)** only after the
   event audit. Include shuffled-history and current-frame duplication controls
   so history cannot win merely by leaking trajectory time or the previous
   action.
6. **Implement a hand-only chunk-relative target adapter (E2)** only if the
   persistence diagnostic remains. Store representation metadata and
   normalization with the checkpoint, reconstruct absolute RH56 targets at the
   existing policy boundary, and keep all safety/action legality unchanged.
7. **Audit Diffusion Policy after the simple main effects.** Match history,
   ImageNet initialization, horizon, action representation and execution before
   attributing a gain to generative modeling. SmolVLA stops after load,
   one-batch and tiny-overfit checks unless a clear transfer signal appears.
8. **Prepare—but do not execute—the equal-time correction protocol.** Define
   operator minutes, frames, intervention starts, rejected/accepted action
   chunks, reset exclusions and stage-wise outcomes. Physical collection waits
   for Thread A and explicit authorization.
9. **Do not start** DP3/RGB-D collection, full S2I/GAP/DataMIL/SARM/PF-DAG,
   complex cross-view fusion, a large VLA sweep, or any robot motion from this
   thread.

No large implementation is warranted until F1--F3 reorder—or fail to
reorder—the roadmap. A negative result is actionable: quality-insensitive
performance kills the scorer track; event-balanced failure redirects effort to
history/action observability; no ImageNet gain kills the encoder tour.
