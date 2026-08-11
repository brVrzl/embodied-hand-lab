# ACT normalization and action semantics

Result: **PASS — action semantic integrity**

Statistics below were computed only from the 3,268 rows in source episodes 65, 66, 67, 69, and 70. They are population statistics, matching the dataset-level convention used for the validation. No force channel participates in standard ACT.

## Native ordering and statistics

Dimensions 1-6 are JAKA measured joints (state) or accepted absolute/native targets (action). Dimensions 7-12 are RH56 measured normalized closure/position values (state) or accepted absolute/native targets (action), ordered index, middle, ring, pinky, thumb-close, thumb-lateral.

| Dim | Semantic | State mean | State std | State min | State max | Action mean | Action std | Action min | Action max |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | JAKA 1 | 1.063208 | 0.241551 | 0.597513 | 1.492117 | 1.059994 | 0.245224 | 0.557077 | 1.504913 |
| 2 | JAKA 2 | -0.996518 | 0.155674 | -1.351932 | -0.578838 | -0.996224 | 0.165664 | -1.376594 | -0.578616 |
| 3 | JAKA 3 | -1.611598 | 0.347122 | -2.151886 | -0.927433 | -1.610035 | 0.355574 | -2.154594 | -0.848742 |
| 4 | JAKA 4 | -3.802186 | 0.178585 | -4.177620 | -3.439136 | -3.799181 | 0.181177 | -4.188760 | -3.417762 |
| 5 | JAKA 5 | -1.207751 | 0.227454 | -1.590414 | -0.773024 | -1.204417 | 0.229400 | -1.596522 | -0.763582 |
| 6 | JAKA 6 | 4.907347 | 0.206042 | 4.465024 | 5.478502 | 4.908896 | 0.209636 | 4.423104 | 5.524969 |
| 7 | RH56 index | 0.150914 | 0.175351 | 0.000000 | 0.407000 | 0.152209 | 0.176008 | 0.000000 | 0.409860 |
| 8 | RH56 middle | 0.163589 | 0.192738 | 0.000000 | 0.448000 | 0.161622 | 0.195616 | 0.000000 | 0.447497 |
| 9 | RH56 ring | 0.191776 | 0.192304 | 0.000000 | 0.485000 | 0.191040 | 0.193461 | 0.000000 | 0.486776 |
| 10 | RH56 pinky | 0.146247 | 0.147566 | 0.002000 | 0.401000 | 0.145161 | 0.148903 | 0.000000 | 0.402500 |
| 11 | RH56 thumb-close | 0.127505 | 0.081621 | 0.004000 | 0.308000 | 0.126850 | 0.084162 | 0.000000 | 0.312500 |
| 12 | RH56 thumb-lateral | 0.735368 | 0.140941 | 0.439000 | 0.970000 | 0.719795 | 0.140794 | 0.423204 | 0.944051 |

No dimension is frozen or near-zero variance. The JAKA and RH56 ranges are on visibly different scales, as expected from arm joint units versus normalized hand values; independent per-dimension normalization prevents one group from dominating solely because of that difference. The similarity of state and target ranges is also consistent with native absolute targets, not evidence for deltas.

## Exact transform path

The selected ACT policy declares `MEAN_STD` normalization independently for `STATE` and `ACTION`. LeRobot's serialized preprocessor applies, per dimension:

`normalized = (native - mean) / (std + epsilon)`

The serialized postprocessor reverses it:

`native = normalized * (std + epsilon) + mean`

There is no channel permutation, subtraction of the current state, integration, or conversion to a delta at either point. The processor state is stored in the checkpoint rather than held only in training memory.

An action from source episode 65, source frame 100 was traced through the actual processor:

```text
raw native absolute action
[ 0.78795558, -0.84502900, -1.75154030, -3.85953116,
 -1.17316985,  4.70137691, -0.00000000,  0.00600000,
  0.01100000,  0.02600000,  0.14700000,  0.66200000]

normalized ACT target
[-1.10934043,  0.91267896, -0.39796454, -0.33309829,
  0.13620198, -0.98913962, -0.86477649, -0.79554701,
 -0.93062878, -0.80026037,  0.23941809, -0.41051212]

inverse-transformed native action
[ 0.78795564, -0.84502900, -1.75154030, -3.85953116,
 -1.17316985,  4.70137691,  0.00000001,  0.00600001,
  0.01100002,  0.02600002,  0.14700000,  0.66200000]
```

Maximum absolute reconstruction error was `5.960464477539063e-08`, within float32 precision. The final native vector remains exactly JAKA 1-6 followed by RH56 index/middle/ring/pinky/thumb-close/thumb-lateral and retains absolute/native target semantics expected by the existing command boundary.

No delta mode was enabled or implemented. This confirms a reversible model-layer transform while leaving the master dataset unchanged.
