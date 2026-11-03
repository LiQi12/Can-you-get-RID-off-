![Project cover](pre_cover.jpg)
![Project overview](COVER.png)

# Can You Get RID of Risk?

Think you can **get RID of risk** in traffic? A gentle tap on the brake, a little extra room, and you are out of trouble. Smooth braking, no drama. Then traffic changes the script.

Many classic safety metrics keep an eye on **“how close”** or **“how soon.”** Yet two encounters can earn the same score while demanding very different responses: one is a calm, controlled slowdown; the other is a last-second, white-knuckle scramble for space. That is the **identity paradox** in everyday terms: **the metric stays chill while the driving task turns nasty.** Same score. Very different story behind the wheel.

**RID (Relative Interaction Domain)** brings the driving task into the picture: the **space an evasive manoeuvre needs** and the **time the encounter leaves**. By combining vehicle geometry, relative motion, and braking-based spatial demand, it gives those deceptively similar encounters a more revealing description.

So, can you **get RID of trouble before trouble gets you?** That is the question behind this project.

## About the paper

This repository provides the implementation accompanying our paper, published in **Transportation Research Part C: Emerging Technologies (TRC)**:

> **Relative interaction Domain (RID): a real-time two--dimensional risk quantification model considering anticipated spatial demand**

The standalone implementation constructs vehicle interaction domains from physical footprints, relative motion, a safety buffer, and braking-based spatial demand. It combines their interaction areas with the anticipated event time, **TANT**, to calculate **RCDI, REDI, and RIDI**.

## What the implementation provides

- **Two-dimensional vehicle geometry.** Oriented rectangular bodies represent vehicle dimensions and headings; relative velocity determines the direction of motion.
- **Braking-based spatial demand.** A configurable deceleration magnitude determines the braking-distance contribution to the domain's longitudinal extent.
- **Anticipated events.** Continuous geometry identifies the first predicted physical contact or the earliest attainment of the minimum future body clearance.
- **Three interaction intensities.** RCDI, REDI, and RIDI describe distinct combinations of physical and buffered interaction areas, normalized by TANT.
- **A standalone Python interface.** Use a single state pair, a pandas DataFrame, or a CSV file. CSV processing supports multiple worker processes.

The implementation is contained in [`rid_model.py`](rid_model.py), with English documentation and usage examples.

## Outputs

Each input row represents one ordered vehicle pair at one synchronized time. Vehicle **A / i** is the reference agent; vehicle **B / j** is the counterpart.

| Output | Meaning | Unit |
|---|---|---|
| `RCDI` | Cropped RCD domain intersected with B's physical projection band, divided by the regularized TANT | m²/s |
| `REDI` | Cropped RID domain intersected with B's physical projection band, divided by the regularized TANT | m²/s |
| `RIDI` | Cropped RID domain intersected with B's safety-buffered projection band, divided by the regularized TANT | m²/s |
| `T_ant` | Anticipated time to the selected collision or qualifying close-pass event | s |
| `rcd_area_m2`, `red_area_m2`, `rid_area_m2` | The three interaction areas before time normalization | m² |
| `current_clearance_m` | Current minimum distance between the physical vehicle bodies | m |
| `minimum_future_clearance_m` | Minimum predicted physical body clearance under the motion model | m |
| `event_distance_m` | Relative translation required to reach the selected event geometry | m |
| `event_status` | Event classification, such as `future_collision`, `future_close_pass`, or `no_future_approach` | — |

Additional diagnostics include the domain radii, relative speed, projected relative acceleration, and current-contact flag. Full field definitions are available in `RIDResult` and the module documentation.

The indicators have the geometric ordering **RCDI ≤ REDI ≤ RIDI**, up to numerical roundoff. They are directed quantities: swapping A and B can change the domain geometry and the resulting areas. Their values describe interaction intensity; they are not calibrated collision probabilities.

## Installation and quick start

The module supports **Python 3.8 or later** and requires NumPy and Shapely:

```bash
python -m pip install numpy shapely
```

Download `rid_model.py`, or clone this repository:

```bash
git clone https://github.com/LiQi12/Can-you-get-RID-off-.git
cd Can-you-get-RID-off-
python rid_model.py --demo
```

The demonstration uses a 4 m × 2 m vehicle travelling at 5 m/s toward an equally sized stationary vehicle whose centre is 20 m ahead. The expected output is approximately:

```text
RCDI: 4.654519
REDI: 4.677758
RIDI: 9.309038
T_ant: 3.2
```

### Python API

```python
from rid_model import AgentState, RIDConfig, compute_rid_metrics

ego = AgentState(
    x=0, y=0, vx=5, vy=0,
    heading=0, length=4, width=2,
)
other = AgentState(
    x=20, y=0, vx=0, vy=0,
    heading=0, length=4, width=2,
)

config = RIDConfig(Ds=1.0, acr=1.67)
result = compute_rid_metrics(ego, other, config)

print(result.rcdi, result.redi, result.ridi, result.tant_s)
print(result.to_dict())
```

Named-field dictionaries are also accepted by `compute_rid_metrics`. The `tant_s` attribute is exported as `T_ant` by `to_dict()` and the CSV interface.

## Default parameters

| Parameter | Default | Role |
|---|---:|---|
| `Ds` | **1.0 m** | Outward safety-buffer distance, additional RID half-width, and upper clearance threshold for a non-collision event |
| `acr` | **1.67 m/s²** | Positive braking-deceleration magnitude used in `v² / (2 × acr)` for both vehicles |
| `domain_exponent` | 20 | Superellipse shape parameter |
| `anticipation_dt_s` | 0.2 s | Interval used in the local acceleration correction to relative speed |
| `superellipse_points` | 100 | Number of vertices used to approximate each superellipse |
| `time_epsilon_s` | 0.000001 s | Minimum denominator used when normalizing areas by TANT |

**Ds is measured outward from each side of the physical footprint.** A 1 m buffer therefore adds 2 m to the total transverse width. Physical collision detection uses the unbuffered bodies.

For a non-collision path, a future event is eligible only when its minimum body clearance satisfies:

```text
0 < minimum_future_clearance < Ds
```

A minimum exactly equal to Ds is excluded. If no qualifying future approach exists, `T_ant` is infinite and all three intensities are zero. A collision path remains eligible independently of this non-collision clearance condition.

The public parameter `acr` corresponds to the effective braking parameter named `a_rid` in the earlier research scripts.

## Paired trajectory data

Each row must contain two vehicle states from the **same recording, location, and timestamp**. The module evaluates existing pairs; trajectory matching and synchronization take place before calculation. Keep recording and intersection identifiers when assembling data from multiple locations.

The following fields are required for **both** suffixes, `_i` and `_j`:

| Fields | Description | Unit |
|---|---|---|
| `x`, `y` | Global coordinates of the physical body centre | m |
| `vx`, `vy` | Global velocity components | m/s |
| `heading` | Body heading, counterclockwise from global +x | rad |
| `length`, `width` | Positive physical body dimensions | m |

For example, use `x_i` and `x_j` for the two x coordinates. The original `psi_kf_i` and `psi_kf_j` heading columns are also supported.

Optional acceleration fields are:

| Fields | Convention |
|---|---|
| `ax`, `ay` | Global acceleration components; provide both together. These take precedence when supplied. |
| `lon_acc`, `lat_acc` | Body-frame acceleration, with longitudinal acceleration positive forward and lateral acceleration positive **right**. |

All accelerations use m/s². Missing accelerations default to zero. If a dataset uses left-positive lateral acceleration, convert its sign or provide global `ax` and `ay` directly.

Metadata such as `source_file`, `frame_id`, `time`, `track_id_i`, `track_id_j`, `pair_id`, and `intersection_id` is preserved in the CSV and DataFrame outputs. Precomputed relative-coordinate fields are not required.

### CSV processing

A minimal `pairs.csv` can contain:

```csv
frame_id,intersection_id,track_id_i,track_id_j,x_i,y_i,vx_i,vy_i,heading_i,length_i,width_i,x_j,y_j,vx_j,vy_j,heading_j,length_j,width_j
0,example,1,2,0,0,5,0,0,4,2,20,0,0,0,0,4,2
```

Calculate the metrics using four worker processes:

```bash
python rid_model.py --input pairs.csv --output metrics.csv --workers 4
```

Parameters can be set explicitly:

```bash
python rid_model.py --input pairs.csv --output metrics_custom.csv --ds 1.0 --acr 1.67
```

Choose a new output filename for each run. The CLI refuses to overwrite an existing file or append duplicate result columns to an input that already contains them.

### DataFrame processing

Install pandas if you want to use the DataFrame interface:

```bash
python -m pip install pandas
```

```python
import pandas as pd
from rid_model import RIDConfig, compute_rid_dataframe

pairs = pd.read_csv("pairs.csv", float_precision="round_trip")
metrics = compute_rid_dataframe(pairs, RIDConfig(Ds=1.0, acr=1.67))
print(metrics[["RCDI", "REDI", "RIDI", "T_ant"]])
```

The output retains the input index and identity columns. For an individual paired row, use `compute_rid_from_row(row, config)`.



## License

This repository is distributed under the [MIT License](LICENSE).
