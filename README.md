![Project cover](/pre_cover.jpg)
![Project cover](/COVER.png)



Think you can **get off the risk** in traffic—smooth brake, no drama? The problem is, many classic safety metrics mostly track **“how close”** or **“how soon,”** so two driving situations can look equally risky on paper while one is a calm, controlled slowdown and the other is a last-second, white-knuckle evasive move. That mismatch is the **identity paradox** in plain terms: the metric stays chill while the driving task turns nasty. 

**RID (Relative Interaction Domain)** is our attempt to make risk feel more like real driving: not just proximity, but whether the encounter is still **avoidable** given the **space you need** and the **time you have left** to react. So here’s the question—can you get **RID** off trouble on the road, before trouble gets you? 

## What this repo does
This repository contains a practical implementation of the **RID** framework from our paper *“Relative Interaction Domain (RID): A Real-Time Two-Dimensional Risk Quantification Model Considering Anticipated Spatial Demand.”* It turns vehicle trajectory data into interpretable, real-time-friendly risk signals by building a geometry-aware **“avoidance space”** between interacting road users and tracking how that space and urgency evolve over time. 

Concretely, the code helps you:
- Compute **ACG (Anticipated Critical Gap)**: the minimum closing-direction separation needed to keep braking-based avoidance feasible under the current state assumptions.
- Construct the **RID Avoidance Area**: a 2D interaction domain that adapts to encounter geometry and vehicle footprint/buffer, so **lateral offsets** and **approach angles** actually matter. 
- Evaluate **EDI (Evasive Difficulty Index)**: a difficulty-style score that rises when the required avoidance space is large **and** the remaining time is short—i.e., when avoiding a crash becomes genuinely hard. 
- Support both **offline** traffic safety analysis and **online-style** monitoring / onboard warning prototypes, with efficiency emphasized in the paper. 

If a “same score” metric can’t tell the difference between an easy brake and an emergency dodge, this repo is here to help you **get RID of that blind spot**. 

## A more formal add-on
Beyond the punchline, **RID** is built for practical traffic safety use: it aims to be more accurate in complex **2D encounters** while still being fast enough for **real-time monitoring** and **onboard warning**. Many classic indicators are proximity-driven and can struggle with geometry-rich interactions, limited scenario generalization, and the **identity paradox**, where similar metric values hide very different avoidability and risk escalation paths. RID addresses these issues with a geometry-grounded conflict representation (**Interaction → Friction → Critical Conflict**) and a severity metric (**EDI**) that cleanly separates **interaction pressure**, **encroachment**, and **imminent collision** states for clearer interpretation. 

RID is also **dynamics-aware**: its avoidance-space demand is conditioned on maneuver capability (e.g., **braking limits**), so the risk signal reflects whether a collision remains physically avoidable under the current motion state—not only whether two vehicles are close. In addition, two intuitive hyperparameters make the framework deployment-friendly and easy to tune: a **buffer width** (safety redundancy / risk tolerance) and a **critical deceleration (capability)** parameter (links the model to vehicle-road-user dynamics). Finally, the paper reports a highly efficient runtime (**0.165 ms per interacting pair** under a 32-thread setting), supporting large-scale pairwise evaluation in real time. 

## Dependencies

Tested with **Python 3.9+** (3.10/3.11 should also work).

Required libraries:
- **numpy**
- **pandas** (reads `.csv` and `.h5`; for `.h5` you may need `pytables` / `tables`)
- **shapely** (geometry / intersection / distance / affine transforms)
- **matplotlib** (optional, for debug plots)

You can install the core dependencies with:

pip install numpy pandas shapely matplotlib tables





## Input data format

RID is computed on **interaction pairs**: each row represents a pair *(i, j)* at a given `frame` / `time`, where agent **i** is treated as the reference (A/ego) and agent **j** as the counterpart (B/target). 

### Supported input files
- **HDF5**: `pd.read_hdf(path, key="data")` 
- **CSV**: `pd.read_csv(path)` (if you adapt the loader accordingly)
- We have provided a test file (RID-0602-128-217-DS2-df_result.csv) that can be used to test RID-main.py


### Indexing & metadata

| Column | Meaning (recommended unit) |
|---|---|
| `source_file` | Source filename / dataset identifier (optional, for traceability). |
| `frame_id` | Frame index (discrete timestep). |
| `time` | Timestamp (seconds, or your dataset’s time unit). |
| `track_id_i` | Track ID of agent *i* (vehicle A / reference). |
| `track_id_j` | Track ID of agent *j* (vehicle B / counterpart). |
| `pair_id` | Pair identifier (optional but recommended for grouping persistent (i, j) pairs). |
| `intersection_id` | Scenario / intersection ID (mainly used for labeling / plots). |

### Agent i (vehicle A): state & geometry

| Column | Meaning (recommended unit) |
|---|---|
| `x_i`, `y_i` | Global position of agent *i* (m). |
| `vx_i`, `vy_i` | Global velocity components of agent *i* (m/s). |
| `hx_i`, `hy_i` | Heading unit-vector components (optional; some datasets provide these). |
| `psi_kf_i` | Heading angle of agent *i* (radians). |
| `length_i`, `width_i` | Vehicle length / width of agent *i* (m), used to build the vehicle footprint (VAD). |
| `lon_acc_i`, `lat_acc_i` | Longitudinal / lateral acceleration (m/s²), used for dynamics-related computations (e.g., acceleration projection). |
| `agent_type_i` | Agent type (optional; e.g., car/truck). |

### Agent j (vehicle B): state & geometry

| Column | Meaning (recommended unit) |
|---|---|
| `x_j`, `y_j` | Global position of agent *j* (m). |
| `vx_j`, `vy_j` | Global velocity components of agent *j* (m/s). |
| `hx_j`, `hy_j` | Heading unit-vector components (optional). |
| `psi_kf_j` | Heading angle of agent *j* (radians). |
| `length_j`, `width_j` | Vehicle length / width of agent *j* (m). |
| `lon_acc_j`, `lat_acc_j` | Longitudinal / lateral acceleration (m/s²). |
| `agent_type_j` | Agent type (optional). |

### Relative quantities (optional if precomputed upstream)

The pipeline can also use relative quantities in a **relative coordinate system** centered at agent *i* (A), where axes are defined using the relative-velocity direction.

| Column | Meaning (recommended unit) |
|---|---|
| `x_rel`, `y_rel` | Relative position coordinates (m). |
| `v_rel` | Relative speed magnitude (m/s). |
| `v_rel_x`, `v_rel_y` | Relative velocity components (m/s). |
| `v_rel_heading` | Relative velocity heading (radians, optional). |
| `a_rel` | Relative-acceleration-related term (optional; definition depends on your pipeline). |
| `a_proj` | Acceleration **projection onto the relative-velocity direction** (used as a dynamics-related input in time-to-criticality computations). |

> Notes  
> - Recommended: standardize your dataset to the column names above, or add a `rename()` mapping at load time.  
> - If you do not provide the relative columns, compute them from the global states in preprocessing or inside the code.


## Outputs

For each interaction pair and timestamp, the pipeline computes (names as in the draft):
- **RIDI / REDI / RCDI**: risk/encroachment-style indicators derived from the RID/RCD domains. 
- **Tant**: anticipated time to the most critical configuration (used as the time resource). 
- **intrusion degree**: degree of intrusion into the RID-related domain. 
- **conflict level**: discrete state label (e.g., detachment / interaction escalation levels). 
- **R_RID_x, R_RID_y** (and RCD counterparts in some functions): RID shape/scale parameters used internally and helpful for debugging. 

### Saved files
- A result table (you can export the returned dataframe to `.csv`/`.h5`).
- Optional debug figures: the draft includes plotting utilities (relative-scene plots, velocity diagrams) and saves `.jpg` outputs under an `OutputData...` folder when enabled. 

## The paper related to this model is currently under review; once it is accepted, we will update the code and the cover accordingly.
​
