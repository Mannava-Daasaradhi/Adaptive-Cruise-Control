# R-10 — Messages, topics, and QoS (`cacc_platoon_msgs`)

## Messages (ament_cmake package)

`VehicleState.msg`:

    builtin_interfaces/Time stamp   # wall time at which this state is current
    int32 index                     # 0 = leader
    float64 t                       # sender's sim time (wall-elapsed)
    float64 position                # front bumper [m]
    float64 velocity                # [m/s]
    float64 acceleration            # realized a [m/s^2]
    float64 u_cmd                   # commanded (saturated) input
    float64 spacing_error           # measured e (0 for the leader)
    # Workstream A: float64 headway, float64 rho_hat

`V2VBeacon.msg`:

    builtin_interfaces/Time stamp
    int32 sender
    float64 t
    float64 accel                   # realized acceleration (CTHP payload)
    float64 u_cmd                   # commanded input (Ploeg-CACC payload)

The channel scales `accel`/`u_cmd` by the per-beacon noise factor w.

## Topic graph

| topic | type | producer → consumer |
|---|---|---|
| `/platoon/v{i}/state` | VehicleState | vehicle i → vehicle i+1 (radar surrogate, **no channel**), recorder, viz |
| `/platoon/v{i}/beacon` | V2VBeacon | vehicle i → channel (raw V2V) |
| `/platoon/v{i}/v2v` | V2VBeacon | channel → vehicle i (impaired V2V) |
| `/platoon/markers` | MarkerArray | viz → rviz2 |
| `/tf` | TFMessage | viz (`map → platoon` at leader) |

The **radar/V2V split is the modeling point**: radar is onboard (direct
topic, fresh, extrapolated by stamp age), V2V goes through the channel
node (delay/loss/noise). Never route the state topics through the channel.

## QoS profiles

All app topics: default RELIABLE, KEEP_LAST(10), VOLATILE. Consequences
measured during development: volatile late-joiners miss pre-match samples
and the first delivered message can be 10–100 ms stale (DDS discovery) —
this is exactly what the D-013 anchor-hold + extrapolation absorb. Do not
"fix" by switching to TRANSIENT_LOCAL (stale replay is worse).

## Adding a field (the A1 recipe)

1. Edit the `.msg`; rebuild both packages (`ros2_sync_build.sh`).
2. Producers fill it; `recorder_node` appends the CSV column.
3. Windows analysis reads columns by *name* (`np.genfromtxt(names=True)`)
   — old CSVs stay readable; never index columns by position.
4. Add the field to the R-10 table above and note it in the experiment
   doc that consumes it.
