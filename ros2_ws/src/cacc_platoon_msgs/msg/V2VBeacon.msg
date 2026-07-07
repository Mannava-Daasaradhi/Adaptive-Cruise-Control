# One V2V broadcast from vehicle `sender` (DSRC/C-V2X style beacon).
# `accel` is the realized acceleration (CTHP feedforward), `u_cmd` the
# commanded acceleration (Ploeg-CACC feedforward). The channel node may
# scale these by a multiplicative noise factor and delay/drop the message.
builtin_interfaces/Time stamp
int32 sender
float64 t            # sender-local simulation time [s]
float64 accel        # a_{sender} [m/s^2] (noise applied by channel)
float64 u_cmd        # u_{sender} [m/s^2] (noise applied by channel)
