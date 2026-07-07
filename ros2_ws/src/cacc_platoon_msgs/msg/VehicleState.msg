# Longitudinal state of one platoon member, stamped with its local sim time.
builtin_interfaces/Time stamp
int32 index          # 0 = leader, 1..N = followers
float64 t            # node-local simulation time [s]
float64 position     # front-bumper position p_i [m]
float64 velocity     # v_i [m/s]
float64 acceleration # realized a_i [m/s^2]
float64 u_cmd        # commanded (saturated) u_i [m/s^2]
float64 spacing_error  # e_i [m] (0 for the leader)
