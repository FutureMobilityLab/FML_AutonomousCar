import numpy as np
from acados_template import AcadosOcpSolver
from ros2_car_control.utilities import get_accel_dist, get_closest_waypoint, get_speed_cmd


class MPCController:
    def __init__(self, waypoints, ctrl_params):
        self.Tf = ctrl_params.get("tf")  # prediction horizon
        self.N = ctrl_params.get("N")  # number of discretization steps
        self.velocity_setpoint = ctrl_params.get("speed_setpoint")  # Velocity Setpoint
        self.max_accel = ctrl_params.get("max_accel")
        self.d_accel = get_accel_dist(self.velocity_setpoint, self.max_accel)
        self.d_decel = self.waypoints.d[-1] - self.d_accel

        self.waypoints = waypoints
        self.xrefs = np.array(waypoints.x)
        self.yrefs = np.array(waypoints.y)
        self.psirefs = np.array(waypoints.psi)
        self.pose_x = self.xrefs[0]
        self.pose_y = self.yrefs[0]
        self.pose_psi = self.psirefs[0]
        # Get Total Path Distances
        sumDist = 0
        srefs = np.array([])
        for i in range(len(self.xrefs)):
            srefs = np.append(srefs, sumDist)
            try:
                sumDist = sumDist + np.sqrt(
                    (self.xrefs[i + 1] - self.xrefs[i]) ** 2
                    + (self.yrefs[i + 1] - self.yrefs[i]) ** 2
                )
            except IndexError:
                continue
        self.srefs = srefs

        self.acados_solver = AcadosOcpSolver(
            None, generate=False, build=False, json_file="acados_ocp.json"
        )

        # Dimensions
        self.nx = ctrl_params.get("nx")  # model.x.size()[0]
        self.nu = ctrl_params.get("nu")  # model.u.size()[0]
        self.ny = self.nx + self.nu

        self.Nsim = int(self.srefs[-1] * self.N / (self.velocity_setpoint * self.Tf))
        self.ds = self.velocity_setpoint * self.Tf / self.N

    def get_commands(self, pose_x, pose_y, pose_psi, v):
        self.pose_x = pose_x
        self.pose_y = pose_y
        self.pose_psi = pose_psi
        # Get Nearest Reference Point
        x0 = np.array([self.pose_x, self.pose_y, 0, 0, 0, self.pose_psi, 0])
        self.acados_solver.set(0, "lbx", x0)
        self.acados_solver.set(0, "ubx", x0)
        normArray = np.sqrt((self.xrefs - x0[0]) ** 2 + (self.yrefs - x0[1]) ** 2)
        start_ref = np.argmin(normArray)
        current_sref = self.srefs[start_ref]

        # Set OCP References
        for j in range(self.N):
            try:
                relative_s_dist = current_sref - self.srefs
                refs0_idx = len(relative_s_dist[relative_s_dist > 0])
                sratio = (current_sref - self.srefs[refs0_idx]) / (
                    self.srefs[refs0_idx + 1] - self.srefs[refs0_idx]
                )
                xrefs_interpolated = (
                    sratio * (self.xrefs[refs0_idx + 1] - self.xrefs[refs0_idx])
                    + self.xrefs[refs0_idx]
                )
                yrefs_interpolated = (
                    sratio * (self.yrefs[refs0_idx + 1] - self.yrefs[refs0_idx])
                    + self.yrefs[refs0_idx]
                )
                psirefs_interpolated = self.psirefs[refs0_idx]
                yref = np.array(
                    [
                        xrefs_interpolated,
                        yrefs_interpolated,
                        0,
                        0,
                        0,
                        psirefs_interpolated,
                        0,
                        0,
                    ]
                )
            except IndexError:
                yref = np.array(
                    [self.xrefs[-1], self.yrefs[-1], 0, 0, 0, self.psirefs[-1], 0, 0]
                )

            self.acados_solver.set(j, "yref", yref)
            current_sref += self.ds
        try:
            yref_N = yref[:-1]
        except IndexError:
            yref_N = np.array(
                [self.xrefs[-1], self.yrefs[-1], 0, 0, 0, self.psirefs[-1], 0]
            )
        self.acados_solver.set(self.N, "yref", yref_N)

        # Solve the OCP Problem and Fetch First Input
        status = self.acados_solver.solve()
        if status != 0:
            self.acados_solver.reset()

        steering_angle = float(self.acados_solver.get(0, "u"))

        waypoints = np.hstack(
            (
                self.waypoints.x[np.newaxis].T,
                self.waypoints.y[np.newaxis].T,
                self.waypoints.psi[np.newaxis].T,
            )
        )
        pose = np.array([[pose_x, pose_y]])
        _, closest_i = get_closest_waypoint(pose, waypoints)
        # Ramp up or down velocity based on distance traveled.
        d = self.waypoints.d[closest_i]
        speed_cmd = get_speed_cmd(
            d, self.d_accel, self.d_decel, self.max_accel, self.velocity_setpoint
        )

        return steering_angle, speed_cmd, self.xrefs[start_ref], self.yrefs[start_ref]
