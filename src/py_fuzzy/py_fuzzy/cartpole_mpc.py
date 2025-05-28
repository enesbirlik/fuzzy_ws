#!/usr/bin/env python3
# lqr_controller.py
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class CartPoleLQR(Node):
    """
    Minimal LQR controller for a real cart-pole driven through
    a single effort interface (/effort_controllers/commands).
    State vector: [cart_pos, cart_vel, pole_angle, pole_ang_vel]
    Control law : u = -K_LQR @ state
    """

    # ---- REPLACE THESE WITH YOUR OWN GAINS ----
    K_LQR = [  -20.5,   -41.8,  1521.0,  273.4 ]      # shape (1×4)

    def __init__(self):
        super().__init__('cartpole_lqr')

        # ---- subscribers / publishers ----
        self.sub = self.create_subscription(
            JointState, '/joint_states', self.cb_joint_states, 20)
        self.pub = self.create_publisher(
            Float64MultiArray, '/effort_controllers/commands', 10)

        # ---- state ----
        self.x_cart      = None     # position (m)
        self.xdot_cart   = None     # velocity (m/s)
        self.theta_pole  = None     # angle   (rad, 0 = upright)
        self.thetadot    = None     # angular vel (rad/s)

        # crude finite-difference buffer for velocity estimation
        self._prev_pos   = None
        self._prev_th    = None
        self._prev_time  = None

        # 200 Hz timer
        self.ctrl_dt = 0.005
        self.timer   = self.create_timer(self.ctrl_dt, self.control_loop)

        self.get_logger().info('LQR controller up')

    # ---------- callbacks ----------
    def cb_joint_states(self, msg: JointState):
        """Grab positions, estimate velocities if topic carries none."""
        try:
            idx_cart = msg.name.index('slider_to_cart')
            idx_pole = msg.name.index('cart_to_pole')
        except ValueError:
            self.get_logger().warn('JointState missing cart or pole joint'); return

        t_now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 \
                if msg.header.stamp.sec or msg.header.stamp.nanosec else \
                self.get_clock().now().nanoseconds * 1e-9

        # positions
        self.x_cart     = msg.position[idx_cart]
        self.theta_pole = msg.position[idx_pole]           # 0 rad = upright

        # velocities – use provided if available, else finite diff
        if msg.velocity and len(msg.velocity) == len(msg.position):
            self.xdot_cart = msg.velocity[idx_cart]
            self.thetadot  = msg.velocity[idx_pole]
        else:
            if self._prev_time is not None:
                dt = t_now - self._prev_time
                if dt > 0.0:
                    self.xdot_cart = (self.x_cart  - self._prev_pos) / dt
                    self.thetadot  = (self.theta_pole - self._prev_th) / dt
            self._prev_pos  = self.x_cart
            self._prev_th   = self.theta_pole
            self._prev_time = t_now

    # ---------- main control loop ----------
    def control_loop(self):
        if None in (self.x_cart, self.xdot_cart, self.theta_pole, self.thetadot):
            return  # wait until we have all four states

        # build state vector
        x = [
            self.x_cart,
            self.xdot_cart,
            self.theta_pole,
            self.thetadot
        ]

        # LQR control
        u = -(CartPoleLQR.K_LQR[0] * x[0]
              + CartPoleLQR.K_LQR[1] * x[1]
              + CartPoleLQR.K_LQR[2] * x[2]
              + CartPoleLQR.K_LQR[3] * x[3])

        # OPTIONAL: saturate if you know motor limits
        u = max(min(u,  25.0), -25.0)

        # publish
        cmd = Float64MultiArray()
        cmd.data = [u]
        self.pub.publish(cmd)

    # ---------- util ----------
    @staticmethod
    def lqr_gain_note():
        """
        Quick reminder on how to compute K with python-control:

        from control import lqr, ctrb
        import numpy as np
        # fill in your A, B matrices (4×4, 4×1)
        Q = np.diag([10, 1, 100, 1])   # tune to taste
        R = np.array([[0.01]])
        K, _, _ = lqr(A, B, Q, R)
        print(-K)   # use negative sign because python-control returns K
        """

def main(args=None):
    rclpy.init(args=args)
    node = CartPoleLQR()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()