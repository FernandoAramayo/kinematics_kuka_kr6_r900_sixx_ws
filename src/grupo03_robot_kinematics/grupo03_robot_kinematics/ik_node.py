import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import numpy as np

def dh(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d],
        [0,   0,        0,       1],
    ], dtype=float)

def get_fk_matrices(q):
    T_base = np.diag([1.0, -1.0, -1.0, 1.0])
    A1 = dh(q[0],            -0.400,  0.025,  np.pi / 2)
    A2 = dh(q[1],             0.0,    0.455,  0.0)
    A3 = dh(q[2] - np.pi/2,   0.0,    0.035,  np.pi / 2)
    A4 = dh(q[3],            -0.420,  0.0,   -np.pi / 2)
    A5 = dh(q[4],             0.0,    0.0,    np.pi / 2)
    A6 = dh(q[5],            -0.080,  0.0,    0.0)
    T_tool = np.diag([-1.0, 1.0, -1.0, 1.0])
    return [T_base, A1, A2, A3, A4, A5, A6, T_tool]

def fk(q):
    m = get_fk_matrices(q)
    return m[0] @ m[1] @ m[2] @ m[3] @ m[4] @ m[5] @ m[6] @ m[7]

def jacobian_geometrico(q):
    m = get_fk_matrices(q)
    T = m[0]
    origins = []
    axes = []

    for i in range(1, 7):
        origins.append(T[:3, 3].copy())
        axes.append(T[:3, 2].copy())
        T = T @ m[i]
        
    T = T @ m[7]
    p_n = T[:3, 3]

    J = np.zeros((6, 6))
    for i in range(6):
        z_i = axes[i]
        p_i = origins[i]
        J[:3, i] = np.cross(z_i, p_n - p_i)
        J[3:, i] = z_i
        
    return J

Q_MIN = np.deg2rad([-170, -190, -120, -185, -120, -350])
Q_MAX = np.deg2rad([170, 45, 156, 185, 120, 350])

def aplicar_limites(q):
    return np.clip(q, Q_MIN, Q_MAX)

def ik_position(p_des, q0):
    q = np.array(q0, dtype=float)

    alfa = 0.5          
    tol_pos = 1e-3       # 1 mm de tolerancia
    max_iterations = 200
    max_step = 0.1

    for k in range(max_iterations):
        T_curr = fk(q)
        p_curr = T_curr[:3, 3]

        e_pos = p_des - p_curr

        if np.linalg.norm(e_pos) < tol_pos:
            return q, True, k, np.linalg.norm(e_pos)

        # Extraer posición de Jacobiano geométrico completo
        J = jacobian_geometrico(q)
        J_v = J[:3, :] 

        M = J_v @ J_v.T
        J_plus = J_v.T @ np.linalg.inv(M)
        dq = J_plus @ e_pos
 
        norm_dq = np.linalg.norm(dq)
        if norm_dq > max_step:
            dq = dq * (max_step / norm_dq)
        
        q = q + alfa * dq
        q = aplicar_limites(q)

    return q, False, max_iterations, np.linalg.norm(p_des - fk(q)[:3, 3])


class IKNode(Node):
    def __init__(self):
        super().__init__("ik_iterative_node")
        self.q = [0.0, -np.pi/2, np.pi/2, 0.0, 0.0, 0.0]
        self.have_solution = True

        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        
        self.subscription = self.create_subscription(
            Point, "/target", self.target_callback, 10
        )
        
        self.timer = self.create_timer(0.1, self.publish_solution)
        self.get_logger().info("Nodo IK activo. Esperando /target (Point)...")

    def target_callback(self, msg):
        p_des = np.array([msg.x, msg.y, msg.z], dtype=float)

        q_sol, success, iterations, error = ik_position(p_des, self.q)

        if success:
            self.q = q_sol.tolist()
            self.have_solution = True
            self.get_logger().info(
                f"Convergencia en {iterations} iteraciones. "
                f"Ángulos: {[round(ang, 4) for ang in self.q]}"
            )
        else:
            self.get_logger().warning(
                f"Fallo de convergencia tras {iterations} iteraciones. "
                f"Error={error:.6f} m. Reseteando semilla a 'home'."
            )
            self.q = [0.0, -np.pi/2, np.pi/2, 0.0, 0.0, 0.0]
            
    def publish_solution(self):
        if not self.have_solution:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f"joint_{i}" for i in range(1, 7)]
        msg.position = self.q
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()