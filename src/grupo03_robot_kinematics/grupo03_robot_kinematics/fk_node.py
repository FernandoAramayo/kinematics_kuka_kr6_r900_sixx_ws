import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
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


def quaternion(R):
    # Matriz de rotación 3x3 a cuaternion [x, y, z, w]
    q = np.zeros(4)
    tr = np.trace(R)

    if tr > 0:
        s = 2 * np.sqrt(tr + 1)
        q[3] = s / 4
        q[0] = (R[2, 1] - R[1, 2]) / s
        q[1] = (R[0, 2] - R[2, 0]) / s
        q[2] = (R[1, 0] - R[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2 * np.sqrt(1 + R[i, i] - R[j, j] - R[k, k])
        q[i] = s / 4
        q[j] = (R[j, i] + R[i, j]) / s
        q[k] = (R[k, i] + R[i, k]) / s
        q[3] = (R[k, j] - R[j, k]) / s

    return q / np.linalg.norm(q)


class JointSubscriber(Node):
    def __init__(self):
        super().__init__('fk_node_numpy')
        
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.sub_callback,
            10
        )
        
        self.target_sub = self.create_subscription(
            Point,
            '/target',
            self.target_callback,
            10
        )
        
        self.p_d = None 
        
        self.get_logger().info("Nodo FK iniciado. Escuchando /joint_states y /target...")

    def target_callback(self, msg):
        # Guardamos el objetivo pd cuando llega desde la terminal
        self.p_d = np.array([msg.x, msg.y, msg.z], dtype=float)

    def sub_callback(self, msg):
        # Evitar errores si el mensaje no tiene todas las articulaciones
        if len(msg.position) < 6:
            return

        q1 = msg.position[0]
        q2 = msg.position[1]
        q3 = msg.position[2]
        q4 = msg.position[3]
        q5 = msg.position[4]
        q6 = msg.position[5]

        # Matrices DH
        T_base = np.diag([1.0, -1.0, -1.0, 1.0])

        A1 = dh(q1,            -0.400,  0.025,  np.pi / 2)
        A2 = dh(q2,             0.0,    0.455,  0.0)
        A3 = dh(q3 - np.pi/2,   0.0,    0.035,  np.pi / 2)
        A4 = dh(q4,            -0.420,  0.0,   -np.pi / 2)
        A5 = dh(q5,             0.0,    0.0,    np.pi / 2)
        A6 = dh(q6,            -0.080,  0.0,    0.0)

        T_tool = np.diag([-1.0, 1.0, -1.0, 1.0])

        T = T_base @ A1 @ A2 @ A3 @ A4 @ A5 @ A6 @ T_tool

        # Extraer posiciones X, Y, Z
        x = T[0, 3]
        y = T[1, 3]
        z = T[2, 3]

        # Extraer matriz de rotación 3x3 y convertir a cuaterniones
        R = T[:3, :3]
        quat = quaternion(R)  

        self.get_logger().info(
            f"x: {x:.3f}  y: {y:.3f}  z: {z:.3f}  |  quat: [{quat[0]:.3f}, {quat[1]:.3f}, {quat[2]:.3f}, {quat[3]:.3f}]"
        )

        # Cálculo de error cuando es pertinente
        if self.p_d is not None:
            p_f = np.array([x, y, z])
            error = np.linalg.norm(self.p_d - p_f)
            
            self.get_logger().info(
                f"   [Verificación IK] pd: ({self.p_d[0]:.4f}, {self.p_d[1]:.4f}, {self.p_d[2]:.4f}) | "
                f"pf: ({p_f[0]:.4f}, {p_f[1]:.4f}, {p_f[2]:.4f}) | Error: {error:.6f} m"
            )


def main(args=None):
    rclpy.init(args=args)
    node = JointSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()