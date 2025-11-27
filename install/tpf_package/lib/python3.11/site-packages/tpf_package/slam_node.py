"""
Nodo de FastSLAM para Grid Mapping (TP Final Parte A).
Integra la logica de particulas del TP5 con el mapeo de ocupacion.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
from typing import List, Tuple, Optional

# --- Funciones Auxiliares (Traidas del TP5) ---

def wrap_angle(angle: float) -> float:
    """Normaliza angulo a [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))

def quaternion_from_yaw(yaw: float):
    """Crea un cuaternion desde yaw (para publicar TF)."""
    class Quat: pass
    q = Quat()
    q.x = 0.0; q.y = 0.0
    q.z = math.sin(yaw * 0.5); q.w = math.cos(yaw * 0.5)
    return q

def yaw_from_quaternion(orientation) -> float:
    """Extrae yaw del mensaje de orientacion."""
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    )

# --- Clase Particula Simplificada ---
class Particle:
    def __init__(self, x=0.0, y=0.0, theta=0.0, weight=0.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight

# --- Nodo Principal ---
class GridSLAM(Node):
    def __init__(self):
        super().__init__('slam_node')
        
        # 1. PARAMETROS
        self.declare_parameters(namespace='', parameters=[
            ('num_particles', 200),        # Menos particulas que en TP5 por costo computacional en Python
            ('resolution', 0.05),         # 5 cm por celda
            ('width', 400),
            ('height', 400),
            ('origin_x', -10.0),
            ('origin_y', -10.0),
            # Alphas del TP5 para el modelo de movimiento
            ('motion_alphas', [0.1, 0.1, 0.05, 0.05]), 
        ])

        # Configuración del mapa
        self.resolution = self.get_parameter('resolution').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.origin_x = self.get_parameter('origin_x').value
        self.origin_y = self.get_parameter('origin_y').value
        
        # Configuración del Filtro de Partículas (Del TP5)
        self.num_particles = self.get_parameter('num_particles').value
        self.alphas = self.get_parameter('motion_alphas').value
        self.rng = np.random.default_rng()
        
        # Inicializar Partículas
        self.particles = [Particle(0,0,0, 1.0/self.num_particles) for _ in range(self.num_particles)]

        # Inicializar Mapa (Log-Odds)
        # 0.0 = desconocido, >0 ocupado, <0 libre
        self.map_log_odds = np.zeros(self.width * self.height, dtype=float)
        
        # Variables de estado
        self.last_odom_pose = None
        self.first_map_created = False
        
        # Publicadores y Suscriptores
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 1)
        self.pose_pub = self.create_publisher(PoseStamped, 'belief', 1)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.create_subscription(Odometry, 'calc_odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        
        # Timer para publicar mapa (1 Hz para no saturar)
        self.create_timer(1.0, self.publish_map)

    # ------------------------------------------------------------------
    # 2. PROCESAMIENTO
    # ------------------------------------------------------------------

    def odom_callback(self, msg: Odometry):
        # Guardamos la ultima odometria cruda para calcular deltas en el scan
        pose = msg.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        self.current_odom_pose = (pose.position.x, pose.position.y, yaw)

    def scan_callback(self, scan_msg: LaserScan):
        if not hasattr(self, 'current_odom_pose'):
            return

        # A. CALCULO DEL DELTA DE MOVIMIENTO (Logica EKFNode del TP5)
        if self.last_odom_pose is None:
            self.last_odom_pose = self.current_odom_pose
            return # Primer frame, solo inicializamos

        dx = self.current_odom_pose[0] - self.last_odom_pose[0]
        dy = self.current_odom_pose[1] - self.last_odom_pose[1]
        
        # Deltas relativos (Odometria)
        current_theta = self.current_odom_pose[2]
        prev_theta = self.last_odom_pose[2]
        
        delta_trans = math.sqrt(dx**2 + dy**2)
        delta_rot1 = wrap_angle(math.atan2(dy, dx) - prev_theta)
        delta_rot2 = wrap_angle(current_theta - prev_theta - delta_rot1)
        
        # Si el movimiento es muy pequeño, asumimos rotación pura o ruido
        if delta_trans < 0.001:
            delta_rot1 = 0.0
            delta_rot2 = wrap_angle(current_theta - prev_theta)

        # B. PREDICCION (Motion Update - Del TP5)
        self.predict_particles(delta_rot1, delta_trans, delta_rot2)
        
        # C. CORRECCION (Measurement Update - Nuevo para Grid)
        if self.first_map_created:
            self.update_weights_with_scan(scan_msg)
            self.resample_particles()
        
        # D. ESTIMACION MEJOR POSE
        best_particle = self.get_best_particle()
        
        # E. MAPEO (Usando la mejor particula)
        self.update_map_with_scan(best_particle, scan_msg)
        self.first_map_created = True
        
        # F. PUBLICAR TF (Map -> Odom)
        # La magia del SLAM: map->odom = map->base * base->odom
        self.publish_tf(best_particle, self.current_odom_pose)
        self.publish_belief(best_particle)
        
        # Actualizar estado anterior
        self.last_odom_pose = self.current_odom_pose

    # ------------------------------------------------------------------
    # 3. LOGICA DE PARTICULAS (Adaptado TP5)
    # ------------------------------------------------------------------

    def predict_particles(self, dr1, dt, dr2):
        """Mueve las particulas agregando ruido gaussiano (Alphas)."""
        a1, a2, a3, a4 = self.alphas
        
        for p in self.particles:
            # Ruido muestreado
            dr1_noisy = dr1 + self.rng.normal(0.0, a1 * abs(dr1) + a2 * dt)
            dt_noisy  = dt  + self.rng.normal(0.0, a3 * dt + a4 * (abs(dr1) + abs(dr2)))
            dr2_noisy = dr2 + self.rng.normal(0.0, a1 * abs(dr2) + a2 * dt)
            
            # Aplicar movimiento
            p.x += dt_noisy * math.cos(p.theta + dr1_noisy)
            p.y += dt_noisy * math.sin(p.theta + dr1_noisy)
            p.theta = wrap_angle(p.theta + dr1_noisy + dr2_noisy)

    def update_weights_with_scan(self, scan):
        """Scan Matching simple: Las particulas que 'calzan' con paredes ganan peso."""
        # Precalcular senos y cosenos del scan para optimizar
        angles = np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)
        # Downsampling: usar 1 de cada 10 rayos para velocidad
        step = 10
        valid_indices = [i for i in range(0, len(scan.ranges), step) 
                         if scan.range_min < scan.ranges[i] < scan.range_max]
        
        total_weight = 0.0
        
        for p in self.particles:
            score = 0
            # Transformar puntos del scan a coordenadas del mundo segun la particula
            for i in valid_indices:
                r = scan.ranges[i]
                # Angulo global del rayo
                a = p.theta + scan.angle_min + i * scan.angle_increment
                
                # Coordenada del punto de impacto
                hit_x = p.x + r * math.cos(a)
                hit_y = p.y + r * math.sin(a)
                
                # Verificar celda en el mapa
                mx, my = self.world_to_map(hit_x, hit_y)
                if mx is not None:
                    idx = my * self.width + mx
                    # Si la celda ya es "Ocupada" (>0 log odds), premiar particula
                    if self.map_log_odds[idx] > 2.0: 
                        score += 1
                    # Si la celda es "Libre" conocida (< -2), penalizar
                    elif self.map_log_odds[idx] < -2.0:
                        score -= 1
            
            # Funcion de peso exponencial
            p.weight = math.exp(score * 0.1) # Factor de ajuste
            total_weight += p.weight

        # Normalizar
        if total_weight > 0:
            for p in self.particles: p.weight /= total_weight
        else:
            # Si todas fallan, resetear a uniforme
            for p in self.particles: p.weight = 1.0 / self.num_particles

    def resample_particles(self):
        """Low variance resampling (Igual al TP5)."""
        weights = [p.weight for p in self.particles]
        # Neff check
        neff = 1.0 / sum(w**2 for w in weights) if sum(weights) > 0 else 0
        if neff > self.num_particles / 2.0:
            return # No resamplear si todavia hay variedad
            
        new_particles = []
        cumulative = np.cumsum(weights)
        step = 1.0 / self.num_particles
        r = self.rng.uniform(0.0, step)
        i = 0
        
        for m in range(self.num_particles):
            U = r + m * step
            while U > cumulative[i] and i < len(weights) - 1:
                i += 1
            p = self.particles[i]
            new_particles.append(Particle(p.x, p.y, p.theta, 1.0/self.num_particles))
            
        self.particles = new_particles

    # ------------------------------------------------------------------
    # 4. LOGICA DE MAPEO (Grid Mapping)
    # ------------------------------------------------------------------
    
    def update_map_with_scan(self, pose: Particle, scan):
        """Pinta el mapa usando Bresenham desde la pose dada."""
        # Parametros de actualizacion log-odds
        L_OCC = 0.85
        L_FREE = -0.4
        
        robot_mx, robot_my = self.world_to_map(pose.x, pose.y)
        if robot_mx is None: return

        angles = np.arange(scan.angle_min, scan.angle_max + scan.angle_increment, scan.angle_increment)
        # Usar subset o todos los rayos segun rendimiento
        step = 5 
        
        for i in range(0, len(scan.ranges), step):
            if i >= len(angles): break
            r = scan.ranges[i]
            
            # Si el rango es inf o max, actualizamos espacio libre hasta max
            if not math.isfinite(r) or r > scan.range_max:
                continue # Opcional: ray casting hasta max_range como libre
                
            angle = pose.theta + angles[i]
            hit_x = pose.x + r * math.cos(angle)
            hit_y = pose.y + r * math.sin(angle)
            
            hit_mx, hit_my = self.world_to_map(hit_x, hit_y)
            if hit_mx is None: continue
            
            # Bresenham para espacio libre
            cells = self.bresenham(robot_mx, robot_my, hit_mx, hit_my)
            for (cx, cy) in cells[:-1]: # Todos menos el ultimo son libres
                self.update_cell(cx, cy, L_FREE)
            
            # Ultima celda es ocupada
            self.update_cell(hit_mx, hit_my, L_OCC)

    def bresenham(self, x0, y0, x1, y1):
        """Algoritmo estandar para trazar lineas en grillas."""
        points = []
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        
        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1: break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy
        return points

    def update_cell(self, x, y, val):
        idx = y * self.width + x
        if 0 <= idx < len(self.map_log_odds):
            self.map_log_odds[idx] += val
            # Clamping para evitar overflow y mantener estabilidad
            self.map_log_odds[idx] = max(-10.0, min(10.0, self.map_log_odds[idx]))

    def world_to_map(self, wx, wy):
        mx = int((wx - self.origin_x) / self.resolution)
        my = int((wy - self.origin_y) / self.resolution)
        if 0 <= mx < self.width and 0 <= my < self.height:
            return mx, my
        return None, None

    # ------------------------------------------------------------------
    # 5. UTILIDADES Y PUBLISHERS
    # ------------------------------------------------------------------

    def get_best_particle(self):
        # Retorna la particula con mayor peso o una promedio
        return max(self.particles, key=lambda p: p.weight)

    def publish_tf(self, particle, odom_pose):
        # Publicamos map -> odom
        # T_map_base = T_map_odom * T_odom_base
        # Queremos T_map_odom.
        # Simplificacion 2D:
        # map_odom_x = particle.x - odom_x (rotado)
        
        # En ROS, usualmente enviamos la transformada correccion.
        # Logica rapida: La diferencia entre donde creo que estoy (particle) y donde dice la odom que estoy.
        
        # Odom frame values
        ox, oy, otheta = odom_pose
        # Map frame values (Best particle)
        mx, my, mtheta = particle.x, particle.y, particle.theta
        
        # Transform map -> odom
        # Esta matemaica es delicada, pero para 2D plano:
        # map_T_odom = map_T_base * base_T_odom
        #            = map_T_base * (odom_T_base)^-1
        
        # Para evitar lios matriciales en este script simple, calculamos el offset visualmente:
        # Es la diferencia vectorial rotada.
        
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'odom' # OJO: Debe coincidir con el frame fijo de tu odom
        
        # Calculo simplificado (asumiendo coincidencia inicial)
        # Esto funciona bien si los ejes estan alineados, sino requiere matrices.
        # Dadas las limitaciones del TP, esto suele bastar para corregir drift lineal.
        
        # Yaw difference
        yaw_diff = mtheta - otheta
        
        # Translation: Rotar la posicion odom al frame map y restar
        # x_map = x_transform + x_odom * cos(theta_trans) - y_odom * sin(theta_trans)
        # Es mas facil pensar: Transform = Pose_Map - Pose_Odom (con rotacion)
        
        # Usaremos aproximación directa por ahora para no complicar el codigo sin numpy matricial completo
        msg.transform.translation.x = mx - (ox * math.cos(yaw_diff) - oy * math.sin(yaw_diff))
        msg.transform.translation.y = my - (ox * math.sin(yaw_diff) + oy * math.cos(yaw_diff))
        msg.transform.translation.z = 0.0
        
        q = quaternion_from_yaw(yaw_diff)
        msg.transform.rotation.x = q.x
        msg.transform.rotation.y = q.y
        msg.transform.rotation.z = q.z
        msg.transform.rotation.w = q.w
        
        self.tf_broadcaster.sendTransform(msg)

    def publish_map(self):
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'map'
        grid.info.resolution = self.resolution
        grid.info.width = self.width
        grid.info.height = self.height
        grid.info.origin.position.x = self.origin_x
        grid.info.origin.position.y = self.origin_y
        grid.info.origin.orientation.w = 1.0
        
        # Convertir log-odds a 0-100
        data = [-1] * (self.width * self.height)
        for i, val in enumerate(self.map_log_odds):
            if val > 2.0: data[i] = 100
            elif val < -2.0: data[i] = 0
            else: data[i] = -1
            
        grid.data = data
        self.map_pub.publish(grid)

    def publish_belief(self, particle):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = particle.x
        msg.pose.position.y = particle.y
        q = quaternion_from_yaw(particle.theta)
        msg.pose.orientation.x = q.x
        msg.pose.orientation.y = q.y
        msg.pose.orientation.z = q.z
        msg.pose.orientation.w = q.w
        self.pose_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GridSLAM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()