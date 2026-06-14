"""Moodul 03: Automaatne folkrace sõitmine.

Ülesanne:
  Kirjuta reaktiivne sõlm, mis sõidab folkrace rajal automaatselt
  ilma seintesse põrkamata. Robot loeb lidarit ja otsustab iga
  mõõtmise põhjal kuhu sõita.

Nõuded:
  - Robot teeb vähemalt ÜHE TÄISRINGI
  - Robot ei põrka seintesse
  - Robot suudab üle SILLA minna

Käivita:
  Terminal 1: ros2 launch yahboom_webots webots.launch.py
  Terminal 2: ros2 run folkrace_driver folkrace_driver

Lidar indeksid (720 kiirt, 360°):
  ranges[0]   = -180° = otse TAGA
  ranges[180] =  -90° = PAREMALE
  ranges[360] =    0° = otse ETTE
  ranges[540] =  +90° = VASAKULE
  ranges[719] = +180° = otse TAGA

  Ette ±15° = indeksid 330..390 (ümber 360)
"""
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class FolkraceDriver(Node):

    def __init__(self):
        super().__init__('folkrace_driver')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.get_logger().info('Folkrace driver käivitatud!')

    def valid_values(self, ranges, start, end):
        values = []

        for r in ranges[start:end]:
            if math.isfinite(r) and 0.12 < r < 8.0:
                values.append(r)

        return values

    def sector_min(self, ranges, start, end, default=8.0):
        values = self.valid_values(ranges, start, end)

        if len(values) == 0:
            return default

        return min(values)

    def sector_avg(self, ranges, start, end, default=8.0):
        values = self.valid_values(ranges, start, end)

        if len(values) == 0:
            return default

        return sum(values) / len(values)

    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def scan_callback(self, msg):
        ranges = msg.ranges

        front = self.sector_min(ranges, 350, 370)

        front_left = self.sector_min(ranges, 385, 470)
        front_right = self.sector_min(ranges, 250, 335)

        left = self.sector_avg(ranges, 500, 620)
        right = self.sector_avg(ranges, 100, 220)

        cmd = Twist()

        
        bridge_like = (
            front < 0.75 and
            front_left > 0.45 and
            front_right > 0.45 and
            left > 0.45 and
            right > 0.45
        )

        
        real_wall_ahead = (
            front < 0.35 and
            not bridge_like and
            (front_left < 0.45 or front_right < 0.45)
        )

        if real_wall_ahead:
            cmd.linear.x = -0.08

            if front_left > front_right:
                cmd.angular.z = -1.0
            else:
                cmd.angular.z = 1.0

        elif front < 0.60 and not bridge_like:
            cmd.linear.x = 0.07

            if front_left > front_right:
                cmd.angular.z = -1.0
            else:
                cmd.angular.z = 1.0

        
        elif front_left < 0.42:
            cmd.linear.x = 0.14
            cmd.angular.z = 0.75

        
        elif front_right < 0.42:
            cmd.linear.x = 0.14
            cmd.angular.z = -0.75

        else:
           
            center_error = right - left

            cmd.linear.x = 0.24
            cmd.angular.z = 0.38 * center_error

          
            if front_left < 0.65:
                cmd.angular.z += 0.30

            if front_right < 0.65:
                cmd.angular.z -= 0.30

          
            if bridge_like:
                cmd.linear.x = 0.18
                cmd.angular.z = self.clamp(cmd.angular.z, -0.35, 0.35)
            else:
                cmd.angular.z = self.clamp(cmd.angular.z, -0.85, 0.85)

            if front < 1.0:
                cmd.linear.x = min(cmd.linear.x, 0.18)

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)

    node = FolkraceDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
