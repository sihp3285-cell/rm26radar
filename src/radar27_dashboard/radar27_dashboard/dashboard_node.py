"""ROS subscriptions and HTTP run in separate threads, in an independent process."""
from pathlib import Path
import signal
import threading

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import struct
import yaml
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from radar27_interfaces import msg

from .adapters.vision import TOPICS
from .dashboard_state import DashboardState
from .http_server import DashboardServer


class DashboardNode(Node):
    def __init__(self):
        super().__init__('radar27_dashboard')
        def param(name, default):
            return self.declare_parameter(name, default).value
        self.host = param('host', '127.0.0.1')
        self.port = param('port', 8765)
        self.radar_hz = param('radar_hz', 5.0)
        self.performance_hz = param('performance_hz', 1.0)
        stale = param('stale_after_s', 2.0)
        length, width = param('field_length', 28.0), param('field_width', 15.0)
        if not (0 < self.radar_hz <= 20 and 0 < self.performance_hz <= 5
                and stale > 0 and length > 0 and width > 0):
            raise ValueError('Invalid rates, stale timeout or field dimensions')
        self.state = DashboardState(stale, length, width, param('world_z_toward_blue', True))
        try:
            default_map = str(Path(get_package_share_directory('radar27_bringup')) / 'config/default/map.yaml')
        except PackageNotFoundError:
            default_map = ''
        map_config = param('map_config', default_map)
        self.map_path = None
        self.state.field['map'] = None
        if map_config:
            config_path = Path(map_config).expanduser().resolve()
            config = yaml.safe_load(config_path.read_text())
            self.map_path = (config_path.parent / config['mapPath']).resolve()
            with self.map_path.open('rb') as file:
                header = file.read(24)
            if header[:8] != b'\x89PNG\r\n\x1a\n':
                raise ValueError('Dashboard mapPath must point to the Qt PNG basemap')
            image_width, image_height = struct.unpack('>II', header[16:24])
            self.state.field.update(length=float(config['race_size'][0]),
                                    width=float(config['race_size'][1]),
                                    map=dict(url='/map.png', width=image_width, height=image_height,
                                             clockwise=bool(config['isflip'])))
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscriptions_ = [self.create_subscription(getattr(msg, type_name), topic,
            lambda message, source=source: self.state.update(source, message), qos)
            for source, (topic, type_name) in TOPICS.items()]
        self.subscriptions_.append(self.create_subscription(
            msg.MatchState, '/match_state', lambda message: self.state.update('match', message),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    server = None
    try:
        node = DashboardNode()
        root = Path(get_package_share_directory('radar27_dashboard')) / 'web'
        server = DashboardServer((node.host, node.port), node.state, root,
                                 node.radar_hz, node.performance_hz, node.map_path)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        node.get_logger().info(f'Dashboard: http://{node.host}:{node.port} (read-only)')
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # A terminal SIGINT reaches both launch and this child; launch may then
        # forward a second one while HTTP shutdown is waiting for its thread.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if server:
            server.close()
        if node:
            node.destroy_node()
        rclpy.try_shutdown()
