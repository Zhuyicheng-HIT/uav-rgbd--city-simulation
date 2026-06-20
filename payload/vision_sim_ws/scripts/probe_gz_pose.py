import sys
import time
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V

world = sys.argv[1] if len(sys.argv) > 1 else 'city_apm_rgbd'
target = sys.argv[2] if len(sys.argv) > 2 else 'apm_iris'
duration = float(sys.argv[3]) if len(sys.argv) > 3 else 15.0
samples = []
node = Node()

def cb(msg):
    for pose in msg.pose:
        if pose.name == target:
            samples.append((time.time(), pose.position.z))
            break

node.subscribe(Pose_V, f'/world/{world}/pose/info', cb)
end = time.time() + duration
while time.time() < end:
    time.sleep(0.1)

if not samples:
    print('pose_seen=False')
    raise SystemExit(2)
zs = [z for _, z in samples]
print('pose_seen=True')
print('samples=', len(samples))
print('z_first=', round(zs[0], 4))
print('z_min=', round(min(zs), 4))
print('z_last=', round(zs[-1], 4))
print('z_last5=', [round(z, 4) for z in zs[-5:]])
