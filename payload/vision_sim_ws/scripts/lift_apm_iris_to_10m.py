import math
import sys
import time
from gz.transport13 import Node
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V

world = sys.argv[1] if len(sys.argv) > 1 else 'city_apm_rgbd'
model = sys.argv[2] if len(sys.argv) > 2 else 'apm_iris'
target_z = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0
rise_time = float(sys.argv[4]) if len(sys.argv) > 4 else 8.0
hold_time = float(sys.argv[5]) if len(sys.argv) > 5 else 3600.0
rate = 30.0
node = Node()
latest = None

def cb(msg):
    global latest
    for p in msg.pose:
        if p.name == model:
            latest = p
            break

node.subscribe(Pose_V, f'/world/{world}/pose/info', cb)
end = time.time() + 10
while time.time() < end and latest is None:
    time.sleep(0.05)
if latest is None:
    print('pose_not_found')
    raise SystemExit(2)

start = latest
x0, y0, z0 = start.position.x, start.position.y, start.position.z
q = start.orientation
print(f'lifting {model}: z {z0:.3f} -> {target_z:.3f}')

def set_pose(z):
    req = Pose()
    req.name = model
    req.position.x = x0
    req.position.y = y0
    req.position.z = z
    req.orientation.CopyFrom(q)
    try:
        node.request(f'/world/{world}/set_pose', req, Pose, Boolean, 100)
    except Exception as exc:
        print('set_pose_error', exc)

start_t = time.time()
while time.time() - start_t < rise_time:
    a = (time.time() - start_t) / rise_time
    a = max(0.0, min(1.0, a))
    smooth = 0.5 - 0.5 * math.cos(math.pi * a)
    set_pose(z0 + (target_z - z0) * smooth)
    time.sleep(1.0 / rate)

hold_end = time.time() + hold_time
while time.time() < hold_end:
    set_pose(target_z)
    time.sleep(1.0 / rate)
