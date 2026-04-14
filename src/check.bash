python - <<'PY'
import rosbag
bag = rosbag.Bag('/home/nvidia/ws_livox/src/FAST_LIO/data/2025-07-11-02-16-40.bag')
for topic,msg,_ in bag.read_messages(topics=['/livox/lidar_192_168_1_125']):
    print(topic, msg._type, [f.name for f in msg.fields])
    break
PY