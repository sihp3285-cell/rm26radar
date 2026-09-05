#!/usr/bin/env python3
"""Read every stored message directly, preserving the bag reception timestamp."""
import collections,json,pathlib,sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict

def export(folder):
    folder=pathlib.Path(folder)
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(folder/'bag'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
    types={t.name:get_message(t.type) for t in reader.get_all_topics_and_types()}
    counts=collections.Counter(); kept=collections.Counter()
    with open(folder/'events.jsonl','w') as f:
        while reader.has_next():
            topic,data,receipt_ns=reader.read_next();counts[topic]+=1
            if topic not in ('/radar_map','/prior_predictions','/pipeline_timing'):continue
            msg=deserialize_message(data,types[topic]);d=message_to_ordereddict(msg)
            f.write(json.dumps({'topic':topic,'receipt_ns':receipt_ns,'message':d},ensure_ascii=False)+'\n');kept[topic]+=1
    info=rosbag2_py.Info().read_metadata(str(folder/'bag'),'sqlite3')
    expected={x.topic_metadata.name:x.message_count for x in info.topics_with_message_count}
    assert dict(counts)==expected,(counts,expected)
    result={'bag_counts':dict(counts),'exported_counts':dict(kept),'verified_all_bag_messages':True}
    (folder/'export_audit.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result),flush=True)

if __name__=='__main__': export(sys.argv[1])
