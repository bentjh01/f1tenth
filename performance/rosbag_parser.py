from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

def get_rosbag_topics(rosbag_path):
    with Reader(rosbag_path) as reader:
        return(list(reader.topics.keys()))

def read_rosbag_scan(rosbag_path, topic='/scan'):
    with Reader(rosbag_path) as reader:
        data = []
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == topic:
                msg = deserialize_cdr(rawdata, connection.msgtype)
                data.append(msg)
        return data

def read_rosbag_odom(rosbag_path, topic='/odom'):
    with Reader(rosbag_path) as reader:
        data = []
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == topic:
                msg = deserialize_cdr(rawdata, connection.msgtype)
                data.append(msg)
        return data

def read_rosbag_ackermann(rosbag_path, topic='/ackermann_cmd'):
    with Reader(rosbag_path) as reader:
        data = []
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == topic:
                msg = deserialize_cdr(rawdata, connection.msgtype)
                data.append(msg)
        return data

def main(args = None):
    get_rosbag_topics('./performance/rosbag2_2024_10_22-13_25_55')

if __name__ == '__main__':
    main()