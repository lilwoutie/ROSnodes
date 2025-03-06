import rospy
from std_msgs.msg import String
from TargetSelector import TargetSelector
#TO BE IMPLEMENTED
# -parse_coordinates(data)
def callback(data):
    coordinates = parse_coordinates(data.data)
    selector = TargetSelector()
    shortest_distance, closest_person = selector.get_shortest_distance(coordinates, 'R', 'P', None)
    
    if shortest_distance is not None and closest_person is not None:
        result = f"Shortest distance: {shortest_distance}, Closest person coordinates: {closest_person}"
        rospy.loginfo(result)
        pub.publish(result)

def parse_coordinates(data):
    # Implement your logic to parse the incoming data into coordinates
    # Example: [(1, 2, 'R'), (4, 6, 'P'), ...]
    # For simplicity, let's assume the data is a string of tuples
    coordinates = eval(data)
    return coordinates

def target_selector_node():
    global pub
    rospy.init_node('target_selector_node', anonymous=True)
    rospy.Subscriber('coordinates_topic', String, callback)
    pub = rospy.Publisher('target', String, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        target_selector_node()
    except rospy.ROSInterruptException:
        pass