import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from autonomy.srv import RobotState

odometry = None
occupancy_grid = None
service = None

def update_odometry(msg):
    global odometry
    odometry = msg

def update_occupancy_grid(msg):
    global occupancy_grid
    occupancy_grid = msg

def subscribe():
    rospy.Subscriber("odometry/filtered_map", Odometry, update_odometry)
    rospy.Subscriber("", OccupancyGrid, ) #todo
    pass

def send_robot_state(req):
    global odometry,occupancy_grid
    return autonomy.srv.RobotStateResponse(time=rospy.get_rostime(), odometry=odometry, grid=occupancy_grid)

def on_shut_down():
    global service
    service.shutdown("robot_state no longer running")

def main():
    global service
    rospy.init_node("robot_state")
    subscribe()
    service = rospy.Service("robot_state", autonomy.srv.RobotState, send_robot_state)
    rospy.spin()
    pass



if __name__ == "__main__":
    main()




