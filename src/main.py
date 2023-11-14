import rospy
import std_msgs.string
from custom_msgs.msg import Reference, Goal, Localization, SemanticInfos, Nav
class FMS:
    def __init__(self):
        # parameters
        self.pub_hz = 1.0
        self.car_ids = ["thor01","thor02","thor03"]
        self.route_ids = ["1an","2an","3an"]

        self.vehicles = []
        self.vehicle_publishers = []

    def getVehicleIds(self):
        return self.car_ids
    
    def checkVehicleNodeSet(self):
        pass

    def pubSpeedLimit(self):
        pass

    def pubNodeSet(self):
        pass
    
class Vehicle:
    def __init__(self, car_id):
        self.car_id = car_id
        self.route = None
        self.nav_fusion = None
        self.pose_estimator = None
        self.is_inter = None
        self.is_arrival = None
        self.route_id = None
        self.semantic_infos = None

        reference_sub = rospy.Subscriber(self.car_id + "/module/globalpath_reader/reference", Reference, self.poseEstimatorCallback, queue_size=1)
        semantic_info_sub = rospy.Subscriber(self.car_id + "/module/map_reader/semantic_info", SemanticInfos, self.poseEstimatorCallback, queue_size=1)
        nav_fusion_sub = rospy.Subscriber(self.car_id + "/sensor/nav/nav_fusion", Nav, self.navFusionCallback, queue_size=1)
        pose_estimator_sub = rospy.Subscriber(self.car_id + "/module/localization/pose_estimator", Localization, self.poseEstimatorCallback, queue_size=1)

    def referenceCallback(self, msg):
        self.route = msg.data.request_id
        self.is_arrival = msg.data.arrival_signal
    
    def navFusionCallback(self, msg):
        self.nav_fusion = msg.data
    
    def poseEstimatorCallback(self, msg):
        self.pose_estimator = msg.data
    
    def semanticInfoCallback(self, msg):
        self.semantic_infos = msg.data

class VehiclePublisher:
    def __init__(self, car_id):
        self.car_id = car_id
        route_pub = rospy.Publisher(self.car_id + "/module/globalpath_planner/goal", Goal, queue_size=1)

    def publish(self, route):
        goal = Goal()
        Goal.route = route
        self.route_pub.publish(goal)
    

    

def main():
    fms = FMS()
    car_ids = fms.getVehicleIds()

    for car_id in car_ids:
        vehicle = Vehicle(car_id)
        vehicle_publisher = VehiclePublisher(car_id)
        fms.vehicles.append(vehicle)
        fms.vehicle_publishers.append(vehicle_publisher)
    
    while not rospy.is_shutdown():
        fms.pubSpeedLimit()
        fms.pubNodeSet()
        rospy.sleep(fms.pub_hz)



if __name__ == "__main__":
    main()