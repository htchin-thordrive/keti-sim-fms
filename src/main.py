import rospy
from std_msgs.msg import String, Float32
from custom_msgs.msg import Reference, Goal, Localization, SemanticInfos, Nav
class FMS:
    def __init__(self):
        # parameters
        self.pub_hz = 1.0
        self.car_ids = ["thor01","thor02","thor03"]
        self.routes = ['1an','2an','3an']
        self.input_speed = 30.0/3.6

        self.vehicles = []
        self.vehicle_publishers = []

    def getVehicleIds(self):
        return self.car_ids
    
    # Check if the vehicle's route is the same as the intended route
    def checkVehicleRouteShouldUpdated(self):
        vehicles_should_be_updated = []
        for vehicle_idx in range(len(self.vehicles)):
            # print(self.vehicles[vehicle_idx].route, self.routes[vehicle_idx])
            if self.vehicles[vehicle_idx].route != self.routes[vehicle_idx]:
                vehicles_should_be_updated.append(vehicle_idx)
        return vehicles_should_be_updated

    def pubSpeedLimit(self, vehicle_idxs):
        for vehicle_idx in vehicle_idxs:
            self.vehicle_publishers[vehicle_idx].pubSpeedLimit(self.input_speed)        

    def pubRoute(self, vehicle_idxs):
        for vehicle_idx in vehicle_idxs:
            self.vehicle_publishers[vehicle_idx].pubRoute(self.routes[vehicle_idx])
    
class Vehicle:
    def __init__(self, car_id):
        self.car_id = car_id
        self.route = None
        self.nav_fusion = None
        self.pose_estimator = None
        self.is_inter = None
        self.is_arrival = None
        self.semantic_infos = None

        reference_sub = rospy.Subscriber("/" + self.car_id + "/module/globalpath_reader/reference", Reference, self.referenceCallback, queue_size=1)
        semantic_info_sub = rospy.Subscriber("/" + self.car_id + "/module/map_reader/semantic_info", SemanticInfos, self.semanticInfoCallback, queue_size=1)
        nav_fusion_sub = rospy.Subscriber("/" + self.car_id + "/sensor/nav/nav_fusion", Nav, self.navFusionCallback, queue_size=1)
        pose_estimator_sub = rospy.Subscriber("/" + self.car_id + "/module/localization/pose_estimator", Localization, self.poseEstimatorCallback, queue_size=1)

    def referenceCallback(self, msg):
        # print("referenceCallback")
        self.route = msg.request_id
        self.is_arrival = msg.arrival_signal
    
    def navFusionCallback(self, msg):
        self.nav_fusion = msg
    
    def poseEstimatorCallback(self, msg):
        self.pose_estimator = msg
    
    def semanticInfoCallback(self, msg):
        self.semantic_infos = msg

class VehiclePublisher:
    def __init__(self, car_id):
        self.car_id = car_id
        self.route_pub = rospy.Publisher("/" + self.car_id + "/module/globalpath_planner/goal", Goal, queue_size=1)
        self.speed_limit_pub = rospy.Publisher("/" + self.car_id + "/module/new_velocity_planner/inputSpeed", Float32, queue_size=1)

    def pubRoute(self, route):
        goal = Goal()
        goal.route = route
        self.route_pub.publish(goal)
    
    def pubSpeedLimit(self, speed_limit):
        speed_msg = Float32()
        speed_msg.data = speed_limit
        self.speed_limit_pub.publish(speed_msg)

    

def main():    
    rospy.init_node('FMS', anonymous=True)
    fms = FMS()
    car_ids = fms.getVehicleIds()

    for car_id in car_ids:
        vehicle = Vehicle(car_id)
        vehicle_publisher = VehiclePublisher(car_id)
        fms.vehicles.append(vehicle)
        fms.vehicle_publishers.append(vehicle_publisher)
    
    while not rospy.is_shutdown():
        # print("\n")
        # publish route and speed limit for the vehicles that don't match with the intended route
        vehicles_should_be_updated =fms.checkVehicleRouteShouldUpdated()
        # print("vehicles_should_be_updated", vehicles_should_be_updated)
        fms.pubRoute(vehicles_should_be_updated)
        fms.pubSpeedLimit(vehicles_should_be_updated)

        rospy.sleep(fms.pub_hz)



if __name__ == "__main__":
    main()