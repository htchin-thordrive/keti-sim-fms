class FMS:
    def __init__(self):
        self.node_set = None
        self.nav_fusion = None
        self.pose_estimator = None
        self.is_inter = None
        self.is_arrival = None

        # parameters
        self.pub_hz = 1.0
        self.car_ids = ["thor01","thor02","thor03"]
        self.route_ids = ["1an","2an","3an"]

    
    def checkVehicleNodeSet(self):
        pass

    def pubSpeedLimit(self):
        pass

    def pubNodeSet(self):
        pass
    
    # subscribers

def main():
    fms = FMS()

if __name__ == "__main__":
    main()