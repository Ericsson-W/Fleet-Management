import numpy as np
def Fleet(self,lead,pack,name):
    lead_vehicle = self.controlled_vehicles[lead]
    lead_v = lead_vehicle.v
    lead_w = lead_vehicle.w

    prev_vehicle = lead_vehicle

    for vehicle_name in pack:
        current_vehicle = self.controlled_vehicles[vehicle_name]

        distance_to_prev_vehicle = np.linalg.norm(current_vehicle.position - prev_vehicle.position)

        current_vehicle.v = prev_vehicle.v
        current_vehicle.w = prev_vehicle.w # change w here based on pack location

        if distance_to_prev_vehicle < 0.2:
            current_vehicle.v *= 0.8
        else:
            current_vehicle.v *= 1.2  

        prev_vehicle = current_vehicle

        return current_vehicle.v,current_vehicle.w
