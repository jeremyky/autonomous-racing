import numpy as np
import math


class DisparityExtender:
    def __init__(self, disparity_distance, safety_extension, max_range):
        self.disparity = disparity_distance
        self._safety_extension = safety_extension
        self._max_range = max_range

    def preprocess_lidar(self, ranges):
        # just to make sure all ranges capped to max
        return np.clip(ranges, 0, self._max_range)
    
    # added a dynamic disparity extension - should help with cornering
    # when obstacle is further away, we rather have a smaller extension so reduce it 
    # when obstacle is closer, we want to increase extension 
    # math:
    # theta = (d / 2pi(r)) * degrees 
    def dynamic_extension(self, obstacle_distance, angle_increment):

        # theta = 2 * math.pi * (self._safety_extension) / (2 * math.pi * obstacle_distance)
        # return theta / angle_incrementn


        # return int(self._safety_extension / angle_increment)


        if obstacle_distance < 0.5:
            return int(self._safety_extension * 3)
        elif obstacle_distance < 0.75:
            return int(self._safety_extension * 2.5)
        elif obstacle_distance < 1:
            return int(self._safety_extension * 1.75)
        elif obstacle_distance < 1.5:
            return int(self._safety_extension * 0.75)
        elif obstacle_distance < 2.0:
            return int(self._safety_extension * 0.5)
        else:
            return self._safety_extension
    

    def extend_disparities(self, original_ranges, angle_increment):
        """
        apply a safety bubble around all obstacles detected'
        -- instead of extending from both directions, we now process range twice: 
        right (extending left) and left (extending right)
        """
        
        original_ranges = self.preprocess_lidar(original_ranges)
        ranges = original_ranges.copy()
        # calculate how many indices to include in the safety bubble
        
        # process from right, extend bubble to left
        for i in range(len(ranges) - 2, -1, -1):  # to avoid index error
            # detect disparity by checking if the distance change is greater than the safety radius
            if original_ranges[i] - original_ranges[i + 1] > self.disparity:
                obstacle_distance = original_ranges[i+ 1]
                if obstacle_distance < 0.05: break
                dynamic_extension = self.dynamic_extension(obstacle_distance, angle_increment)
                # print(obstacle_distance, dynamic_extension)
                # dynsmic extension based on distance
                start_idx = max(0, i - int(dynamic_extension))
                # start_idx = max(0, i - self._safety_extension)
                # extend the bubble left by setting points within range to the obstacle distance
                for j in range(i, start_idx - 1, -1):
                    ranges[j] = min(original_ranges[j], obstacle_distance)

        # process from left, extend bubble to right
        for i in range(1, len(original_ranges)):
            # detect a disparity ^ same way
            if original_ranges[i] - original_ranges[i - 1] > self.disparity:
                obstacle_distance = original_ranges[i-1]
                if obstacle_distance < 0.05: break
                dynamic_extension = self.dynamic_extension(obstacle_distance, angle_increment)
                # print(obstacle_distance, dynamic_extension)
                end_idx = min(len(ranges), i + int(dynamic_extension))
                # end_idx = min(len(ranges), i + self._safety_extension)
                # extend right 
                for j in range(i, end_idx):
                    ranges[j] = min(original_ranges[j], obstacle_distance)

        return ranges