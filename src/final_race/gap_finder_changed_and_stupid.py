import numpy as np
import math
import rospy


# TODO: We should also consider other ways to choosing gaps/choosing point in gap
class GapFinder:
    def __init__(self, gap_selection, point_selection, min_gap_size, min_gap_distance, cornering_distance):
        gap_selection_functions = {
            "deepest" : self.find_deepest_gap,
            "widest": self.find_widest_gap,
            "least_steering": self.find_least_steering_gap,
            "largest_integral": self.find_largest_integral_gap,
            "best" : self.find_deepest_then_widest_then_left_gap
        }

        point_selection_functions = {
            "deepest": self.find_deepest_point,
            "middle": self.find_middle_point,
            "least_steering": self.find_least_steering_point,
            "best" : self.find_deepest_then_middle_point
        }

        self.gap_selection = gap_selection_functions[gap_selection]
        self.point_selection = point_selection_functions[point_selection]
        self.min_gap_size = min_gap_size
        self.min_gap_distance = min_gap_distance
        self.cornering_distance = cornering_distance

        self.ranges = None
        self.data = None
        self.i = 0

        # NEW CODE TO FIX GAP SWITCHING; Initialize gap history
        self.previous_gaps = []
        self.gap_history_size = 6 # change (27 memory read in 3.5 sec)

        self.og_min_gap_distance = min_gap_distance  # store orginial min gap distance



        # NEW CODE TO FIX TURNING/CORNERING ISSUE ; track vehicle speed for dynamic cornering
        self.current_speed = 0
        self.prev_steering_angle = 90 # start straight
        self.steering_smoothing = .3 # smoother = lower value


    def update_data(self, ranges, data):
        self.ranges = ranges
        self.data = data

    def get_gap(self):
        valid_gaps = self.get_gaps() # for if no gaps found 
 

            
        while not valid_gaps:
            self.min_gap_distance -= .025  # increase min gap distance in increments
            if self.min_gap_distance > 2 * self.og_min_gap_distance:
                self.min_gap_distance = self.og_min_gap_distance  # reset if maxed out
            return 0, len(self.ranges) - 1  # default
        
        # reset min_gap_distance if valid gaps are found
        self.min_gap_distance = self.og_min_gap_distance


        # -----
    
        # ADD GAPS TO GAP HISTORY, ENSURE IT FOLLOWS GAP HISTORY SIZE FOR MEMORY
        current_gap = self.gap_selection()
        self.previous_gaps.append(current_gap)
        if len(self.previous_gaps) > self.gap_history_size:
            self.previous_gaps.pop(0)
        
        # if we have atleast 2 gaps in history
        if len(self.previous_gaps) > 1:
            # average center of previous gaps in history (p[0] and p[1] start and end indicies)
            prev_gap_center = sum((p[0] + p[1]) for p in self.previous_gaps[:-1]) / (2 * (len(self.previous_gaps)-1))
            # center of current gap
            current_gap_center = (current_gap[0] + current_gap[1]) / 2
            # if the current gap center differ to much from history 
            threshold = len(self.ranges) * .35 # 35% OF LIDAR READING INDEX IS THRESHOLD  
            if abs(current_gap_center - prev_gap_center) > threshold:
                # return valid_gaps[0]
                return self.previous_gaps[-2] # last stable gap we were using
        return current_gap
    




    def get_point(self, start_i, end_i):

        # OLD COOOODE ----

        # Check cornering
        min_index = self.get_index_of(90)
        max_index = self.get_index_of(180)

        close = 0
        for i in range(min_index, max_index):
            if 0.05 < self.ranges[i] < self.cornering_distance:
                close += 1
        if close > 10:
            print("cornering")
            return self.get_index_of(90)
        return self.point_selection(start_i, end_i) 
    
    # NEW CODE HELPER METHOD (cornering/turning)
    def get_angle_from_index(self, index):
        ''' convert lidar to angle in degree'''
        angle_rad = index * self.data.angle_increment + self.data.angle_min
        return math.degrees(angle_rad)
    
    # NEW CODE HELPER METHOD (cornering/turning)
    def smooth_steering(self, target_angle):
        ''' smoothly transition between current steering angle and target angle'''
        ''' formula: new angle = prev angle * (1 - smooth) + target_angle * smooth'''
        smoothed_angle = (self.prev_steering_angle * (1 - self.steering_smoothing) + target_angle * self.steering_smoothing)
        self.prev_steering_angle = smoothed_angle
        return smoothed_angle

    

    def get_point_to_go_to(self):
        start_i, end_i = self.get_gap()
        best_point = self.get_point(start_i, end_i)
        return best_point
    
    def find_middle_point(self, start_i, end_i):
        """
        pick the middle point in the identified gap
        """
        return (start_i + end_i) // 2


    def find_least_steering_point(self, start_i, end_i):
        """
        Find the point in the gap that requires the least steering
        (closest to straight ahead)
        """
        center_index = self.get_index_of(90)

        # If gap contains center, return center
        if start_i <= center_index <= end_i:
            return center_index
        
        # If gap is to the lrft of center, return leftmost point
        if start_i > center_index:
            return start_i
            
        # If gap is to the right of center, return leftmost point
        return end_i


    def find_deepest_point(self, start_i, end_i):
        """
        pick furthest point in the identified gap ^ from there
        """
        # return index of the best point
        return np.argmax(self.ranges[start_i:end_i + 1]) + start_i
    

    def find_deepest_then_middle_point(self, start_i, end_i):
        deepest_value = max(self.ranges[start_i:end_i + 1])

        # find number of points that are equal to deepest value
        num_at_max = 0
        for i in range(start_i, end_i+1):
            if self.ranges[i] == deepest_value:
                num_at_max += 1

        middle_index = int(num_at_max / 2) + 1 # middle point but round up

        # go to the nth instance of deepest point, where n is in the middle
        cur_index = 0
        for i in range(start_i, end_i+1):
            if self.ranges[i] == deepest_value:
                cur_index += 1
                if cur_index == middle_index:
                    return i
                



    def find_widest_gap(self):
        """
        find widest gap
        """
        valid_gaps = self.get_gaps()

        if not valid_gaps:
            rospy.logwarn("No valid gaps detected")
            return 0, len(self.ranges) - 1  # default to full range if no valid gaps

        # find largest gap based on its length, then return start and end indices of the largest gap
        largest_gap = max(valid_gaps, key=lambda x : x[-1] - x[0])

        return largest_gap[0], largest_gap[-1]
    
    def find_deepest_then_widest_then_left_gap(self):
        valid_gaps = self.get_gaps()
        center_index = self.get_index_of(90)

        if not valid_gaps:
            rospy.logwarn("No valid gaps detected")
            raise ValueError()
            return 0, len(self.ranges) - 1  # default to full range if no valid gaps

        def sort_key(x):
            gap = self.ranges[x[0]: x[-1]+1]
            return (max(gap), len(gap) - (len(gap) % 50), -abs(center_index-gap[0]))
        
        sorted_gaps = sorted(valid_gaps, key=sort_key)

        # find largest gap based on its length, then return start and end indices of the largest gap
        largest_gap = sorted_gaps[-1]

        return largest_gap[0], largest_gap[-1]



    def find_deepest_gap(self):
        """
        find deepest gap
        """
        valid_gaps = self.get_gaps()

        if not valid_gaps:
            rospy.logwarn("No valid gaps detected")
            return 0, len(self.ranges) - 1  # default to full range if no valid gaps

        # find largest gap based on its length, then return start and end indices of the largest gap
        largest_gap = max(valid_gaps, key=lambda x : max(self.ranges[x[0]: x[-1]]))
        return largest_gap[0], largest_gap[-1]
    
    def find_deepest_favor_left_gap(self):
        """
        find deepest gap
        """
        valid_gaps = self.get_gaps()

        if not valid_gaps:
            rospy.logwarn("No valid gaps detected")
            return 0, len(self.ranges) - 1  # default to full range if no valid gaps

        # find largest gap based on its length, then return start and end indices of the largest gap
        deepest_gap = max(valid_gaps, key=lambda x : max(self.ranges[x[0]: x[-1]]))
        deepest_point = max(self.ranges[deepest_gap[0]:deepest_gap[-1]])
        # Filter to only gaps that have a deepest point within 0.2 meters of the deepest point
        potential_gaps = list(filter(lambda x : deepest_point - max(self.ranges[x[0]: x[-1]]) < 0.2, valid_gaps))
        largest_gap = max(potential_gaps, key=lambda x : x[-1])
        return largest_gap[0], largest_gap[-1]

    
    def find_largest_integral_gap(self):
        """
        find deepest gap
        """
        valid_gaps = self.get_gaps()

        if not valid_gaps:
            rospy.logwarn("No valid gaps detected")
            return 0, len(self.ranges) - 1  # default to full range if no valid gaps

        # find largest gap based on its length, then return start and end indices of the largest gap
        largest_gap = max(valid_gaps, key=lambda x : sum(self.ranges[x[0]: x[-1]]))
        return largest_gap[0], largest_gap[-1]


    def filter_gaps(self, gaps):
        min_index = self.get_index_of(0)
        max_index = self.get_index_of(180)
        valid_gaps = []
        for gap in gaps:
            if len(gap) <= self.min_gap_size or gap[-1] < min_index or gap[0] > max_index:
                continue
            if gap[0] < min_index:
                gap[0] = min_index
            if gap[-1] > max_index:
                gap[-1] = max_index
            valid_gaps.append(gap)
        return valid_gaps


    def find_least_steering_gap(self):
        """
        Find the gap that requires the least steering (most aligned with car's forward direction)
        """
        valid_gaps = self.get_gaps()

        if not valid_gaps:
            rospy.logwarn("No valid gaps detected")
            return 0, len(self.ranges) - 1  # default to full range if no valid gaps

        
        # Calculate the center index (represents straight ahead)
        center_index = self.get_index_of(90)
        
        # Find the gap with center closest to the car's forward direction
        min_distance_to_center = float('inf')
        best_gap = valid_gaps[0]
        
        for gap in valid_gaps:
            gap_center = (gap[0] + gap[-1]) // 2
            distance_to_center = abs(gap_center - center_index)
            
            if distance_to_center < min_distance_to_center:
                min_distance_to_center = distance_to_center
                best_gap = gap
        
        return best_gap[0], best_gap[-1]
    
    def get_gaps(self):
        too_close = np.where(self.ranges < self.min_gap_distance)[0]
        
        # Split into gaps
        gaps = np.split(np.arange(len(self.ranges)), too_close)
        
        # Filter out small gaps
        valid_gaps = self.filter_gaps(gaps)

        #checking if there are valid gaps in the raceline
        valid_gaps_in_raceline = []
        for gap in valid_gaps:
            if self.is_gap_in_raceline(gap):
                valid_gaps_in_raceline.append(gap)

        if not valid_gaps_in_raceline:
            rospy.logwarn("No valid gaps detected in the raceline, switching to cc.")
            return 0, len(self.ranges) - 1  # full range for cc

        return valid_gaps_in_raceline
    
    def is_gap_in_raceline(self, gap):
        start_idx, end_idx = gap  
        for raceline in self.racelines:
            if start_idx >= raceline[0] and end_idx <= raceline[1]:
                return True
        return False
    
    def get_index_of(self, degrees):
        angle_rad = math.radians(degrees)
        angle_min = -(self.data.angle_min % math.pi)
        index = int((angle_rad - angle_min) / self.data.angle_increment)
        return index

