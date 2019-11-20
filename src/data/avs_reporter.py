#!/usr/bin/env python

import random
import time
import numpy as np
import math

# TODO: change time to frame #
def get_distance(a,b):
    if type(a) == type(np.array([0,0])):
        return math.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2)
    target = np.array([a.x-b.x,a.y-b.y])
    norm = np.linalg.norm(target)
    return norm

class AVSReporter:
    # Wrote as class to save computations
    def __init__(self,time,v,mapp,actor_list,interval,fov):
        self.time = float(time)
        self.interval = float(interval)
        self.fov = float(fov)
        self._v = v
        self._v_loc = self._v.get_location()
        self._v_trans = self._v.get_transform()
        self._v_orientation = self._v.get_transform().rotation.yaw
        self._v_forward_vector = np.array([math.cos(math.radians(self._v_orientation)), 
                                           math.sin(math.radians(self._v_orientation))])
        self._v_real_vector = np.array([self._v.get_location().x,self._v.get_location().y])

        self._v_wp = mapp.get_waypoint(self._v_loc,project_to_road=True)
        self._v_wp_orientation = self._v_wp.transform.rotation.yaw
        self._v_wp_forward_vector = np.array([math.cos(math.radians(self._v_wp_orientation)), 
                                              math.sin(math.radians(self._v_wp_orientation))])
        self._v_wp_real_vector = np.array([self._v_wp.transform.location.x,self._v_wp.transform.location.y])
        self._d_distance = get_distance(self._v_real_vector,self._v_wp_real_vector)

        # m/s
        self._velocity = math.sqrt(self._v.get_velocity().x**2 + self._v.get_velocity().y**2 + 
                                   self._v.get_velocity().z**2)
        # is_intersection -> is_junction in 0.9.6
        # self._inJunction = self._v_wp.is_junction
        self._inInter = self._v_wp.is_intersection

        self._IN_LANE, self._ON_MARKING = self._which_system()
        self._angle = self._get_angle()
        self._toMarking_ML = self._get_toMarking_ML()
        self._toMarking_MR = self._get_toMarking_MR()
        self._toMarking_LL = self._get_toMarking_LL()
        self._toMarking_RR = self._get_toMarking_RR()
        self._toMarking_M = self._get_toMarking_M()
        self._toMarking_L = self._get_toMarking_L()
        self._toMarking_R = self._get_toMarking_R()
        self._dist_MM, self._dist_LL, self._dist_RR, self._dist_L, self._dist_R = self._get_dists(actor_list,mapp)
        self._lanes = self._count_lanes()

    def _update(self,v,mapp,actor_list):
        self._v = v
        self._v_loc = self._v.get_location()
        self._v_trans = self._v.get_transform()
        self._v_orientation = self._v.get_transform().rotation.yaw
        self._v_forward_vector = np.array([math.cos(math.radians(self._v_orientation)), 
                                           math.sin(math.radians(self._v_orientation))])
        self._v_real_vector = np.array([self._v.get_location().x,self._v.get_location().y])

        self._v_wp = mapp.get_waypoint(self._v_loc,project_to_road=True)
        self._v_wp_orientation = self._v_wp.transform.rotation.yaw
        self._v_wp_forward_vector = np.array([math.cos(math.radians(self._v_wp_orientation)), 
                                              math.sin(math.radians(self._v_wp_orientation))])
        self._v_wp_real_vector = np.array([self._v_wp.transform.location.x,self._v_wp.transform.location.y])
        self._d_distance = get_distance(self._v_real_vector,self._v_wp_real_vector)

        #self._velocity = self._v.get_velocity()
        #m/s
        self._velocity = math.sqrt(self._v.get_velocity().x**2 + self._v.get_velocity().y**2 + 
                                   self._v.get_velocity().z**2)
        # is_intersection -> is_junction in 0.9.6
        # self._inJunction = self._v_wp.is_junction
        self._inInter = self._v_wp.is_intersection

        self._IN_LANE, self._ON_MARKING = self._which_system()
        self._angle = self._get_angle()
        self._toMarking_ML = self._get_toMarking_ML()
        self._toMarking_MR = self._get_toMarking_MR()
        self._toMarking_LL = self._get_toMarking_LL()
        self._toMarking_RR = self._get_toMarking_RR()
        self._toMarking_M = self._get_toMarking_M()
        self._toMarking_L = self._get_toMarking_L()
        self._toMarking_R = self._get_toMarking_R()
        self._dist_MM, self._dist_LL, self._dist_RR, self._dist_L, self._dist_R = self._get_dists(actor_list,mapp)
        self.time += self.interval
        self._lanes = self._count_lanes()

    def _count_half_lanes(self,curr,dirx):
        lanes = 0
        if dirx == 'left':
            if not (curr.get_left_lane() and curr.get_left_lane().lane_id * curr.lane_id > 0):
                return 0
            else:
                return 1 + self._count_half_lanes(curr.get_left_lane(),'left')
        elif dirx == 'right':
            if not (curr.get_right_lane() and curr.get_right_lane().lane_id * curr.lane_id > 0):
                return 0
            else:
                return 1 + self._count_half_lanes(curr.get_right_lane(),'right')
    
    def _count_lanes(self):
        lanes = 0
        lanes += self._count_half_lanes(self._v_wp,'left')
        lanes += self._count_half_lanes(self._v_wp,'right')
        return lanes

    def _on_right(self,v_real_vector,v_wp_real_vector,v_wp_forward_vector):
        v_to_wp = v_wp_real_vector - v_real_vector
        v_dif = np.cross(v_to_wp, v_wp_forward_vector)
        if v_dif >= 0:
            return True
        else:
            return False

    def _get_angle(self):
        try:
            _angle = math.degrees(math.acos(np.dot(self._v_forward_vector, self._v_wp_forward_vector) / 
                            (np.linalg.norm(self._v_forward_vector)*np.linalg.norm(self._v_wp_forward_vector))))
        except ValueError as e:
            print(e)
            _angle = 0.0
        return _angle

    def _which_system(self):
        # v on the right
        if self._on_right(self._v_real_vector, self._v_wp_real_vector, self._v_wp_forward_vector):
            toMarkingML = 0.5*self._v_wp.lane_width + self._d_distance
            toMarkingMR = 0.5*self._v_wp.lane_width - self._d_distance
            
        else:
            toMarkingML = 0.5*self._v_wp.lane_width + self._d_distance
            toMarkingMR = 0.5*self._v_wp.lane_width - self._d_distance

        if 0.5 < toMarkingML < 1.2:
            _IN_LANE = True
            _ON_MARKING = True
        if 0.5 < toMarkingMR < 1.2:
            _IN_LANE = True
            _ON_MARKING = True
        if toMarkingML > 1.2 and toMarkingMR > 1.2:
            _IN_LANE = True
            _ON_MARKING = False
        if toMarkingML < 0.5 or toMarkingMR < 0.5:
            _IN_LANE = False
            _ON_MARKING = True
        return _IN_LANE, _ON_MARKING

    def _get_toMarking_ML(self):
        if self._IN_LANE:
            # v on the right
            if self._on_right(self._v_real_vector, self._v_wp_real_vector, self._v_wp_forward_vector):
                _toMarking_ML = 0.5*self._v_wp.lane_width + self._d_distance
            else:
                _toMarking_ML = 0.5*self._v_wp.lane_width - self._d_distance
        else:
            _toMarking_ML = None
        return _toMarking_ML

    def _get_toMarking_MR(self):
        if self._IN_LANE:
            # v on the right
            if self._on_right(self._v_real_vector, self._v_wp_real_vector, self._v_wp_forward_vector):
                _toMarking_MR = 0.5*self._v_wp.lane_width - self._d_distance
            else:
                _toMarking_MR = 0.5*self._v_wp.lane_width + self._d_distance
        else:
            _toMarking_MR = None
        return _toMarking_MR

    def _get_toMarking_LL(self):
        #!! Need to consider if adjacent lane is not car lane
        v_wp_l = self._v_wp.get_left_lane()
        if v_wp_l == None or self._IN_LANE == False:
            _toMarking_LL = None
        else: 
            # v on the right
            if self._on_right(self._v_real_vector, self._v_wp_real_vector, self._v_wp_forward_vector):
                _toMarking_LL = 0.5*self._v_wp.lane_width + v_wp_l.lane_width + self._d_distance
            # left
            else:
                _toMarking_LL = 0.5*self._v_wp.lane_width + v_wp_l.lane_width - self._d_distance
        return _toMarking_LL

    def _get_toMarking_RR(self):
        #!! Need to consider if adjacent lane is not car lane
        v_wp_r = self._v_wp.get_right_lane()
        if v_wp_r == None or self._IN_LANE == False:
            _toMarking_RR = None
        else: 
            # v on the right
            if self._on_right(self._v_real_vector, self._v_wp_real_vector, self._v_wp_forward_vector):
                _toMarking_RR = 0.5*self._v_wp.lane_width + v_wp_r.lane_width - self._d_distance
            # left
            else:
                _toMarking_RR = 0.5*self._v_wp.lane_width + v_wp_r.lane_width + self._d_distance
        return _toMarking_RR

    def _get_toMarking_M(self):
        if self._ON_MARKING:
            _toMarking_M = 0.5*self._v_wp.lane_width - self._d_distance
        else:
            _toMarking_M = None
        return _toMarking_M

    def _get_toMarking_L(self):
        if self._ON_MARKING:
            if self._on_right(self._v_real_vector, self._v_wp_real_vector, self._v_wp_forward_vector):
                _toMarking_L = 0.5*self._v_wp.lane_width + self._d_distance
            else:
                v_wp_l = self._v_wp.get_left_lane()
                if v_wp_l == None:
                    _toMarking_L = None
                else: 
                    _toMarking_L = 0.5*self._v_wp.lane_width + v_wp_l.lane_width - self._d_distance
        else:
            _toMarking_L = None
        return _toMarking_L
    
    def _get_toMarking_R(self):
        if self._ON_MARKING:
            if self._on_right(self._v_real_vector, self._v_wp_real_vector, self._v_wp_forward_vector):
                v_wp_r = self._v_wp.get_right_lane()
                if v_wp_r == None:
                    _toMarking_R = None
                else: 
                    _toMarking_R = 0.5*self._v_wp.lane_width + v_wp_r.lane_width - self._d_distance

            else:
                _toMarking_R = 0.5*self._v_wp.lane_width + self._d_distance

        else:
            _toMarking_R = None
        return _toMarking_R

    def _preceding_dist(self,a):
        a_loc = a.get_location()
        target_vector = np.array([a_loc.x - self._v_loc.x, a_loc.y - self._v_loc.y])
        norm_target = np.linalg.norm(target_vector)
        try:
            d_angle = math.degrees(math.acos(np.dot(self._v_forward_vector, target_vector) / norm_target))
        except ValueError as e:
            # !! Make sure what causes the error
            print(e)
            d_angle = 0.0
        if d_angle < self.fov/2.0:
            return norm_target
        else:
            return False

    def _get_dists(self,actor_list,mapp): 
        MM_precedings = []
        RR_precedings = []
        LL_precedings = []
        L_precedings = []
        R_precedings = []
        _dist_MM = None
        _dist_LL = None
        _dist_RR = None
        _dist_L = None
        _dist_R = None

        for a in actor_list:
            if a.type_id.split(".")[0] == "vehicle":
                a_wp = mapp.get_waypoint(a.get_location(),project_to_road=True)
                # Don't need to count vehicles in opposite direction
                if (a_wp.lane_id * self._v_wp.lane_id > 0):
                    if self._IN_LANE:
                        if a_wp.lane_id == self._v_wp.lane_id:
                            dist = self._preceding_dist(a)
                            if dist:
                                MM_precedings.append((dist,a))

                        v_wp_r = self._v_wp.get_right_lane()
                        if v_wp_r == None:
                            _dist_RR = None
                        elif a_wp.lane_id == v_wp_r.lane_id:
                            dist = self._preceding_dist(a)
                            if dist:
                                RR_precedings.append((dist,a))
                            
                        v_wp_l = self._v_wp.get_left_lane()
                        if v_wp_l == None:
                            _dist_LL = None
                        elif a_wp.lane_id == v_wp_l.lane_id:
                            dist = self._preceding_dist(a)
                            if dist:
                                LL_precedings.append((dist,a))

                        if MM_precedings:
                            diag, nearest = sorted(MM_precedings,key=lambda x: x[0])[0]
                            horizontal =  get_distance(self._v_wp_real_vector, self._v_real_vector)
                            _dist_MM = math.sqrt(abs(diag**2-horizontal**2)) 
                        else: _dist_MM = None
                        if RR_precedings:
                            diag, nearest = sorted(RR_precedings,key=lambda x: x[0])[0]
                            v_wp_r = self._v_wp.get_right_lane()
                            v_wp_r_real_vector = np.array([v_wp_r.transform.location.x,v_wp_r.transform.location.y])
                            horizontal =  get_distance(v_wp_r_real_vector, self._v_real_vector)
                            _dist_RR = math.sqrt(abs(diag**2-horizontal**2)) 
                        else: _dist_RR = None
                        if LL_precedings:
                            diag, nearest = sorted(LL_precedings,key=lambda x: x[0])[0]
                            v_wp_l = self._v_wp.get_left_lane()
                            v_wp_l_real_vector = np.array([v_wp_l.transform.location.x,v_wp_l.transform.location.y])
                            horizontal =  get_distance(v_wp_l_real_vector, self._v_real_vector)
                            _dist_LL = math.sqrt(abs(diag**2-horizontal**2)) 
                        else: _dist_LL = None

                    if self._ON_MARKING:
                        if self._on_right(self._v_real_vector, self._v_wp_real_vector, self._v_wp_forward_vector):
                            if a_wp.lane_id == self._v_wp.lane_id:
                                dist = self._preceding_dist(a)
                                if dist:
                                    L_precedings.append((dist,a))
                                    
                            v_wp_r = self._v_wp.get_right_lane()
                            if v_wp_r == None:
                                _dist_R = None
                            elif a_wp.lane_id == v_wp_r.lane_id:
                                dist = self._preceding_dist(a)
                                if dist:
                                    R_precedings.append((dist,a))

                            if L_precedings:
                                diag, nearest = sorted(L_precedings,key=lambda x: x[0])[0]
                                horizontal =  get_distance(self._v_wp_real_vector, self._v_real_vector)
                                _dist_L = math.sqrt(abs(diag**2-horizontal**2))
                            else: _dist_L = None

                            if R_precedings:
                                diag, nearest = sorted(R_precedings,key=lambda x: x[0])[0]
                                v_wp_r = self._v_wp.get_right_lane()
                                v_wp_r_real_vector = np.array([v_wp_r.transform.location.x,v_wp_r.transform.location.y])
                                horizontal =  get_distance(v_wp_r_real_vector, self._v_real_vector)
                                _dist_R = math.sqrt(abs(diag**2-horizontal**2)) 
                            else: _dist_R = None

                        else:
                            if a_wp.lane_id == self._v_wp.lane_id:
                                dist = self._preceding_dist(a)
                                if dist:
                                    R_precedings.append((dist,a))

                            v_wp_l = self._v_wp.get_left_lane()
                            if v_wp_l == None:
                                _dist_L = None
                            elif a_wp.lane_id == v_wp_l.lane_id:
                                dist = self._preceding_dist(a)
                                if dist:
                                    L_precedings.append((dist,a))

                            if R_precedings:
                                diag, nearest = sorted(R_precedings,key=lambda x: x[0])[0]
                                horizontal =  get_distance(self._v_wp_real_vector, self._v_real_vector)
                                _dist_R = math.sqrt(abs(diag**2-horizontal**2))
                            else: _dist_R = None

                            if L_precedings:
                                diag, nearest = sorted(L_precedings,key=lambda x: x[0])[0]
                                v_wp_l = self._v_wp.get_left_lane()
                                v_wp_l_real_vector = np.array([v_wp_l.transform.location.x,v_wp_l.transform.location.y])
                                horizontal =  get_distance(v_wp_l_real_vector, self._v_real_vector)
                                _dist_L = math.sqrt(abs(diag**2-horizontal**2)) 
                            else: _dist_L = None

        res = [_dist_MM, _dist_LL, _dist_RR, _dist_L, _dist_R]
        return res


    def report(self,v,mapp,actor_list):
        avs = []
        report = ""
        report += "Time({})\n".format(self.time)
        report += "Vehicle {}\n".format(self._v_trans)
        report += "  Nearest {}\n".format(self._v_wp)
        report += "  Lane(Width({}), id({}))\n".format(self._v_wp.lane_width,self._v_wp.lane_id)
        report += "  Lane Marking Width(Left({}), Right({}))\n".format(self._v_wp.left_lane_marking.width,self._v_wp.right_lane_marking.width)
        report += "  In Lane:{}; On Marking:{}\n".format(self._IN_LANE, self._ON_MARKING)
        report += "  angle: {}\n".format(self._angle)
        report += "  toMarking_L: {}\n".format(self._toMarking_L)
        report += "  toMarking_M: {}\n".format(self._toMarking_M)
        report += "  toMarking_R: {}\n".format(self._toMarking_R)
        report += "  dist_L: {}\n".format(self._dist_L)
        report += "  dist_R: {}\n".format(self._dist_R)
        report += "  toMarking_LL: {}\n".format(self._toMarking_LL)
        report += "  toMarking_ML: {}\n".format(self._toMarking_ML)
        report += "  toMarking_MR: {}\n".format(self._toMarking_MR)
        report += "  toMarking_RR: {}\n".format(self._toMarking_RR)
        report += "  dist_LL: {}\n".format(self._dist_LL)
        report += "  dist_MM: {}\n".format(self._dist_MM)
        report += "  dist_RR: {}\n".format(self._dist_RR)
        report += "  velocity(m/s): {}\n".format(self._velocity)
        report += "  In intersection: {}\n".format(1 if self._inInter else 0)
        report += "  Lanes: {}\n".format(self._lanes)

        
        avs.append(self._angle)
        avs.append(self._toMarking_L)
        avs.append(self._toMarking_M)
        avs.append(self._toMarking_R)
        avs.append(self._dist_L)
        avs.append(self._dist_R)
        avs.append(self._toMarking_LL)
        avs.append(self._toMarking_ML)
        avs.append(self._toMarking_MR)
        avs.append(self._toMarking_RR)
        avs.append(self._dist_LL)
        avs.append(self._dist_MM)
        avs.append(self._dist_RR)
        avs.append(self._velocity)
        avs.append(1 if self._inInter else 0)
        avs.append(self._lanes)
        
        self._update(v,mapp,actor_list)
        
        return report, avs

