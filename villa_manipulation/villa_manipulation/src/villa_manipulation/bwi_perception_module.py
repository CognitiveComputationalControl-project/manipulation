import rospy
import sensor_msgs.point_cloud2 as pc2

import math
import tf
import tf2_ros
import geometry_msgs.msg
import tf2_geometry_msgs

from roslib import message
from bwi_tabletop_perception.srv import TabletopPerception
from bwi_tabletop_perception.srv import TabletopPerceptionRequest

import operator


class DetectedObject:
    def __init__(self, x, y, z, w, l, h, bbox=None, label=''):
        self.x = x
        self.y = y
        self.z = z
        self.width = w
        self.length = l
        self.height = h
        self.bbox = bbox
        self.label = label


class BWISegmentation:
    def __init__(self):
        self.service_client = rospy.ServiceProxy('tabletop_object_detection_service', TabletopPerception)
        self.service_client.wait_for_service(timeout=1.0)

    def service_call(self, table):
        req = TabletopPerceptionRequest()
        req.override_filter_z = True
        req.filter_z_value = 2.0
        req.table_height = -0.01  # TODO: Set the filter based on the detected table height

        try:
            res = self.service_client(req)
        # A bug in the service causes it to crash instead of returning no plane
        except rospy.service.ServiceException as e:
            rospy.logerr("Tabletop segmentation threw an exception")
            rospy.logerr(e)
            return None
        num_objects = 0
        objects = {}

        if not res.is_plane_found:
            return None
        max_table_dim = max(table.scale.x, table.scale.y)
        half_max_dim = max_table_dim / 2
        for box in res.tabletop_AA_bounding_box:
            xdim = box.scale.x
            ydim = box.scale.y
            zdim = box.scale.z
            x = box.pose.position.x
            y = box.pose.position.y
            z = box.pose.position.z
            if abs(x) > half_max_dim or abs(y) > half_max_dim or z < 0:
                rospy.loginfo("Ignoring object that doesn't fit on previously detected table...")
                continue
            name = "obj" + str(num_objects)
            num_objects += 1
            objects[name] = DetectedObject(x, y, z, xdim, ydim, zdim, box)
        return objects


if __name__ == "__main__":
    rospy.init_node('bwi_tabletop_segmentation')
    segmentor = BWISegmentation()
    segmentor.service_call()
