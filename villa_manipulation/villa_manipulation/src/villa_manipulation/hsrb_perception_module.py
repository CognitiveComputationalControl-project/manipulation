
import rospy
import sensor_msgs.point_cloud2 as pc2

import math
import tf
import tf2_ros
import geometry_msgs.msg
import tf2_geometry_msgs

from roslib import message
from tmc_tabletop_segmentator.srv import TabletopSegmentation
from tmc_tabletop_segmentator.srv import TabletopSegmentationRequest

import operator


class detectedObject:
    
    def __init__(self,x,y,z,w,l,h,xlist,ylist,zlist):
        self.x = x
        self.y = y
        self.z = z
        self.width = w
        self.length = l
        self.height = h
        #self.cloud = cloud
        self.xlist = xlist
        self.ylist = ylist
        self.zlist = zlist
            

class hsrb_segmentation:  

    def __init__(self):

        self.service_client = rospy.ServiceProxy('/tabletop_segmentator_node/execute', TabletopSegmentation)
        self.service_client.wait_for_service(timeout=1.0)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        
        self.req = TabletopSegmentationRequest()
        self.req.crop_enabled = True  # limit the processing area
        self.req.crop_x_max = 0.8     # X coordinate maximum value in the area
        self.req.crop_x_min = -0.8    # X coordinate minimum value in the area
        self.req.crop_y_max = 0.7     # Y coordinate maximum value in the area
        self.req.crop_y_min = -0.7    # Y coordinate minimum value in the area
        self.req.crop_z_max = 1.8     # Z coordinate maximum value in the area
        self.req.crop_z_min = 0.5     # Z coordinate minimum value in the area
        self.req.cluster_z_max = 0.5  # maximum height value of cluster on table
        self.req.cluster_z_min = 0.05  # minimum height value of cluster on table
        self.req.remove_bg = True    # remove the background of the segment image
        #self.table = {}
        self.objects = {}
        self.tb_ct = 0
        self.obj_ct = 0
        self.largest_table = None
        # self.nearestObject = []

    '''def clearTables(self):
        self.table.clear()
        self.tb_ct = 0'''
        
    def clearObjects(self):
        self.objects.clear()
        self.obj_ct = 0
        
    def post_process(self):
        merge_list = []
        for i in range(len(self.objects)):
            obj0 = self.objects["obj"+str(i)]
            for j in range(i+1,len(self.objects)):
                obj1 = self.objects["obj"+str(j)]
                dist = math.sqrt((obj0.x-obj1.x)**2+(obj0.y-obj1.y)**2+(obj0.z-obj1.z)**2)
                if dist < 0.1 and abs(obj0.z-obj1.z) - obj0.height/2 - obj1.height/2 < 0.05:
                    if obj0.z > obj1.z:
                        merge_list.append((i,j))
                    else:
                        merge_list.append((j,i))
        
        for pair in merge_list:
            print "merging", pair
        
            x_list = list(self.objects["obj"+str(pair[0])].xlist)
            x_list.extend(self.objects["obj"+str(pair[1])].xlist)
            y_list = list(self.objects["obj"+str(pair[0])].ylist)
            y_list.extend(self.objects["obj"+str(pair[1])].ylist)
            z_list = list(self.objects["obj"+str(pair[0])].zlist)
            z_list.extend(self.objects["obj"+str(pair[1])].zlist)
            x_list.sort()
            y_list.sort()
            z_list.sort()
            
            i=len(x_list)
                        
            x = sum(x_list)/i
            y = sum(y_list)/i
            z = sum(z_list)/i 
         
            xdim = x_list[-2] - x_list[1]
            ydim = y_list[-2] - y_list[1]
            zdim = z_list[-2] - z_list[1]
            
            name = "obj"+str(self.obj_ct)
            self.obj_ct += 1
            
            self.objects[name] = detectedObject(x,y,z,xdim,ydim,zdim,x_list,y_list,z_list)
            del self.objects["obj"+str(pair[0])]
            del self.objects["obj"+str(pair[1])]
            
        # Nearest Object
        obj_dict = {}
        for obj in self.objects:
            obj_dict[obj] = self.objects[obj].width*self.objects[obj].length*self.objects[obj].height
        self.sorted_objects = sorted(obj_dict.items(), key=operator.itemgetter(1))


    def service_call(self, table, table_width, table_length, process=False ):
    
        res = self.service_client(self.req)
        rospy.loginfo('Number of detected objects={0}'.format(len(res.segmented_objects_array.table_objects_array)))
        rospy.loginfo('Number of detected planes={0}'.format(len(res.table_array.tables)))
        
        transform = self.tfBuffer.lookup_transform('map','head_rgbd_sensor_rgb_frame',rospy.Time(0), rospy.Duration(1.0))
        # objects
        
        for obj in res.segmented_objects_array.table_objects_array:
            for cloud in obj.points_array:
                #print type(cloud)
                gen = pc2.read_points(cloud, skip_nans=True, field_names=("x","y","z"))        
                i = 0
                x_list = []
                y_list = []
                z_list = []
                for g in gen:
                    if g[0] != 0 or g[1] != 0 or g[2] != 0:
                        i += 1
                        
                        ps = geometry_msgs.msg.PoseStamped()
                        ps.header.stamp = rospy.Time.now()
                        ps.header.frame_id = 'head_rgbd_sensor_rgb_frame'
                        ps.pose.position.x = g[0]
                        ps.pose.position.y = g[1]
                        ps.pose.position.z = g[2]
                        
                        pt = tf2_geometry_msgs.do_transform_pose(ps, transform)
                        pt.header.stamp = rospy.Time.now()
                        pt.header.frame_id = 'map'
                
                        if pt.pose.position.z < table.pose.position.z + 0.02: #too low
                            continue
                            
                        x_list.append(pt.pose.position.x)
                        y_list.append(pt.pose.position.y)
                        z_list.append(pt.pose.position.z)
                
                if len(x_list) < 10:
                    continue
                x = sum(x_list)/i
                y = sum(y_list)/i
                z = sum(z_list)/i 
                if x > table.pose.position.x+table_width: #too far
                    continue
                if y > table.pose.position.y+table_length: #too far
                    continue
                    
                if z < table.pose.position.z: #too low
                    continue
                xdim = max(x_list)-min(x_list)
                ydim = max(y_list)-min(y_list)
                zdim = max(z_list)-min(z_list)
                if z - zdim/2 > table.pose.position.z + 0.1: #too high
                    continue
                
                name = "obj"+str(self.obj_ct)
                self.obj_ct += 1
                
                self.objects[name] = detectedObject(x,y,z,xdim,ydim,zdim,x_list,y_list,z_list)


        if process:
            self.post_process()

    def service_call2(self, process=False ):
        res = self.service_client(self.req)
        rospy.loginfo('Number of detected objects={0}'.format(len(res.segmented_objects_array.table_objects_array)))
        rospy.loginfo('Number of detected planes={0}'.format(len(res.table_array.tables)))
        
        transform = self.tfBuffer.lookup_transform('map','head_rgbd_sensor_rgb_frame',rospy.Time(0), rospy.Duration(1.0))
        # objects
        
        for obj in res.segmented_objects_array.table_objects_array:
            for cloud in obj.points_array:
                #print type(cloud)
                gen = pc2.read_points(cloud, skip_nans=True, field_names=("x","y","z"))        
                i = 0
                x_list = []
                y_list = []
                z_list = []
                for g in gen:
                    if g[0] != 0 or g[1] != 0 or g[2] != 0:
                        i += 1
                        
                        ps = geometry_msgs.msg.PoseStamped()
                        ps.header.stamp = rospy.Time.now()
                        ps.header.frame_id = 'head_rgbd_sensor_rgb_frame'
                        ps.pose.position.x = g[0]
                        ps.pose.position.y = g[1]
                        ps.pose.position.z = g[2]
                        
                        pt = tf2_geometry_msgs.do_transform_pose(ps, transform)
                        pt.header.stamp = rospy.Time.now()
                        pt.header.frame_id = 'map'
                        
                        x_list.append(pt.pose.position.x)
                        y_list.append(pt.pose.position.y)
                        z_list.append(pt.pose.position.z)
                
                x = sum(x_list)/i
                y = sum(y_list)/i
                z = sum(z_list)/i 
                
                xdim = max(x_list)-min(x_list)
                ydim = max(y_list)-min(y_list)
                zdim = max(z_list)-min(z_list)
                
                name = "obj"+str(self.obj_ct)
                self.obj_ct += 1
                
                self.objects[name] = detectedObject(x,y,z,xdim,ydim,zdim,x_list,y_list,z_list)
                print name, x,y,z


        if process:
            self.post_process()

if __name__ == "__main__":
    rospy.init_node('hsrb_tabletop_segmentation')
    segmentor = hsrb_segmentation()
    segmentor.service_call2(process=True)
    
