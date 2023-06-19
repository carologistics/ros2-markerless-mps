import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
import math
import signal

from tf_transformations import euler_from_quaternion, quaternion_from_euler
qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    depth=1
)


class TFSniffer(Node):
    
    def __init__(self):
        super().__init__('tf_sniffer')

        self.min_count = 6
        self.min_count_laser = 30
        self.tf_buffer = tf2_ros.Buffer(rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        #transform publisher
        self.tf_broadcaster = TransformBroadcaster(self)


        self.mps_magenta_arr = [[0 for j in range(9)] for i in range(8)]
        self.mps_cyan_arr = [[0 for j in range(9)] for i in range(8)]
        
        self.orientation_magenta_arr = [[[0 for k in range(4)]  for k in range(9)] for i in range(8)]
        self.orientation_cyan_arr = [[[0 for k in range(4)]  for k in range(9)] for i in range(8)]

        self.classes_magenta_arr = [[[0 for k in range(5)] for j in range(9)] for i in range(8)]
        self.classes_type_cyan_arr = [[[0 for k in range(5)] for j in range(9)] for i in range(8)]
        
        self.tl_magenta_arr = [[[0.0 for k in range(3)] for j in range(9)] for i in range(8)]
        self.tl_cyan_arr = [[[0.0 for k in range(3)] for j in range(9)] for i in range(8)]
        
        self.output_magenta = [[['   ' for k in range(2)]  for k in range(9)] for i in range(8)]
        self.output_cyan = [[['   ' for k in range(2)]  for k in range(9)] for i in range(8)]

        self.classes = ['SS', 'RS', 'CS', 'DS', 'BS']
        self.classes3 = [' SS', ' RS', ' CS', ' DS', ' BS']
        self.orientation = ['  0', ' 45', ' 90', '135']
        self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10)
        #call ros timer every 5 seconds
        backgroud_task_period = 2.0
        check_task = self.create_timer(backgroud_task_period, self.timer_callback)

    def get_position_in_rcll(self, transform):
        x, y = 0, 0
        if (transform.transform.translation.x > 0):
            x = int(transform.transform.translation.x) + 1
        else:
            x = int(transform.transform.translation.x) - 1
        if transform.transform.translation.y > 0:
            y = int(transform.transform.translation.y) + 1
        return x, y

    def get_real_rotation_magenta(self, x, y, cls, rot):
        
        if(self.tl_magenta_arr[x][y][0] > self.min_count):
            xoff = self.tl_magenta_arr[x][y][1] / self.tl_magenta_arr[x][y][0]
            yoff = self.tl_magenta_arr[x][y][2] / self.tl_magenta_arr[x][y][0]
            if rot == 0:
                if yoff < 0.5:
                    if cls == ' SS':
                        return '  0'
                    return '180'
                if cls == ' SS':
                    return '180'
                return '  0'
            if rot == 2:
                if xoff < 0.5:
                    if cls == ' SS':
                        return ' 90'
                    return '270'
                if cls == ' SS':
                    return '270'
                return ' 90'
            if rot == 1:
                if (xoff + yoff) > 1.0 :
                    if cls == ' SS':
                        return '225'
                    return ' 45'
                if cls == ' SS':
                    return ' 45'
                return '225'
            if rot == 3:
                if xoff > yoff:
                    if cls == ' SS':
                        return '135'
                    return '315'
                if cls == ' SS':
                    return '315'
                return '135'
        return '   '
    
    def get_real_rotation_cyan(self, x, y, cls, rot):
       
        if(self.tl_cyan_arr[x][y][0] > self.min_count):
            xoff = self.tl_cyan_arr[x][y][1] / self.tl_cyan_arr[x][y][0]
            yoff = self.tl_cyan_arr[x][y][2] / self.tl_cyan_arr[x][y][0]
            if rot == 0:
                if yoff < 0.5:
                    if cls == ' SS':
                        return '  0'
                    return '180'
                if cls == ' SS':
                    return '180'
                return '  0'
            if rot == 2:
                if xoff > 0.5:
                    if cls == ' SS':
                        return ' 90'
                    return '270'
                if cls == ' SS':
                    return '270'
                return ' 90'
            if rot == 1:
                if (xoff + yoff) > 1.0 :
                    if cls == ' SS':
                        return '135'
                    return '315'
                if cls == ' SS':
                    return '315'
                return '135'
            if rot == 3:
                if xoff < yoff:
                    if cls == ' SS':
                        return '225'
                    return ' 45'
                if cls == ' SS':
                    return ' 45'
                return '225'
        return '   '

        
    def round_to_nearest(self, degrees):
        degrees = degrees % 180
        steps = [0, 45, 90, 135, 180]
        for i in range(len(steps)):
            if degrees < (steps[i] + 22.5):
                if i == 4:
                    return 0
                return i
        return 0
    
    def tf_callback(self, msg):
        # Check if the transform is from the MPS frame to some other frame
        #FIX: FILTER TFS THAT ARE PUBLISHED BY THIS NODE

        for transform in msg.transforms:
            if 'BOX' in transform.child_frame_id:
                continue
            if  'MPS' in transform.child_frame_id:
                #transform = self.tf_buffer.lookup_transform('map', 'MPS_0', transform.header.stamp)
                try:
                    transform = self.tf_buffer.lookup_transform(
                        'map', transform.child_frame_id, rclpy.time.Time())
                except Exception as e:
                    self.get_logger().error('Failed to lookup transform: %s' % str(e))
                    continue
                x, y = self.get_position_in_rcll(transform)
                #self.get_logger().info('Got transform: x: %d , y: %d' % (x,y))
                if x > 0 and  x < 8 and y > 0 and y < 9:
                    self.mps_cyan_arr[x][y] += 1
                elif x < 0 and x > -8 and y > 0 and y < 9:
                    self.mps_magenta_arr[x*-1][y] += 1
            
            #check if tl is in the child frame
            if 'TL' in transform.child_frame_id:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        'map', transform.child_frame_id, rclpy.time.Time())
                except Exception as e:
                    #self.get_logger().error('Failed to lookup transform: %s' % str(e))
                    continue
                x, y = self.get_position_in_rcll(transform)
                xoff = abs(transform.transform.translation.x) % 1.0
                yoff = abs(transform.transform.translation.y) % 1.0
                if x > 0 and  x < 8 and y > 0 and y < 9:
                    self.tl_cyan_arr[x][y][0] += 1.0
                    self.tl_cyan_arr[x][y][1] += xoff
                    self.tl_cyan_arr[x][y][2] += yoff
                    #self.tl_cyan_arr[x][y][1] /= self.tl_cyan_arr[x][y][0]
                    #self.tl_cyan_arr[x][y][2] /= self.tl_cyan_arr[x][y][0]
                elif x < 0 and x > -8 and y > 0 and y < 9:
                    self.tl_magenta_arr[x*-1][y][0] += 1.0
                    self.tl_magenta_arr[x*-1][y][1] += xoff
                    self.tl_magenta_arr[x*-1][y][2] += yoff
                    #self.tl_magenta_arr[x*-1][y][1] /= self.tl_magenta_arr[x*-1][y][0]
                    #self.tl_magenta_arr[x*-1][y][2] /= self.tl_magenta_arr[x*-1][y][0]
                #self.get_logger().info('Got TL transform: x: %d , y: %d, xoff: %f, yoff: %f' % (x,y,xoff,yoff))
            # check if one of the classes is in the child frame id
            for i in range(len(self.classes)):
                if self.classes[i] in transform.child_frame_id:
                    try:
                        transform = self.tf_buffer.lookup_transform(
                            'map', transform.child_frame_id, rclpy.time.Time())#transform.header.stamp)
                    except Exception as e:
                        self.get_logger().error('Failed to lookup transform: %s' % str(e))
                        continue #TODO: check if this is correct
                    x, y = self.get_position_in_rcll(transform)
                    #self.get_logger().info('Got transform: x: %d , y: %d' % (x,y))
                    if x > 0 and  x < 8 and y > 0 and y < 9:
                        #todo check if mps already found at position
                        self.classes_type_cyan_arr[x][y][i] += 1
                    elif x < 0 and x > -8 and y > 0 and y < 9:
                        #todo check if mps already found at position
                        self.classes_magenta_arr[x*-1][y][i] += 1
            
            
            # check for laser lines in tf tree
            # TODO: calculate average of endpoints of laser line 
            if 'laser_line_avg' in transform.child_frame_id:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        'map', transform.child_frame_id, rclpy.time.Time())#transform.header.stamp)
                    if 'e1' in transform.child_frame_id:
                        transform2 = self.tf_buffer.lookup_transform(
                            'map', transform.child_frame_id[:-1] + '2', rclpy.time.Time())
                    else:
                        transform2 = self.tf_buffer.lookup_transform(
                            'map', transform.child_frame_id[:-1] + '1', rclpy.time.Time())
                except Exception as e:
                    #self.get_logger().error('Failed to lookup transform: %s' % str(e))
                    continue
                #if(int(transform.transform.translation.x) != int(transform2.transform.translation.x) or int(transform.transform.translation.y) != int(transform2.transform.translation.y)):
                #    return
                
                # get average of both endpoints
                transform.transform.translation.x = (transform.transform.translation.x + transform2.transform.translation.x) / 2
                transform.transform.translation.y = (transform.transform.translation.y + transform2.transform.translation.y) / 2
                xint = int(transform.transform.translation.x)
                yint = int(transform.transform.translation.y)
                #filter for edge cases
                if xint == 0:
                    xint = 1
                if yint == 0:
                    yint = 1
                xhelp = abs(transform.transform.translation.x %  xint)
                yhelp = abs(transform.transform.translation.y % yint)
                if xhelp < 0.15 or xhelp > 0.85 or yhelp < 0.15 or  yhelp > 0.85:
                    continue
                    
                x, y = self.get_position_in_rcll(transform)
                quaternion = (
                        transform.transform.rotation.x,
                        transform.transform.rotation.y,
                        transform.transform.rotation.z,
                        transform.transform.rotation.w)
                yaw = math.degrees(euler_from_quaternion(quaternion)[2])
                if (yaw < 0):
                    yaw += 360
                yaw = self.round_to_nearest(yaw)
                #self.get_logger().info('Got Line: x: %d , y: %d, yaw: %d' % (x,y,yaw))
                if x > 0 and  x < 8 and y > 0 and y < 9:
                    #todo check if mps already found at position
                    self.orientation_cyan_arr[x][y][yaw] += 1
                elif x < 0 and x > -8 and y > 0 and y < 9:
                    #todo check if mps already found at position
                    self.orientation_magenta_arr[x*-1][y][yaw]  += 1
                    
    def publish_transforms(self):
        for x in range(1,8):
            for y in range(1,9):
                if self.output_magenta[x][y][0] != '   ':
                    #name without leading blank
                    cls = self.output_magenta[x][y][0][1:]
                    if (self.output_magenta[x][y][1] != '   '):
                        rot = int(self.output_magenta[x][y][1])
                        #publish transform
                        t = TransformStamped()
                        t.header.stamp = self.get_clock().now().to_msg()
                        t.header.frame_id = 'map'
                        t.child_frame_id = 'BOX_' + cls + '_M_' + str(x) + str(y) + '_' + str(rot)
                        t.transform.translation.x = x*(-1)+0.5
                        t.transform.translation.y = y - 0.5
                        t.transform.translation.z = 0.0
                        r = quaternion_from_euler(0, 0, math.radians(rot))
                        t.transform.rotation.x = r[0]
                        t.transform.rotation.y = r[1]
                        t.transform.rotation.z = r[2]
                        t.transform.rotation.w = r[3]
                        self.tf_broadcaster.sendTransform(t)
                        
                if self.output_cyan[x][y][0] != '   ':
                    cls = self.output_cyan[x][y][0][1:]
                    if (self.output_cyan[x][y][1] != '   '):
                        rot = int(self.output_cyan[x][y][1])
                        #publish transform
                        t = TransformStamped()
                        t.header.stamp = self.get_clock().now().to_msg()
                        t.header.frame_id = 'map'
                        t.child_frame_id = 'BOX_' + cls + '_C_' + str(x) + str(y) + '_' + str(rot)
                        t.transform.translation.x = x-0.5
                        t.transform.translation.y = y - 0.5
                        t.transform.translation.z = 0.0
                        r = quaternion_from_euler(0, 0, math.radians(rot))
                        t.transform.rotation.x = r[0]
                        t.transform.rotation.y = r[1]
                        t.transform.rotation.z = r[2]
                        t.transform.rotation.w = r[3]
                        self.tf_broadcaster.sendTransform(t)                   
                    
                
    def timer_callback(self):
        # make table for orientation
        for y in range(1,9):
            for x in range(1,8):
                class_name = ''
                max_value = 0
                for i in range(len(self.classes)):
                    # get max value and class
                    if self.classes_magenta_arr[x][y][i] > max_value:
                        max_value = self.classes_magenta_arr[x][y][i]
                        class_name = self.classes3[i]
                if max_value > self.min_count:
                    self.output_magenta[x][y][0] = class_name
                
                    max_value = 0
                    orientation = 5
                    for i in range(len(self.orientation)):
                        if self.orientation_magenta_arr[x][y][i] > max_value:
                            max_value = self.orientation_magenta_arr[x][y][i]
                            orientation = i
                    if max_value > self.min_count_laser:
                        self.output_magenta[x][y][1] = self.get_real_rotation_magenta(x, y, class_name, orientation)
                
                class_name = ''
                max_value = 0
                for i in range(len(self.classes)):
                    # get max value and class
                    if self.classes_type_cyan_arr[x][y][i] > max_value:
                        max_value = self.classes_type_cyan_arr[x][y][i]
                        class_name = self.classes3[i]
                if max_value > self.min_count:
                    self.output_cyan[x][y][0] = class_name
                    max_value = 0
                    orientation = 5
                    for i in range(len(self.orientation)):
                        if self.orientation_cyan_arr[x][y][i] > max_value:
                            max_value = self.orientation_cyan_arr[x][y][i]
                            orientation = i
                    if max_value > self.min_count_laser:
                        self.output_cyan[x][y][1] = self.get_real_rotation_cyan(x, y, class_name, orientation)
                    
        self.get_logger().info('   -7  -6  -5  -4  -3  -2  -1  1   2   3   4   5   6   7  ')
        self.get_logger().info('   --- --- --- --- --- --- --- --- --- --- --- --- --- --- ')
        for y in range(1,9):
            string1 = str(9-y) + ' |'
            string2 = '  |'
            for x in range(1,8):
                string1 += self.output_magenta[8-x][9-y][0] + '|'
                string2 += self.output_magenta[8-x][9-y][1] + '|'
            for x in range(1,8):
                string1 += self.output_cyan[x][9-y][0] + '|'
                string2 += self.output_cyan[x][9-y][1] + '|'
            self.get_logger().info(string1)
            self.get_logger().info(string2)
            self.get_logger().info('   --- --- --- --- --- --- --- --- --- --- --- --- --- --- ')
        self.publish_transforms()
            

        
        
                
        
    def shutdown_callback(self):
        for x in range(1,8):
            for y in range(1,9):
                if self.mps_magenta_arr[x][y] > 0:
                    self.get_logger().info('Magenta MPS %d %d: %d' % (x,y,self.mps_magenta_arr[x][y]))
        for x in range(1,8):
            for y in range(1,9):
                if self.mps_cyan_arr[x][y] > 0:
                    self.get_logger().info('Cyan MPS %d %d: %d' % (x,y,self.mps_cyan_arr[x][y]))

        # print out arrays in table for class with highest count
        for x in range(1,8):
            for y in range(1,9):
                class_name = ''
                max_value = 0
                for i in range(len(self.classes)):
                    # get max value and class
                    if self.classes_magenta_arr[x][y][i] > max_value:
                        max_value = self.classes_magenta_arr[x][y][i]
                        class_name = self.classes[i]
                if max_value > 0:
                    self.get_logger().info('Magenta %s %d %d: %d' % (class_name,x,y,max_value))
        # same for cyan
        for x in range(1,8):
            for y in range(1,9):
                class_name = ''
                max_value = 0
                for i in range(len(self.classes)):
                    # get max value and class
                    if self.classes_type_cyan_arr[x][y][i] > max_value:
                        max_value = self.classes_type_cyan_arr[x][y][i]
                        class_name = self.classes[i]
                if max_value > 0:
                    self.get_logger().info('Cyan %s %d %d: %d' % (class_name,x,y,max_value))
        
        #make table for magenta
        #TODO: only print highest values per class (DS, SS, BS, 2xRS, 2xCS)
        self.get_logger().info('   -7 -6 -5 -4 -3 -2 -1 1  2  3  4  5  6  7  ')
        self.get_logger().info('   -- -- -- -- -- -- -- -- -- -- -- -- -- -- ')
        
        for y in range(1,9):
            output = str(9-y) + ' |'
            for x in range(1,8):
                class_name = '  '
                max_value = 0
                for i in range(len(self.classes)):
                    # get max value and class
                    if self.classes_magenta_arr[8-x][9-y][i] > max_value:
                        max_value = self.classes_magenta_arr[8-x][9-y][i]
                        class_name = self.classes[i]
                if max_value < 5:
                    #self.get_logger().info('Magenta %s %d %d: %d' % (class_name,x,y,max_value))
                    class_name = '  '
                output += class_name + '|'
            for x in range(1,8):
                class_name = '  '
                max_value = 0
                for i in range(len(self.classes)):
                    # get max value and class
                    if self.classes_type_cyan_arr[x][9-y][i] > max_value:
                        max_value = self.classes_type_cyan_arr[x][9-y][i]
                        class_name = self.classes[i]
                if max_value < 5:
                    #self.get_logger().info('Magenta %s %d %d: %d' % (class_name,x,y,max_value))
                    class_name = '  '
                output += class_name + '|'
            self.get_logger().info(output)
            self.get_logger().info('   -- -- -- -- -- -- -- -- -- -- -- -- -- -- ')
        
        # make table for orientation
        self.get_logger().info('   -7  -6  -5  -4  -3  -2  -1  1   2   3   4   5   6   7  ')
        self.get_logger().info('   --- --- --- --- --- --- --- --- --- --- --- --- --- --- ')

        for y in range(1,9):
            output = str(9-y) + ' |'
            for x in range(1,8):
                max_value = 0
                orientation = 5
                for i in range(len(self.orientation)):
                    if self.orientation_magenta_arr[8-x][9-y][i] > max_value:
                        max_value = self.orientation_magenta_arr[8-x][9-y][i]
                        orientation = i
                if max_value > 5:
                    real_orientation = self.get_real_rotation_magenta(8-x, 9-y, 'CS', orientation)
                    output += real_orientation + '|'
                else:
                    output += '   |'
            for x in range(1,8):
                max_value = 0
                orientation = 5
                for i in range(len(self.orientation)):
                    if self.orientation_cyan_arr[x][9-y][i] >= max_value:
                        max_value = self.orientation_cyan_arr[x][9-y][i]
                        orientation = i
                if max_value > 5:
                    real_orientation = self.get_real_rotation_cyan(x, 9-y, 'CS', orientation)
                    output += real_orientation + '|'
                else:
                    output += '   |'
            self.get_logger().info(output)
            self.get_logger().info('   --- --- --- --- --- --- --- --- --- --- --- --- --- --- ')
        
        self.get_logger().info('Node is shutting down...')

            
def shutdown_node(signum, frame):
    print("Node is shutting down...")
    rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    node = TFSniffer()
    #rclpy.get_default_context().on_shutdown(node.shutdown_callback)
    # Set up signal handler for SIGINT (Ctrl-C)
    def sigint_handler(*args):
        node.get_logger().info('Received SIGINT signal')
        node.shutdown_callback()
        rclpy.shutdown()
    signal.signal(signal.SIGINT, sigint_handler)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
