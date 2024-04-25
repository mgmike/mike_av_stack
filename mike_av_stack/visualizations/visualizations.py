
import argparse
import rclpy
import numpy as np
import cv2
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

from cv_bridge import CvBridge


colors_map={2:(0,255,0), 0:(0,0,255), 1:(100,100,0)}
def colors(cls):
    if int(cls) in colors_map:
        return colors_map[cls]
    else:
        return (255,0,0)

def addBoxes(results):
    for result in results:
        boxes = result.boxes
        probs = result.probs
        img = result.orig_img
        classes = boxes.cls.cpu().numpy()
        names = result.names
        # print(boxes, ', prob:')
        for i, xyxy in enumerate(boxes.xyxy.cpu().numpy()):
            # print(xyxy[0])
            cls = classes[i]

            if int(cls) in colors_map:
                color = colors_map[cls]
                cv2.rectangle(img, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), thickness=2, color=color)
                cv2.putText(img, names[int(cls)], (int(xyxy[0]), int(xyxy[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color=color, thickness=2)
            
    cv2.imshow('Camera and detected objects', img)
    cv2.waitKey(0) 
    return img

class Visualizer(Node):
    def __init__(self):
        super().__init__('visualizer')
        self.get_logger().debug('Starting Visualizer node')
        self.br = CvBridge()
        sub_cb_group = ReentrantCallbackGroup()
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.pub_img_with_boxes = self.create_publisher(
            Image,
            "/sensor_fusion/detection/camera/front/img_with_boxes",
            10
            )

        self.subscriber_pc = self.create_subscription(
        msg_type=Detection2DArray,
        topic="/sensor_fusion/detection/camera/front", 
        callback=self.detection_callback,
        qos_profile=qos_profile,
        callback_group=sub_cb_group
        )
        self.subscriber_pc
        
    def detection_callback(self, detection2DArray: Detection2DArray):
        self.get_logger().info('Got detection')
        image = Image()
        image_mat = np.zeros([480, 640, 3])
        for i, detection in enumerate(detection2DArray.detections):
            # convert class from alpha numeric ie. '1:person' to int 1
            cls = int(detection.results[0].id.split(':')[0])
            cls_name = detection.results[0].id.split(':')[1]
            color = colors_map[cls]
            if i == 0 and detection.source_img is not None and len(detection.source_img.data) > 0:
                image_matBGRA = self.br.imgmsg_to_cv2(detection.source_img, desired_encoding='passthrough')
                image_mat = cv2.cvtColor(image_matBGRA, cv2.COLOR_BGRA2BGR)
            cv2.rectangle(image_mat, (int(detection.bbox.center.x - detection.bbox.size_x), 
                                      int(detection.bbox.center.y - detection.bbox.size_y)), 
                                     (int(detection.bbox.center.x + detection.bbox.size_x), 
                                      int(detection.bbox.center.y + detection.bbox.size_y)),
                                      thickness=2, 
                                      color=color)
            cv2.putText(image_mat, cls_name, 
                        (int(detection.bbox.center.x - detection.bbox.size_x), 
                         int(detection.bbox.center.y - detection.bbox.size_y) - 10), 
                         cv2.FONT_HERSHEY_SIMPLEX,
                         0.6,
                         color=color,
                         thickness=2)
        result_image = self.br.cv2_to_imgmsg(image_mat, encoding='passthrough')
        self.pub_img_with_boxes.publish(result_image)
            

def main(args=None):
    # argparser = argparse.ArgumentParser(
    #     description=__doc__)
    # argparser.add_argument(
    #     '--host',
    #     metavar='H',
    #     default='127.0.0.1',
    #     help='IP of the host server (default: 127.0.0.1)')
    # argparser.add_argument(
    #     '-p', '--port',
    #     metavar='P',
    #     default=2000,
    #     type=int,
    #     help='TCP port to listen to (default: 2000)')   
    # args = argparser.parse_args()

    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    viz = Visualizer()
    executor.add_node(viz)

    try:
        print('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        print('Keyboard interrupt, shutting down.\n')
        viz.destroy_node()
    rclpy.shutdown()
    return

if __name__ == '__main__':
    main()

    
