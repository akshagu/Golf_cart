#!/usr/bin/env python
"""camera_tf_trt.py

This is a Camera TensorFlow/TensorRT Object Detection sample code for
Jetson TX2 or TX1.  This script captures and displays video from either
a video file, an image file, an IP CAM, a USB webcam, or the Tegra
onboard camera, and do real-time object detection with example TensorRT
optimized SSD models in NVIDIA's 'tf_trt_models' repository.  Refer to
README.md inside this repository for more information.

This code is written and maintained by JK Jung <jkjung13@gmail.com>.
"""


import sys
import time
import logging
import argparse
import rospy
import std_msgs.msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Header
import numpy as np
import cv2
import tensorflow as tf
import tensorflow.contrib.tensorrt as trt

from mobilenet_detect.msg import cvbox
from mobilenet_detect.msg import cvboxarray

from utils.camera import Camera
from utils.od_utils import read_label_map, build_trt_pb, load_trt_pb, \
                           write_graph_tensorboard, detect
from utils.visualization import BBoxVisualization


# Constants
DEFAULT_MODEL = 'ssd_mobilenet_v1_coco'
DEFAULT_LABELMAP = 'third_party/models/research/object_detection/' \
                   'data/mscoco_label_map.pbtxt'
WINDOW_NAME = 'CameraTFTRTDemo'
BBOX_COLOR = (0, 255, 0)  # green


def parse_args():
    """Parse input arguments."""
    desc = ('This script captures and displays live camera video, '
            'and does real-time object detection with TF-TRT model '
            'on Jetson TX2/TX1')
    parser = argparse.ArgumentParser(description=desc)
    parser.add_argument('--file', dest='use_file',
                        help='use a video file as input (remember to '
                        'also set --filename)',
                        action='store_true')
    parser.add_argument('--image', dest='use_image',
                        help='use an image file as input (remember to '
                        'also set --filename)',
                        action='store_true')
    parser.add_argument('--filename', dest='filename',
                        help='video file name, e.g. test.mp4',
                        default=None, type=str)
    parser.add_argument('--rtsp', dest='use_rtsp',
                        help='use IP CAM (remember to also set --uri)',
                        action='store_true')
    parser.add_argument('--uri', dest='rtsp_uri',
                        help='RTSP URI, e.g. rtsp://192.168.1.64:554',
                        default=None, type=str)
    parser.add_argument('--latency', dest='rtsp_latency',
                        help='latency in ms for RTSP [200]',
                        default=200, type=int)
    parser.add_argument('--usb', dest='use_usb',
                        help='use USB webcam (remember to also set --vid)',
                        action='store_true')
    parser.add_argument('--vid', dest='video_dev',
                        help='device # of USB webcam (/dev/video?) [1]',
                        default=1, type=int)
    parser.add_argument('--width', dest='image_width',
                        help='image width [1280]',
                        default=1280, type=int)
    parser.add_argument('--height', dest='image_height',
                        help='image height [720]',
                        default=720, type=int)
    parser.add_argument('--model', dest='model',
                        help='tf-trt object detecion model '
                        '[{}]'.format(DEFAULT_MODEL),
                        default=DEFAULT_MODEL, type=str)
    parser.add_argument('--build', dest='do_build',
                        help='re-build TRT pb file (instead of using'
                        'the previously built version)',
                        action='store_true')
    parser.add_argument('--tensorboard', dest='do_tensorboard',
                        help='write optimized graph summary to TensorBoard',
                        action='store_true')
    parser.add_argument('--labelmap', dest='labelmap_file',
                        help='[{}]'.format(DEFAULT_LABELMAP),
                        default=DEFAULT_LABELMAP, type=str)
    parser.add_argument('--num-classes', dest='num_classes',
                        help='(deprecated and not used) number of object '
                        'classes', type=int)
    parser.add_argument('--confidence', dest='conf_th',
                        help='confidence threshold [0.3]',
                        default=0.3, type=float)
    args = parser.parse_args()
    return args


rospy.init_node('mobilenet_out', anonymous=False)
image_pub1 = rospy.Publisher("cv_test_image",Image, queue_size=1000)
image_pub2 = rospy.Publisher("cv_test_box",cvboxarray, queue_size=1000)

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)
# Ask tensorflow logger not to propagate logs to parent (which causes
# duplicated logging)
logging.getLogger('tensorflow').propagate = False

args = parse_args()
print(args)
logger.info('called with args: %s' % args)

# build the class (index/name) dictionary from labelmap file
logger.info('reading label map')
cls_dict = read_label_map(args.labelmap_file)

pb_path = './data/{}_trt.pb'.format(args.model)
log_path = './logs/{}_trt'.format(args.model)
if args.do_build:
    logger.info('building TRT graph and saving to pb: %s' % pb_path)
    build_trt_pb(args.model, pb_path)


logger.info('loading TRT graph from pb: %s' % pb_path)
trt_graph = load_trt_pb(pb_path)

logger.info('starting up TensorFlow session')
conf_th = .7
od_type = 'faster_rcnn'
tf_config = tf.ConfigProto()
tf_config.gpu_options.allow_growth = True
tf_sess = tf.Session(config=tf_config, graph=trt_graph)
bridge = CvBridge()
vis = BBoxVisualization(cls_dict)

class image_converter:

  def callback(data):
    try:
      cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    box, conf, cls = detect(cv_image, tf_sess, conf_th, od_type=od_type)
    cls = cls-1
    cv_image = vis.draw_bboxes(cv_image, box, conf, cls)


    #MESSAGE STUFF ///////////////////////

    msg_array = cvboxarray()
    msg = [None]*len(box)
    header = std_msgs.msg.Header()
    msg_array.header.seq = id(image_converter)
    msg_array.header.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
    
    for i in range(len(box)):
       msg[i] = cvbox()
       p1 = (box[i][0], box[i][1])
       p2 = (box[i][2], box[i][3])
   
       msg[i].class_name = str(cls_dict.get(cls[i]))
       msg[i].confidence = conf[i]
       msg[i].x_min = np.float32(p1[1]).item()
       msg[i].x_max = np.float32(p2[1]).item()
       msg[i].y_max = np.float32(750-p1[0]).item()
       msg[i].y_min = np.float32(750-p2[0]).item()

       msg_array.objects.append(msg[i])

    try:
      image_pub1.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      image_pub2.publish(msg_array)
    except CvBridgeError as e:
      print(e)

    #FOR DISPLAYING THE IMAGE WITH BOUNDING BOXES /////////////////
    #cv2.imshow("SSD", cv_image)
    #k = cv2.waitKey(1) & 0xff
        #Exit if ESC pressed
    #if k == 27 : return False
    #return True

  image_sub = rospy.Subscriber("/v4l/camera/image_raw",Image,callback)

def main():
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
