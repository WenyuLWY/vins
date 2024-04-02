#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import time
import numpy as np

from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator, colors
# from collections import defaultdict





class ObjectDetector:
    def __init__(self):
		# Load parameters
		
        self.WEIGHT_PATH = rospy.get_param("~weight_path")
        self.IMAGE_TOPIC = rospy.get_param("~image_topic")
        self.IAMGE_HEIGHT = rospy.get_param("~image_height")
        self.IMAGE_WIDTH = rospy.get_param("~image_width")
        self.KERNEL_SIZE = rospy.get_param("~kernel_size")
        self.DILATE_ITER= rospy.get_param("~dilate_iter")
        self.model = YOLO(self.WEIGHT_PATH)   # segmentation model
        
        # self.model.names
        """
            
            { 0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 4: 'airplane', 5: 'bus',
              6: 'train', 7: 'truck', 8: 'boat', 9: 'traffic light', 10: 'fire hydrant', 
              11: 'stop sign', 12: 'parking meter', 13: 'bench', 14: 'bird', 15: 'cat', 
              16: 'dog', 17: 'horse', 18: 'sheep', 19: 'cow', 20: 'elephant', 21: 'bear', 
              22: 'zebra', 23: 'giraffe', 24: 'backpack', 25: 'umbrella', 26: 'handbag', 
              27: 'tie', 28: 'suitcase', 29: 'frisbee', 30: 'skis', 31: 'snowboard', 
              32: 'sports ball', 33: 'kite', 34: 'baseball bat', 35: 'baseball glove', 
              36: 'skateboard', 37: 'surfboard', 38: 'tennis racket', 39: 'bottle', 
              40: 'wine glass', 41: 'cup', 42: 'fork', 43: 'knife', 44: 'spoon', 
              45: 'bowl', 46: 'banana', 47: 'apple', 48: 'sandwich', 49: 'orange', 
              50: 'broccoli', 51: 'carrot', 52: 'hot dog', 53: 'pizza', 54: 'donut', 
              55: 'cake', 56: 'chair', 57: 'couch', 58: 'potted plant', 59: 'bed', 
              60: 'dining table', 61: 'toilet', 62: 'tv', 63: 'laptop', 64: 'mouse', 
              65: 'remote', 66: 'keyboard', 67: 'cell phone', 68: 'microwave', 69: 'oven', 
              70: 'toaster', 71: 'sink', 72: 'refrigerator', 73: 'book', 74: 'clock', 
              75: 'vase', 76: 'scissors', 77: 'teddy bear', 78: 'hair drier', 79: 'toothbrush'}

        """
 
        rospy.logwarn("Model loaded")
        # self.track_history = defaultdict(lambda: [])
        self.bridge = CvBridge()

        
        rospy.Subscriber(self.IMAGE_TOPIC, Image, self.object_detection, queue_size=100)

        self.pubResults = rospy.Publisher('~detection_results', Image, queue_size=100)
        self.pubMask = rospy.Publisher('~mask_image', Image, queue_size=100)

        
    def object_detection(self, msg):

        start = time.time()

        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        annotator = Annotator(image, line_width=2)


        results = self.model.track(image, verbose=False, persist=True,
                                #    conf=0.3,
                                #    imgsz=(544, 960),
                                #    classes=[0]
                                   device='cuda',
                                   classes=[0,1,2,3,5,6,7]
                                   )
        
        # all results without filtering    result_img = results[0].plot() 

        mask_image = np.ones(image.shape[:2], dtype=np.uint8) * 255

        if results[0].boxes.id is not None and results[0].masks is not None:
            masks = results[0].masks.xy
            track_ids = results[0].boxes.id.int().cpu().tolist()
            for mask, track_id in zip(masks, track_ids):
                annotator.seg_bbox(mask=mask,mask_color=colors(track_id, True),track_label=str(track_id))
                cv2.fillPoly(mask_image, [np.int32([mask])], color=(0,0,0))  
        
        # mask_image = cv2.dilate(mask_image, self.generate_diamond_kernel(99) )
        mask_image = self.expandmask(mask_image, self.KERNEL_SIZE)

        result_msg=self.bridge.cv2_to_imgmsg(image, "bgr8")
        result_msg.header = msg.header
        mask_msg=self.bridge.cv2_to_imgmsg(mask_image, "mono8")
        mask_msg.header = msg.header
        self.pubResults.publish(result_msg)
        self.pubMask.publish(mask_msg)
		
        rospy.logwarn("Time taken for detection: %s ms", (time.time() - start)*1000) 

    def expandmask(self, mask, size):
        assert size % 2 == 1, "Size must be odd"
        inverted_mask = 255 - mask
        center = size // 2
        kernel = np.zeros((size, size), dtype=np.uint8)
        for i in range(size):
            for j in range(size):
                if abs(i - center) + abs(j - center) <= center:
                    kernel[i, j] = 1
        dilated_mask = cv2.dilate(inverted_mask, kernel, iterations=self.DILATE_ITER)
        final_mask = 255 - dilated_mask
        return final_mask   	

def main():
	rospy.init_node('object_detection_node', anonymous=True)
	ObjectDetector()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	main()