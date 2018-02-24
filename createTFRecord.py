from PIL import Image
import tensorflow as tf
import re
from object_detection.utils import dataset_util

def getRectInfo():
    with open("rectinfo.csv") as rectInfo:
        fileLines = rectInfo.readlines()

    lineList = []
    paths = []
    classes = []
    rect_x = []
    rect_y = []
    rect_w = []
    rect_h = []

    for i in range(1,len(fileLines)):
        line = fileLines[i].split(",")

        if line[1] is '1' or line[1] is not '2':
            paths.append(re.sub(r"\\", "/", line[0]))
            classes.append(line[1]) 
            rect_x.append(line[2])  
            rect_y.append(line[3])
            rect_w.append(line[4])
            rect_h.append(line[5])

    return paths, classes, rect_x, rect_y, rect_w, rect_h

def getImageFile(filename):
    im = Image.open(filename)
    width, height = im.size
    return None

def create_tf_example(example):
  # TODO: Populate the following variables from your example.
  height = None # Image height
  width = None # Image width
  filename = None # Filename of the image. Empty if image is not from file
  encoded_image_data = None # Encoded image bytes
  image_format = None # b'jpeg' or b'png'

  xmins = [] # List of normalized left x coordinates in bounding box (1 per box)
  xmaxs = [] # List of normalized right x coordinates in bounding box
             # (1 per box)
  ymins = [] # List of normalized top y coordinates in bounding box (1 per box)
  ymaxs = [] # List of normalized bottom y coordinates in bounding box
             # (1 per box)
  classes_text = [] # List of string class name of bounding box (1 per box)
  classes = [] # List of integer class id of bounding box (1 per box)

  tf_example = tf.train.Example(features=tf.train.Features(feature={
      'image/height': dataset_util.int64_feature(height),
      'image/width': dataset_util.int64_feature(width),
      'image/filename': dataset_util.bytes_feature(filename),
      'image/source_id': dataset_util.bytes_feature(filename),
      'image/encoded': dataset_util.bytes_feature(encoded_image_data),
      'image/format': dataset_util.bytes_feature(image_format),
      'image/object/bbox/xmin': dataset_util.float_list_feature(xmins),
      'image/object/bbox/xmax': dataset_util.float_list_feature(xmaxs),
      'image/object/bbox/ymin': dataset_util.float_list_feature(ymins),
      'image/object/bbox/ymax': dataset_util.float_list_feature(ymaxs),
      'image/object/class/text': dataset_util.bytes_list_feature(classes_text),
      'image/object/class/label': dataset_util.int64_list_feature(classes),
  }))
  return tf_example



if __name__ == "__main__":
    paths, classes, rect_x, rect_y, rect_w, rect_h = getRectInfo()

