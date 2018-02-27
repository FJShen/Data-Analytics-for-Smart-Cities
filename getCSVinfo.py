from PIL import Image
import csv
import re
import random

def getRectInfo():
    paths = []
    classes = []
    rect_x = []
    rect_y = []
    rect_w = []
    rect_h = []

    with open("rectinfo.csv") as rectInfo:
        fileLines = csv.DictReader(rectInfo)

        for data in fileLines:
            if data[' view_class'] == ' 1' or data[' view_class'] == ' 2':
                path = re.sub(r"\\", r"/", data['image_path'])
                paths.append(path)
                classes.append(data[' view_class'])
                rect_x.append(data[' rect_x'])
                rect_y.append(data[' rect_y'])
                rect_w.append(data[' rect_w'])
                rect_h.append(data[' rect_h'])

    return [paths, classes, rect_x, rect_y, rect_w, rect_h]

def chooseData(im_list):
    train_data = []
    test_data = []

    no_train_data = round(len(im_list[0]) * 0.8)
    no_test_data = len(im_list[0]) - no_train_data

    #generate rand index for training data
    train_ind = set()
    while len(train_ind) < no_train_data:
        train_ind.add(random.randint(0, len(im_list[0])))

    # generate rand index for testing data
    test_ind = set()
    for i in range(0, len(im_list[0])):
        if(i not in train_ind):
            test_ind.add(i)

    for i in train_ind:
        train_data.append(im_list[i])

    for i in test_ind:
        test_data.append(im_list[i])

    return train_data, test_data

def getImageFile(filename):
    im = Image.open(filename)
    width, height = im.size
    return im, width, height

#def calculateXY_MinMax(rect_x, rect_y, rect_w, rect_h, im_width, im_height):
#
#    return xmin, xmax, ymin, ymax


if __name__ == "__main__":
    im_list = getRectInfo()
    chooseData(im_list)

    im, width, height = getImageFile(paths[0])