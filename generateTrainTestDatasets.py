from PIL import Image
import csv
import re
import random

def getRectInfo(filename):
    im_list = []
    with open(filename) as rectInfo:
        fileLines = csv.DictReader(rectInfo)

        for data in fileLines:
            if data['class'] == '1' or data['class'] == '2':
                im_list.append([data['filename'], data['width'], data['height'], data['class'], data['xmin'], data['ymin'], data['xmax'], data['ymax']])

    return im_list

def chooseData(im_list):
    train_data = []
    test_data = []

    no_train_data = round(len(im_list) * 0.8)
    no_test_data = len(im_list) - no_train_data

    #generate rand index for training data
    train_ind = set()
    while len(train_ind) < no_train_data:
        train_ind.add(random.randint(0, len(im_list) - 1))

    # generate rand index for testing data
    test_ind = set()
    for i in range(0, len(im_list)):
        if(i not in train_ind):
            test_ind.add(i)

    for i in train_ind:
        train_data.append(im_list[i])

    for i in test_ind:
        test_data.append(im_list[i])

    return train_data, test_data

def generateDataCSV(train_data, test_data):

    with open("train_labels.csv", "w+") as trainFile:
        trainFile.write("filename,width,height,class,xmin,ymin,xmax,ymax\n")

        wr = csv.writer(trainFile, quoting=csv.QUOTE_ALL)

        for data in train_data:
            wr.writerow(data)

    with open("test_labels.csv", "w+") as testFile:
        wr = csv.writer(testFile, quoting=csv.QUOTE_ALL)

        for data in test_data:
            wr.writerow(data)


if __name__ == "__main__":
    filename = input("This script randomly generates a training and testing set from WLSP and Morganza datasets.\n"
                     "Input filename of csv file with all labels: ")
    im_list = getRectInfo(filename)
    train_data, test_data = chooseData(im_list)
    generateDataCSV(train_data, test_data)
