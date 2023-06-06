# Note : this file is automaticly converted from Python Jupyter Notebook .ipynb to python .py so you may need to modify the code
# To get the original notebook head to https://gist.github.com/witcherxz/423dc737bdfaa1c505cbb853ff7c1838


import json
import numpy as np
import numpy.linalg as la
import tensorflow as tf
from sklearn.linear_model import LinearRegression
from sklearn.metrics import r2_score
from collections import defaultdict
import matplotlib.pyplot as plt
import pickle
import math
import csv
import re
from os import listdir
from os.path import isfile

# Global variables
data = None
test_data = None
features = None
test_features = None
labels = None
test_labels = None
all_Y = None
X = None
Y = None
test_X = None
test_Y = None
model = None
arucos_coff = None

def get_features(data):
    f = defaultdict(list)
    for markar in data['camera'].items():
        for record in markar[1]: 
            f[markar[0]].append(list(record.values()))
    return f

def get_labels(data):
    labels = defaultdict(list)
    for record in data['real']:
        r = record.copy()
        ids = r['ids']
        r.pop('ids', None)
        for id in ids: 
            labels[id].append(list(r.values()))
    return labels

def load_data(file_path, test_file_path, train, test):
    global data, test_data, features, test_features, labels, test_labels, all_Y, X, Y, test_X, test_Y
    file = open(file_path)
    test_file = open(test_file_path)
    data = json.load(file)
    test_data = json.load(test_file)
    features = get_features(data)
    test_features = get_features(test_data)
    labels = get_labels(data)
    test_labels = get_labels(test_data)
    comp_labels = []
    for l in labels.items():
        for r in l[1]:
            comp_labels.append(r)
    all_Y = np.array(comp_labels)
    X = np.array(features[f"_{train}"])[:, :3]
    test_X = np.array(test_features[f"_{test}"])[:, :3]
    Y = np.array(labels[train])[:, :2]
    test_Y = np.array(test_labels[test])[:, :2]

def train_linear_regression(model_file, train):
    global X, Y, model
    model = LinearRegression()
    model.fit(X[:, :2], Y[:, :2])
    with open(model_file, 'wb') as file:
        pickle.dump(model, file)

def load_linear_regression_model(model_file):
    global model
    with open(model_file, 'rb') as file:
        model = pickle.load(file)

def load_aruco_models(models_dir_path):
    global arucos_coff
    models_name = [f for f in listdir(models_dir_path) if isfile(join(models_dir_path, f))]
    arucos_coff = {}
    for model_name in models_name:
        with open((models_dir_path + "\\" + model_name), 'rb') as file:
            aruco_id = re.findall(r'\d+(?:\.\d+)?', model_name)[0]
            m = pickle.load(file)
            coefs = m.coef_
            intercepts = m.intercept_.reshape((2, 1))
            arucos_coff[aruco_id] = np.hstack([intercepts, coefs])

def save_aruco_coff_json(json_file):
    global arucos_coff
    json_body = {}
    for key, value in arucos_coff.items():
        aruco = {"opencv-matrix" : "opencv-matrix", "rows": 2, "cols": 3, "dt": "d"}
        aruco["data"] = np.squeeze(value.reshape(1, 6)).tolist()
        json_body[key] = aruco
    with open(json_file, 'w') as file:
        json.dump(json_body, file, indent=4)
