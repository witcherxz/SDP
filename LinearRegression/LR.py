import json
import numpy as np
from sklearn.linear_model import LinearRegression

file = open("systemCalibration.json")
data = json.load(file)
# records = 11                  #number of records
# no_Features = 4
no_Aruco = 2

realX = [[] for _ in range(no_Aruco)]
realY = [[] for _ in range(no_Aruco)]
realTheta = [[] for _ in range(no_Aruco)]

for item in data["real"]:
    for id in item['ids']:
        realX[id].append(item['x'])
        realY[id].append(item['y'])
        realTheta[id].append(item['theta'])

# loop to add 1 on the top of realX
for i in realX:
    i.insert(0,1)

# loop to add 1 on the top of realY
for i in realY:
    i.insert(0,1)
# loop to add 1 on the top of realTheta
for i in realTheta:
    i.insert(0,1) 


# Variables X, Y and Z
camera_X =  [[] for _ in range(no_Aruco)]
camera_Y =  [[] for _ in range(no_Aruco)]
camera_Z =  [[] for _ in range(no_Aruco)]
# Variables Theta X, theta Y and theta Z
camera_thetaX =  [[] for _ in range(no_Aruco)]
camera_thetaY =  [[] for _ in range(no_Aruco)]
camera_thetaZ =  [[] for _ in range(no_Aruco)]

# Get the list of ids in the object, because it is in shape of string
keys = list(data['camera'].keys())

for key in keys:
    id = int(key.split('_')[1])
    objects = data['camera'][key]
    for obj in objects:
        camera_X[id].append(obj['x'])
        camera_Y[id].append(obj['y'])
        camera_Z[id].append(obj['z'])
        camera_thetaX[id].append(obj['thetaX'])
        camera_thetaY[id].append(obj['thetaY'])
        camera_thetaZ[id].append(obj['thetaZ'])

# For X values
model1 = LinearRegression()
# For Y values
model2 = LinearRegression()
# For Theta values
model3 = LinearRegression()

#   Next, Still needs to improvements to fit all ids featrues Auto 
#   The code below only for testing

# Merging features and adding 1 at top of the featrues
features = np.vstack([camera_X[0],camera_Y[0],camera_Z[0],camera_thetaX[0],camera_thetaY[0],camera_thetaZ[0]]).T
features = np.vstack([np.ones(features.shape[1]),features])

model1.fit(features, realX[0])
model2.fit(features, realY[0])
model3.fit(features, realTheta[0])

print(model1.predict(features))
###################################################################################
# for i in range(no_Aruco):
#     model1.fit(features, realX) 
#     model2.fit(features, realY)
#     model2.fit(features, realTheta)


# Print the arrays to verify the data was stored correctly
# print(y)
# print(theta)

# for i in data["real"]:
#     for j in i["ids"]:
#         realX = np.append(j,i["x"],axis=j)
# print(realX)

# print(len(ID[8]))
# real_x_values = [obj["x"] for obj in data["real"]]
# real_x_values = np.array([real_x_values]).T
# n = int(len(real_x_values[0])) # length of features
# real_x_values = np.vstack((np.ones(n), real_x_values))

# real_y_values = [obj["y"] for obj in data["real"]]
# real_y_values = np.array([real_y_values]).T
# n = int(len(real_y_values[0])) # length of features
# real_y_values = np.vstack((np.ones(n), real_y_values))

# # real_theta_values = [obj["theta"] for obj in data["real"]]
# # real_theta_values = np.array([real_theta_values]).T
# # n = int(len(real_theta_values[0])) # length of features
# # real_theta_values = np.vstack((np.ones(n), real_theta_values))


# camera_x_values = [obj["x"] for obj in data["camera"]]
# camera_y_values = [obj["y"] for obj in data["camera"]]
# # camera_z_values = [obj["y"] for obj in data["camera"]]

# camera_thetaX_values = [obj["thetaX"] for obj in data["camera"]]
# camera_thetaY_values = [obj["thetaY"] for obj in data["camera"]]
# # camera_thetaZ_values = [obj["thetaY"] for obj in data["camera"]]

# real_x_values = np.array(real_x_values)
# real_y_values = np.array(real_y_values)
# # real_theta_values = np.array(real_theta_values)

# camera_x_values = np.array(camera_x_values).reshape(-1, 1)
# camera_y_values = np.array(camera_y_values).reshape(-1, 1)
# # camera_z_values = np.array(camera_z_values).reshape(-1, 1)

# camera_thetaX_values = np.array(camera_thetaX_values).reshape(-1, 1)
# camera_thetaY_values = np.array(camera_thetaY_values).reshape(-1, 1)
# # camera_thetaZ_values = np.array(camera_thetaY_values).reshape(-1, 1)


# features = np.array([camera_x_values, camera_y_values, camera_thetaX_values, camera_thetaY_values ]).reshape(no_Features,records).T
# n = int(len(features[0])) # length of features
# features = np.vstack((np.ones(n), features))
# # print (features)

# model1 = LinearRegression()
# model2 = LinearRegression()
# model3 = LinearRegression()


# model1.fit(features, real_x_values) 
# model2.fit(features, real_y_values)
# # model3.fit(featuers, real_theta_values)

# # print("Coff of X: ", model1.coef_)
# # print("Coff of Y: ", model2.coef_)
# # print("Coff of ThetaX: ", model3.coef_)


# # print("perdiction of real x ", model1.predict(features))
# # print("perdiction of real y ", model2.predict(features))
# # print(model3.predict(featuers))

# #----------------testing------------------
# test_record = np.array([1,1,1,1])  # [7 * 1]

# models_coff = np.array([model1.coef_, model2.coef_ ]).reshape(2,no_Features)   # [3 * 7]

# predicted_Values = np.matmul(models_coff, test_record)

# print(predicted_Values)