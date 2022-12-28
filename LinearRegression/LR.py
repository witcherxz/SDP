import json
import numpy as np
from sklearn.linear_model import LinearRegression

file = open("systemCalibration.json")
data = json.load(file)
n = 11                                  #number of records


real_x_values = [obj["x"] for obj in data["real"]]
n = int(len(real_x_values[0])) # length of features
features = np.vstack((np.ones(n), real_x_values))

real_y_values = [obj["y"] for obj in data["real"]]
n = int(len(real_y_values[0])) # length of features
features = np.vstack((np.ones(n), real_y_values))

# real_theta_values = [obj["theta"] for obj in data["real"]]

camera_x_values = [obj["x"] for obj in data["camera"]]
camera_y_values = [obj["y"] for obj in data["camera"]]
# camera_z_values = [obj["y"] for obj in data["camera"]]

camera_thetaX_values = [obj["thetaX"] for obj in data["camera"]]
camera_thetaY_values = [obj["thetaY"] for obj in data["camera"]]
# camera_thetaZ_values = [obj["thetaY"] for obj in data["camera"]]

real_x_values = np.array(real_x_values)
real_y_values = np.array(real_y_values)
# real_theta_values = np.array(real_theta_values)

camera_x_values = np.array(camera_x_values).reshape(-1, 1)
camera_y_values = np.array(camera_y_values).reshape(-1, 1)
# camera_z_values = np.array(camera_z_values).reshape(-1, 1)

camera_thetaX_values = np.array(camera_thetaX_values).reshape(-1, 1)
camera_thetaY_values = np.array(camera_thetaY_values).reshape(-1, 1)
# camera_thetaZ_values = np.array(camera_thetaY_values).reshape(-1, 1)

features = np.array([camera_x_values, camera_y_values, camera_thetaX_values, camera_thetaY_values ]).reshape(4,n).T
n = int(len(features[0])) # length of features
features = np.vstack((np.ones(n), features))
# print (features)

model1 = LinearRegression()
model2 = LinearRegression()
model3 = LinearRegression()

model1.fit(features, real_x_values) 
model2.fit(features, real_y_values)
# model3.fit(featuers, real_theta_values)

# print("Coff of X: ", model1.coef_)
# print("Coff of Y: ", model2.coef_)
# print("Coff of ThetaX: ", model3.coef_)


# print("perdiction of real x ", model1.predict(features))
# print("perdiction of real y ", model2.predict(features))
# print(model3.predict(featuers))

#----------------testing------------------
test_record = np.array([1, 1, 1, 1, 1, 1, 1])
print(test_record.shape)