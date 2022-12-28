import json
import numpy as np
from sklearn.linear_model import LinearRegression


file = open("systemCalibration.json")
data = json.load(file)

real_x_values = [obj["x"] for obj in data["real"]]
camera_x_values = [obj["x"] for obj in data["camera"]]

real_y_values = [obj["y"] for obj in data["real"]]
camera_y_values = [obj["y"] for obj in data["camera"]]

real_thetaX_values = [obj["thetaX"] for obj in data["real"]]
camera_thetaX_values = [obj["thetaX"] for obj in data["camera"]]

real_thetaY_values = [obj["thetaY"] for obj in data["real"]]
camera_thetaY_values = [obj["thetaY"] for obj in data["camera"]]

x1 = np.array(camera_y_values).reshape(-1, 1)
y1 = np.array(real_x_values)

x2 = np.array(camera_x_values).reshape(-1, 1)
y2 = np.array(real_y_values)

x3 = np.array(camera_thetaY_values).reshape(-1, 1)
y3 = np.array(real_thetaX_values)

x4 = np.array(camera_thetaX_values).reshape(-1, 1)
y4 = np.array(real_thetaY_values)

xlabels = np.array([x1,x2,x3, x4]).reshape(4,11).T


model1 = LinearRegression()
model2 = LinearRegression()
model3 = LinearRegression()
model4 = LinearRegression()
model1.fit(xlabels, y1)
model2.fit(xlabels, y2)
model3.fit(xlabels, y3)
model4.fit(xlabels, y4)

print("Coff of X: ", model1.coef_)
print("Coff of Y: ", model2.coef_)
print("Coff of ThetaX: ", model3.coef_)
print("Coff of ThetaY: ", model4.coef_)

print(model1.predict(xlabels))
print(model2.predict(xlabels))
print(model3.predict(xlabels))
print(model4.predict(xlabels))
