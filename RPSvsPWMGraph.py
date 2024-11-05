import csv
import pandas as pd
import numpy as np
from matplotlib import pyplot as plt
from sklearn.linear_model import LinearRegression

df = pd.read_csv("Downloads/Percent_to_Rate_to_PWM_Motor_1_Tacho_2.csv")
df.head()

cols = df.columns

percent = cols[0]
pwm = cols[1]
rps = cols[2]

plt.figure()
plt.title("Angular Velocity (RPS) vs. PWM (Negative)")
plt.plot(df[pwm], df[rps])
plt.xlabel("PWM (Negative)")
plt.ylabel("Angular Velocity (RPS)")
plt.show()

#linear fit
model = LinearRegression()
model.fit(df[[pwm]], df[rps])
slope = model.coef_[0]
intercept = model.intercept_
print("Slope: ", slope)
print("Intercept: ", intercept)

# Fit a quadratic curve
coefficients = np.polyfit(df[pwm], df[rps], 2)

# Create a function for the quadratic fit
fit_function = np.poly1d(coefficients)
print("Quadratic Function: ", fit_function)