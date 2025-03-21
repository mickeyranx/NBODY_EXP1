import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


df = pd.read_table("output.txt")
x_1 = df.iloc[:,1]
y_1 = df.iloc[:,2]
x_2 = df.iloc[:,4]
y_2 = df.iloc[:,5]



for i in range(len(x_1)):
    fig, ax = plt.subplots()
    ax.set_xlim(-3,3)
    ax.set_ylim(-3,3)
    ax.grid(True)
    ax.scatter(x_1[i], y_1[i])
    ax.scatter(x_2[i], y_2[i])
    plt.show()

