import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import streamlit as st

input_path = "C:/Users/Miki/uni/CPPrakt/NBODY_V1/NBODY_EXP1/N-Body/"
output_path = "C:/Users/Miki/uni/CPPrakt/NBODY_V1/NBODY_EXP1/N-Body/diashow/"
df = pd.read_table(input_path + "output.txt")
x_1 = df.iloc[:,1]
y_1 = df.iloc[:,2]
x_2 = df.iloc[:,4]
y_2 = df.iloc[:,5]

figs = []

for i in range(len(x_1)):
    fig, ax = plt.subplots()
    ax.set_xlim(-3,3)
    ax.set_ylim(-3,3)
    ax.grid(True)
    ax.scatter(x_1[i], y_1[i])
    ax.scatter(x_2[i], y_2[i])
    ax.set_title(f'{i}')
    fig.savefig(output_path + "plot_" + str(i) + ".png")
    plt.close()


#with st.expander('Using a slider'):

 #   plot = st.container()

    # use st.slider to select
  #  index = st.slider('figure index', 1, len(figs))

    # use st.pyplot
   # with plot:
    #    st.pyplot(figs[index-1])
#
#with st.expander('Using tabs'):
#    tabs = st.tabs(list(np.array(range(1,len(figs))).astype(str)))
#
#    for i in range(len(figs)):
#        tabs[i].pyplot(figs[i])