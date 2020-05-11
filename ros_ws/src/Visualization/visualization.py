import os
import pandas as pd
from collections import Counter
import matplotlib.pyplot as plt

# getting files from directory and reading from files
files = os.listdir('out_10_transformed')
os.chdir('out_10_transformed')
print(len(files))

object_dictionary = dict()

#Printing file name and size
for file in files:
    print(file)

    file_text = open(file, 'r').readlines()
    for line in file_text:
        if 'POINTS' in line:
            print(line.strip().split(' ')[1])
            value = int(line.strip().split(' ')[1])
            key = file.split('-')[0]
            # Adding key and value to object dictionary
            if key in object_dictionary:
                object_dictionary[key].append(value)
            else:
                object_dictionary[key] = [value]
            break
#printing each object with points in each Cloud file
print(object_dictionary)

#Plotting Histogram for objects
for object in object_dictionary.keys():
    dataframe = pd.DataFrame(object_dictionary[object], columns=[object])
    print(dataframe)
    ax = dataframe.plot.hist(bins=100, legend=False, title=object)
    plt.xlabel('Number of points')
    plt.ylabel('Number of clouds')
    plt.show()
    fig = ax.get_figure()











