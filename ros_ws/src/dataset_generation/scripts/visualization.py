import os
import pandas as pd
from collections import Counter
import matplotlib.pyplot as plt

files = os.listdir('out_10_transformed')
os.chdir('out_10_transformed')
print(len(files))
object_dictionary = dict()

for file in files:
    print(file)

    file_text = open(file, 'r').readlines()
    for line in file_text:
        if 'POINTS' in line:
            print(line.strip().split(' ')[1])
            value = int(line.strip().split(' ')[1])
            key = file.split('-')[0]
            if key in object_dictionary:
                object_dictionary[key].append(value)
            else:
                object_dictionary[key] = [value]
            break

print(object_dictionary)

object_points = dict()

for key in object_dictionary:
    object_points[key] = Counter(object_dictionary[key])
    object_points[key] = dict(object_points[key])

print(object_points)

for object in object_points.keys():
    dataframe = pd.DataFrame.from_dict(object_points[object], orient='index')
    print(dataframe)

    ax = dataframe.plot.hist(bins=10, legend=False, title=object)
    plt.show()
    fig = ax.get_figure()
    fig.savefig(object+'.png')










