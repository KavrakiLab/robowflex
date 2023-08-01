import yaml
from dataclasses import dataclass
from numpy.typing import NDArray
import matplotlib.pyplot as plt
import numpy as np

@dataclass
class ResultFile:
    algorithm: str
    monitoring_rate: NDArray

with open('../result/results_RRTConnect1000.txt', 'r') as params_yaml:
    data = yaml.load(params_yaml, Loader=yaml.FullLoader)
    result = ResultFile(**data)

mr = np.array(result.monitoring_rate.split(' '))
dat1 = mr.astype(float)
name1 = result.algorithm

with open('../result/results_RRTConnectPost1000.txt', 'r') as params_yaml:
    data2 = yaml.load(params_yaml, Loader=yaml.FullLoader)
    result2 = ResultFile(**data2)

mr = np.array(result2.monitoring_rate.split(' '))
dat2 = mr.astype(float)
name2 = result2.algorithm

monitoring_data = {name1: dat1, name2: dat2}

fig, ax = plt.subplots()
ax.boxplot(monitoring_data.values())
ax.set_xticklabels(monitoring_data.keys())

plt.show()
