import copy
import random
from typing import TYPE_CHECKING, List, Tuple, Type, Any, Dict, Union, Optional
import csv
import pandas as pd
pd.plotting.register_matplotlib_converters()
import matplotlib.pyplot as plt
import seaborn as sns


df = pd.read_csv('pbtestgraph.csv')
dffull = pd.read_csv('pbtestfull900.csv')

#df_t = df.T     

print(df)
print('********************************')
print(dffull)

#fig, ax = plt.subplots()
#fig.set_size_inches(11, 9)




plot2, axs1 = plt.subplots(ncols=2)
plot2.set_size_inches(18, 12)

# 'Algorithm': algostring, 'Time': res_proc["time_alldata"][n], 'Distance': res_proc['distance_alldata'][n],'Distance from Goal': 
# res_proc['distance_from_goal_alldata'][n], 'Original Distance from Goal': res_proc['original_distance_from_goal_alldata'][n]}

# p1f = sns.violinplot(x="Algorithm", y="Path Deviation", data=dffull, ax=axs1[0])
# p1f.set_xticklabels(p1f.get_xticklabels(), rotation=40, ha="right", fontsize=9)
# p1f.set_title('Path Deviation vs. Algorithm ')
# p1f.set_ylabel('Path Deviation (m)')

# p2f = sns.violinplot(x="Algorithm", y='Search Space', data=dffull, ax=axs1[1])
# p2f.set_xticklabels(p1f.get_xticklabels(), rotation=40, ha="right", fontsize=9)
# p2f.set_title('Search Space vs. Algorithm ')
# p2f.set_ylabel('Search Space')

# p3f = sns.violinplot(x="Algorithm", y='Memory', data=dffull, ax=axs1[2])
# p3f.set_xticklabels(p1f.get_xticklabels(), rotation=40, ha="right", fontsize=9)
# p3f.set_title('Memory vs. Algorithm ')
# p3f.set_ylabel('Memory')

# p4f = sns.violinplot(x="Algorithm", y='Original Distance from Goal', data=dffull, ax=axs1[3])
# p4f.set_xticklabels(p1f.get_xticklabels(), rotation=40, ha="right", fontsize=9)
# p4f.set_title('Original Distance from Goal vs. Algorithm ')
# p4f.set_ylabel('Original Distance from Goal')

# p5f = sns.violinplot(x="Algorithm", y='Success Rate', data=dffull, ax=axs1[0])
# p5f.set_xticklabels(pf.get_xticklabels(), rotation=40, ha="right", fontsize=9)
# p5f.set_title('Success Rate vs. Algorithm ')
# p5f.set_ylabel('Success Rate')

p1f = sns.violinplot(x="Algorithm", y="Time", data=dffull, ax=axs1[0])
p1f.set_xticklabels(p1f.get_xticklabels(), rotation=40, ha="right", fontsize=9)
p1f.set_title('Time vs. Algorithm ')
p1f.set_ylabel('Time (s)')


plot1, axs = plt.subplots(ncols=2)
plot1.set_size_inches(18, 11)

#bar plots

p1 = sns.barplot(x="Algorithm", y="Average Path Deviation", data=df, ax=axs[0])
p1.set_xticklabels(p1.get_xticklabels(), rotation=40, ha="right", fontsize=9)
p1.set_title('Average Path Deviation vs. Algorithm ')
p1.set_ylabel('Average Path Deviation')


# p2 = sns.barplot(x="Algorithm", y="Average Search Space", data=df, ax=axs[1])
# p2.set_xticklabels(p1.get_xticklabels(), rotation=40, ha="right", fontsize=9)
# p2.set_title('Average Search Space vs. Algorithm')
# p2.set_ylabel('Average Search Space')

p7 = sns.barplot(x="Algorithm", y="Average Time", data=df, ax=axs[1])
p7.set_xticklabels(p7.get_xticklabels(), rotation=40, ha="right", fontsize=9)
p7.set_title('Average Time vs. Algorithm')
p7.set_ylabel('Average Time')

# p3 = sns.barplot(x="Algorithm", y="Average Memory", data=df, ax=axs[2])
# p3.set_xticklabels(p1.get_xticklabels(), rotation=40, ha="right", fontsize=9)
# p3.set_title('Average Memory vs. Algorithm ')
# p3.set_ylabel('Average Memory')      


# p5 = sns.barplot(x="Algorithm", y="Average Distance from Goal", data=df, ax=axs[4])
# p5.set_xticklabels(p1.get_xticklabels(), rotation=40, ha="right", fontsize=9)
# p5.set_title('Average Distance from Goal vs. Algorithm ')
# p5.set_ylabel('Average Distance from Goal')  

# p6 = sns.barplot(x="Algorithm", y='Success Rate', data=df, ax=axs[0])
# p6.set_xticklabels(p6.get_xticklabels(), rotation=40, ha="right", fontsize=9)
# p6.set_title('Success Rate vs. Algorithm ')
# p6.set_ylabel('Success Rate') 

# p4 = sns.barplot(x="Algorithm", y='Average Distance', data=df, ax=axs[1])
# p4.set_xticklabels(p4.get_xticklabels(), rotation=40, ha="right", fontsize=9)
# p4.set_title('Average Distance from Goal vs. Algorithm ')
# p4.set_ylabel('Average Distance from Goal') 

#ax = sns.barplot(x="Algorithm", y="Average Time", data=df)
#plt.xticks(rotation=45)
plt.tight_layout(pad=2.5, w_pad=1.5, h_pad=1.5)
plt.show()