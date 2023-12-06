import pandas as pd
import matplotlib.pyplot as plt
import os
import glob
import numpy as np

ekfFiles = glob.glob('*_ekf.csv')
print(f'{ekfFiles = }')
for ind, f in enumerate(ekfFiles):
    print(f'[{ind}]: {f}')

f = ekfFiles[int(input('Please input file index: '))]

df = pd.read_csv(f)
df = df.dropna()
df['Time'] = df['TimestampSecond'] - df['TimestampSecond'].iloc[0]

print(f'{df.head() = }')

fig = plt.figure(figsize=(8,8))
ax = fig.add_subplot(111)
ekfX = [df[f'usvEKFx[{i}][0]'] for i in range(6)]
ax.scatter(df['Time'], df['usvEKFx[0][0]'], label='x')
ax.scatter(df['Time'], df['usvEKFx[1][0]'], label='y')
ax.scatter(df['Time'], df['usvEKFx[2][0]'], label='z')

df['EKFr'] = np.sqrt()

ax.legend()
ax.set_title(f)
fig.savefig(f[:-4] + '.png')
plt.show()
        