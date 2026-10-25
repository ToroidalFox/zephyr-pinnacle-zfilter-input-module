import numpy as np


def biased_rounding(x):
    return np.floor(x + 0.3)


rows = 7 + 3
columns = 9 + 4

maxz = 32

y, x = np.indices((rows, columns))
r = np.sqrt((x / columns) ** 2 + (y / rows) ** 2)

zval = r**0.8

map = maxz * np.clip(1 - zval, 0.0, 1)

print(np.flip(biased_rounding(map))[3:, 4:])
