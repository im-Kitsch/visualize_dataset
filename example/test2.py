import matplotlib.pyplot as plt
import numpy as np

from scipy import zeros, signal, random


def filter_sbs():
    t = np.linspace(-1, 1, 1501)
    x = (np.sin(2 * np.pi * 0.75 * t * (1 - t) + 2.1) +
         0.1 * np.sin(2 * np.pi * 1.25 * t + 1) +
         0.18 * np.cos(2 * np.pi * 3.85 * t))
    data = x + np.random.standard_normal(len(t)) * 0.08
    b = signal.firwin(150, 0.004)
    z = signal.lfilter_zi(b, 1) * data[0]
    result = zeros(data.size)
    for i, x in enumerate(data):
        result[i], z = signal.lfilter(b, 1, [x], zi=z)

    plt.figure()
    plt.plot(data)
    plt.plot(result)
    plt.show()
    return result


if __name__ == '__main__':
    filter_sbs()
