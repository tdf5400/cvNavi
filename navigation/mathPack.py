# 数学包，用于存储本项目用到的数学函数
import numpy as np
import math


def Gaussian_distribution(x, u, sigma):
    """
    高斯分布公式
    X∼N(μ,σ2),
    :param x:     传入参数
    :param u:     即μ
    :param sigma: 即σ
    :return:
    """
    y = np.exp(-(x - u) ** 2 / (2 * sigma ** 2)) / (sigma * math.sqrt(2 * math.pi))
    return y


# test
if __name__ == '__main__' and 1:
    import matplotlib.pyplot as plt

    x = np.arange(0, 480)
    # y = []
    # for i in x:
    #     y.append(Gaussian_distribution(i, 0, 100) / Gaussian_distribution(0, 0, 100))
    #
    # plt.scatter(x, y)
    # plt.show()

    points = np.zeros(480, dtype=np.int16)
    for i in range(0, len(points)):
        points[i] = 100
    points[0] = 0
    points[405:420] = 0
    plt.subplot(121)
    plt.scatter(x, points)

    newList = [points[0], ]
    P = 0.1 / Gaussian_distribution(0, 0, 100)
    for i in range(1, len(points)):
        newList.append(newList[i - 1] + int(P * (points[i] - newList[i - 1]) *
                                            Gaussian_distribution(i - 1, 0, 100)))
    points = newList
    print(points[2]>1)

    plt.subplot(122)
    plt.scatter(x, points)
    plt.show()
