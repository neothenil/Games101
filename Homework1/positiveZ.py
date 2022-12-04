import numpy

# n = 0.1
# f = 50
n = -0.1
f = -50
persp = numpy.array(
    [[n, 0, 0, 0], [0, n, 0, 0], [0, 0, n + f, -n * f], [0, 0, 1, 0]]
)
points = [
    numpy.array([2, 0, -2, 1]),
    numpy.array([0, 2, -2, 1]),
    numpy.array([-2, 0, -2, 1]),
]
for point in points:
    print(point, end=" => ")
    homo = numpy.matmul(persp, point)
    print(homo, end=" => ")
    norm = homo / homo[-1]
    print(norm)
    print(norm, end=" => ")
    print(numpy.matmul(numpy.linalg.inv(persp), norm), end=", ")
    print(norm, end=" => ")
    print(norm * homo[-1], end=" => ")
    print(numpy.matmul(numpy.linalg.inv(persp), norm * homo[-1]))
