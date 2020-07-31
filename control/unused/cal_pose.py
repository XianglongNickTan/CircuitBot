import pybullet as p

# x = 0.14578
# y = 0.98924
# z = -0.0085346
# w = 0.0084136

# x = 0.14578
# y = 0.98924
# z = -0.00853
# w = 0.00841
# #
# a = p.getEulerFromQuaternion([x, y, z, w])
#
# print(a)


b = p.getQuaternionFromEuler([-3.05, 0, 2.8])
#
#
print ("wpose.orientation.x =", b[0])
print ("wpose.orientation.y =", b[1])
print ("wpose.orientation.z =", b[2])
print ("wpose.orientation.w =", b[3])
