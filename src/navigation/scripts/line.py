#! /usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import numpy as np
from skimage.morphology import skeletonize
from skimage.util import invert
from scipy.spatial import distance as d
from scipy import ndimage
import std_msgs.msg

def DistMatrix(x): #vypocet matice urcujici vzdalenosti mezi jednotlivymi body
    dim = len(x)
    M = np.zeros((dim, dim))

    for i in range(dim):
        for j in range(i+1, dim):
            M[i][j] = d.euclidean(x[i], x[j])
            M[j][i] = M[i][j]

    return M


def sortCenterLine(v): #serazeni bodu stredove cary
    new = np.zeros((len(v), 2))
    v = np.array(v)
    idx = list(v[:, 0]).index(min(i for i in v[:, 0] if i >= 0))
    D = DistMatrix(v)
    minimum = list(D[idx]).index(np.min(D[idx][np.nonzero(D[idx])]))
    D = np.delete(D, idx, 0)
    D = np.delete(D, idx, 1)
    new[0][0] = v[idx][0]
    new[0][1] = v[idx][1]
    v = np.delete(v, idx, axis=0)
    if idx < minimum:
        idx = minimum - 1
    else:
        idx = minimum

    for i in range(1, len(new) - 1):
        minimum = list(D[idx]).index(np.min(D[idx][np.nonzero(D[idx])]))
        new[i][0] = v[idx][0]
        new[i][1] = v[idx][1]
        v = np.delete(v, idx, axis=0)
        D = np.delete(D, idx, 0)
        D = np.delete(D, idx, 1)
        if idx < minimum:
            idx = minimum - 1
        else:
            idx = minimum
    new[len(new) - 1][0] = v[0][0]
    new[len(new) - 1][1] = v[0][1]

    return new

def callback(map):
    #Prevedeni 1D pole na 2D
    grid = np.zeros((map.info.height, map.info.width))
    for i in range(0, map.info.height):
        for j in range(0, map.info.width):
            grid[i][j] = map.data[i * map.info.height + j]

    image = grid

    #Uprava mapy
    for i in range(0, map.info.height):
        for j in range(0, map.info.width):
            if grid[i][j] == 100 or grid[i][j] == -1:
                image[i][j] = 1
            else:
                image[i][j] = 0


    #Dilatace prekazek
    for i in range(0, 8):
        image = ndimage.binary_dilation(image).astype(image.dtype)

    #Vytvoreni skeletonu
    image = invert(image)
    skeleton = skeletonize(image).astype(np.uint16)

    point_pub = rospy.Publisher('/path', PointCloud, latch=True, queue_size=10)
    pc = PointCloud()

    while not rospy.is_shutdown():
        #Priprava skeletonu pro nasledne serazeni
        points = []
        v = []
        for i in range(0, len(skeleton)):
            for j in range(0, len(skeleton)):
                if skeleton[j][i] == True:
                    v.append((i, j))

        #Serazeni skeletonu
        #new = sortCenterLine(v)
        m = []
        #Prevedeni z indexu pole do souradneho systemu mapy
        for i in range(len(v)-1, 0, -1):
            p = Point32()
            #p.x = new[i][0] * map.info.resolution + map.info.origin.position.x
            #p.y = new[i][1] * map.info.resolution + map.info.origin.position.y
            p.x = v[i][0] * map.info.resolution + map.info.origin.position.x
            p.y = v[i][1] * map.info.resolution + map.info.origin.position.y
            m.append((p.x, p.y))
            p.z = 0
            #points.append(p)
        new = sortCenterLine(m)

        for i in range(len(new)-1, 0, -1):
            p = Point32()
            #p.x = new[i][0] * map.info.resolution + map.info.origin.position.x
            #p.y = new[i][1] * map.info.resolution + map.info.origin.position.y
            p.x = new[i][0]
            p.y = new[i][1]
            #m.append(p.x, p.y)
            p.z = 0
            points.append(p)

        #Poslani zpravy
        pc.header.stamp = rospy.Time.now()
        pc.header = std_msgs.msg.Header()
        pc.header.frame_id = 'map'
        pc.points = points
        point_pub.publish(pc)
        print("Done!")
        rospy.spin()

def subscribe():
    print("Creating the centerline ...")
    rospy.init_node('centerline', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid, callback)
    rospy.spin()


if __name__ == '__main__':
    '''
    Tento program vytvori v mape stredovou caru
    '''
    subscribe()
