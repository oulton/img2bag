import time, sys, os
from ros import rosbag
import roslib
import rospy
import cv2
roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# import ImageFile
from PIL import ImageFile
from PIL import Image as ImagePIL


'''get image from dir'''
def GetFilesFromDir(dir):
    '''Generates a list of files from the directory'''
    print( "Searching directory %s" % dir )
    color_image = []
    multi_imgs = []

    if os.path.exists(dir):
        for path, names, files in os.walk(dir + 'color-images'):
            for f in sorted(files):
                color_image.append( os.path.join( path, f ) )
        for path, names, files in os.walk(dir + 'multi-images'):
            for f in sorted(files):
                multi_imgs.append( os.path.join( path, f ) )

    return color_image, multi_imgs

def CreateMonoBag(color_imgs, multi_imgs, bagname, timestamps):
    
    '''read time stamps'''
    file = open(timestamps, 'r')
    timestampslines = file.readlines()
    file.close()
    '''Creates a bag file with camera images'''
    bag =rosbag.Bag(bagname, 'w')
    cb = CvBridge()

    try:
        for i in range(len(color_imgs)):
            # A factor of 10 is used to adjust the time.txt timestamp
            Stamp = rospy.rostime.Time.from_sec(10*float(timestampslines[i]))
            #print(Stamp)

            img_color = cv2.imread(color_imgs[i], cv2.IMREAD_UNCHANGED)
            print("Adding %s" % color_imgs[i])

            multi_color = cv2.imread(multi_imgs[i], cv2.IMREAD_UNCHANGED)
            print("Adding %s" % multi_imgs[i])

            image_color = cb.cv2_to_imgmsg(img_color, encoding='bgr8')
            image_color.header.stamp = rospy.rostime.Time.from_sec(10*float(timestampslines[i]))
            image_color.header.frame_id = "camera"

            multi_color = cb.cv2_to_imgmsg(multi_color, encoding='16UC1')
            multi_color.header.stamp = rospy.rostime.Time.from_sec(10*float(timestampslines[i]))
            multi_color.header.frame_id = "camera"

            bag.write('/camera/color/image_raw', image_color, Stamp)
            bag.write('/camera/multi/image_raw', multi_color, Stamp)

            
    finally:
        bag.close()       


def CreateBag(args):
    '''Creates the actual bag file by successively adding images'''
    color_imgs, multi_imgs = GetFilesFromDir(args[0])
    #print(args)

    # create bagfile 
    CreateMonoBag(color_imgs, multi_imgs, args[1], args[2])    

if __name__ == "__main__":
    if len( sys.argv ) == 4:
        CreateBag(sys.argv[1:])
    else:
        print( "Usage: img2bag_Stereo.py imagedir bagfilename timestamp")
