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
    all_image = []

    if os.path.exists(dir):
        for path, names, files in os.walk(dir):
            for f in sorted(files):
                all_image.append( os.path.join( path, f ) )
    return all_image

def CreateMonoBag(imgs, bagname, timestamps):
    
    '''read time stamps'''
    file = open(timestamps, 'r')
    timestampslines = file.readlines()
    file.close()
    '''Creates a bag file with camera images'''
    bag =rosbag.Bag(bagname, 'w')
    cb = CvBridge()

    try:
        for i in range(len(imgs)):
            # A factor of 10 is used to adjust the time.txt timestamp
            Stamp = rospy.rostime.Time.from_sec(10*float(timestampslines[i]))
            #print(Stamp)

            img = cv2.imread(imgs[i], cv2.IMREAD_UNCHANGED)
            print("Adding %s" % imgs[i])

            image = cb.cv2_to_imgmsg(img, encoding='16UC1')
            image.header.stamp = rospy.rostime.Time.from_sec(10*float(timestampslines[i]))
            image.header.frame_id = "camera"

            bag.write('camera/image_raw', image, Stamp)

            
    finally:
        bag.close()       


def CreateBag(args):
    '''Creates the actual bag file by successively adding images'''
    all_imgs = GetFilesFromDir(args[0])
    #print(args)

    # create bagfile 
    CreateMonoBag(all_imgs, args[1], args[2])    

if __name__ == "__main__":
    if len( sys.argv ) == 4:
        CreateBag(sys.argv[1:])
    else:
        print( "Usage: img2bag.py imagedir bagfilename timestamp")
