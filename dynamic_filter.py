import sklearn
import scipy
import scipy.signal

import rospy


def main():
    TOPIC = '/unkown_name13'
    rospy.init_node('transformer', anonymous=True)
    listener = tf.TransformListener()
    b = scipy.signal.firwin(150, 0.004)
    quaternion_buffer = []
    z = None
    while True:
        try:
            shoulder, quaternion = listener.lookupTransform('/world', TOPIC, rospy.Time(0) )
            if z is None:
                z = scipy.signal.lfilter_zi(b, 1) * quaternion[0]
            else:
                w, z = scipy.signal.lfilter(b, 1, [quaternion[0]], zi=z)

    return


if __name__ == '__main__':
    main()
