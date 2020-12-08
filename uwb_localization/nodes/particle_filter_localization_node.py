#!/usr/bin/python2
import rospy
from uwb_localization.particle_filter_localization import ParticleFilterLocalization

if __name__ == '__main__':
    try:
        particle_filter = ParticleFilterLocalization()
    except rospy.exceptions.ROSInterruptException:
        pass
