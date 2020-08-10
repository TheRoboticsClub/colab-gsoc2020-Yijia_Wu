import threading
import numpy
from pick_and_place import Pick_Place
from std_msgs.msg import Bool
import os, rospkg, yaml
import rospy


class Algorithm:
    def __init__(self):
        self.is_on = False

        self.stopevent = threading.Event()
        self.pauseevent = threading.Event()

        self.startalgorithm_sub = rospy.Subscriber("/start_algorithm", Bool, self.start_callback)
        self.stopalgorithm_sub = rospy.Subscriber("/stop_algorithm", Bool, self.stop_callback)
        self.pausealgorithm_sub = rospy.Subscriber("/pause_algorithm", Bool, self.pause_callback)
        self.stopalgorithm_pub = rospy.Publisher("/stop_algorithm", Bool, queue_size=0)

    def start_callback(self, msg):
        if msg.data == True:
            self.stopevent.set()
            self.pauseevent.set()
            self.myalgorithm()

            self.pick_place.send_message("Algorithm is finished")

            msg = Bool()
            msg.data = True
            self.stopalgorithm_pub.publish(msg)        

    def stop_callback(self, msg):
        if msg.data == True:
            self.pauseevent.clear()
            self.stopevent.clear()

    def pause_callback(self, msg):
        if msg.data == True:
            self.pauseevent.clear()
        else:
            self.pauseevent.set()

    def set_pick_and_place(self, pick_place):
        self.pick_place = pick_place

    def build_map(self):
        ############## Insert your code here ###############
        self.pick_place.send_message("Building map")

        ####################################################

    def get_object_position(self, object_name):
        ############## Insert your code here ###############

        return position
        ####################################################

    def myalgorithm(self):
        ###### code for recording video ######
        self.pick_place.buildmap()
        self.pick_place.send_message("Map is ready")
        rospy.sleep(3)

        # insert following two lines where you want to stop the algorithm 
        # with the stop button in GUI
        while (not self.pauseevent.isSet()) or (not self.stopevent.isSet()):
            if not self.stopevent.isSet():
                return

        ### find red cylinder
        object_name = "red_cylinder"
        height, width, length, shape, color = self.pick_place.get_object_info(object_name)
        self.pick_place.start_color_filter(color, 255, 100, 50, 0, 50, 0)
        self.pick_place.start_shape_filter(color, shape, width/2+0.01)
        
        ### detect object position
        position = self.pick_place.get_object_position(object_name)
        its = 0
        while its < 5:
            while position == None:
                position = self.pick_place.get_object_position(object_name)
            its += 1
            rospy.sleep(1)
        self.pick_place.stop_shape_filter(color, shape)

        ### adjust position and pick up object
        position.z = -0.2+height+0.005
        self.pick_place.pickup(object_name, position, distance = 0.15)

        while (not self.pauseevent.isSet()) or (not self.stopevent.isSet()):
            if not self.stopevent.isSet():
                return

        ### place object
        position = self.pick_place.get_target_position("target7")
        self.pick_place.place(object_name, position)

        while (not self.pauseevent.isSet()) or (not self.stopevent.isSet()):
            if not self.stopevent.isSet():
                return

        self.pick_place.back_to_home()


if __name__=="__main__":
    rospy.init_node("pick_place_basic")

    algo = Algorithm()
    algo.set_pick_and_place(Pick_Place())
    print("You can start your algorithm with GUI")

    rospy.spin()