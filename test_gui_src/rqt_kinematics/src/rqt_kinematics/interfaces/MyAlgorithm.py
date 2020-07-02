import threading
import numpy


class Algorithm:
    def __init__(self):
        self.is_on = False

    def set_pick_and_place(self, pick_place):
        self.pick_place = pick_place

    def myalgorithm(self, event):
        self.pick_place.back_to_home()

        # insert following two lines where you want to stop the algorithm 
        # with the stop button in GUI
        if not event.isSet():
            return

        # pick cylinder
        object_name = "cylinder"
        pose = self.pick_place.get_object_pose(object_name)
        print(pose.position.y)
        pose.position.z += 0.16
        self.pick_place.pickup(object_name, pose)

        if not event.isSet():
            return

        roll = 0.0
        pitch = numpy.deg2rad(90.0)
        yaw = 0.0
        x = 0
        y = 0.6
        z = pose.position.z + 0.01
        place_pose = self.pick_place.pose2msg(roll, pitch, yaw, x, y, z)
        self.pick_place.place(place_pose)

        if not event.isSet():
            return

        self.pick_place.back_to_home()

        if not event.isSet():
            return

        # pick box
        object_name = "box"
        pose = self.pick_place.get_object_pose(object_name)
        print(pose.position.y)
        pose.position.z += 0.15
        self.pick_place.pickup(object_name, pose)

        if not event.isSet():
            return

        roll = 0.0
        pitch = numpy.deg2rad(90.0)
        yaw = 0.0
        x = 0.15
        y = 0.6
        z = pose.position.z + 0.01
        place_pose = self.pick_place.pose2msg(roll, pitch, yaw, x, y, z)
        self.pick_place.place(place_pose)

        if not event.isSet():
            return

        self.pick_place.back_to_home()

        if not event.isSet():
            return

        # pick ball
        object_name = "ball"
        pose = self.pick_place.get_object_pose(object_name)
        print(pose.position.y)
        pose.position.z += 0.14
        self.pick_place.pickup(object_name, pose)

        if not event.isSet():
            return

        roll = 0.0
        pitch = numpy.deg2rad(90.0)
        yaw = 0.0
        x = -0.15
        y = 0.6
        z = pose.position.z + 0.01
        place_pose = self.pick_place.pose2msg(roll, pitch, yaw, x, y, z)
        self.pick_place.place(place_pose)

        if not event.isSet():
            return

        self.pick_place.back_to_home()

