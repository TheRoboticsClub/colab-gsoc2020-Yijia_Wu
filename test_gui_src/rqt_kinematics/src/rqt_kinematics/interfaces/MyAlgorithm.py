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

        # pick blue ball
        object_name = "blue_ball"
        pose = self.pick_place.get_object_pose(object_name)
        print(pose.position)
        pose.position.z += 0.145

        grasp = self.pick_place.generate_grasp("vertical", pose.position, 0.27)
        self.pick_place.pickup(object_name, [grasp])

        if not event.isSet():
            return

        goal_position = self.pick_place.get_target_position("blue_target")
        x = goal_position.x
        y = goal_position.y
        z = goal_position.z+0.15
        place_pose = self.pick_place.pose2msg(0,0,0, x, y, z)
        print(place_pose.position)
        self.pick_place.place("vertical", place_pose.position)

        if not event.isSet():
            return

        self.pick_place.back_to_home()

        if not event.isSet():
            return

        # pick yellow box
        object_name = "yellow_box"
        pose = self.pick_place.get_object_pose(object_name)
        print(pose.position)
        pose.position.z += 0.16

        grasp = self.pick_place.generate_grasp("vertical", pose.position, 0.55, yaw = 90)
        self.pick_place.pickup(object_name, [grasp])

        if not event.isSet():
            return

        goal_position = self.pick_place.get_target_position("yellow_target")
        x = goal_position.x
        y = goal_position.y
        z = goal_position.z+0.15
        place_pose = self.pick_place.pose2msg(0,0,0, x, y, z)
        print(place_pose.position)
        self.pick_place.place("vertical", place_pose.position)

        if not event.isSet():
            return

        self.pick_place.back_to_home()

        if not event.isSet():
            return

        # pick red box
        object_name = "red_box"
        pose = self.pick_place.get_object_pose(object_name)
        print(pose.position)
        pose.position.z += 0.16

        grasp = self.pick_place.generate_grasp("vertical", pose.position, 0.4)
        self.pick_place.pickup(object_name, [grasp])

        if not event.isSet():
            return

        goal_position = self.pick_place.get_target_position("red_target")
        x = goal_position.x
        y = goal_position.y
        z = goal_position.z+0.15
        place_pose = self.pick_place.pose2msg(0,0,0, x, y, z)
        print(place_pose.position)
        self.pick_place.place("vertical", place_pose.position)

        if not event.isSet():
            return

        self.pick_place.back_to_home()

        if not event.isSet():
            return

        # pick yellow ball
        object_name = "yellow_ball"
        pose = self.pick_place.get_object_pose(object_name)
        print(pose.position)
        pose.position.z += 0.16

        grasp = self.pick_place.generate_grasp("vertical", pose.position, 0.55)
        self.pick_place.pickup(object_name, [grasp])

        if not event.isSet():
            return

        goal_position = self.pick_place.get_target_position("yellow_target")
        x = goal_position.x
        y = goal_position.y
        z = goal_position.z+0.15
        place_pose = self.pick_place.pose2msg(0,0,0, x, y, z)
        print(place_pose.position)
        self.pick_place.place("vertical", place_pose.position)

        if not event.isSet():
            return

        self.pick_place.back_to_home()

        if not event.isSet():
            return

        # pick green cylinder
        object_name = "green_cylinder"
        pose = self.pick_place.get_object_pose(object_name)
        print(pose.position)
        pose.position.x -= 0.13
        pose.position.z += 0.02

        grasp = self.pick_place.generate_grasp("horizontal", pose.position, 0.3)
        self.pick_place.pickup(object_name, [grasp])

        if not event.isSet():
            return

        goal_position = self.pick_place.get_target_position("green_target")
        x = goal_position.x
        y = goal_position.y
        z = goal_position.z+0.15
        place_pose = self.pick_place.pose2msg(0,0,0, x, y, z)
        print(place_pose.position)
        self.pick_place.place("vertical", place_pose.position)

        if not event.isSet():
            return

        self.pick_place.back_to_home()

        if not event.isSet():
            return

        # pick red cylinder
        object_name = "red_cylinder"
        pose = self.pick_place.get_object_pose(object_name)
        print(pose.position)
        pose.position.x -= 0.14
        pose.position.z += 0.01

        grasp = self.pick_place.generate_grasp("horizontal", pose.position, 0.45)
        self.pick_place.pickup(object_name, [grasp])

        if not event.isSet():
            return

        goal_position = self.pick_place.get_target_position("red_target")
        x = goal_position.x
        y = goal_position.y
        z = goal_position.z+0.15
        place_pose = self.pick_place.pose2msg(0,0,0, x, y, z)
        print(place_pose.position)
        self.pick_place.place("vertical", place_pose.position)

        if not event.isSet():
            return

        self.pick_place.back_to_home()

        if not event.isSet():
            return

        # pick blue box
        object_name = "blue_box"
        pose = self.pick_place.get_object_pose(object_name)
        print(pose.position)
        pose.position.z += 0.145
        pose.position.x -= 0.07

        grasp = self.pick_place.generate_grasp("horizontal", pose.position, 0.4, pitch = 60)
        self.pick_place.pickup(object_name, [grasp])

        if not event.isSet():
            return

        goal_position = self.pick_place.get_target_position("blue_target")
        x = goal_position.x
        y = goal_position.y
        z = goal_position.z+0.15
        place_pose = self.pick_place.pose2msg(0,0,0, x, y, z)
        print(place_pose.position)
        self.pick_place.place("vertical", place_pose.position)

        if not event.isSet():
            return

        self.pick_place.back_to_home()

        # pick green ball
        object_name = "green_ball"
        pose = self.pick_place.get_object_pose(object_name)
        print(pose.position)
        pose.position.z += 0.147
        pose.position.x -= 0.025

        grasp = self.pick_place.generate_grasp("horizontal", pose.position, 0.37, pitch = 80)
        self.pick_place.pickup(object_name, [grasp])

        if not event.isSet():
            return

        goal_position = self.pick_place.get_target_position("green_target")
        x = goal_position.x
        y = goal_position.y
        z = goal_position.z+0.15
        place_pose = self.pick_place.pose2msg(0,0,0, x, y, z)
        print(place_pose.position)
        self.pick_place.place("vertical", place_pose.position)

        if not event.isSet():
            return

        self.pick_place.back_to_home()