import threading


class Algorithm:
    def __init__(self):
        self.is_on = False

    def set_pick_and_place(self, pick_place):
        self.pick_place = pick_place

    def myalgorithm(self, event):
        self.pick_place.back_to_home()

