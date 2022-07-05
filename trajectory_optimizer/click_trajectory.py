import cv2
import numpy as np
import os
import scipy.ndimage
from scipy import spatial
import yaml
import matplotlib.pyplot as plt
from scipy import interpolate


class Traj_Planner():
    def __init__(self, track_name, load_points=False, points_file=None, debug=False):
        self.track_name = track_name
        self.track_points = []
        self.image_points = None
        self.mode = None
        self.insert_point = False
        self.running = True
        self.debug = debug

        self.csv_path = self.track_name + "_clicked" + ".csv"
        self.csv_path = os.path.join("../", "racelines", self.csv_path)

        if load_points:
            # Open csv and read the waypoint data
            with open(self.csv_path, 'r') as f:
                lines = (line for line in f if not line.startswith('#'))
                data = np.loadtxt(lines, delimiter=',')
            self.image_points = data[:, 2:4].astype(np.int)

        # Read settings and info from yaml file
        yaml_name = os.path.join(os.path.dirname(os.getcwd()), "maps", self.track_name + ".yaml")
        with open(yaml_name, 'r') as stream:
            yaml_dict = yaml.safe_load(stream)
        self.resolution = yaml_dict["resolution"]
        self.origin = yaml_dict["origin"]
        self.origin = self.origin[:2]
        self.image_name = yaml_dict["image"]

        # Load image
        self.img = cv2.imread(os.path.join(os.path.dirname(os.getcwd()), "maps", self.image_name), cv2.IMREAD_GRAYSCALE)

        # Display image
        new_img = self.img.copy()
        if load_points:
            self.draw_spline(new_img)
        cv2.imshow('image', new_img)
        cv2.waitKey(0)
        print("####SPLINE CLICKER####")
        print("Mode 0: Insert points")
        print("Mode 1: Delete points")
        print("Mode 2: Insert between points")
        print("Mode 3: Save and write points")
        print("Press esc to change mode")


    def edit_points(self, mode):
        new_img = self.img.copy()
        if np.any(self.image_points):
            for i in range(self.image_points.shape[0]):
                cv2.circle(new_img, (self.image_points[i, 0], self.image_points[i, 1]), 4, (0, 0, 255), -1)
        cv2.imshow('image', new_img)
        self.mode = mode

        # setting mouse handler for the image
        # and calling the click_event() function
        cv2.setMouseCallback('image', self.click_event)

        # wait for a key to be pressed to exit
        cv2.waitKey(0)



    def click_event(self, event, x, y, flags, params):
        # checking for left mouse clicks
        if self.mode == 0:
            if event == cv2.EVENT_LBUTTONDOWN:
                if not np.any(self.image_points):
                    self.image_points = np.array((x, y)).reshape(1, 2)
                    self.image_points = np.append(self.image_points, [[x, y]], axis=0)
                else:
                    self.image_points = self.image_points[:-1, :]  # Delete duplicate point
                    self.image_points = np.append(self.image_points, [[x, y]], axis=0)
                    self.image_points = np.append(self.image_points, [self.image_points[0, :]], axis=0)
                # displaying the coordinates
                # on the Shell
                if self.debug:
                    print("Added point")
                # displaying the coordinates
                # on the image window
                new_img = self.img.copy()
                for i in range(self.image_points.shape[0]):
                    cv2.circle(new_img, (self.image_points[i, 0], self.image_points[i, 1]), 4, (0, 0, 255), -1)

                new_img = self.draw_spline(new_img)
                cv2.imshow('image', new_img)
        if self.mode == 1:
            if event == cv2.EVENT_LBUTTONDOWN:
                point = np.array([x, y])
                self.image_points = self.image_points[:-1, :]  # Delete duplicate point

                # Find closest point
                dist_array = self.image_points - point
                idx = np.argmin(np.linalg.norm(dist_array, axis=1))
                self.image_points = np.delete(self.image_points, idx, 0)
                self.image_points = np.append(self.image_points, [self.image_points[0, :]], axis=0)

                new_img = self.img.copy()
                for i in range(self.image_points.shape[0]):
                    cv2.circle(new_img, (self.image_points[i, 0], self.image_points[i, 1]), 3, (0, 0, 255), -1)

                new_img = self.draw_spline(new_img)
                cv2.imshow('image', new_img)
                if self.debug:
                    print("Point deleted")

        if self.mode == 2:
            if event == cv2.EVENT_LBUTTONDOWN:
                if not self.insert_point:
                    # delete point
                    point = np.array([x, y])
                    self.image_points = self.image_points[:-1, :]  # Delete duplicate point

                    # Find closest point
                    dist_array = self.image_points - point
                    idx = np.argmin(np.linalg.norm(dist_array, axis=1))
                    self.insert_idx = idx
                    self.image_points = np.append(self.image_points, [self.image_points[0, :]], axis=0)
                    self.insert_point = True
                    if self.debug:
                        print("point identified")
                else:
                    point = np.array([x, y])
                    self.image_points = self.image_points[:-1, :]  # Delete duplicate point
                    self.image_points = np.insert(self.image_points, self.insert_idx + 1, point, axis=0)
                    self.image_points = np.append(self.image_points, [self.image_points[0, :]], axis=0)
                    self.insert_point = False
                    new_img = self.img.copy()
                    for i in range(self.image_points.shape[0]):
                        cv2.circle(new_img, (self.image_points[i, 0], self.image_points[i, 1]), 3, (0, 0, 255), -1)

                    new_img = self.draw_spline(new_img)
                    cv2.imshow('image', new_img)

        if self.mode == 3:
            if event == cv2.EVENT_LBUTTONDOWN:
                cv2.destroyAllWindows()
                self.running = False

                x, y = self.image2real()
                with open(self.csv_path , "w") as f:
                    f.write("# x_m,y_m, x_i, y_i\n")
                    for i in range(len(x)):
                        f.write("{},{},{},{}\n".format(x[i], y[i], self.image_points[i, 0], self.image_points[i, 1]))
                    print("Wrote to file")

    def image2real(self):
        max_y = self.img.shape[0]
        x = (self.image_points[:, 0] * self.resolution + self.origin[0]) * 10
        y = ((max_y - self.image_points[:, 1]) * self.resolution + self.origin[1]) * 10
        return x, y

    def draw_spline(self, new_img):
        if (self.image_points.shape[0] > 6):
            spline_data, m = interpolate.splprep([self.image_points[:, 0], self.image_points[:, 1]], s=0, per=True)
            self.x_spline, self.y_spline = interpolate.splev(np.linspace(0, 1, 1000), spline_data)
            self.spline_points = np.vstack((self.x_spline, self.y_spline, np.zeros((len(self.y_spline)))))
            for i in range(self.spline_points.shape[1]):
                cv2.circle(new_img, (int(self.spline_points[0, i]), int(self.spline_points[1, i])), 1, (0, 255, 255),
                           -1)
        return new_img


if __name__=="__main__":

    # Flags
    load_flag = False
    planner = Traj_Planner('0425racex', load_points=False)

    while planner.running == True:
        mode = int(input("Enter mode: "))
        planner.edit_points(mode)

    # close the window
    cv2.destroyAllWindows()
