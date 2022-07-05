import cv2
import numpy as np
import os
import scipy.ndimage
from scipy import spatial
import yaml
import matplotlib.pyplot as plt


def rdp(points, epsilon=0.5):
    # Referenced and adapted from https://rdp.readthedocs.io/en/latest/
    # get the start and end points
    start = np.tile(np.expand_dims(points[0], axis=0), (points.shape[0], 1))
    end = np.tile(np.expand_dims(points[-1], axis=0), (points.shape[0], 1))

    # find distance from other_points to line formed by start and end
    dist_point_to_line = np.abs(np.cross(points - start, end - start)) / np.linalg.norm(end - start, axis=-1)
    # get the index of the points with the largest distance
    max_idx = np.argmax(dist_point_to_line)
    max_value = dist_point_to_line[max_idx]

    result = []
    if max_value > epsilon:
        partial_results_left = rdp(points[:max_idx+1], epsilon)
        result += [list(i) for i in partial_results_left if list(i) not in result]
        partial_results_right = rdp(points[max_idx:], epsilon)
        result += [list(i) for i in partial_results_right if list(i) not in result]
    else:
        result += [points[0], points[-1]]

    return result

def interp_contour(contour, interp_pixels):
    """
    Adds extra points to the contour. This is done by using np linspace between each point
    :param contour: nx2 array of contour points
    :param interp_pixels: how often to interpolate, in pixels
    :return: nx2 array of interpolated contour points
    """
    dist = np.linalg.norm(contour - np.roll(contour, -1, axis=0), axis=1)
    num_interp = np.round(dist / interp_pixels).astype(int) + 2  # +2 to make sure we have enough points
    contour_new = contour.copy()

    for i in range(len(dist) - 1):
        pointsx = np.linspace(contour[i, 0], contour[i+1, 0], num_interp[i])
        pointsy = np.linspace(contour[i, 1], contour[i+1, 1], num_interp[i])

        points = np.stack((pointsx,pointsy), axis=1)
        points = points[1:-1]
        insert_idx = np.argmax((contour_new == contour[i, :]).all(axis=1)) + 1
        contour_new = np.insert(contour_new, insert_idx, points.reshape(-1, 2), axis=0)

    return contour_new

def remove_close_points(points, close_thresh):
    done = False
    # Remove close points
    for i in range(len(points)):
        dist = np.linalg.norm(points[i, :] - points, axis=1)
        keep_idx = dist > close_thresh
        keep_idx[dist == 0] = True
        if np.all(keep_idx):
            #If we keep all the points
            if i == len(points) - 1:
                done = True
        else:
            points = points[keep_idx, :]
            break

    return points, done

def find_and_save_centerline(map_name, display_images=True, write_csv=True, csv_scaler=10, point_space=50, epsilon_val = .5, close_point_thresh=40):
    """
    Finds the centerline of the map and saves it to a csv file
    :param map_name: name of the map to find the centerline of
    :param display_images: whether to display images of the centerline
    :param write_csv: whether to write the centerline to a csv file, writes to the map named "traj_optimizer_inputs"
    :param csv_scaler: how much to scale the csv file by
    :param point_space: how many pixels between each interpolated point
    :param epsilon_val: ramer-douglas-peucker epsilon value, don't worry about this too much
    :param close_point_thresh: how close points can be to each other to be considered the same point
    :return: Nothing, saves the centerline to a csv file
    """

    # Read settings and info from yaml file
    yaml_name = os.path.join(os.path.dirname(os.getcwd()), "maps", map_name + ".yaml")
    with open(yaml_name, 'r') as stream:
        yaml_dict = yaml.safe_load(stream)
    resolution = yaml_dict["resolution"]
    origin = yaml_dict["origin"]
    origin = origin[:2]
    image_name = yaml_dict["image"]

    # Load image
    src = cv2.imread(os.path.join(os.path.dirname(os.getcwd()), "maps", image_name), cv2.IMREAD_GRAYSCALE)
    # rotate image
    src = cv2.rotate(src, cv2.ROTATE_180)
    src = cv2.flip(src, 1)

    # Threshold for walls only
    thresh = cv2.threshold(src, 240, 255, cv2.THRESH_BINARY)[1]
    rgb_thresh = cv2.cvtColor(thresh, cv2.COLOR_GRAY2RGB)
    rgb_thresh_copy = rgb_thresh.copy()

    # Find contours
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # find the 2 largest contours by the area
    contours_sorted = sorted(contours, key=cv2.contourArea, reverse=True)
    outer_contour = contours_sorted[0]
    inner_contour = contours_sorted[1]
    cv2.drawContours(rgb_thresh, np.expand_dims(outer_contour, axis=1), -1, (0, 255, 0), 3)
    cv2.drawContours(rgb_thresh, np.expand_dims(inner_contour, axis=1), -1, (0, 255, 255), 3)
    plt.imshow(cv2.cvtColor(rgb_thresh, cv2.COLOR_BGR2RGB))
    plt.show()

    # Use RDP to simplify the contour
    outer_contour = outer_contour[:, 0, :]
    inner_contour = inner_contour[:, 0, :]
    mid_num_inner = round(inner_contour.shape[0] / 2)
    mid_num_outer = round(outer_contour.shape[0] / 2)

    outer_contour = np.append(np.array(rdp(outer_contour[:mid_num_outer, :], epsilon=epsilon_val)),
                              np.array(rdp(outer_contour[mid_num_outer:, :], epsilon=epsilon_val)), axis=0)
    inner_contour = np.append(np.array(rdp(inner_contour[:mid_num_inner, :], epsilon=epsilon_val)),
                              np.array(rdp(inner_contour[mid_num_inner:, :], epsilon=epsilon_val)), axis=0)

    cv2.drawContours(rgb_thresh, np.expand_dims(outer_contour, axis=1), -1, (0, 255, 0), 3)
    cv2.drawContours(rgb_thresh, np.expand_dims(inner_contour, axis=1), -1, (0, 255, 255), 3)

    # Add extra points to the contour
    outer_contour = np.append(outer_contour, outer_contour[0, :].reshape(-1, 2), axis=0)
    inner_contour = np.append(inner_contour, inner_contour[0, :].reshape(-1, 2), axis=0)
    outer_contour = interp_contour(outer_contour, point_space)
    inner_contour = interp_contour(inner_contour, point_space)

    # Remove points too close
    done = False
    while not done:
        outer_contour, done = remove_close_points(outer_contour, close_point_thresh)

    # done = False
    # while not done:
    #    inner_contour, done = remove_close_points(inner_contour, close_thresh)

    # Find the nearest point for each point in the outer contour
    dist, idx = spatial.KDTree(inner_contour).query(outer_contour)
    p = ((outer_contour - inner_contour[idx]) / 2) + inner_contour[idx]
    width = np.multiply(dist.reshape(len(dist), 1), np.ones((len(outer_contour), 2)))

    # Display center points
    for point in p:
        cv2.circle(rgb_thresh_copy, (int(point[0]), int(point[1])), 5, (255, 0, 0), -1)

    if display_images:
        cv2.drawContours(rgb_thresh_copy, np.expand_dims(outer_contour, axis=1), -1, (0, 255, 0), 3)
        cv2.drawContours(rgb_thresh_copy, np.expand_dims(inner_contour, axis=1), -1, (0, 255, 255), 3)
        plt.imshow(cv2.cvtColor(rgb_thresh_copy, cv2.COLOR_BGR2RGB))
        plt.show()

    # Write to csv
    if write_csv:
        x = (p[:, 0] * resolution + origin[0]) * csv_scaler
        y = (p[:, 1] * resolution + origin[1]) * csv_scaler
        w_inner = width[:, 0] * resolution * csv_scaler * .4
        w_outer = width[:, 1] * resolution * csv_scaler * .25

        cwd = os.path.dirname(os.getcwd())
        directory = os.path.join(cwd, "maps", "csv_maps")
        csv_name = image_name[:-4] + ".csv"
        path = os.path.join(directory, csv_name)
        if not os.path.exists(os.path.join(directory)):
            os.makedirs(os.path.join(directory))
        with open(path, "w") as f:
            f.write("# x_m,y_m,w_tr_right_m,w_tr_left_m\n")
            for i in range(len(x)):
                f.write("{},{},{},{}\n".format(x[i], y[i], w_inner[i], w_outer[i]))
            print("Wrote to {}".format(image_name[:-4] + ".csv"))
            print(f"Included a scale factor of {csv_scaler}")

    cwd = os.getcwd()
    csv_name = image_name[:-4] + ".csv"
    path = os.path.join(cwd, "traj_optimizer_inputs", "tracks", csv_name)
    if not os.path.exists(os.path.join(cwd, directory)):
        os.makedirs(os.path.join(cwd, directory))
    with open(path, "w") as f:
        f.write("# x_m,y_m,w_tr_right_m,w_tr_left_m\n")
        for i in range(len(x)):
            f.write("{},{},{},{}\n".format(x[i], y[i], w_inner[i], w_outer[i]))
        print("Wrote to {}".format(image_name[:-4] + ".csv"))
        print(f"Included a scale factor of {csv_scaler}")

    return True

if __name__ == '__main__':
    print(f"Don't run me!")




# See PyCharm help at https://www.jetbrains.com/help/pycharm/
