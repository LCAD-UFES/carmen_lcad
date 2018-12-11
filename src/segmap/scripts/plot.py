
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

FIELDS = {'gt': 5, 'pmean': 9, 'pmode': 13} 


def normalize_theta(theta):
    if theta >= -np.pi and theta < np.pi:
        return theta

    multiplier = np.floor(theta / (2 * np.pi))
    theta = theta - multiplier * 2 * np.pi

    if theta >= np.pi:
        theta -= 2 * np.pi
    if theta < -np.pi:
        theta += 2 * np.pi
    return theta


def angle_mean(angles):
    return np.arctan2(np.mean(np.sin(angles)), np.mean(np.cos(angles)))


def angle_diff(th1, th2):
    return np.asarray([normalize_theta(x) for x in th1 - th2]) 


def compare_and_print_stats(localizer_poses, gt_poses, localizer_label, gt_label):
    dx = np.abs(localizer_poses[:, 0] - gt_poses[:, 0])
    dy = np.abs(localizer_poses[:, 1] - gt_poses[:, 1])
    dth = np.abs(angle_diff(localizer_poses[:, 1], gt_poses[:, 1]))
    dists = np.sqrt(dx ** 2 + dy ** 2)
    
    print(localizer_label + '_' + gt_label)
    print('RMSE:', np.mean(dists), 'Std:', np.std(dists), 'MaxDist:', np.max(dists))
    print('RMSE_X:', np.mean(dx), 'RMSE_Y:', np.mean(dy), 'RMSE_TH:', np.rad2deg(angle_mean(dth)))
    print('% losses:', 100. * np.mean(dists > 1.))
    print()


def generate_reports(lines, optimized_poses):
    pmeans = lines[:, FIELDS['pmean']:FIELDS['pmean']+3].astype(np.float)
    pmodes = lines[:, FIELDS['pmode']:FIELDS['pmode']+3].astype(np.float)     
    gps_gts = lines[:, FIELDS['gt']:FIELDS['gt']+3].astype(np.float)
    graph_gts = optimized_poses[:, 1:4].astype(np.float)

    offset_x = 7757677.517731
    offset_y = -363602.117405
    graph_gts[:, 0] -= offset_x
    graph_gts[:, 1] -= offset_y
    graph_gts[:, 1] = -graph_gts[:, 1]
    graph_gts[:, 2] = -graph_gts[:, 2]
    print(graph_gts)

    compare_and_print_stats(pmeans, gps_gts, 'mean', 'gps')
    compare_and_print_stats(pmodes, gps_gts, 'mode', 'gps')
    compare_and_print_stats(pmeans, graph_gts, 'mean', 'graphslam')
    compare_and_print_stats(pmodes, graph_gts, 'mode', 'graphslam')

    """
    plt.figure()
    mpl.style.use('seaborn')
    plt.title('Dist')
    plt.plot(means, '-r')
    plt.plot(modes, '-b')
    plt.figure()
    mpl.style.use('seaborn')
    plt.title('Orientation')
    plt.plot(np.rad2deg(omeans), '-r')
    plt.plot(np.rad2deg(omodes), '-b')
    plt.show()
    """


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("use python %s <result_name> <path_dataset>" % sys.argv[0])
    else:
        lines = open(sys.argv[1], 'r').readlines()
        lines = [l.rstrip().rsplit() for l in lines if 'Step' in l]
        optimized_poses = open(sys.argv[2] + "/optimized.txt", 'r').readlines()
        optimized_poses = [p.rstrip().rsplit() for p in optimized_poses][1:]

        lines = np.asarray(lines)
        optimized_poses = np.asarray(optimized_poses)
        
        generate_reports(lines, optimized_poses)

