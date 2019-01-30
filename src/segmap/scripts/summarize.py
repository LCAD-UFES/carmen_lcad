
import os
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

def log_size(gt_poses):
    d = 0
    for i in range(1, len(gt_poses)):
        dx = gt_poses[i, 0] - gt_poses[i-1, 0]
        dy = gt_poses[i, 1] - gt_poses[i-1, 1]
        d += np.sqrt(dx ** 2 + dy ** 2) 
    return d


def compute_local_diffs(gt_poses, localizer_poses):
    dx, dy = [], []
    for i in range(len(localizer_poses)):
        x, y, th = localizer_poses[i]

        c, s = np.cos(-th), np.sin(-th)
        R = np.matrix('{} {}; {} {}'.format(c, -s, s, c))
        
        t = R * np.array([-x, -y]).reshape((2, 1))
        diff = R * gt_poses[i].reshape((2, 1)) + t

        #print(x, y, th, gt_poses[i], diff)

        dx.append(diff[0, 0])
        dy.append(diff[1, 0])

    return dx, dy


def compare_and_save_stats(localizer_poses, gt_poses, localizer_label, gt_label, log_name):
    dx = localizer_poses[:, 0] - gt_poses[:, 0]
    dy = localizer_poses[:, 1] - gt_poses[:, 1]
    #dth = angle_diff(localizer_poses[:, 1], gt_poses[:, 1])
    dists = np.sqrt(dx ** 2 + dy ** 2)
    
    dx_local, dy_local = compute_local_diffs(gt_poses, localizer_poses)

    f = open('/home/filipe/stats_basic_' + log_name, 'w')
    f.write('RMSE: %f Std: %f MaxDist: %f DX_LOCAL: %f DY_LOCAL: %f\n' % (
        np.mean(dists), np.std(dists), np.max(dists), 
        np.mean(np.abs(dx_local)), np.mean(np.abs(dy_local))
    ))

    f.close()

    print(log_name.replace('data_log', '')[1:], '&', 
        '%.1f & %.1f & %.1f & %.1f & %.1f & %.1f \\\\' % (
        log_size(gt_poses) / 1000., 
        np.mean(dists),  
        np.std(dists), 
        np.max(dists), 
        np.mean(np.abs(dx_local)), 
        np.mean(np.abs(dy_local)), 
    ))

    #print('RMSE_X:', np.mean(dx), 'RMSE_Y:', np.mean(dy), 'RMSE_TH:', np.rad2deg(angle_mean(dth)))
    #print('% losses:', 100. * np.mean(dists > 1.))

    f = open('/home/filipe/stats_complement_' + log_name, 'w')

    for i in range(len(dx)):
        f.write(str(dx_local[i]) + ' ' + str(dy_local[i]) + '\n')

    f.close()


def generate_reports(lines, sync, log_name):
    pmeans = lines[:, FIELDS['pmean']:FIELDS['pmean']+3].astype(np.float)
    gps_gts = sync[:, 13:15].astype(np.float)[1:]

    offset_x = 7757677.517731
    offset_y = -363602.117405
    gps_gts[:, 0] -= offset_x
    gps_gts[:, 1] -= offset_y
    gps_gts[:, 1] = -gps_gts[:, 1]
    #gps_gts[:, 2] = -graph_gts[:, 2]
    #print(graph_gts)

    compare_and_save_stats(pmeans, gps_gts, 'mean', 'gps', log_name)
    #compare_and_print_stats(pmodes, gps_gts, 'mode', 'gps')
    #compare_and_print_stats(pmeans, graph_gts, 'mean', 'graphslam')
    #compare_and_print_stats(pmodes, graph_gts, 'mode', 'graphslam')

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
    d1 = 'results/0006_rerun_com_parametros_do_paper/'
    reports = [f for f in os.listdir(d1) if 'report_data' in f]

    print('Log Size (km) & RMSE & STD & max & RMSE X & RMSE Y')

    for r in reports:
        print('')
        log_name = 'data' + r.rsplit('data')[1][:-1]
        sync = '/dados/data/%s/sync.txt' % log_name

        lines = open(d1 + '/' + r, 'r').readlines()
        lines = [l.rstrip().rsplit() for l in lines if 'Step' in l]

        gt = open(sync, 'r').readlines()
        gt = [p.rstrip().rsplit() for p in gt]

        lines = np.asarray(lines)
        gt = np.asarray(gt)
    
        generate_reports(lines, gt, log_name)



