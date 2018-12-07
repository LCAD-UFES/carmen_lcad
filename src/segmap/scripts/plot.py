
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

FIELDS = {'mean': 17, 'mode': 19, 'gt': 5, 'pmean': 9, 'pmode': 13, 'omean': 21, 'omode': 23} 


def angle_mean(angles):
    return np.arctan2(np.mean(np.sin(angles)), np.mean(np.cos(angles)))


def generate_reports(lines):
    lines = np.asarray(lines)
    means = lines[:, FIELDS['mean']].astype(np.float)
    modes = lines[:, FIELDS['mode']].astype(np.float)
    gts = lines[:, FIELDS['gt']:FIELDS['gt']+3].astype(np.float)
    pmeans = lines[:, FIELDS['pmean']:FIELDS['pmean']+3].astype(np.float)
    pmodes = lines[:, FIELDS['pmode']:FIELDS['pmode']+3].astype(np.float)     
    omeans = lines[:, FIELDS['omean']].astype(np.float)     
    omodes = lines[:, FIELDS['omode']].astype(np.float)     
    
    print('RMSE Mean Particle:', np.mean(means), 'Std:', np.std(means), 'Max Dist:', np.max(means))
    print('RMSE Mode particle:', np.mean(modes), 'Std:', np.std(modes), 'Max Dist:', np.max(modes))    
    print('Mean RMSE X:', np.mean(np.abs(gts[:,0] - pmeans[:, 0])), 'Y:', np.mean(np.abs(gts[:,1] - pmeans[:, 1])), 'TH:', np.rad2deg(angle_mean(omeans)))
    print('Mode RMSE X:', np.mean(np.abs(gts[:,0] - pmodes[:, 0])), 'Y:', np.mean(np.abs(gts[:,1] - pmodes[:, 1])), 'TH:', np.rad2deg(angle_mean(omodes)))

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


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("use python %s <result_name>" % sys.argv[0])
    else:
        lines = open(sys.argv[1], 'r').readlines()
        lines = [l.rstrip().rsplit() for l in lines if 'Step' in l]
        generate_reports(lines)

