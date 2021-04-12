#!/usr/bin/python
import sys, getopt
import os
import pandas as pd
import numpy as np
from scipy import signal
from scipy.spatial.transform import Rotation as R


def main(argv):
    inputfile = ''
    calfile = ''
    outputfile = ''
    try:
        opts, args = getopt.getopt(argv,"hi:c:o:",["ifile=", "cfile=","ofile="])
    except getopt.GetoptError:
        print('test.py -i <inputfile> -c <calfile> -o <outputfile>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
         print('test.py -i <inputfile> -c calfile -o <outputfile>')
         sys.exit()
        elif opt in ("-i", "--ifile"):
         inputfile = arg
        elif opt in ("-c", "--ifile"):
         calfile = arg
        elif opt in ("-o", "--ofile"):
         outputfile = arg


    def orientation_matrix(q0, q1, q2, q3):
       # based on https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/
       r11 = 2 * (q0 ** 2 + q1 ** 2) - 1
       r12 = 2 * (q1 * q2 - q0 * q3)
       r13 = 2 * (q1 * q3 + q0 * q2)
       r21 = 2 * (q1 * q2 + q0 * q3)
       r22 = 2 * (q0 ** 2 + q2 ** 2) - 1
       r23 = 2 * (q2 * q3 - q0 * q1)
       r31 = 2 * (q1 * q3 - q0 * q2)
       r32 = 2 * (q2 * q3 + q0 * q1)
       r33 = 2 * (q0 ** 2 + q3 ** 2) - 1

       return r11, r12, r13, r21, r22, r23, r31, r32, r33

    def compute_relative_orientation(seg, cal):
       '''
       Calculating the relative orientation between two matrices. This is used for the initial normalization
       procedure using the standing calibration
       '''
       R_11 = np.array([])
       R_12 = np.array([])
       R_13 = np.array([])
       R_21 = np.array([])
       R_22 = np.array([])
       R_23 = np.array([])
       R_31 = np.array([])
       R_32 = np.array([])
       R_33 = np.array([])

       for i in range(seg.shape[0]):
           segment = np.asmatrix([
               [np.array(seg['o11'])[i], np.array(seg['o12'])[i], np.array(seg['o13'])[i]],
               [np.array(seg['o21'])[i], np.array(seg['o22'])[i], np.array(seg['o23'])[i]],
               [np.array(seg['o31'])[i], np.array(seg['o32'])[i], np.array(seg['o33'])[i]]
           ])

           segment_cal = np.asmatrix([
               [np.array(cal['o11'])[i], np.array(cal['o12'])[i], np.array(cal['o13'])[i]],
               [np.array(cal['o21'])[i], np.array(cal['o22'])[i], np.array(cal['o23'])[i]],
               [np.array(cal['o31'])[i], np.array(cal['o32'])[i], np.array(cal['o33'])[i]]
           ])
           # normalization
           r = np.matmul(segment, segment_cal.T)

           new_orientations = np.asarray(r).reshape(-1)

           R_11 = np.append(R_11, new_orientations[0])
           R_12 = np.append(R_12, new_orientations[1])
           R_13 = np.append(R_13, new_orientations[2])
           R_21 = np.append(R_21, new_orientations[3])
           R_22 = np.append(R_22, new_orientations[4])
           R_23 = np.append(R_23, new_orientations[5])
           R_31 = np.append(R_31, new_orientations[6])
           R_32 = np.append(R_32, new_orientations[7])
           R_33 = np.append(R_33, new_orientations[8])

       return R_11, R_12, R_13, R_21, R_22, R_23, R_31, R_32, R_33

    def compute_joint_angle(df, child, parent):
       c = df[df[' jointType'] == child]
       p = df[df[' jointType'] == parent]
       ml = np.array([])
       ap = np.array([])
       v = np.array([])

       # Compute Rotation Matrix Components
       for i in range(c.shape[0]):
           segment = np.asmatrix([
               [np.array(c['n_o11'])[i], np.array(c['n_o12'])[i], np.array(c['n_o13'])[i]],
               [np.array(c['n_o21'])[i], np.array(c['n_o22'])[i], np.array(c['n_o23'])[i]],
               [np.array(c['n_o31'])[i], np.array(c['n_o32'])[i], np.array(c['n_o33'])[i]]
           ])

           reference_segment = np.asmatrix([
               [np.array(p['n_o11'])[i], np.array(p['n_o12'])[i], np.array(p['n_o13'])[i]],
               [np.array(p['n_o21'])[i], np.array(p['n_o22'])[i], np.array(p['n_o23'])[i]],
               [np.array(p['n_o31'])[i], np.array(p['n_o32'])[i], np.array(p['n_o33'])[i]]
           ])

           # transformation of segment to reference segment
           r = np.matmul(reference_segment.T, segment)
           # decomposition to Euler angles
           rotations = R.from_matrix(r).as_euler('xyz', degrees=True)
           ml = np.append(ml, rotations[0])
           ap = np.append(ap, rotations[1])
           v = np.append(v, rotations[2])

       return ml, ap, v

    def butterworth_filter(data, fc, sf, order=4):
       # data = data to be filtered
       # fc = frequency cutoff of low-pass filter (i.e. all frequency components below this threshold are kept)
       w = fc / (sf / 2)  # Normalize the frequency; sf = sample requency
       b, a = signal.butter(order, w, 'low')  # order refers to the maximum number of delay elements used in the filter
       output = signal.filtfilt(b, a, data, padlen=250)
       return output

    dir = os.getcwd()

    cal = pd.read_csv(os.path.join(dir, calfile))
    df = pd.read_csv(os.path.join(dir, inputfile))
    df['# timestamp'] = df['# timestamp'] * 10 ** -3
    cal['# timestamp'] = cal['# timestamp'] * 10 ** -3

    df_reoriented = df.copy()
    cal_reoriented = cal.copy()
    print('... Reorienting LCSs')
    # Hips
    df_reoriented.loc[df[' jointType'] == 16, ' orientation.X'] = df.loc[df[' jointType'] == 16, ' orientation.Z']
    df_reoriented.loc[df[' jointType'] == 16, ' orientation.Y'] = df.loc[df[' jointType'] == 16, ' orientation.X']
    df_reoriented.loc[df[' jointType'] == 16, ' orientation.Z'] = df.loc[df[' jointType'] == 16, ' orientation.Y']

    cal_reoriented.loc[cal[' jointType'] == 16, ' orientation.X'] = cal.loc[cal[' jointType'] == 16, ' orientation.Z']
    cal_reoriented.loc[cal[' jointType'] == 16, ' orientation.Y'] = cal.loc[cal[' jointType'] == 16, ' orientation.X']
    cal_reoriented.loc[cal[' jointType'] == 16, ' orientation.Z'] = cal.loc[cal[' jointType'] == 16, ' orientation.Y']

    df_reoriented.loc[df[' jointType'] == 12, ' orientation.X'] = df.loc[df[' jointType'] == 12, ' orientation.Z']
    df_reoriented.loc[df[' jointType'] == 12, ' orientation.Y'] = df.loc[df[' jointType'] == 12, ' orientation.X'] * -1
    df_reoriented.loc[df[' jointType'] == 12, ' orientation.Z'] = df.loc[df[' jointType'] == 12, ' orientation.Y'] * -1

    cal_reoriented.loc[cal[' jointType'] == 12, ' orientation.X'] = cal.loc[cal[' jointType'] == 12, ' orientation.Z']
    cal_reoriented.loc[cal[' jointType'] == 12, ' orientation.Y'] = cal.loc[cal[' jointType'] == 12, ' orientation.X'] * -1
    cal_reoriented.loc[cal[' jointType'] == 12, ' orientation.Z'] = cal.loc[cal[' jointType'] == 12, ' orientation.Y'] * -1

    # Knees
    df_reoriented.loc[df[' jointType'] == 17, ' orientation.X'] = df.loc[df[' jointType'] == 17, ' orientation.X'] * -1
    df_reoriented.loc[df[' jointType'] == 17, ' orientation.Y'] = df.loc[df[' jointType'] == 17, ' orientation.Y'] * -1
    df_reoriented.loc[df[' jointType'] == 17, ' orientation.Z'] = df.loc[df[' jointType'] == 17, ' orientation.Z']

    cal_reoriented.loc[cal[' jointType'] == 17, ' orientation.X'] = cal.loc[cal[' jointType'] == 17, ' orientation.X'] * -1
    cal_reoriented.loc[cal[' jointType'] == 17, ' orientation.Y'] = cal.loc[cal[' jointType'] == 17, ' orientation.Y'] * -1
    cal_reoriented.loc[cal[' jointType'] == 17, ' orientation.Z'] = cal.loc[cal[' jointType'] == 17, ' orientation.Z']

    df_reoriented.loc[df[' jointType'] == 13, ' orientation.X'] = df.loc[df[' jointType'] == 13, ' orientation.X']
    df_reoriented.loc[df[' jointType'] == 13, ' orientation.Y'] = df.loc[df[' jointType'] == 13, ' orientation.Y'] * -1
    df_reoriented.loc[df[' jointType'] == 13, ' orientation.Z'] = df.loc[df[' jointType'] == 13, ' orientation.Z'] * -1

    cal_reoriented.loc[cal[' jointType'] == 13, ' orientation.X'] = cal.loc[cal[' jointType'] == 13, ' orientation.X']
    cal_reoriented.loc[cal[' jointType'] == 13, ' orientation.Y'] = cal.loc[cal[' jointType'] == 13, ' orientation.Y'] * -1
    cal_reoriented.loc[cal[' jointType'] == 13, ' orientation.Z'] = cal.loc[cal[' jointType'] == 13, ' orientation.Z'] * -1

    # Ankles
    df_reoriented.loc[df[' jointType'] == 18, ' orientation.X'] = df.loc[df[' jointType'] == 18, ' orientation.X'] * -1
    df_reoriented.loc[df[' jointType'] == 18, ' orientation.Y'] = df.loc[df[' jointType'] == 18, ' orientation.Y'] * -1
    df_reoriented.loc[df[' jointType'] == 18, ' orientation.Z'] = df.loc[df[' jointType'] == 18, ' orientation.Z']

    cal_reoriented.loc[cal[' jointType'] == 18, ' orientation.X'] = cal.loc[cal[' jointType'] == 18, ' orientation.X'] * -1
    cal_reoriented.loc[cal[' jointType'] == 18, ' orientation.Y'] = cal.loc[cal[' jointType'] == 18, ' orientation.Y'] * -1
    cal_reoriented.loc[cal[' jointType'] == 18, ' orientation.Z'] = cal.loc[cal[' jointType'] == 18, ' orientation.Z']

    df_reoriented.loc[df[' jointType'] == 14, ' orientation.X'] = df.loc[df[' jointType'] == 14, ' orientation.X']
    df_reoriented.loc[df[' jointType'] == 14, ' orientation.Y'] = df.loc[df[' jointType'] == 14, ' orientation.Y'] * -1
    df_reoriented.loc[df[' jointType'] == 14, ' orientation.Z'] = df.loc[df[' jointType'] == 14, ' orientation.Z'] * -1

    cal_reoriented.loc[cal[' jointType'] == 14, ' orientation.X'] = cal.loc[cal[' jointType'] == 14, ' orientation.X']
    cal_reoriented.loc[cal[' jointType'] == 14, ' orientation.Y'] = cal.loc[cal[' jointType'] == 14, ' orientation.Y'] * -1
    cal_reoriented.loc[cal[' jointType'] == 14, ' orientation.Z'] = cal.loc[cal[' jointType'] == 14, ' orientation.Z'] * -1

    df_reoriented['o11'], df_reoriented['o12'], df_reoriented['o13'], df_reoriented['o21'], df_reoriented['o22'], \
    df_reoriented['o23'], df_reoriented['o31'], df_reoriented['o32'], df_reoriented['o33'] \
        = orientation_matrix(df_reoriented[' orientation.W'], df_reoriented[' orientation.X'],
                             df_reoriented[' orientation.Y'], df_reoriented[' orientation.Z'])

    cal_reoriented['o11'], cal_reoriented['o12'], cal_reoriented['o13'], cal_reoriented['o21'], cal_reoriented['o22'], \
    cal_reoriented['o23'], cal_reoriented['o31'], cal_reoriented['o32'], cal_reoriented['o33'] \
        = orientation_matrix(cal_reoriented[' orientation.W'], cal_reoriented[' orientation.X'],
                             cal_reoriented[' orientation.Y'], cal_reoriented[' orientation.Z'])

    df_reoriented.set_index(' jointType', inplace=True)
    cal_reoriented.set_index(' jointType', inplace=True)
    cal_reoriented = cal_reoriented.groupby(' jointType').mean().drop(columns=['# timestamp'])
    cal_reoriented = pd.concat([cal_reoriented] * np.int(df_reoriented.shape[0] / 25))

    print('... Normalizing to calibration pose')
    # Normalize orientations to calibration pose
    df_reoriented['n_o11'], df_reoriented['n_o12'], df_reoriented['n_o13'], df_reoriented['n_o21'], df_reoriented[
        'n_o22'], \
    df_reoriented['n_o23'], df_reoriented['n_o31'], df_reoriented['n_o32'], df_reoriented['n_o33'] \
        = np.array(compute_relative_orientation(cal_reoriented, df_reoriented))

    df_reoriented.reset_index(inplace=True)

    print('... Computing joint angles')
    r_hipFlexion, r_hipAbduction, r_hipV = compute_joint_angle(df_reoriented, child=17, parent=16)
    l_hipFlexion, l_hipAbduction, l_hipV = compute_joint_angle(df_reoriented, child=13, parent=12)
    r_kneeFlexion, r_kneeAbduction, r_kneeV = compute_joint_angle(df_reoriented, child=18, parent=17)
    l_kneeFlexion, l_kneeAbduction, l_kneeV = compute_joint_angle(df_reoriented, child=14, parent=13)

    new_df = pd.DataFrame({
        'frame': np.arange(df_reoriented['# timestamp'].unique().shape[0]),
        'timeStamp': df_reoriented['# timestamp'].unique(),
        'r_hipFlexion': r_hipFlexion,
        'l_hipFlexion': l_hipFlexion * -1,
        'r_hipAbduction': r_hipAbduction * -1,
        'l_hipAbduction': l_hipAbduction,
        'r_hipV': r_hipV * -1,
        'l_hipV': l_hipV * -1,
        'r_kneeFlexion': r_kneeFlexion * -1,
        'l_kneeFlexion': l_kneeFlexion,
        'r_kneeAdduction': r_kneeAbduction,
        'l_kneeAdduction': l_kneeAbduction * -1,
        'r_kneeV': r_kneeV * -1,
        'l_kneeV': l_kneeV
    })

    new_df.to_csv(str(outputfile))
    print('... Finished processing')

if __name__ == "__main__":
    main(sys.argv[1:])