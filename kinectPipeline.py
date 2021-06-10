#!/usr/bin/python
import sys, getopt
import os
import pandas as pd
import numpy as np
import pyquaternion as pyq
from pyquaternion import Quaternion
from scipy import signal
from scipy.spatial.transform import Slerp
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

    # Creating Functions
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

    def resample_df(d, new_freq=30, method='linear'):
        # Resamples data at 30Hz unless otherwise specified
        joints_without_quats = [3, 15, 19, 21, 22, 23, 24]
        resampled_df = pd.DataFrame(
            columns=['# timestamp', ' jointType', ' orientation.X', ' orientation.Y', ' orientation.Z',
                     ' orientation.W', ' position.X', ' position.Y', ' position.Z'])
        new_df = pd.DataFrame()
        for i in d[' jointType'].unique():
            current_df = d.loc[d[' jointType'] == i].copy()
            old_times = np.array(current_df['# timestamp'])
            new_times = np.arange(min(current_df['# timestamp']), max(current_df['# timestamp']), 1 / new_freq)

            o_x = np.array(current_df[' orientation.X'])
            o_y = np.array(current_df[' orientation.Y'])
            o_z = np.array(current_df[' orientation.Z'])
            o_w = np.array(current_df[' orientation.W'])

            p_x = np.array(current_df[' position.X'])
            p_y = np.array(current_df[' position.Y'])
            p_z = np.array(current_df[' position.Z'])

            if i in joints_without_quats:
                orientation_x = np.repeat(0.0, len(new_times))
                orientation_y = np.repeat(0.0, len(new_times))
                orientation_z = np.repeat(0.0, len(new_times))
                orientation_w = np.repeat(0.0, len(new_times))
            else:
                if method == "linear":
                    orientation_x = np.interp(new_times, old_times, o_x)
                    orientation_y = np.interp(new_times, old_times, o_y)
                    orientation_z = np.interp(new_times, old_times, o_z)
                    orientation_w = np.interp(new_times, old_times, o_w)
                elif method == 'slerp':
                    quats = []
                    for t in range(len(old_times)):
                        quats.append([o_x[t], o_y[t], o_z[t], o_w[t]])
                    # Create rotation object
                    quats_object = R.from_quat(quats)

                    # Spherical Linear Interpolation
                    slerp = Slerp(np.array(current_df['# timestamp']), quats_object)
                    interp_rots = slerp(new_times)
                    new_quats = interp_rots.as_quat()

                    # Create new orientation objects
                    orientation_x = np.array([item[0] for item in new_quats])
                    orientation_y = np.array([item[1] for item in new_quats])
                    orientation_z = np.array([item[2] for item in new_quats])
                    orientation_w = np.array([item[3] for item in new_quats])
                else:
                    raise ValueError("Method must be either linear or spherical (slerp) interpolation.")

            position_x = signal.resample(p_x, num=int(max(current_df['# timestamp']) * new_freq))
            position_y = signal.resample(p_y, num=int(max(current_df['# timestamp']) * new_freq))
            position_z = signal.resample(p_z, num=int(max(current_df['# timestamp']) * new_freq))

            new_df['# timestamp'] = pd.Series(new_times)
            new_df[' jointType'] = pd.Series(np.repeat(i, len(new_times)))

            new_df[' orientation.X'] = pd.Series(orientation_x)
            new_df[' orientation.Y'] = pd.Series(orientation_y)
            new_df[' orientation.Z'] = pd.Series(orientation_z)
            new_df[' orientation.W'] = pd.Series(orientation_w)

            new_df[' position.X'] = pd.Series(position_x)
            new_df[' position.Y'] = pd.Series(position_y)
            new_df[' position.Z'] = pd.Series(position_z)

            resampled_df = resampled_df.append(new_df, ignore_index=True)

        return resampled_df

    def smooth_rotations(o_x, o_y, o_z, o_w):
        o_x = np.array(o_x)
        o_y = np.array(o_y)
        o_z = np.array(o_z)
        o_w = np.array(o_w)
        trajNoisy = []
        for i in range(len(o_x)):
            trajNoisy.append([o_x[i], o_y[i], o_z[i], o_w[i]])

        trajNoisy = np.array(trajNoisy)
        # This code was adapted from https://ww2.mathworks.cn/help/nav/ug/lowpass-filter-orientation-using-quaternion-slerp.html

        # As explained in the link above, "The interpolation parameter to slerp is in the closed-interval [0,1], so the output of dist
        # must be re-normalized to this range. However, the full range of [0,1] for the interpolation parameter gives poor performance,
        # so it is limited to a smaller range hrange centered at hbias."
        hrange = 0.4
        hbias = 0.4
        low = max(min(hbias - (hrange / 2), 1), 0)
        high = max(min(hbias + (hrange / 2), 1), 0)
        hrangeLimited = high - low

        # initial filter state is the quaternion at frame 0
        y = trajNoisy[0]
        qout = []

        for i in range(1, len(trajNoisy)):
            x = trajNoisy[i]
            # x = mathutils.Quaternion(x)
            # y = mathutils.Quaternion(y)
            # d = x.rotation_difference(y).angle
            x = pyq.Quaternion(x)
            y = pyq.Quaternion(y)
            d = (x.conjugate * y).angle
            # Renormalize dist output to the range [low, high]

            hlpf = (d / np.pi) * hrangeLimited + low
            # y = y.slerp(x, hlpf)
            y = Quaternion.slerp(y, x, hlpf).elements
            qout.append(np.array(y))

        # because a frame of data is lost during this process, I've (arbitrarily) decided to append an extra quaternion at the end of the trial
        # that is identical to the n-1th frame. This keeps the length consistent (so there is no issues with merging later) and should not
        # negatively impact the data since the last frame is rarely of interest (and the data collector can decide to collect for a split second
        # after their trial of interest has completed to attenuate any of these "errors" that may propogate in the analyses)
        qout.append(qout[int(len(qout) - 1)])

        orientation_x = [item[0] for item in qout]
        orientation_y = [item[1] for item in qout]
        orientation_z = [item[2] for item in qout]
        orientation_w = [item[3] for item in qout]

        return orientation_x, orientation_y, orientation_z, orientation_w

    def smooth_quaternions(d):
        for i in d[' jointType'].unique():
            current_df = d.loc[d[' jointType'] == i].copy()

            current_df[' orientation.X'], current_df[' orientation.Y'], current_df[' orientation.Z'], current_df[
                ' orientation.W'] = smooth_rotations(current_df[' orientation.X'], current_df[' orientation.Y'],
                                                     current_df[' orientation.Z'], current_df[' orientation.W'])

            d[d[' jointType'] == i] = current_df

        return d

    def compute_segment_angle(df, SEGMENT):
        s = df[df[' jointType'] == SEGMENT]
        ml = np.array([])
        ap = np.array([])
        v = np.array([])

        # Compute Rotation Matrix Components
        for i in range(s.shape[0]):
            segment = np.asmatrix([
                [np.array(s['n_o11'])[i], np.array(s['n_o12'])[i], np.array(s['n_o13'])[i]],
                [np.array(s['n_o21'])[i], np.array(s['n_o22'])[i], np.array(s['n_o23'])[i]],
                [np.array(s['n_o31'])[i], np.array(s['n_o32'])[i], np.array(s['n_o33'])[i]]
            ])

            # decomposition to Euler angles
            rotations = R.from_matrix(segment).as_euler('xyz', degrees=True)
            ml = np.append(ml, rotations[0])
            ap = np.append(ap, rotations[1])
            v = np.append(v, rotations[2])

        return ml, ap, v

    dir = os.getcwd()

    # Loading Data
    print('... Loading data')
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

    # Resampling data to 30Hz
    df_reoriented = resample_df(df_reoriented, new_freq=30, method='slerp')
    # Smooth Quaternion Rotations
    df_reoriented = smooth_quaternions(df_reoriented)
    # need to re-sort and reset the index following the resampling
    df_reoriented = df_reoriented.sort_values(by=['# timestamp', ' jointType']).reset_index()

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
    cal_reoriented = pd.concat([cal_reoriented] * np.int64(df_reoriented.shape[0] / 25))

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

    # Note that 16 or 12 can be used for the pelvis (given Kinect's definitions)
    pelvis_rotation = compute_segment_angle(df_reoriented, 16)[0]
    r_thigh_rotation = compute_segment_angle(df_reoriented, 17)[0]
    l_thigh_rotation = compute_segment_angle(df_reoriented, 13)[0]
    r_shank_rotation = compute_segment_angle(df_reoriented, 18)[0]
    l_shank_rotation = compute_segment_angle(df_reoriented, 14)[0]

    new_df = pd.DataFrame({
        'frame': np.arange(df_reoriented['# timestamp'].unique().shape[0]),
        'timeStamp': df_reoriented['# timestamp'].unique(),
        'r_hipFlexion' : r_hipFlexion,
        'l_hipFlexion' : l_hipFlexion*-1,
        'r_hipAbduction' : r_hipAbduction*-1,
        'l_hipAbduction' : l_hipAbduction,
        'r_hipV' : r_hipV *-1,
        'l_hipV' : l_hipV *-1,
        'r_kneeFlexion' : r_kneeFlexion*-1,
        'l_kneeFlexion' : l_kneeFlexion,
        'r_kneeAdduction' : r_kneeAbduction,
        'l_kneeAdduction' : l_kneeAbduction*-1,
        'r_kneeV' : r_kneeV*-1,
        'l_kneeV' : l_kneeV,
        'pelvis_rotation': pelvis_rotation,
        'r_thigh_rotation': r_thigh_rotation,
        'l_thigh_rotation': l_thigh_rotation*-1,
        'r_shank_rotation': r_shank_rotation,
        'l_shank_rotation': l_shank_rotation*-1,
        'r_hip_x': np.array(df_reoriented[df_reoriented[' jointType'] == 16][' position.X']),
        'r_hip_y': np.array(df_reoriented[df_reoriented[' jointType'] == 16][' position.Y']),
        'r_hip_z': np.array(df_reoriented[df_reoriented[' jointType'] == 16][' position.Z']),
        'l_hip_x': np.array(df_reoriented[df_reoriented[' jointType'] == 12][' position.X']),
        'l_hip_y': np.array(df_reoriented[df_reoriented[' jointType'] == 12][' position.Y']),
        'l_hip_z': np.array(df_reoriented[df_reoriented[' jointType'] == 12][' position.Z']),
        'r_knee_x': np.array(df_reoriented[df_reoriented[' jointType'] == 17][' position.X']),
        'r_knee_y': np.array(df_reoriented[df_reoriented[' jointType'] == 17][' position.Y']),
        'r_knee_z': np.array(df_reoriented[df_reoriented[' jointType'] == 17][' position.Z']),
        'l_knee_x': np.array(df_reoriented[df_reoriented[' jointType'] == 13][' position.X']),
        'l_knee_y': np.array(df_reoriented[df_reoriented[' jointType'] == 13][' position.Y']),
        'l_knee_z': np.array(df_reoriented[df_reoriented[' jointType'] == 13][' position.Z']),
        'r_ankle_x': np.array(df_reoriented[df_reoriented[' jointType'] == 18][' position.X']),
        'r_ankle_y': np.array(df_reoriented[df_reoriented[' jointType'] == 18][' position.Y']),
        'r_ankle_z': np.array(df_reoriented[df_reoriented[' jointType'] == 18][' position.Z']),
        'l_ankle_x': np.array(df_reoriented[df_reoriented[' jointType'] == 14][' position.X']),
        'l_ankle_y': np.array(df_reoriented[df_reoriented[' jointType'] == 14][' position.Y']),
        'l_ankle_z': np.array(df_reoriented[df_reoriented[' jointType'] == 14][' position.Z']),
        'r_foot_x': np.array(df_reoriented[df_reoriented[' jointType'] == 19][' position.X']),
        'r_foot_y': np.array(df_reoriented[df_reoriented[' jointType'] == 19][' position.Y']),
        'r_foot_z': np.array(df_reoriented[df_reoriented[' jointType'] == 19][' position.Z']),
        'l_foot_x': np.array(df_reoriented[df_reoriented[' jointType'] == 15][' position.X']),
        'l_foot_y': np.array(df_reoriented[df_reoriented[' jointType'] == 15][' position.Y']),
        'l_foot_z': np.array(df_reoriented[df_reoriented[' jointType'] == 15][' position.Z']),
        'spinebase_x': np.array(df_reoriented[df_reoriented[' jointType'] == 0][' position.X']),
        'spinebase_y': np.array(df_reoriented[df_reoriented[' jointType'] == 0][' position.Y']),
        'spinebase_z': np.array(df_reoriented[df_reoriented[' jointType'] == 0][' position.Z']),
        'spinemid_x': np.array(df_reoriented[df_reoriented[' jointType'] == 1][' position.X']),
        'spinemid_y': np.array(df_reoriented[df_reoriented[' jointType'] == 1][' position.Y']),
        'spinemid_z': np.array(df_reoriented[df_reoriented[' jointType'] == 1][' position.Z']),
        'neck_x': np.array(df_reoriented[df_reoriented[' jointType'] == 2][' position.X']),
        'neck_y': np.array(df_reoriented[df_reoriented[' jointType'] == 2][' position.Y']),
        'neck_z': np.array(df_reoriented[df_reoriented[' jointType'] == 2][' position.Z']),
        'head_x': np.array(df_reoriented[df_reoriented[' jointType'] == 3][' position.X']),
        'head_y': np.array(df_reoriented[df_reoriented[' jointType'] == 3][' position.Y']),
        'head_z': np.array(df_reoriented[df_reoriented[' jointType'] == 3][' position.Z']),
        'r_shoulder_x': np.array(df_reoriented[df_reoriented[' jointType'] == 8][' position.X']),
        'r_shoulder_y': np.array(df_reoriented[df_reoriented[' jointType'] == 8][' position.Y']),
        'r_shoulder_z': np.array(df_reoriented[df_reoriented[' jointType'] == 8][' position.Z']),
        'r_elbow_x': np.array(df_reoriented[df_reoriented[' jointType'] == 9][' position.X']),
        'r_elbow_y': np.array(df_reoriented[df_reoriented[' jointType'] == 9][' position.Y']),
        'r_elbow_z': np.array(df_reoriented[df_reoriented[' jointType'] == 9][' position.Z']),
        'r_wrist_x': np.array(df_reoriented[df_reoriented[' jointType'] == 10][' position.X']),
        'r_wrist_y': np.array(df_reoriented[df_reoriented[' jointType'] == 10][' position.Y']),
        'r_wrist_z': np.array(df_reoriented[df_reoriented[' jointType'] == 10][' position.Z']),
        'l_shoulder_x': np.array(df_reoriented[df_reoriented[' jointType'] == 4][' position.X']),
        'l_shoulder_y': np.array(df_reoriented[df_reoriented[' jointType'] == 4][' position.Y']),
        'l_shoulder_z': np.array(df_reoriented[df_reoriented[' jointType'] == 4][' position.Z']),
        'l_elbow_x': np.array(df_reoriented[df_reoriented[' jointType'] == 5][' position.X']),
        'l_elbow_y': np.array(df_reoriented[df_reoriented[' jointType'] == 5][' position.Y']),
        'l_elbow_z': np.array(df_reoriented[df_reoriented[' jointType'] == 5][' position.Z']),
        'l_wrist_x': np.array(df_reoriented[df_reoriented[' jointType'] == 6][' position.X']),
        'l_wrist_y': np.array(df_reoriented[df_reoriented[' jointType'] == 6][' position.Y']),
        'l_wrist_z': np.array(df_reoriented[df_reoriented[' jointType'] == 6][' position.Z'])
    })

    new_df.to_csv(str(outputfile))
    print('... Finished processing')

if __name__ == "__main__":
    main(sys.argv[1:])