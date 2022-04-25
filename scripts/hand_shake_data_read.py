import numpy as np
from human_robot_interaction_data import read_hh_hr_data

def read_data(joint_names, path, seg_path):
    data_p, data_q, names, times = read_hh_hr_data.read_data(path)
    extracted_data_p, extracted_data_q = [], []
    for joint in joint_names:
        idx = read_hh_hr_data.joints_dic[joint]
        extracted_data_p.append(data_p[:, idx])
        extracted_data_q.append(data_q[:, idx])
    extracted_data_p = np.stack(extracted_data_p, axis=1)
    extracted_data_q = np.stack(extracted_data_q, axis=1)
    assert extracted_data_p.ndim == 3
    assert extracted_data_p.ndim == 3
    assert extracted_data_p.shape[2] == 3
    assert extracted_data_q.shape[2] == 4
    assert extracted_data_p.shape[1] == len(joint_names)
    assert extracted_data_q.shape[1] == len(joint_names)

    extracted_data_p_seg, extracted_data_q_seg = [], []
    seg = np.load(seg_path)
    for _seg in seg:
        t_begin = _seg[0]
        t_end = _seg[0] + np.around((_seg[1]-_seg[0])/2).astype(int)
        extracted_data_p_seg.append(extracted_data_p[t_begin:t_end])
        extracted_data_q_seg.append(extracted_data_q[t_begin:t_end])

    # extracted_data_p = np.stack(extracted_data_p, axis=0)
    # extracted_data_q = np.stack(extracted_data_q, axis=0)
    return extracted_data_p_seg, extracted_data_q_seg
