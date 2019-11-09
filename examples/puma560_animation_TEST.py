import robopy.base.model as model
import numpy as np
import robopy.base.transforms as tr

def main():
    blueprint = np.array([
        [[1, 0, 0], [1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [1, 1, 1]],
        [[1, 0, 0], [1, 0, 0], [1, 0, 0], [1, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 0, 0], [1, 1, 0], [0, 0, 0], [1, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 0, 0], [1, 0, 0], [0, 0, 0], [1, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 0, 0], [0, 0, 0], [1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
    ])

    robot = model.Puma560_TEST(base=np.matrix([[1, 0, 0, 0.5],
                                              [0, 1, 0, 0.5],
                                              [0, 0, 1, 1],
                                              [0, 0, 0, 1]]), blueprint=blueprint)

    num_steps = 60
    a = np.transpose(np.asmatrix(np.linspace(1, -180, num_steps)))
    b = np.transpose(np.asmatrix(np.linspace(1, 180, num_steps)))
    c = np.transpose(np.asmatrix(np.linspace(1, 90, num_steps)))



    p1 = np.transpose(np.asmatrix(np.linspace(-6.33614205e-16, 2.07242110e-16, num_steps)))
    p2 = np.transpose(np.asmatrix(np.linspace(2.59624572e+01, 1.85638575e+01, num_steps)))
    p3 = np.transpose(np.asmatrix(np.linspace(-3.23802002e+01, -3.71277150e+01, num_steps)))
    p4 = np.transpose(np.asmatrix(np.linspace(-8.35353681e+01, -7.13892536e+01, num_steps)))

    e = np.asmatrix(np.zeros((num_steps, 1)))

    forward_1 = np.transpose(np.asmatrix(np.linspace(1, 30, num_steps)))
    forward_2 = np.transpose(np.asmatrix(np.linspace(1, -60, num_steps)))
    forward_3 = np.transpose(np.asmatrix(np.linspace(1, -50, num_steps)))

    # forward_1 = np.transpose(np.asmatrix(np.linspace(1, 155, num_steps)))
    # forward_2 = np.transpose(np.asmatrix(np.linspace(1, 56, num_steps)))
    # forward_3 = np.transpose(np.asmatrix(np.linspace(1, 150, num_steps)))

    # back_1 = np.transpose(np.asmatrix(np.linspace(30, 139, num_steps)))
    # back_2 = np.transpose(np.asmatrix(np.linspace(-60, 89, num_steps)))



    back_1 = np.transpose(np.asmatrix(np.linspace(30, 50, num_steps)))
    back_2 = np.transpose(np.asmatrix(np.linspace(-60, -100, num_steps)))
    back_3 = np.transpose(np.asmatrix(np.linspace(-50, -40, num_steps)))

    # d = np.transpose(np.asmatrix(np.linspace(1, 450, num_steps)))
    e = np.asmatrix(np.zeros((num_steps, 1)))
    g = np.concatenate((e, back_1, back_2, back_3), axis=1)
    f = np.concatenate((e, forward_1, forward_2, forward_3), axis=1)
    h = np.concatenate((e, back_1, back_2, back_3), axis=1)
    # g = np.concatenate((f[-1, 0], f[-1, 1], f[-1, 2], back_3), axis=1)
    p_test = np.concatenate((p1, p2, p3, p4), axis=1)

    two_step = np.concatenate((f, h, f, h))
    robot.animate(stances=p_test, frame_rate=30, unit='deg', num_steps=num_steps)
    # robot.plot(stance=f[-1], unit='deg')

    ik_angles = robot.ikineConstrained([3.5, 0.5, 1])*180/np.pi
    print(ik_angles)

    forward_1 = np.transpose(np.asmatrix(np.linspace(1, ik_angles[0], num_steps)))
    forward_2 = np.transpose(np.asmatrix(np.linspace(1, ik_angles[1], num_steps)))
    forward_3 = np.transpose(np.asmatrix(np.linspace(1, ik_angles[2], num_steps)))
    forward_4 = np.transpose(np.asmatrix(np.linspace(1, ik_angles[3], num_steps)))

    ik_test = np.concatenate((forward_1, forward_2, forward_3, forward_4), axis=1)

    # ik_angles = robot.ikineConstrained([1.5, -1.5, 1]) * 180 / np.pi
    # print(ik_angles)
    #
    # forward_1_2 = np.transpose(np.asmatrix(np.linspace(forward_1[-1], ik_angles[0], num_steps)))
    # forward_2_2 = np.transpose(np.asmatrix(np.linspace(forward_2[-1], ik_angles[1], num_steps)))
    # forward_3_2 = np.transpose(np.asmatrix(np.linspace(forward_3[-1], ik_angles[2], num_steps)))
    # forward_4_2 = np.transpose(np.asmatrix(np.linspace(forward_4[-1], ik_angles[3], num_steps)))
    #
    # ik_test_2 = np.concatenate((forward_1_2, forward_2_2, forward_3_2, forward_4_2), axis=1)
    #
    # ik_two_step = np.concatenate((ik_test, ik_test_2))


    # robot.animate(stances=ik_test, frame_rate=30, unit='deg', num_steps=num_steps)
    # robot.plot(stance=f[-1], unit='deg')


    # print(f[-1])

    # new_base = robot.fkine(np.array([[[0, 30, -60, -50]]]), timer=0)
    # ee_pos = new_base[0:3, 3].flatten().tolist()

    # ee_pos = [[3.5, 0.5, 1]]
    # print("EE_POS: {}".format(ee_pos))
    # new_base = new_base + tr.trotz(-90, unit='deg')
    # new_base = tr.trotz(180, unit="deg", xyz=ee_pos[0])
    # new_base=tr.r2t(tr.rotx(0, unit="deg"))
    # new_base =np.matrix([[1, 0, 0, 0.5],
    #                                           [0, 1, 0, 0.5],
    #                                           [0, 0, 1, 1],
    #                                           [0, 0, 0, 1]])
    # robot = model.Puma560_TEST(base=new_base, blueprint=blueprint)

    # angles = robot.ikine(np.matrix([[1, 0, 0, 0.5],
    #                                           [0, 1, 0, 0.5],
    #                                           [0, 0, 1, 1],
    #                                           [0, 0, 0, 1]]))

    # print("Angles: {}".format(angles))
    # robot.animate(stances=h, frame_rate=30, unit='deg', gif="robot_test")
    # robot.plot(stance=h[-1], unit='deg')

if __name__ == '__main__':
    main()


