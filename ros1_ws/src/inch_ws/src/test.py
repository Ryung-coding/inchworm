#!/usr/bin/env python3
# solver_publisher.py

import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np
from scipy import sparse
import osqp

def solve_qp_problem():
    # Discrete time model of a quadcopter
    Ad = sparse.csc_matrix([
    [1.,      0.,     0., 0., 0., 0., 0.1,     0.,     0.,  0.,     0.,     0.    ],
    [0.,      1.,     0., 0., 0., 0., 0.,      0.1,    0.,  0.,     0.,     0.    ],
    [0.,      0.,     1., 0., 0., 0., 0.,      0.,     0.1, 0.,     0.,     0.    ],
    [0.0488,  0.,     0., 1., 0., 0., 0.0016,  0.,     0.,  0.0992, 0.,     0.    ],
    [0.,     -0.0488, 0., 0., 1., 0., 0.,     -0.0016, 0.,  0.,     0.0992, 0.    ],
    [0.,      0.,     0., 0., 0., 1., 0.,      0.,     0.,  0.,     0.,     0.0992],
    [0.,      0.,     0., 0., 0., 0., 1.,      0.,     0.,  0.,     0.,     0.    ],
    [0.,      0.,     0., 0., 0., 0., 0.,      1.,     0.,  0.,     0.,     0.    ],
    [0.,      0.,     0., 0., 0., 0., 0.,      0.,     1.,  0.,     0.,     0.    ],
    [0.9734,  0.,     0., 0., 0., 0., 0.0488,  0.,     0.,  0.9846, 0.,     0.    ],
    [0.,     -0.9734, 0., 0., 0., 0., 0.,     -0.0488, 0.,  0.,     0.9846, 0.    ],
    [0.,      0.,     0., 0., 0., 0., 0.,      0.,     0.,  0.,     0.,     0.9846]
    ])
    Bd = sparse.csc_matrix([
    [0.,      -0.0726,  0.,     0.0726],
    [-0.0726,  0.,      0.0726, 0.    ],
    [-0.0152,  0.0152, -0.0152, 0.0152],
    [-0.,     -0.0006, -0.,     0.0006],
    [0.0006,   0.,     -0.0006, 0.0000],
    [0.0106,   0.0106,  0.0106, 0.0106],
    [0,       -1.4512,  0.,     1.4512],
    [-1.4512,  0.,      1.4512, 0.    ],
    [-0.3049,  0.3049, -0.3049, 0.3049],
    [-0.,     -0.0236,  0.,     0.0236],
    [0.0236,   0.,     -0.0236, 0.    ],
    [0.2107,   0.2107,  0.2107, 0.2107]])
    [nx, nu] = Bd.shape

    # Constraints
    u0 = 10.5916
    umin = np.array([9.6, 9.6, 9.6, 9.6]) - u0
    umax = np.array([13., 13., 13., 13.]) - u0
    xmin = np.array([-np.pi/6,-np.pi/6,-np.inf,-np.inf,-np.inf,-1.,
                    -np.inf,-np.inf,-np.inf,-np.inf,-np.inf,-np.inf])
    xmax = np.array([ np.pi/6, np.pi/6, np.inf, np.inf, np.inf, np.inf,
                    np.inf, np.inf, np.inf, np.inf, np.inf, np.inf])

    # Objective function
    Q = sparse.diags([0., 0., 10., 10., 10., 10., 0., 0., 0., 5., 5., 5.])
    QN = Q
    R = 0.1*sparse.eye(4)

    # Initial and reference states
    x0 = np.zeros(12)
    xr = np.array([0.,0.,1.,0.,0.,0.,0.,0.,0.,0.,0.,0.])

    # Prediction horizon
    N = 100

    # Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))
    # - quadratic objective
    P = sparse.block_diag([sparse.kron(sparse.eye(N), Q), QN,
                        sparse.kron(sparse.eye(N), R)], format='csc')
    # - linear objective
    q = np.hstack([np.kron(np.ones(N), -Q@xr), -QN@xr, np.zeros(N*nu)])
    # - linear dynamics
    Ax = sparse.kron(sparse.eye(N+1),-sparse.eye(nx)) + sparse.kron(sparse.eye(N+1, k=-1), Ad)
    Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, N)), sparse.eye(N)]), Bd)
    Aeq = sparse.hstack([Ax, Bu])
    leq = np.hstack([-x0, np.zeros(N*nx)])
    ueq = leq
    # - input and state constraints
    Aineq = sparse.eye((N+1)*nx + N*nu)
    lineq = np.hstack([np.kron(np.ones(N+1), xmin), np.kron(np.ones(N), umin)])
    uineq = np.hstack([np.kron(np.ones(N+1), xmax), np.kron(np.ones(N), umax)])
    # - OSQP constraints
    A = sparse.vstack([Aeq, Aineq], format='csc')
    l = np.hstack([leq, lineq])
    u = np.hstack([ueq, uineq])


    # Create an OSQP object
    prob = osqp.OSQP()

    # Setup workspace and change alpha parameter
    prob.setup(P, q, A, l, u, alpha=1.0)

    # Solve problem
    res = prob.solve()
    ctrl = res.x[-N*nu:-(N-1)*nu]

    return ctrl

def publisher():
    rospy.init_node('solver_publisher', anonymous=True)
    pub = rospy.Publisher('solver_results', Float64MultiArray, queue_size=10)
    rate = rospy.Rate(1000) # 1000 Hz

    while not rospy.is_shutdown():
        solution = solve_qp_problem()

        # Preparing the message
        msg = Float64MultiArray()
        msg.data = solution

        # Publishing
        rospy.loginfo(f'Publishing: {msg.data}')
        pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
