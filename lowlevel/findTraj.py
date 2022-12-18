import numpy as np

def calc_control_command(x_diff, y_diff, theta, theta_goal):
    """
    Returns the control command for the linear and angular velocities as
    well as the distance to goal
    Parameters
    ----------
    x_diff : The position of target with respect to current robot position
    in x direction
    y_diff : The position of target with respect to current robot position
    in y direction
    theta : The current heading angle of robot with respect to x axis
    theta_goal: The target angle of robot with respect to x axis
    Returns
    -------
    rho : The distance between the robot and the goal position
    v : Command linear velocity
    w : Command angular velocity
    """

    # Description of local variables:
    # - alpha is the angle to the goal relative to the heading of the robot
    # - beta is the angle between the robot's position and the goal
    #   position plus the goal angle
    # - Kp_rho*rho and Kp_alpha*alpha drive the robot along a line towards
    #   the goal
    # - Kp_beta*beta rotates the line so that it is parallel to the goal
    #   angle
    #
    # Note:
    # we restrict alpha and beta (angle differences) to the range
    # [-pi, pi] to prevent unstable behavior e.g. difference going
    # from 0 rad to 2*pi rad with slight turn

    Kp_rho, Kp_alpha, Kp_beta = 5.0, 16.0, 4.0

    rho = np.hypot(x_diff, y_diff)
    alpha = (np.arctan2(y_diff, x_diff) - theta + np.pi) % (2 * np.pi) - np.pi
    beta = (theta_goal - theta - alpha + np.pi) % (2 * np.pi) - np.pi
    v = Kp_rho * rho
    w = Kp_alpha * alpha - Kp_beta * beta

    if alpha > np.pi / 2 or alpha < -np.pi / 2:
        v = -v

    if rho < .05:
        v = 0
        w = 0

    if np.abs(v) > 0.1:
        v = np.sign(v)*.1
    if np.abs(w) > 0.1:
        w = np.sign(w) * .1

    return rho, v, w
