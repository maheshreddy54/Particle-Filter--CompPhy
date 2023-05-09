
import numpy as np
import matplotlib.pyplot as plt
import ipywidgets as widgets
from IPython.display import display
from matplotlib.lines import Line2D

### Setup parameters for Particle Filter simulations

def linear_move_robot(true_pos: np.ndarray, u: np.ndarray, noise_std: float, world_size: np.ndarray) -> np.ndarray:
    '''
    Move the robot with action u from position true_pos subject to Gaussian noise with stdev=noise_std
    while staying within the world_size bounds
    '''

    new_pos = np.clip(true_pos + u + np.random.normal(0, noise_std), world_size[0], world_size[1])
    return new_pos    

def linear_sense_distance(true_pos: np.ndarray, landmarks: np.ndarray, noise_std: float) -> float:
    '''
    Find the distance between the robot and the nearest landmark
    '''
    min_distance = np.min(np.abs(true_pos - landmarks))
    return min_distance + np.random.normal(0, noise_std)

def visualize_linear_world(robot_position: np.ndarray, estimates: np.ndarray, landmarks: np.ndarray, world_size: np.ndarray):
    '''
    robot_position: single value [x]
    landmarks: vector of landmark positions [l1, l2, l3, ...]
    world_size: [min, max]
    '''

    robot_position = np.clip(robot_position, world_size[0], world_size[1])


    fig, ax = plt.subplots()
    ax.hlines(0, world_size[0], world_size[1], color='black', linewidth=3)

    # Plot robot position
    ax.plot([robot_position, robot_position], [-0.1, 0], color='green', linewidth=3)

    # Plot estimates as bar chart
    if (estimates is not None and estimates.shape[0] > 0):
        ax.plot([estimates[:,0], estimates[:,0]], [np.zeros_like(estimates[:,1]), estimates[:,1]/np.max(estimates[:,1])], color='red', linewidth=1)
    est_line = Line2D([], [], color='red', linewidth=1, label="Estimated Position")
    pos_line = Line2D([], [], color='green', linewidth=1, label="Actual Position")
    landmark_line = Line2D([], [], color='blue', linewidth=1, label="Landmarks")
    
    # Plot landmarks
    ax.scatter(landmarks, -0.1 * np.ones_like(landmarks), c='blue', marker='s')


    ax.set_ylim(-0.15, 1.0)
    ax.set_xlim(world_size[0], world_size[1])
    ax.set_xlabel('Horizontal Position')
    ax.set_ylabel('Confidence')
    ax.set_title('Robot Position and Estimates')
    ax.legend(handles=[est_line, pos_line, landmark_line], loc='best')
      

    plt.show()

    # return fig, ax

def run_linear_particle_filter_simulation(pf, world_size: np.ndarray, landmarks: np.ndarray, motion_noise_std: float, sensor_noise_std: float, true_initial_pos: np.ndarray, control_sequence: np.ndarray):
    '''
    Execute the particle filter for a given start state and sequence of controls.
    '''
    true_pos = true_initial_pos
    estimated_positions = []
    true_positions = []

    for u in control_sequence:
        true_pos = linear_move_robot(true_pos, u, motion_noise_std, world_size)
        z = linear_sense_distance(true_pos, landmarks, sensor_noise_std)
        pf.resample()
        pf.predict(u)
        pf.update(z)
        estimated_positions.append(np.vstack([pf.particles, pf.weights]).T)
        true_positions.append(true_pos)
    return estimated_positions, true_positions