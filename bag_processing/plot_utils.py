import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib import transforms

def plot_map_2d(fig_name, num_spots=19, occupancy=np.ones((2, 19)), spot_width=0.32, spot_length=0.55, lane_width=0.5, v_length=0.5, v_width=0.2):
    """
    plot parking lot map in 2D
    """
    plt.figure(fig_name)
    ax = plt.gca()
    
    # Plot parking lanes
    plt.plot([0, num_spots*spot_width], [-lane_width-spot_length, -
                          lane_width-spot_length], 'k')
    plt.plot([0, num_spots*spot_width], [lane_width+spot_length, lane_width +
                          spot_length], 'k')
    for i in range(num_spots+1):
        plt.plot([i*spot_width, i*spot_width], [lane_width, lane_width+spot_length], 'k')
        plt.plot([i*spot_width, i*spot_width],
                 [-lane_width, -lane_width-spot_length], 'k')
    
    # Plot static vehicles
    for i in range(occupancy.shape[1]):
        if occupancy[0, i]:
            rect = patches.Rectangle(((i+0.5)*spot_width-0.5*v_width, lane_width +
                                      0.5*spot_length-0.5*v_length), v_width, v_length, facecolor="#d5d7db")
            ax.add_patch(rect)
        
        if occupancy[1, i]:
            rect = patches.Rectangle(((i+0.5)*spot_width-0.5*v_width, -lane_width -
                                      0.5*spot_length-0.5*v_length), v_width, v_length, facecolor="#d5d7db")
            ax.add_patch(rect)


def plot_car_frames(ax, pos_list, car_img, plot_shape=[0.265, 0.52]):
    """
    plot car images along a list of positions with discreasing transparency
    """
    img_shape = car_img.shape[:2]
    num_list = len(pos_list)
    for idx, pos in enumerate(pos_list):
        car = car_img.copy()

        # Center the vehicle to (0, 0), scale it, and turn it to heading angle 0
        tf = transforms.Affine2D().translate(
            tx=-img_shape[1]/2, ty=-img_shape[0]/2).scale(sx=plot_shape[0]/img_shape[1], sy=plot_shape[1]/img_shape[0]).rotate_around(x=0, y=0, theta=np.pi/2)

        # # Rotate it with the heading angle
        tf = tf.rotate_around(x=0, y=0, theta=pos[2])

        # # Translate it to the target position
        tf = tf.translate(tx=pos[0], ty=pos[1])

        im = ax.imshow(car, origin='lower', alpha=(idx+1)/num_list)
        im.set_transform(tf + ax.transData)
        

def plot_map_3d(fig_name, height=10, num_spots=19, occupancy=np.ones((2, 19)), spot_width=0.32, spot_length=0.55, lane_width=0.5, v_length=0.5, v_width=0.2):
    """
    plot the parking lot map in 3D
    """
    fig = plt.figure(fig_name)
    ax = fig.gca(projection='3d')

    # Plot static vehicles
    for i in range(occupancy.shape[1]):
        if occupancy[0, i]:
            center = [(i+0.5)*spot_width, lane_width+0.5*spot_length, 0.5*height]
            dimension = [v_width, v_length, height]
            plot_cube(ax, center, dimension)

        if occupancy[1, i]:
            center = [(i+0.5)*spot_width, -lane_width-0.5*spot_length, 0.5*height]
            dimension = [v_width, v_length, height]
            plot_cube(ax, center, dimension)

    ax.axes.set_xlim3d(left=-0.5, right=6.5)
    ax.axes.set_ylim3d(bottom=-1.5, top=1.5)
    ax.axes.set_zlim3d(bottom=0, top=height)
    
def plot_cube(ax, center=[0, 0, 0], dimension=[2, 2, 2], color="#d5d7db", alpha=0.2):
    """
    plot a cube in 3d fig
    """
    # Bottom
    x = [center[0] - 0.5*dimension[0], center[0] + 0.5*dimension[0]]
    y = [center[1] - 0.5*dimension[1], center[1] + 0.5*dimension[1]]
    X, Y = np.meshgrid(x, y)
    Z = (center[2] - 0.5*dimension[2]) * np.ones((2, 2))
    ax.plot_surface(X, Y, Z, color=color, alpha=alpha)

    # Top
    x = [center[0] - 0.5*dimension[0], center[0] + 0.5*dimension[0]]
    y = [center[1] - 0.5*dimension[1], center[1] + 0.5*dimension[1]]
    X, Y = np.meshgrid(x, y)
    Z = (center[2] + 0.5*dimension[2]) * np.ones((2,2))
    ax.plot_surface(X, Y, Z, color=color, alpha=alpha)

    # Front
    x = [center[0] - 0.5*dimension[0], center[0] + 0.5*dimension[0]]
    z = [center[2] - 0.5*dimension[2], center[2] + 0.5*dimension[2]]
    X, Z = np.meshgrid(x, z)
    Y = (center[1] - 0.5*dimension[1]) * np.ones((2,2))
    ax.plot_surface(X, Y, Z, color=color, alpha=alpha)

    # Back
    x = [center[0] - 0.5*dimension[0], center[0] + 0.5*dimension[0]]
    z = [center[2] - 0.5*dimension[2], center[2] + 0.5*dimension[2]]
    X, Z = np.meshgrid(x, z)
    Y = (center[1] + 0.5*dimension[1]) * np.ones((2, 2))
    ax.plot_surface(X, Y, Z, color=color, alpha=alpha)

    # Left
    y = [center[1] - 0.5*dimension[1], center[1] + 0.5*dimension[1]]
    z = [center[2] - 0.5*dimension[2], center[2] + 0.5*dimension[2]]
    Y, Z = np.meshgrid(y, z)
    X = (center[0] - 0.5*dimension[0]) * np.ones((2,2))
    ax.plot_surface(X, Y, Z, color=color, alpha=alpha)

    # Right
    y = [center[1] - 0.5*dimension[1], center[1] + 0.5*dimension[1]]
    z = [center[2] - 0.5*dimension[2], center[2] + 0.5*dimension[2]]
    Y, Z = np.meshgrid(y, z)
    X = (center[0] + 0.5*dimension[0]) * np.ones((2, 2))
    ax.plot_surface(X, Y, Z, color=color, alpha=alpha)
