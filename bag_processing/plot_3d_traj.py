import numpy as np
import matplotlib.pyplot as plt

from plot_utils import plot_map_3d

np.random.seed(0)

if __name__ == "__main__":
    fig_name = "ParkingLot3D"

    # Generate occupancy randomly
    occupancy = np.random.rand(2, 19)
    occupancy[occupancy <= 0.5] = 0
    occupancy[occupancy > 0.5] = 1
    # Set the destination of target vehicle to be empty
    occupancy[0, 10] = 0

    fig = plt.figure(fig_name, figsize=(6, 6))

    # plot_map_3d(fig_name, occupancy=occupancy)
    plot_map_3d(fig_name, height=5000, occupancy=occupancy)

    ax = plt.gca()

    # ax.plot3D(EV.x, EV.y, np.arange(len(EV.x)), linewidth=4)
    # ax.plot3D(TV.x, TV.y, np.arange(len(TV.x)), linewidth=4)
    plt.show()
