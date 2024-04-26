import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def read_data(file_path):
    t = []
    x = []
    y = []
    z = []
    vx = []
    vy = []
    vz = []
    mu = []
    l = []
    h = []
    with open(file_path, "r") as file:
        next(file)
        for line in file:
            data = line.strip().split()
            t.append(float(data[0]))
            vx.append(float(data[1]))
            vy.append(float(data[2]))
            vz.append(float(data[3]))
            mu.append(float(data[4])*180.0/3.14159265358979323846)
            l.append(float(data[5])*180.0/3.14159265358979323846)
            h.append(float(data[6]))
            x.append(float(data[7]))
            y.append(float(data[8]))
            z.append(float(data[9]))

    return t, vx, vy, vz, x, y, z, mu, l, h

file_path = "build/src/simulation_results.txt"
t, vx, vy, vz, x, y, z, mu, l, h = read_data(file_path)


fig, axs = plt.subplots(3, 1)

axs[0].plot(t, vx, label="Vx")
axs[0].plot(t, vy, label="Vy")
axs[0].plot(t, vz, label="Vz")
axs[0].set_title("Velocities")
axs[0].grid(True)
axs[0].legend(loc="best")

axs[1].plot(x, z, label="z")
axs[1].set_title("altitude")
axs[1].legend(loc="best")
axs[1].grid(True)

axs[2].plot(x, y, label="position")
axs[2].set_title("position")
axs[2].legend(loc="best")
axs[2].grid(True)


plt.tight_layout()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot trajectory
ax.plot(x, y, z, label='Trajectory')

# Plot starting point
ax.scatter(x[0], y[0], z[0], color='red', label='Starting Point')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Trajectory')
ax.legend()
x_range = max(x) - min(x)
y_range = max(y) - min(y)
z_range = max(z) - min(z)
max_range = max(x_range, y_range, z_range)

# Set aspect ratio based on the maximum range
ax.set_box_aspect([x_range/max_range, y_range/max_range, z_range/max_range])

plt.show()

