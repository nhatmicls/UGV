import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from random import seed
from random import randint

# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
ax.axes.get_xaxis().set_visible(False)
ax.axes.get_yaxis().set_visible(False)
# ax.get_xaxis().tick_bottom()
# ax.set_xticks([])
xs = []
ys = []

# This function is called periodically from FuncAnimation
seed(1)


def animate(i, xs, ys):

    # Read temperature (Celsius) from TMP102
    temp_c = randint(50, 100)

    # Add x and y to lists
    xs.append(dt.datetime.now().strftime('%S.%f'))
    ys.append(temp_c)

    # Limit x and y lists to 20 items
    xs = xs[-20:]
    ys = ys[-20:]

    # Draw x and y lists
    ax.clear()
    ax.plot(xs, ys)

    # Format plot
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('TMP102 Temperature over Time')
    plt.ylabel('Temperature (deg C)')


# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=500)
plt.show()
