import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from matplotlib.widgets import Button, Slider

#Using Earth's X and Y, we can build matrices to get X and Y for lagrange points.
def get_lagrange_locations(earth_pos):
    L1_mat = [[0.99005,0],[0,0.99005]]
    L2_mat = [[1.01004,0],[0,1.01004]]
    L3_mat = [[-1.00018,0],[0,-1.00018]]
    L4_mat = [[np.cos(np.pi/3),-np.sin(np.pi/3)],[np.sin(np.pi/3),np.cos(np.pi/3)]]
    L5_mat = [[np.cos(-np.pi/3),-np.sin(-np.pi/3)],[np.sin(-np.pi/3),np.cos(-np.pi/3)]]
    return np.array([np.dot(mat,earth_pos) for mat in [L1_mat,L2_mat,L3_mat,L4_mat,L5_mat]])

SUN_MU = 39.423
EARTH_MU = 1.184e-4
#Precision of physics simulation
timestep = 0.00001
#Number of physics steps per rendered step. Speed that you see is roughly timestep * interp.
interp_factor = 500


fig, axes = plt.subplot_mosaic("AAB;AAC")
plt.subplots_adjust(bottom=0.15)
ax1 = axes["A"]
ax2 = axes["B"]
ax3 = axes["C"]

sun = plt.Circle((0, 0), 0.2, color='y')
earth_1 = plt.Circle((1,0), 0.01, color='b')
earth_2 = plt.Circle((0,0), 0.001, color='b')
#put satellite at 10,10 until we place it
sat_patch = plt.Circle((10,10), 0.01, color='r')
sat_patch_2 = plt.Circle((10,10), 0.001, color='r')
sat_patch_L5 = plt.Circle((10,10), 0.003, color='r')

earth_pos = np.array([1,0])
earth_vel = np.array([0,2*np.pi])

sat_pos = np.array([10,10])
sat_vel = np.array([0,0])

lagrange_coords = get_lagrange_locations(earth_pos)
lagrange_points = ax1.scatter(*zip(*lagrange_coords))
lagrange_shifted = [x - earth_pos for x in lagrange_coords]
lagrange_points_zoom = ax2.scatter(*zip(*lagrange_shifted))

ax1.set_xlim(-2,2)
ax1.set_ylim(-2,2)
ax1.set_aspect('equal')
ax1.set_title('Overall view')

ax2.set_xlim(-0.02,0.02)
ax2.set_ylim(-0.02,0.02)
ax2.set_aspect('equal')
ax2.set_title('L1/L2 view')
ax2.annotate("L1", (-0.01,0))
ax2.annotate("L2", (0.01,0))

ax3.set_xlim(-0.06,0.06)
ax3.set_ylim(-0.06,0.06)
ax3.set_aspect('equal')
ax3.set_title('L5 view')
ax3.scatter(0,0)
ax3.annotate("L5", (0,0))


ax1.add_patch(sun)
ax1.add_patch(earth_1)
ax2.add_patch(earth_2)
ax1.add_patch(sat_patch)
ax2.add_patch(sat_patch_2)
ax3.add_patch(sat_patch_L5)

def animate(frame):
    global earth_pos,earth_vel, sat_pos, sat_vel,anim
    #Find position of earth, and thus lagrange points

    for _ in range(interp_factor):
        earth_accel_vec = (-SUN_MU / np.linalg.norm(earth_pos)**3) * earth_pos
        earth_vel = earth_vel + (earth_accel_vec * timestep)
        earth_pos = earth_pos + (earth_vel * timestep)
        

        #Calculate overall force vector from earth and sun
        sun_sat_accel = (-SUN_MU / np.linalg.norm(sat_pos)**3) * sat_pos
        sat_rel_pos = sat_pos - earth_pos
        if np.linalg.norm(sat_rel_pos) < 4e-5:
            print("Crash")
            sat_pos = earth_pos
            anim.pause()
        else:
            earth_sat_accel = (-EARTH_MU / np.linalg.norm(sat_rel_pos)**3) * sat_rel_pos
            sat_accel_vec = sun_sat_accel + earth_sat_accel
            sat_vel = sat_vel + (sat_accel_vec * timestep)
            sat_pos = sat_pos + (sat_vel * timestep)

    #Update big plot
    earth_1.set_center(earth_pos)
    lagrange_points.set_offsets(get_lagrange_locations(earth_pos))
    sat_patch.set_center(sat_pos)

    #Update zoomed plot
    #Get position of sat relative to earth
    rel_pos = sat_pos - earth_pos
    #Convert with rotation matrix to earth coordinates
    earth_angle = -np.arctan2(earth_pos[1],earth_pos[0]) #earth angle, earth angle, will you be mine
    rotation_matrix = [[np.cos(earth_angle),-np.sin(earth_angle)],[np.sin(earth_angle),np.cos(earth_angle)]]
    rotated_relpos = np.dot(rotation_matrix,rel_pos)
    sat_patch_2.set_center(rotated_relpos)

    #Do same thing for L5
    L5_mat = [[np.cos(-np.pi/3),-np.sin(-np.pi/3)],[np.sin(-np.pi/3),np.cos(-np.pi/3)]]
    L5_pos = np.dot(L5_mat,earth_pos)
    rel_pos = sat_pos - L5_pos
    L5_angle = -np.arctan2(L5_pos[1],L5_pos[0])
    rotation_matrix = [[np.cos(L5_angle),-np.sin(L5_angle)],[np.sin(L5_angle),np.cos(L5_angle)]]
    rotated_relpos = np.dot(rotation_matrix,rel_pos)
    sat_patch_L5.set_center(rotated_relpos)
    
    return [earth_1, sat_patch,lagrange_points, sat_patch_2, sat_patch_L5]

def on_click(event):
    global sat_pos, sat_vel, sat_patch, earth_pos
    print(f'{event.xdata}, {event.ydata}')
    if event.inaxes == ax1:
        x = event.xdata
        y = event.ydata
    elif event.inaxes == ax2:
        earth_angle = np.arctan2(earth_pos[1],earth_pos[0])
        rotation_matrix = [[np.cos(earth_angle),-np.sin(earth_angle)],[np.sin(earth_angle),np.cos(earth_angle)]]
        rotated_relpos = np.dot(rotation_matrix,[event.xdata,event.ydata])
        x = earth_pos[0] + rotated_relpos[0]
        y = earth_pos[1] + rotated_relpos[1]
        sat_patch_2.set_center((x,y))
    elif event.inaxes == ax3:
        L5_mat = [[np.cos(-np.pi/3),-np.sin(-np.pi/3)],[np.sin(-np.pi/3),np.cos(-np.pi/3)]]
        L5_pos = np.dot(L5_mat,earth_pos)
        L5_angle = np.arctan2(L5_pos[1],L5_pos[0])
        rotation_matrix = [[np.cos(L5_angle),-np.sin(L5_angle)],[np.sin(L5_angle),np.cos(L5_angle)]]
        rotated_relpos = np.dot(rotation_matrix,[event.xdata,event.ydata])
        x = L5_pos[0] + rotated_relpos[0]
        y = L5_pos[1] + rotated_relpos[1]
        sat_patch_L5.set_center((x,y))
    else:
        print("Invalid click")
        return
    sat_pos = np.array([x,y])
    #Give the satellite the same angular velocity as the earth
    #first get the satellite angle
    distance_to_sat = np.linalg.norm(sat_pos)
    #Earth's angular velocity is 2pi radians per year.
    #Angular velocity = linear velocity / r
    sat_vel_mag = 2*np.pi * distance_to_sat
    #Need to look into this further but for some reason L5 is much more stable like this
    if event.inaxes == ax3:
        sat_vel_mag = 2*np.pi / distance_to_sat
    print(sat_vel_mag)
    #Satellite velocity needs to be 90 degrees ahead of the angle to the sat
    angle_to_sat = np.arctan2(y,x)
    sat_vel_angle = angle_to_sat + np.pi/2
    sat_vel_x = sat_vel_mag * np.cos(sat_vel_angle)
    sat_vel_y = sat_vel_mag * np.sin(sat_vel_angle)
    sat_vel = (sat_vel_x, sat_vel_y)
    sat_patch.set_center(sat_pos)
    global fig
    fig.canvas.draw()
     
def start_animation(event):
    global anim
    anim = animation.FuncAnimation(fig=fig, func=animate, interval=1,blit=True)
    plt.show()

def reset_positions(event):
    global earth_pos,earth_vel, sat_pos, sat_vel

    earth_pos = np.array([1,0])
    earth_vel = np.array([0,2*np.pi])

    sat_pos = np.array([0.99,0])
    sat_vel = np.array([0,2*np.pi])
def set_interp(slider_val):
    global interp_factor
    interp_factor = slider_val

axstart = fig.add_axes([0.05, 0.9, 0.1, 0.075])
axreset = fig.add_axes([0.2, 0.9, 0.1, 0.075])
axslider = fig.add_axes([0.05, 0.05, 0.9, 0.075])


bstart = Button(axstart, 'Start Animation')
breset = Button(axreset, 'Reset Animation')

speed_slider = Slider(axslider, 'Speed', 10,1000,500,valstep=1)
speed_slider.on_changed(set_interp)
bstart.on_clicked(start_animation)
breset.on_clicked(reset_positions)

plt.connect('button_press_event', on_click)
plt.show()

