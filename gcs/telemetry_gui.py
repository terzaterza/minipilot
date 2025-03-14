import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

# Requires tkinter to be installed
# sudo apt install python3-tk
mpl.use("TkAgg")

def create_vector_plot(name, dimensions = 3, buffer_size = 100, traces = 1):
    fig, axs = plt.subplots(dimensions)
    fig.suptitle(name)

    for ax in axs:
        ax.set_ylim([-5, 5])
    
    # For each trace there is a history
    buffers = [np.zeros((buffer_size, dimensions)) for _ in range(traces)]

    bgs = [fig.canvas.copy_from_bbox(ax.bbox) for ax in axs]
    lines = [[axs[d].plot(buffers[t][:, d])[0] for t in range(traces)] for d in range(dimensions)]
    
    def update(data, time):
        # Data is an array of n-dimensional vectors for each trace
        assert len(data) == traces

        for t in range(traces):
            # TODO: Try to do this roll in-place
            buffers[t] = np.roll(buffers[t], -1, axis=0)
            buffers[t][-1, :] = data[t]

        for d in range(dimensions):
            # Restore background
            fig.canvas.restore_region(bgs[d])

            # Update each of the traces
            for t in range(traces):
                lines[d][t].set_ydata(buffers[t][:, d])
                axs[d].draw_artist(lines[d][t])
            
            fig.canvas.blit(axs[d].bbox)
        fig.canvas.flush_events()

    return update


def create_state_plot(
    show_position = False,
    show_velocity = True,
    show_acceleration = True,
    show_rotation_q = False,
    show_angular_velocity = True
):
    update_position = create_vector_plot("Position") if show_position else lambda d, t: None
    update_velocity = create_vector_plot("Velocity") if show_velocity else lambda d, t: None
    update_acceleration = create_vector_plot("Acceleration") if show_acceleration else lambda d, t: None
    update_rotation_q = create_vector_plot("Rotation Q", 4) if show_rotation_q else lambda d, t: None
    update_angular_velocity = create_vector_plot("Angular Velocity") if show_angular_velocity else lambda d, t: None

    def update_state(pb_state, time_current):
        pb_vec3_to_np = lambda v: np.array([v.x, v.y, v.z])
        pb_vec4_to_np = lambda v: np.array([v.w, v.x, v.y, v.z])
        update_position([pb_vec3_to_np(pb_state.position)], time_current)
        update_velocity([pb_vec3_to_np(pb_state.velocity)], time_current)
        update_acceleration([pb_vec3_to_np(pb_state.acceleration)], time_current)
        update_rotation_q([pb_vec4_to_np(pb_state.rotation)], time_current)
        update_angular_velocity([pb_vec3_to_np(pb_state.angular_velocity)], time_current)

    return update_state
