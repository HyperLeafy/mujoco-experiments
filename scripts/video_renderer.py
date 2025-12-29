"""
Docstring for scripts.video_renderer
A stand alone script to render video and save to a specifiec loacition
- should be used for simple body with nact<2 i.e a single actuator

"""

import numpy as np
from dm_control import mujoco
# from dm_control.mujoco.wrapper import enums
from dm_control.mujoco.wrapper import mjbindings
import matplotlib.pyplot as plt
from matplotlib import animation


def sine_controller(physics: mujoco.Physics, amplitude: float=1.0, frequency:float =5.0) -> float:
    return amplitude * np.sin(frequency * physics.data.time)

def save_simulation_video(physics: mujoco.Physics, control_fn,duration:float =2.0, framerate: int=30, filename:str = "sample"):
    """Simulates and returns an HTML5 video player.
        - has joint axis visulaizer enabled
        - Change the flag to false to no axis visualaztion
    """

    scene_option = mujoco.wrapper.core.MjvOption()
    scene_option.flags[mjbindings.enums.mjtVisFlag.mjVIS_JOINT] = True
    
    frames = []
    physics.reset()

    while physics.data.time < duration:
        # check if there is manitpulator given for contorl
        if control_fn:
            physics.data.ctrl[0] = control_fn(physics)

        physics.step()

        if len(frames) < physics.data.time * framerate:
            pixels = physics.render(scene_option=scene_option)
            frames.append(pixels)

    # Minimal Animation logic
    fig, ax = plt.subplots(figsize=(frames[0].shape[1]/70, frames[0].shape[0]/70))
    ax.set_axis_off()
    ax.set_position([0, 0, 1, 1])
    im = ax.imshow(frames[0])
    
    def update(frame):
        im.set_data(frame)
        return [im]
    
    anim = animation.FuncAnimation(fig, update, frames=frames, interval=1000/framerate, blit=True)

    # SAVE the video
    anim.save(f'results/{filename}', writer='ffmpeg', fps=framerate)
    print(f"Video saved as {filename}.mp4")
    plt.close() # Prevents static plot from showing
    


# Testing the script 
# Example - A child body with a joint { vertical-output: true }

if __name__=="__main__":
    swinging_body = """
    <mujoco>
    <option gravity="0 0 -9.81"/>
    <visual>
        <global offwidth="640" offheight="480"/>
    </visual>
    <asset>
        <texture name="skybox" type="skybox" builtin="gradient" rgb1=".6 .8 1" rgb2="2 1 1" width="800" height="800"/>
    </asset>
    <worldbody>
        <light directional="true" diffuse=".8 .8 .8" pos="0 0 10" dir="0 1 0"/>
        <body name="base" pos="0 0 0">
        <geom type="box" size="0.1 0.1 0.1" rgba="0.7 0.7 0.7 1"/>
        <body name="swing" pos="0 0 0.3" euler="0 0 90">
            <joint name="hinge" type="hinge" axis="0 1 0" pos="0 0 0"/>
            <geom type="capsule" fromto="0 0 0 0 0 0.3" size="0.05" rgba="0.2 0.5 0.8 1"/>
        </body>
        </body>
    </worldbody>

    <actuator>
        <motor name="swing_motor" joint="hinge" ctrlrange="-1 1" gear="50"/>
    </actuator>
    </mujoco>
    """

    physics = mujoco.Physics.from_xml_string(swinging_body)
# Run with custom parameters
    save_simulation_video(
        physics, 
        lambda p: sine_controller(p, amplitude=1.0, frequency=10.0), 
        filename='fast_swing.mp4',
        duration=4.0
    )