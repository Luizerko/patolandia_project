import sys
import pyglet
from pyglet.window import key
from duckievillage import create_env, FRONT_VIEW_MODE, TOP_DOWN_VIEW_MODE, FULL_VIEW_MODE
import numpy as np
import os

from agent import Agent

def main():
    print("MAC0318 - Final Project")

    map_name = None
    with open(sys.argv[1]) as file:
        map_name = file.readline().strip()

    env = create_env(
        raw_motor_input = True,
        noisy = True,
        mu_l = 0.007123895,
        mu_r = -0.000523123,
        std_l = 1e-7,
        std_r = 1e-7,
        seed = 101,
        map_name = map_name,
        mailbox_file = sys.argv[1],
        enable_mailbox = True,
        enable_eval = True,
        enable_topomap = True,
        enable_mileage = True,
        draw_curve = False,
        draw_bbox = False,
        domain_rand = False,
        distortion = False,
        top_down = False,
        cam_height = 10,
        is_external_map = True,
        randomize_maps_on_reset = False,
        # To record your agent, set the variable below to some file path. It must end in ".mp4".
        video_path = None,
    )

    # For a top-down view of the map, change this to TOP_DOWN_VIEW_MODE. For both, FULL_VIEW_MODE.
    env.set_view(FRONT_VIEW_MODE)
    env.render()

    @env.unwrapped.window.event
    def on_key_press(symbol, modifiers):
        if symbol == key.ESCAPE: # exit simulation
            print("Energy usage:", env.mileage.mileage())
            # Dump log to file
            with open("/tmp/output.txt", 'w') as file:
                for k in env.eval._log:
                    for l in env.eval._log[k]: file.write(str(l[0]) + " " + str(l[1]) + '\n')
                file.write(str(env.mileage.mileage()) + " = energy usage")

            np.save(os.environ['PWD']+"/assignments/mailduck/data/images1.npy", agent.images, allow_pickle = True)
            np.save(os.environ['PWD']+"/assignments/mailduck/data/vira1.npy", agent.vira, allow_pickle = True)
            np.save(os.environ['PWD']+"/assignments/mailduck/data/labels1.npy", agent.labels, allow_pickle = True)

            env.close()
            sys.exit(0)
            
        elif symbol == key.DELETE: # exit simulation
            print("Energy usage:", env.mileage.mileage())
            # Dump log to file
            with open("/tmp/output.txt", 'w') as file:
                for k in env.eval._log:
                    for l in env.eval._log[k]: file.write(str(l[0]) + " " + str(l[1]) + '\n')
                file.write(str(env.mileage.mileage()) + " = energy usage")
            env.close()
            sys.exit(0)

    agent = Agent(env)

    track_dt = 0

    def loop(dt: float):
        agent.send_commands(dt)
        nonlocal track_dt
        track_dt += dt
        if track_dt > 0.5:
            env.eval.track()
            track_dt = 0
        env.render()

    pyglet.clock.schedule_interval(loop, 1.0 / env.unwrapped.frame_rate)
    pyglet.app.run()
    env.close()

if __name__ == '__main__':
    main()
