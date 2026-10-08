#!/usr/bin/env python
"""
| File: 4_python_single_vehicle.py
| Author: Marcelo Jacinto and Joao Pinto (marcelo.jacinto@tecnico.ulisboa.pt, joao.s.pinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: This files serves as an example on how to use the control backends API to create a custom controller 
for the vehicle from scratch and use it to perform a simulation, without using PX4 nor ROS.
"""

# Imports to start Isaac Sim from this script
import carb
from isaacsim import SimulationApp

# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import, otherwise the simulator will crash
# as this is the object that will load all the extensions and load the actual simulator.
simulation_app = SimulationApp({"headless": False})

# -----------------------------------
# The actual script should start here
# -----------------------------------
import omni.timeline
from omni.isaac.core.world import World

# Import the Pegasus API for simulating drones
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.thrusters import QuadraticThrustCurve

CUSTOM_USD = (
    "/home/pranathikaruturi/Downloads/GitHub/"
    "Aero_Manipulator/scenes/f550_pegasus_axis_test.usd"
)

# Import the custom python control backend
import sys, os
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)) + '/utils')
from nonlinear_controller import NonlinearController

# Auxiliary scipy and numpy modules
from scipy.spatial.transform import Rotation

# Use pathlib for parsing the desired trajectory from a CSV file
from pathlib import Path


class PegasusApp:
    """
    A Template class that serves as an example on how to build a simple Isaac Sim standalone App.
    """

    def __init__(self):
        """
        Method that initializes the PegasusApp and is used to setup the simulation environment.
        """

        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        # Start the Pegasus Interface
        self.pg = PegasusInterface()

        # Acquire the World, .i.e, the singleton that controls that is a one stop shop for setting up physics, 
        # spawning asset primitives, etc.
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Launch one of the worlds provided by NVIDIA
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])

        # Get the current directory used to read trajectories and save results
        self.curr_dir = str(Path(os.path.dirname(os.path.realpath(__file__))).resolve())

        # Create the vehicle 1
        # Try to spawn the selected robot in the world to the specified namespace
        config_multirotor1 = MultirotorConfig()

        config_multirotor1.thrust_curve = QuadraticThrustCurve({
            "num_rotors": 6,

            # Temporary starting values
            "rotor_constant": [
                8.54858e-6,
                8.54858e-6,
                8.54858e-6,
                8.54858e-6,
                8.54858e-6,
                8.54858e-6,
            ],

            "rolling_moment_coefficient": [
                1e-6,
                1e-6,
                1e-6,
                1e-6,
                1e-6,
                1e-6,
            ],

            # Temporary alternating rotation directions
            "rot_dir": [-1, 1, -1, 1, -1, 1],

            "min_rotor_velocity": [0, 0, 0, 0, 0, 0],

            "max_rotor_velocity": [
                1100,
                1100,
                1100,
                1100,
                1100,
                1100,
            ],
        })

        config_multirotor1.backends = [
            NonlinearController(
                trajectory_file=None,
                results_file=None,
                num_rotors=6,
            )
        ]

        Multirotor(
            "/World/aeromanipulator1",
            CUSTOM_USD,
            0,
            [2.3, -1.5, 3.0],
            Rotation.from_euler(
                "XYZ",
                [0.0, 0.0, 0.0],
                degrees=True,
            ).as_quat(),
            config=config_multirotor1,
        )

        # Reset the simulation environment so that all articulations (aka robots) are initialized
        self.world.reset()

    def run(self):
        """
        Method that implements the application main loop, where the physics steps are executed.
        """

        # Start the simulation
        self.timeline.play()

        # The "infinite" loop
        while simulation_app.is_running():

            # Update the UI of the app and perform the physics step
            self.world.step(render=True)
        
        # Cleanup and stop
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()

def main():

    # Instantiate the template app
    pg_app = PegasusApp()

    # Run the application loop
    pg_app.run()

if __name__ == "__main__":
    main()
