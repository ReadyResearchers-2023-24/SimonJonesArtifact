#!/usr/bin/env python

from pcg_gazebo.generators import WorldGenerator
from pcg_gazebo.generators.creators import extrude
from pcg_gazebo.generators.shapes import random_rectangles
from pcg_gazebo.simulation import SimulationModel
from pcg_gazebo.visualization import plot_workspace, plot_occupancy_grid
from pathlib import Path

import random
import rospy
import os


rospy.init_node("pcg")
OUTPUT_DIRECTORY = str(Path(os.environ["HOME"]) / Path(".gazebo") / Path(""))
N_ROOMS=10


def generate_room(n_rectangles: float, filename: str) -> None:
    """Randomly generate a room and save it to a file."""
    # credit: https://github.com/boschresearch/pcg_gazebo/blob/master/examples/gen_grid_map.ipynb
    world_gen = WorldGenerator()
    wall_thickness = 0.15 # m
    wall_height = random.randint(3, 6) # m
    wall_polygon = random_rectangles(
        n_rect=n_rectangles,
        delta_x_min=6,
        delta_x_max=10,
        delta_y_min=6,
        delta_y_max=10,
    )
    walls_model = extrude(
        polygon=wall_polygon,
        thickness=wall_thickness,
        height=wall_height,
        pose=[0, 0, wall_height / 2., 0, 0, 0],
        extrude_boundaries=True,
        color="xkcd",
    )
    walls_model.name = "walls"
    ceiling_model = extrude(
        polygon=wall_polygon,
        thickness=10,
        height=wall_thickness,
        pose=[0, 0, wall_height, 0, 0, 0],
        extrude_boundaries=True,
        color="xkcd",
    )
    ceiling_model.name = "ceiling"

    # reset world generator
    world_gen.init()
    world_gen.engines.reset()

    # add walls to the world
    world_gen.world.add_model(
        tag=walls_model.name,
        model=walls_model,
    )
    # add ceiling to the world
    world_gen.world.add_model(
        tag=ceiling_model.name,
        model=ceiling_model,
    )
    # add ground plane to the world
    world_gen.world.add_model(
        tag="ground_plane",
        model=SimulationModel.from_gazebo_model("ground_plane")
    )

    free_space_polygon = world_gen.world.get_free_space_polygon(
        ground_plane_models=[walls_model.name],
        ignore_models=["ground_plane", ceiling_model.name],
    )

    # Add the workspace constraint to the generator
    world_gen.add_constraint(
        name='room_workspace',
        type='workspace',
        frame='world',
        geometry_type='polygon',
        polygon=free_space_polygon,
    )

    NUM_BOXES = 4
    NUM_CYLINDER = 4

    placement_policy = dict(
        models=['dyn_box', 'static_cylinder'],
        config=[
            dict(
                dofs=['x', 'y'],
                tag='workspace',
                workspace='room_workspace'
            ),
            dict(
                dofs=['yaw'],
                tag='uniform',
                min=-3.141592653589793,
                max=3.141592653589793
            )
        ]
    )

    a_ascii = 97
    z_ascii = 122
    world_gen.add_engine(
        tag="".join([chr(random.randint(a_ascii, a_ascii)) for i in range(5)]),
        engine_name='random_pose',
        models=['dyn_box', 'static_cylinder'],
        max_num=dict(
            dyn_box=NUM_BOXES,
            static_cylinder=NUM_CYLINDER),
        model_picker='random',
        no_collision=True,
        policies=[placement_policy],
        constraints=[
            dict(
                model='dyn_box',
                constraint='tangent_to_ground_plane'),
            dict(
                model='static_cylinder',
                constraint='tangent_to_ground_plane')
        ]
    )

    world_gen.run_engines(attach_models=True)

    # save world
    world_gen.export_world(
        output_dir=OUTPUT_DIRECTORY,
        filename=filename,
        with_default_ground_plane=False
    )


if __name__=="__main__":
    for i in [number+2 for number in range(N_ROOMS)]:
        generate_room(n_rectangles=i, filename=f"{i}-rectangles-walls")
