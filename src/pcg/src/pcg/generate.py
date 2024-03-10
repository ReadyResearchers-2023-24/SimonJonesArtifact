from pcg_gazebo.generators import WorldGenerator
from pcg_gazebo.generators.creators import extrude, box
from pcg_gazebo.generators.shapes import random_rectangles
from pcg_gazebo.simulation import SimulationModel
from pcg_gazebo.utils import generate_random_string
from geometry_msgs.msg import Pose
from typing import List

import random
import rospy
import os
import rospkg


rospy.init_node("generate")

# FIXME: add parquet_plane model. currently,
# the only way to add it is manually

def generate_room(
    n_rectangles: float,
    filename: str,
    worlds_dir_path: str = None,
    models_dir_path: str = None,
) -> None:
    """Randomly generate a room and save it to a file."""
    # credit: https://github.com/boschresearch/pcg_gazebo/blob/master/examples/gen_grid_map.ipynb
    world_gen = WorldGenerator()
    world_gen.add_asset(
        tag="dyn_box",
        description=dict(
            type="box",
            args=dict(
                size="5 * __import__('pcg_gazebo').random.rand(3)",
                name="cuboid",
                mass="max(0.1, __import__('pcg_gazebo').random.rand())",
                color="xkcd",
            ),
        ),
    )
    # assert if dyn_box was added correctly
    assert "dyn_box" in world_gen.assets.tags

    world_gen.add_asset(
        tag="static_cylinder",
        description=dict(
            type="cylinder",
            args=dict(
                length="2 * __import__('pcg_gazebo').random.rand()",
                radius="2 * __import__('pcg_gazebo').random.rand()",
                name="cylinder",
                color="xkcd",
            ),
        ),
    )
    # assert if static_cylinder was added correctly
    assert "static_cylinder" in world_gen.assets.tags

    world_gen.add_constraint(
        name="tangent_to_ground_plane",
        type="tangent",
        frame="world",
        reference=dict(type="plane", args=dict(origin=[0, 0, 0], normal=[0, 0, 1])),
    )
    # assert that tangent_to_ground_plane constraint was added
    assert "tangent_to_ground_plane" in list(world_gen.constraints.tags)

    wall_thickness = 0.15  # m
    wall_height = random.randint(3, 6)  # m
    wall_polygon = random_rectangles(
        n_rect=n_rectangles,
        delta_x_min=6,
        delta_x_max=10,
        delta_y_min=6,
        delta_y_max=10,
    )

    # create model the size of the clover's collision box to later
    # use for deciding an open space to place the clover model
    clover_sized_box_model = box(
        size=[0.35, 0.35, 0.124],
        mass=1,
        name="clover_sized_box",
    )
    walls_model = extrude(
        polygon=wall_polygon,
        thickness=wall_thickness,
        height=wall_height,
        pose=[0, 0, wall_height / 2.0, 0, 0, 0],
        extrude_boundaries=True,
        color="xkcd",
    )
    walls_model.name = f"walls_{n_rectangles}"
    ceiling_model = extrude(
        polygon=wall_polygon,
        thickness=10,
        height=wall_thickness,
        pose=[0, 0, wall_height, 0, 0, 0],
        extrude_boundaries=True,
        color="xkcd",
    )
    ceiling_model.name = f"ceiling_{n_rectangles}"

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
        tag="ground_plane", model=SimulationModel.from_gazebo_model("ground_plane")
    )

    free_space_polygon = world_gen.world.get_free_space_polygon(
        ground_plane_models=[walls_model.name],
        ignore_models=["ground_plane", ceiling_model.name],
    )

    # Add the workspace constraint to the generator
    world_gen.add_constraint(
        name="room_workspace",
        type="workspace",
        frame="world",
        geometry_type="polygon",
        polygon=free_space_polygon,
    )

    NUM_BOXES = 2
    NUM_CYLINDER = 2

    placement_policy = dict(
        models=["dyn_box", "static_cylinder"],
        config=[
            dict(dofs=["x", "y"], tag="workspace", workspace="room_workspace"),
            dict(
                dofs=["yaw"],
                tag="uniform",
                min=-3.141592653589793,
                max=3.141592653589793,
            ),
        ],
    )

    world_gen.add_engine(
        tag=generate_random_string(5),
        engine_name="random_pose",
        models=["dyn_box", "static_cylinder"],
        max_num=dict(dyn_box=NUM_BOXES, static_cylinder=NUM_CYLINDER),
        model_picker="random",
        no_collision=True,
        policies=[placement_policy],
        constraints=[
            dict(model="dyn_box", constraint="tangent_to_ground_plane"),
            dict(model="static_cylinder", constraint="tangent_to_ground_plane"),
        ],
    )

    world_gen.run_engines(attach_models=True)

    # FIXME: publish this in the form of a service or topic
    # FIXME: publish file namees of maps in the form of a service or topic
    # FIXME: test if this takes too long

    free_poses_to_broadcast: List[Pose] = []
    while len(free_poses_to_broadcast) < 2:
        # take 100 random free spots to determine free spots in the map

        random_free_poses = world_gen.world.get_random_free_spots(
            model=clover_sized_box_model,
            n_spots=100,
        )[0]

        for random_free_pose in random_free_poses:
            # determine if free space is within the room workspace (inside the room)
            if world_gen.constraints.get("room_workspace").contains_point(
                    [random_free_pose.x, random_free_pose.y]
                ):
                # broadcast the valid pose within the room
                pose_to_broadcast = Pose()
                pose_to_broadcast.position.x = random_free_pose.x
                pose_to_broadcast.position.y = random_free_pose.y
                free_poses_to_broadcast.append(pose_to_broadcast)


    rospack = rospkg.RosPack()
    clover_simulation_path = rospack.get_path("clover_simulation")

    if worlds_dir_path is None:
        worlds_dir_path = os.path.join(
            clover_simulation_path, "resources", "worlds"
        )
    if models_dir_path is None:
        models_dir_path = os.path.join(
            clover_simulation_path, "models"
        )

    # save world
    world_gen.export_world(
        filename=filename,
        output_dir=worlds_dir_path,
        models_output_dir=models_dir_path,
        with_default_ground_plane=False,
    )
