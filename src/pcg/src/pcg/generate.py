from pcg_gazebo.generators import WorldGenerator
from pcg_gazebo.generators.creators import extrude, box
from pcg_gazebo.generators.shapes import random_rectangles
from pcg_gazebo.simulation import SimulationModel
from pcg_gazebo.utils import generate_random_string

import random
import rospy
import os


rospy.init_node("generate")
N_ROOMS = 10


def generate_room(n_rectangles: float, filename: str) -> None:
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
    print(world_gen.constraints.get("room_workspace").get_random_position())
    print(
        world_gen.world.get_random_free_spots(model=clover_sized_box_model, n_spots=1)[
            0
        ]
    )

    # save world
    # default location is
    #   ~/.gazebo/worlds/ for worlds,
    #   ~/.gazebo/models/ for models
    world_gen.export_world(
        filename=filename,
        output_dir=os.path.join(os.environ["HOME"], ".gazebo", "worlds"),
        with_default_ground_plane=False,
    )


if __name__ == "__main__":
    # using +2 because minimum number of rectangles is 2
    # when using plural method
    for i in [number + 2 for number in range(N_ROOMS)]:
        generate_room(n_rectangles=i, filename=f"{i}-rectangles-walls")
