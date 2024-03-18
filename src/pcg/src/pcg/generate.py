from pcg_gazebo.generators import WorldGenerator
from pcg_gazebo.generators.creators import extrude, box
from pcg_gazebo.generators.shapes import random_rectangles
from pcg_gazebo.simulation import SimulationModel
from pcg_gazebo.utils import generate_random_string
from geometry_msgs.msg import Pose
from typing import List

import random
import rospy
import shutil
import os
import rospkg
import xml.etree.ElementTree as ET


rospy.init_node("generate")

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
        pose=[0, 0, wall_height / 2, 0, 0, 0],
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

    # find random points within the workspace where the clover model could be
    # spawned.
    free_poses_to_persist: List[Pose] = []
    while len(free_poses_to_persist) < 2:
        print(f"({n_rectangles} rectangles) looking for random free spots...")
        # take 100 random free spots to determine free spots in the map

        random_free_poses = world_gen.world.get_random_free_spots(
            model=clover_sized_box_model,
            n_spots=100,
        )

        for random_free_pose in random_free_poses:
            # determine if free space is within the room workspace (inside the room)
            if world_gen.constraints.get("room_workspace").contains_point(
                    [random_free_pose.x, random_free_pose.y]
                ):
                # persist the valid pose within the room
                pose_to_persist = Pose()
                pose_to_persist.position.x = random_free_pose.x
                pose_to_persist.position.y = random_free_pose.y
                free_poses_to_persist.append(pose_to_persist)

    rospack = rospkg.RosPack()
    pcg_path = rospack.get_path("pcg")

    if worlds_dir_path is None:
        worlds_dir_path = os.path.join(
            pcg_path, "resources", "worlds"
        )
    if models_dir_path is None:
        models_dir_path = os.path.join(
            pcg_path, "models"
        )

    # save world
    world_gen.export_world(
        filename=filename,
        output_dir=worlds_dir_path,
        with_default_ground_plane=False,
    )

    # move models from default directory to custom one
    gazebo_models_path = os.path.join(os.path.expanduser("~"), ".gazebo", "models")
    walls_dir = os.path.join(models_dir_path, f"walls_{n_rectangles}")
    ceiling_dir = os.path.join(models_dir_path, f"ceiling_{n_rectangles}")
    # clean up existing models
    if os.path.exists(walls_dir):
        shutil.rmtree(walls_dir)
    if os.path.exists(ceiling_dir):
        shutil.rmtree(ceiling_dir)
    shutil.copytree(os.path.join(gazebo_models_path, f"walls_{n_rectangles}"), walls_dir, dirs_exist_ok=True)
    shutil.copytree(os.path.join(gazebo_models_path, f"ceiling_{n_rectangles}"), ceiling_dir, dirs_exist_ok=True)

    path_to_world = os.path.join(worlds_dir_path, f"{filename}.world")
    customize_world_file(path_to_world=path_to_world)
    create_free_poses_xml(path_to_xml=f"{path_to_world}-free-poses.xml", free_poses_to_persist=free_poses_to_persist)



def create_free_poses_xml(path_to_xml: str, free_poses_to_persist: List[Pose]) -> None:
    """Persist calculated free poses in xml format in the same directory where worlds are stored."""
    poses_strs = [f"<pose><x>{p.position.x}</x><y>{p.position.y}</y></pose>" for p in free_poses_to_persist]
    poses_elements = [ET.fromstring(ps) for ps in poses_strs]
    root = ET.Element("poses")
    for e in poses_elements:
        root.append(e)
    tree = ET.ElementTree(root)
    tree.write(path_to_xml)


def customize_world_file(path_to_world: str) -> None:
    """Modify just-written xml world file to include custom parameters."""
    new_physics = """
    <physics name='default_physics' default='0' type='ode'>
        <gravity>0 0 -9.8066</gravity>
        <ode>
        <solver>
            <type>quick</type>
            <iters>10</iters>
            <sor>1.3</sor>
            <use_dynamic_moi_rescaling>0</use_dynamic_moi_rescaling>
        </solver>
        <constraints>
            <cfm>0</cfm>
            <erp>0.2</erp>
            <contact_max_correcting_vel>100</contact_max_correcting_vel>
            <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
        </ode>
        <max_step_size>0.004</max_step_size>
        <real_time_factor>1</real_time_factor>
        <real_time_update_rate>250</real_time_update_rate>
        <magnetic_field>6.0e-6 2.3e-5 -4.2e-5</magnetic_field>
    </physics>
    """
    scene = """
    <scene>
      <ambient>0.8 0.8 0.8 1</ambient>
      <background>0.8 0.9 1 1</background>
      <shadows>false</shadows>
      <grid>false</grid>
      <origin_visual>false</origin_visual>
    </scene>
    """
    parquet_plane = """
    <include>
      <uri>model://parquet_plane</uri>
      <pose>0 0 0 0 0</pose>
    </include>
    """
    tree = ET.parse(path_to_world)
    sdf = tree.getroot()
    world = sdf.find("world")
    # parse out old physics params from xml file
    old_physics = world.find("physics")
    # remove old physics params
    world.remove(old_physics)
    # parse physics params and parquet_plane model from string into xml
    new_physics_xml = ET.fromstring(new_physics)
    scene_xml = ET.fromstring(scene)
    parquet_plane_xml = ET.fromstring(parquet_plane)

    # append custom physics params and parquet_plane model into xml file
    world.append(new_physics_xml)
    world.append(scene_xml)
    world.append(parquet_plane_xml)

    # write modified element tree to original filepath
    tree.write(path_to_world)
