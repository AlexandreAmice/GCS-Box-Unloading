"""
Template file that shows how to build a generic MultibodyPlant containing one of
the 9 test scenes.
"""

from pydrake.all import (
    StartMeshcat,
    AddDefaultVisualization,
    Simulator,
    VisibilityGraph,
    RobotDiagramBuilder,
    VPolytope,
    HPolyhedron,
    SceneGraphCollisionChecker,
    RandomGenerator,
    PointCloud,
    Rgba,
    Quaternion,
    RigidTransform,
    IrisFromCliqueCoverOptions,
    IrisInConfigurationSpaceFromCliqueCoverV2,
    SaveIrisRegionsYamlFile,
)
from iris import IrisRegionGenerator

import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from station import MakeHardwareStation, load_scenario
from scenario import scenario_yaml_for_iris
from utils import ik

import numpy as np
import importlib
from scipy.spatial.transform import Rotation
from scipy.sparse import find

# TEST_SCENE = "3DOFFLIPPER"
# TEST_SCENE = "5DOFUR3"
# TEST_SCENE = "6DOFUR3"
TEST_SCENE = "7DOFIIWA"
# TEST_SCENE = "7DOFBINS"
# TEST_SCENE = "7DOF4SHELVES"
# TEST_SCENE = "14DOFIIWAS"
# TEST_SCENE = "15DOFALLEGRO"
# TEST_SCENE = "BOXUNLOADING"

rng = RandomGenerator(1234)

# scene_yaml_file = os.path.dirname(os.path.abspath(__file__)) + "/../../data/iris_benchmarks_scenes_urdf/yamls/" + TEST_SCENE + ".dmd.yaml"
src_directory = os.path.dirname(os.path.abspath(__file__))
parent_directory = os.path.dirname(src_directory)
data_directory = os.path.join(parent_directory, "data")
scene_yaml_file = os.path.join(
    data_directory, "iris_benchmarks_scenes_urdf", "yamls", TEST_SCENE + ".dmd.yaml"
)
region_file = os.path.join(data_directory, "iris_regions" + TEST_SCENE + ".yaml")

meshcat = StartMeshcat()
robot_diagram_builder = RobotDiagramBuilder()
parser = robot_diagram_builder.parser()
iris_environement_assets = os.path.join(
    data_directory, "iris_benchmarks_scenes_urdf", "iris_environments", "assets"
)
parser.package_map().Add("iris_environments", iris_environement_assets)
if TEST_SCENE == "BOXUNLOADING":
    robot_model_instances = parser.AddModelsFromString(
        scenario_yaml_for_iris, ".dmd.yaml"
    )
else:
    robot_model_instances = parser.AddModels(scene_yaml_file)
plant = robot_diagram_builder.plant()
plant.Finalize()
AddDefaultVisualization(robot_diagram_builder.builder(), meshcat=meshcat)
diagram = robot_diagram_builder.Build()

# Roll forward sim a bit to show the visualization
simulator = Simulator(diagram)
simulator.AdvanceTo(0.001)

plant_context = plant.CreateDefaultContext()

num_robot_positions = plant.num_positions()

collision_checker_params = {}
collision_checker_params["robot_model_instances"] = robot_model_instances
collision_checker_params["model"] = diagram
collision_checker_params["edge_step_size"] = 0.125
collision_checker = SceneGraphCollisionChecker(**collision_checker_params)

options = IrisFromCliqueCoverOptions()
# options.num_points_per_coverage_check = 10
options.num_points_per_visibility_round = 1000

options.coverage_termination_threshold = 0.8
options.iteration_limit = 10
options.sample_outside_of_sets = True
options.partition = False

generator = RandomGenerator(0)

sets_cover = IrisInConfigurationSpaceFromCliqueCoverV2(
    checker=collision_checker, options=options, generator=generator, sets=[]
)
set_dict = {f"sets_cover{i}": sets_cover[i] for i in range(len(sets_cover))}

generator = RandomGenerator(0)
options.partition = True

sets_partition = IrisInConfigurationSpaceFromCliqueCoverV2(
    checker=collision_checker, options=options, generator=generator, sets=[]
)
set_dict = {f"sets_partition{i}": sets_partition[i] for i in range(len(sets_partition))}

SaveIrisRegionsYamlFile(region_file, set_dict)
iris_gen = IrisRegionGenerator(meshcat, collision_checker, region_file, DEBUG=True)
ee_frame_name = "iiwa_link_7"
iris_gen.test_iris_region(
    plant,
    plant_context,
    meshcat,
    sets_cover,
    coverage=False,
    ee_frame_name="iiwa_link_7",
    name="cover",
)
iris_gen.test_iris_region(
    plant,
    plant_context,
    meshcat,
    sets_partition,
    coverage=False,
    ee_frame_name="iiwa_link_7",
    name="partition",
)
print(len(sets_partition))
