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
    ConfigurationSpaceObstacleCollisionChecker,
    RandomGenerator,
    PointCloud,
    Rgba,
    Quaternion,
    RigidTransform,
    PRM,
    FastCliqueInflation,
    FastCliqueInflationOptions,
)

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from station import MakeHardwareStation, load_scenario
from scenario import scenario_yaml_for_iris
from iris import IrisRegionGenerator
from utils import ik

import numpy as np
import importlib
from scipy.spatial.transform import Rotation
from scipy.sparse import find

# TEST_SCENE = "3DOFFLIPPER"
# TEST_SCENE = "5DOFUR3"
# TEST_SCENE = "6DOFUR3"
# TEST_SCENE = "7DOFIIWA"
# TEST_SCENE = "7DOFBINS"
# TEST_SCENE = "7DOF4SHELVES"
# TEST_SCENE = "14DOFIIWAS"
# TEST_SCENE = "15DOFALLEGRO"
TEST_SCENE = "BOXUNLOADING"

rng = RandomGenerator(1234)

scene_yaml_file = os.path.dirname(os.path.abspath(__file__)) + "/../../data/iris_benchmarks_scenes_urdf/yamls/" + TEST_SCENE + ".dmd.yaml"

meshcat = StartMeshcat()


robot_diagram_builder = RobotDiagramBuilder()
parser = robot_diagram_builder.parser()
parser.package_map().Add("iris_environments", os.path.dirname(os.path.abspath(__file__)) + "/../../data/iris_benchmarks_scenes_urdf/iris_environments/assets")
if TEST_SCENE == "BOXUNLOADING":
    robot_model_instances = parser.AddModelsFromString(scenario_yaml_for_iris, ".dmd.yaml")
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

ambient_dim = plant.num_positions()

collision_checker_params = {}
collision_checker_params["robot_model_instances"] = robot_model_instances
collision_checker_params["model"] = diagram
collision_checker_params["edge_step_size"] = 0.125
collision_checker = SceneGraphCollisionChecker(**collision_checker_params)
cspace_obstacle_collision_checker = ConfigurationSpaceObstacleCollisionChecker(collision_checker, [])

# Sample to build PRM
N = 1000
points = np.zeros((ambient_dim, N))  # ambient_dim x N
domain = HPolyhedron.MakeBox(plant.GetPositionLowerLimits(), plant.GetPositionUpperLimits())
last_polytope_sample = domain.UniformSample(rng, domain.ChebyshevCenter())
for i in range(np.shape(points)[1]):
    last_polytope_sample = domain.UniformSample(rng, last_polytope_sample)

    while not cspace_obstacle_collision_checker.CheckConfigCollisionFree(last_polytope_sample):
        last_polytope_sample = domain.UniformSample(rng, last_polytope_sample)

    points[:, i] = last_polytope_sample

# Build PRM
RADIUS = np.pi/4  # Radians
print(np.shape(points))
prm = PRM(cspace_obstacle_collision_checker, points, RADIUS)

print(prm)

# Follow a "path" through cspace, inflating regions along the way.
# Jump to a new "path" if the current path ends
regions = {}
options = FastCliqueInflationOptions()
options.parallelize = False
last_point_idx = 0  # Start with the first point
cspace_coverage = 0
COVERAGE_THRESH = 0.35
# Used for IRIS utility functions
iris_gen = IrisRegionGenerator(meshcat, cspace_obstacle_collision_checker, f"data/iris_regions_prm_{TEST_SCENE}.yaml", DEBUG=True)

while cspace_coverage < COVERAGE_THRESH:
    # Find the first point that connects to last_point_idx
    found_edge = False
    for i in range(N):
        if i == last_point_idx:
            continue

        if prm[last_point_idx, i] != 0:
            # Remove that edge from prm to ensure we don't find it again
            prm[last_point_idx, i] = 0
            prm[i, last_point_idx] = 0

            # Only inflate a region here if both points forming the clique are not in other regions
            print(regions.values())
            if all(not (r.PointInSet(points[:, last_point_idx]) or r.PointInSet(points[:, i])) for r in regions.values()):            
                line_clique = np.hstack((points[:,last_point_idx:last_point_idx+1], points[:,i:i+1]))  # ambient_dim x 2
                region = FastCliqueInflation(cspace_obstacle_collision_checker, line_clique, domain, options)
                regions[f"{last_point_idx},{i}"] = region

            # Update last_point_idx
            last_point_idx = i
            found_edge = True
            break
    
    # If this point had no edges, pick another next
    if not found_edge:
        last_point_idx += 1

    cspace_coverage = iris_gen.estimate_coverage(regions, num_samples=500)

# Visualize IRIS regions
iris_gen.test_iris_region(plant, plant_context, meshcat, regions)