from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    Hyperellipsoid,
    HPolyhedron,
    VPolytope,
    Intersection,
    IrisFromCliqueCoverOptions,
    IrisInConfigurationSpaceFromCliqueCover,
    FastIris,
    FastIrisOptions,
    IrisInConfigurationSpace,
    IrisOptions,
    SceneGraphCollisionChecker,
    SaveIrisRegionsYamlFile,
    LoadIrisRegionsYamlFile,
    RandomGenerator,
    PointCloud,
    RobotDiagramBuilder,
    MathematicalProgram,
    Rgba,
    RigidTransform,
)
from manipulation.meshcat_utils import AddMeshcatTriad

import numpy as np
from pathlib import Path
import pydot
import matplotlib.pyplot as plt


class IrisRegionGenerator():
    def __init__(self, meshcat, collision_checker, regions_file, DEBUG=False):
        self.meshcat = meshcat
        self.collision_checker = collision_checker  # ConfigurationObstacleCollisionChecker
        self.plant = collision_checker.plant()
        self.plant_context = collision_checker.plant_context()

        self.regions_file = Path(regions_file)

        self.DEBUG = DEBUG


    @staticmethod
    def visualize_connectivity(iris_regions, output_file='../iris_connectivity.svg'):
        """
        Create and save SVG graph of IRIS Region connectivity.

        iris_regions can be a list of ConvexSets or a dictionary with keys as
        labels and values as ConvexSets.
        """
        numEdges = 0
        numNodes = 0

        graph = pydot.Dot("IRIS region connectivity")

        if isinstance(iris_regions, dict):
            items = list(iris_regions.items())
        else:
            items = list(enumerate(iris_regions))

        for i, (label1, v1) in enumerate(items):
            numNodes += 1
            graph.add_node(pydot.Node(label1))
            for j in range(i + 1, len(items)):
                label2, v2 = items[j]
                if v1.IntersectsWith(v2):
                    numEdges += 1
                    graph.add_edge(pydot.Edge(label1, label2, dir="both"))

        # Add text annotations for numNodes and numEdges
        annotation = f"Nodes: {numNodes}, Edges: {numEdges}"
        graph.add_node(pydot.Node("annotation", label=annotation, shape="none", fontsize="12", pos="0,-1!", margin="0"))

        svg = graph.create_svg()

        with open(output_file, 'wb') as svg_file:
            svg_file.write(svg)

        return numNodes, numEdges
    

    def estimate_coverage(self, plant, regions, num_samples=100000, seed=42):
        rng = RandomGenerator(seed)
        sampling_domain = HPolyhedron.MakeBox(plant.GetPositionLowerLimits(), plant.GetPositionUpperLimits())
        last_sample = sampling_domain.UniformSample(rng)

        self.collision_checker.SetConfigurationSpaceObstacles([])  # We don't want to account for any c-space obstacles during the coverge estimate

        num_samples_in_regions = 0
        num_samples_collision_free = 0
        for _ in range(num_samples):
            last_sample = sampling_domain.UniformSample(rng, last_sample)

            # Check if sample is in collision
            if self.collision_checker.CheckConfigCollisionFree(last_sample):
                num_samples_collision_free += 1

                # If sample is collsion-free, check if sample falls in regions
                for r in regions:
                    if r.PointInSet(last_sample):
                        num_samples_in_regions += 1
                        break

        return num_samples_in_regions / num_samples_collision_free


    def generate_overlap_histogram(self, regions, seed=42):
        """
        Measure region overlap by randomly sampling 100 points in each region
        and checking how many other regions that sample also falls in.
        Generally, the less overlap the better.
        """
        rng = RandomGenerator(seed)

        data = {}

        for r in regions:
            last_sample = r.UniformSample(rng, mixing_steps=5)
            for _ in range(100):
                last_sample = r.UniformSample(rng, last_sample, mixing_steps=5)
                last_sample_num_regions = 0

                # Count the number of sets the sample appears in
                for r_ in regions:
                    if r_.PointInSet(last_sample):
                        last_sample_num_regions += 1

                if last_sample_num_regions in data.keys():
                    data[last_sample_num_regions] += 1
                else:
                    data[last_sample_num_regions] = 0

        # Plot data
        num_regions = list(data.keys())
        samples = list(data.values())

        # Plotting the histogram
        if self.DEBUG:
            plt.figure(figsize=(10, 6))
            bars = plt.bar(num_regions, samples, color='blue', edgecolor='black')
            plt.xlabel('Number of Regions Sample Appears In')
            plt.ylabel('Number of Samples')
            plt.title('Histogram of Sample Distribution Across Regions')
            plt.xticks(num_regions)  # Ensure all x-axis labels are shown
            plt.grid(axis='y', linestyle='--', alpha=0.7)
            # Add text annotations on top of each bar
            for bar in bars:
                height = bar.get_height()
                plt.text(bar.get_x() + bar.get_width() / 2.0, height, f'{height}', ha='center', va='bottom')
            plt.show(block=False)
            plt.pause(1)  # Allow the plot to be displayed


    def test_iris_region(self, plant, plant_context, meshcat, regions, seed=42, num_sample=10000, colors=None, name="regions"):
        """
        Plot small spheres in the volume of each region. (we are using forward
        kinematics to return from configuration space to task space.)

        regions is a list of ConvexSets.
        """
        if not self.DEBUG:
            print("IrisRegionGenerator: DEBUG set to False; skipping region visualization.")
            return

        self.generate_overlap_histogram(regions)

        num_nodes, num_edges = IrisRegionGenerator.visualize_connectivity(regions)
        print("Connectivity graph saved to ../iris_connectivity.svg.")
        print(f"Number of nodes and edges: {num_nodes}, {num_edges}")
        print("\n\n")

        coverage = self.estimate_coverage(plant, regions)
        print(f"Estimated region coverage fraction: {coverage}")
        self.generate_source_iris_regions(coverage_check_only=True)  # clique covers will just print the coverage estimate then terminate

        world_frame = plant.world_frame()
        ee_frame = plant.GetFrameByName("arm_eef")

        rng = RandomGenerator(seed)

        # Allow caller to input custom colors
        if colors is None:
            colors = [Rgba(0.5,0.0,0.0,0.5),
                      Rgba(0.0,0.5,0.0,0.5),
                      Rgba(0.0,0.0,0.5,0.5),
                      Rgba(0.5,0.5,0.0,0.5),
                      Rgba(0.5,0.0,0.5,0.5),
                      Rgba(0.0,0.5,0.5,0.5),
                      Rgba(0.2,0.2,0.2,0.5),
                      Rgba(0.5,0.2,0.0,0.5),
                      Rgba(0.2,0.5,0.0,0.5),
                      Rgba(0.5,0.0,0.2,0.5),
                      Rgba(0.2,0.0,0.5,0.5),
                      Rgba(0.0,0.5,0.2,0.5),
                      Rgba(0.0,0.2,0.5,0.5),
                     ]

        for i in range(len(regions)):
            region = regions[i]

            xyzs = []  # List to hold XYZ positions of configurations in the IRIS region

            q_sample = region.UniformSample(rng)
            prev_sample = q_sample

            plant.SetPositions(plant_context, q_sample)
            xyzs.append(plant.CalcRelativeTransform(plant_context, frame_A=world_frame, frame_B=ee_frame).translation())

            for _ in range(num_sample-1):
                q_sample = region.UniformSample(rng, prev_sample)
                prev_sample = q_sample

                plant.SetPositions(plant_context, q_sample)
                xyzs.append(plant.CalcRelativeTransform(plant_context, frame_A=world_frame, frame_B=ee_frame).translation())
            
            # Create pointcloud from sampled point in IRIS region in order to plot in Meshcat
            xyzs = np.array(xyzs)
            pc = PointCloud(len(xyzs))
            pc.mutable_xyzs()[:] = xyzs.T
            meshcat.SetObject(f"{name}/region {i}", pc, point_size=0.025, rgba=colors[i % len(colors)])


    def load_and_test_regions(self, name="regions"):
        regions = LoadIrisRegionsYamlFile(self.regions_file)
        regions = [hpolyhedron for hpolyhedron in regions.values()]
        self.test_iris_region(self.plant, self.plant_context, self.meshcat, regions, name=name)


    def generate_source_region_at_q_nominal(self, q):
        """
        Generate a region around a nominal position so we guarantee good
        coverage around that position.
        """
        # Explicitely set plant positions at q as as seed for IRIS
        self.plant.SetPositions(self.plant_context, q)

        options = FastIrisOptions()
        options.random_seed = 0
        options.verbose = True
        domain = HPolyhedron.MakeBox(self.plant.GetPositionLowerLimits(),
                                     self.plant.GetPositionUpperLimits())
        kEpsilonEllipsoid = 1e-5
        clique_ellipse = Hyperellipsoid.MakeHypersphere(kEpsilonEllipsoid, self.plant.GetPositions(self.plant_context))
        region = FastIris(self.collision_checker, clique_ellipse, domain, options)

        regions_dict = {f"set0" : region}
        SaveIrisRegionsYamlFile(self.regions_file, regions_dict)
        
        # This source region will be drawn in black
        self.test_iris_region(self.plant, self.plant_context, self.meshcat, [region], colors=[Rgba(0.0,0.0,0.0,0.5)])


    def generate_source_iris_regions(self, minimum_clique_size=12, coverage_threshold=0.35, num_points_per_visibility_round=500, use_previous_saved_regions=True, coverage_check_only=False):
        """
        Source IRIS regions are defined as the regions considering only self-
        collision with the robot, and collision with the walls of the empty truck
        trailer (excluding the back wall).

        This function automatically searches the regions_file for existing
        regions, and begins with those.
        """
        options = IrisFromCliqueCoverOptions()
        options.num_points_per_coverage_check = 1000
        options.num_points_per_visibility_round = num_points_per_visibility_round
        options.coverage_termination_threshold = coverage_threshold
        options.minimum_clique_size = minimum_clique_size  # minimum of 7 points needed to create a shape with volume in 6D
        options.iteration_limit = 1  # Only build 1 visibility graph --> cliques --> region in order not to have too much region overlap
        options.fast_iris_options.max_iterations = 1
        options.fast_iris_options.require_sample_point_is_contained = True
        options.fast_iris_options.mixing_steps = 10  # default 50
        options.fast_iris_options.random_seed = 0
        options.fast_iris_options.verbose = True
        options.use_fast_iris = True

        if coverage_check_only:
            options.iteration_limit = 0
            regions = LoadIrisRegionsYamlFile(self.regions_file)
            regions = [hpolyhedron for hpolyhedron in regions.values()]
            self.collision_checker.SetConfigurationSpaceObstacles([])  # We don't want to account for any c-space obstacles during the coverge estimate
        elif use_previous_saved_regions:
            regions = LoadIrisRegionsYamlFile(self.regions_file)
            regions = [hpolyhedron for hpolyhedron in regions.values()]

            # Scale down previous regions and use as obstacles in new round of Clique Covers
            # Encourages exploration while still allowing small degree of region overlap
            region_obstacles = [hpolyhedron.Scale(0.995) for hpolyhedron in regions]

            # Set previous regions as obstacles to encourage exploration
            # options.iris_options.configuration_obstacles = region_obstacles  # No longer needed bc of the line below
            self.collision_checker.SetConfigurationSpaceObstacles(region_obstacles)  # Set config. space obstacles in collision checker so FastIRIS will also respect them
        else:
            regions = []

        regions = IrisInConfigurationSpaceFromCliqueCover(
            checker=self.collision_checker, options=options, generator=RandomGenerator(42), sets=regions
        )  # List of HPolyhedrons

        # Remove redundant hyperplanes
        regions = [r.ReduceInequalities() for r in regions]

        if not coverage_check_only:
            regions_dict = {f"set{i}" : regions[i] for i in range(len(regions))}
            SaveIrisRegionsYamlFile(self.regions_file, regions_dict)

            self.test_iris_region(self.plant, self.plant_context, self.meshcat, regions)
        
    
    @staticmethod
    def post_process_iris_regions(regions_dict, edge_count_threshold=0.75):
        """
        Simplify IRIS regions using SimplifyByIncrementalFaceTranslation()
        procedure. This reduces the number of faces on the HPolyhedron which
        potentially removes region intersections. This function is meant to be
        used in line with any calls to `LoadIrisRegionsYamlFile()`.

        Note: we intentionally do not saved the simplified HPolytopes to file
        since we want to keep the detailed original polyhedrons.

        regions_dict is a dictionary that maps region names to regions.

        edge_count_threshold is a tunable value that controls what fraction of
        edges relative to the average warrants allowing that vertex's edges
        to be removed. Lower --> more edges are removed.
        """
        # First find number of edges on each region
        edge_counts = {}
        for s, r in regions_dict.items():
            for s_, r_ in regions_dict.items():
                if r.IntersectsWith(r_):
                    if s not in edge_counts.keys():
                        edge_counts[s] = 0.5  # Add 0.5 instead of 1 since we're going to double count every edge
                    else:
                        edge_counts[s] += 0.5
                    
                    if s_ not in edge_counts.keys():
                        edge_counts[s_] = 0.5
                    else:
                        edge_counts[s_] += 0.5

        avg_edge_count = sum(ct for ct in edge_counts.values()) / len(edge_counts)
        print(f"IRIS region avg_edge_count: {avg_edge_count}")
                    
        # Then perform simplifications on each HPolyhedron
        output_regions = {}
        for s, r in regions_dict.items():
            intersecting_polytopes = []
            for s_, r_ in regions_dict.items():
                if r.IntersectsWith(r_) and edge_counts[s_] < avg_edge_count * edge_count_threshold:
                    intersecting_polytopes.append(r_)

            r_simplified = r.SimplifyByIncrementalFaceTranslation(min_volume_ratio=0.1,
                                                                  max_iterations=1,
                                                                  intersecting_polytopes=intersecting_polytopes,
                                                                  random_seed=42)
            print("finished call to SimplifyByIncrementalFaceTranslation.")
            
            output_regions[s] = r_simplified
        
        # FOR TESTING ONLY
        SaveIrisRegionsYamlFile("../data/TEMPORARY.yaml", output_regions)
        IrisRegionGenerator.visualize_connectivity(output_regions, output_file='../TEMPORARY.svg')

        return output_regions

