"""
Utility functions related to simulation setup, including simulation scenario 
YAML definitions and box poses.
"""

from pydrake.all import (
    RigidTransform,
    RotationMatrix,
    ExternallyAppliedSpatialForce,
    SpatialForce,
)

import numpy as np
import os
import re

NUM_BOXES = 40
BOX_DIM = 0.5  # box edge length in m

GRIPPER_DIM = 0.25  # edge length of gripper in m
GRIPPER_THICKNESS = 0.045  # distance from gripper face to end of the last robot link
PREPICK_MARGIN = 0.2  # how far pre-pick pose is from the box face

q_nominal = [0.0, -2.2, 2.2, 0.0, 1.57, 0.0]  # Neutral "pulled back" position
q_place_nominal = [1.55738942, -2.2127606, 2.02098258, 0.02843482, 1.7572682, 1.52160177]  # nominal configuration for dropping box onto conveyor

robot_pose = RigidTransform([0.0,0.0,0.58])

relative_path_to_robot_base = '../data/unload-gen0/robot_base.urdf'
relative_path_to_robot_arm = '../data/unload-gen0/robot_arm.urdf'

""" Note: It is necessary to split the truck trailer into individual parts since
    Drake automatically takes the convex hull of the collision geometry, which 
    would make the hollow shipping container no longer hollow."""
relative_path_to_truck_trailer_floor = '../data/Truck_Trailer_Floor.sdf'
relative_path_to_truck_trailer_back = '../data/Truck_Trailer_Back.sdf'
relative_path_to_truck_trailer_right_side = '../data/Truck_Trailer_Right_Side.sdf'
relative_path_to_truck_trailer_left_side = '../data/Truck_Trailer_Left_Side.sdf'
relative_path_to_truck_trailer_roof = '../data/Truck_Trailer_Roof.sdf'

absolute_path_to_robot_base = os.path.abspath(relative_path_to_robot_base)
absolute_path_to_robot_arm = os.path.abspath(relative_path_to_robot_arm)
absolute_path_to_truck_trailer_floor = os.path.abspath(relative_path_to_truck_trailer_floor)
absolute_path_to_truck_trailer_back = os.path.abspath(relative_path_to_truck_trailer_back)
absolute_path_to_truck_trailer_right_side = os.path.abspath(relative_path_to_truck_trailer_right_side)
absolute_path_to_truck_trailer_left_side = os.path.abspath(relative_path_to_truck_trailer_left_side)
absolute_path_to_truck_trailer_roof = os.path.abspath(relative_path_to_truck_trailer_roof)

scenario_yaml = f"""
directives:
- add_model:
    name: robot_base
    file: file://{absolute_path_to_robot_base}
- add_model:
    name: kuka
    file: file://{absolute_path_to_robot_arm}
    default_joint_positions:
        arm_a1: [{q_nominal[0]}]
        arm_a2: [{q_nominal[1]}]
        arm_a3: [{q_nominal[2]}]
        arm_a4: [{q_nominal[3]}]
        arm_a5: [{q_nominal[4]}]
        arm_a6: [{q_nominal[5]}]
- add_weld:
    parent: robot_base::base
    child: kuka::base_link        


- add_model: 
    name: Truck_Trailer_Floor
    file: file://{absolute_path_to_truck_trailer_floor}
- add_weld:
    parent: world
    child: Truck_Trailer_Floor::Truck_Trailer_Floor


- add_model: 
    name: Truck_Trailer_Right_Side
    file: file://{absolute_path_to_truck_trailer_right_side}
- add_weld:
    parent: world
    child: Truck_Trailer_Right_Side::Truck_Trailer_Right_Side


- add_model: 
    name: Truck_Trailer_Left_Side
    file: file://{absolute_path_to_truck_trailer_left_side}
- add_weld:
    parent: world
    child: Truck_Trailer_Left_Side::Truck_Trailer_Left_Side
    
    
- add_model: 
    name: Truck_Trailer_Roof
    file: file://{absolute_path_to_truck_trailer_roof}


- add_model: 
    name: Truck_Trailer_Back
    file: file://{absolute_path_to_truck_trailer_back}
- add_weld:
    parent: world
    child: Truck_Trailer_Back::Truck_Trailer_Back
"""


scenario_yaml_for_iris = scenario_yaml.replace(
f"""
model_drivers:
    kuka: !ForceDriver {{}}  # ForceDriver allows access to desired_state and desired_acceleration input ports for station (results in better traj following)
""",
""
)

scenario_yaml_for_iris = scenario_yaml_for_iris.replace(
f"""
- add_model: 
    name: Truck_Trailer_Roof
    file: file://{absolute_path_to_truck_trailer_roof}
""",
f"""
- add_model: 
    name: Truck_Trailer_Roof
    file: file://{absolute_path_to_truck_trailer_roof}
- add_weld:
    parent: world
    child: Truck_Trailer_Roof::Truck_Trailer_Roof
"""
)


robot_yaml = f"""
directives:
- add_model:
    name: kuka
    file: file://{absolute_path_to_robot_arm}
    default_joint_positions:
        arm_a1: [{q_nominal[0]}]
        arm_a2: [{q_nominal[1]}]
        arm_a3: [{q_nominal[2]}]
        arm_a4: [{q_nominal[3]}]
        arm_a5: [{q_nominal[4]}]
        arm_a6: [{q_nominal[5]}]

- add_weld:
    parent: world
    child: kuka::base_link
"""


def set_hydroelastic(enable_hydroelastic, sdf_path='../data/Box_0_5_0_5_0_5.sdf'):
    """
    Trade off simulation accuracy for speed.
    """
    with open(sdf_path, 'r') as file:
        sdf_content = file.read()

    if enable_hydroelastic:
        # Uncomment hydroelastic line if it is commented out
        sdf_content = re.sub(
            r'<!--\s*<drake:compliant_hydroelastic\s*/>\s*-->',
            r'<drake:compliant_hydroelastic/>',
            sdf_content
        )
    else:
        # Comment out hydroelastic line if it is uncommented
        sdf_content = re.sub(
            r'(?<!<!--)\s*<drake:compliant_hydroelastic\s*/>(?!\s*-->)',
            r'\n\t\t\t\t\t<!-- <drake:compliant_hydroelastic/> -->',
            sdf_content
        )

    with open(sdf_path, 'w') as file:
        file.write(sdf_content)


def get_fast_box_poses():
    box_poses_string = """
    {BodyIndex(31): RigidTransform(
    R=RotationMatrix([
        [-0.8536127893242504, -0.047723328745299276, -0.5187173505827053],
        [-0.30954973549914616, 0.8473706828154717, 0.4314416381820953],
        [0.41895604441812473, 0.5288529188709367, -0.7380991959399557],
    ]),
    p=[3.1869236552206166, 0.447422027159258, 0.49705483014631335],
    ), BodyIndex(32): RigidTransform(
    R=RotationMatrix([
        [0.04019812282118512, -0.9979837167884987, 0.04911834633481049],
        [-0.5328830404469128, -0.06299597670738571, -0.8438407267504551],
        [0.8452335630615402, 0.007746479438814569, -0.53434091732769],
    ]),
    p=[3.371096502984189, -0.14463774007467486, 0.5182361336575764],
    ), BodyIndex(33): RigidTransform(
    R=RotationMatrix([
        [0.8709837416796266, -0.43341121877320093, 0.23139152355107173],
        [-0.1503673493289795, -0.683519496683131, -0.7142763876188375],
        [0.46773601741317106, 0.5873293906280935, -0.6605052648684171],
    ]),
    p=[2.909715693519941, -0.10852603699090088, 1.363685778726578],
    ), BodyIndex(34): RigidTransform(
    R=RotationMatrix([
        [0.9998873525146329, -0.014642882547183564, -0.0032967062331356986],
        [0.014634480796876832, 0.9998896372175946, -0.0025583894685656857],
        [0.003333804595961195, 0.0025098556883636934, 0.999991293147765],
    ]),
    p=[2.7964014147701395, 0.6469846339043066, 0.5241989521271369],
    ), BodyIndex(35): RigidTransform(
    R=RotationMatrix([
        [0.1887068959781537, 0.5841503818671944, 0.7894035968848263],
        [0.8254737812613565, -0.5297928011429291, 0.19471164399492621],
        [0.531961224019541, 0.6148885421127473, -0.5821763795604318],
    ]),
    p=[1.3812103860317329, -0.9141717359667607, 0.21386289673084052],
    ), BodyIndex(36): RigidTransform(
    R=RotationMatrix([
        [0.25588193724317143, -0.9617277290915596, -0.0980010677955771],
        [-0.6894241481318253, -0.11048233009494668, -0.7158826710498711],
        [0.6776568292029443, 0.25074574738803196, -0.6913087530195378],
    ]),
    p=[3.825328134801734, -0.24759653203564952, 1.5084963933561102],
    ), BodyIndex(37): RigidTransform(
    R=RotationMatrix([
        [0.23678450527599637, 0.04082548226432009, 0.9707040630692173],
        [0.8256207105728885, -0.5351161921146679, -0.17888852173858466],
        [0.5121362517283071, 0.8437914084266225, -0.16041358649159534],
    ]),
    p=[3.3171899264354936, -0.5464413756624368, 0.8580918446129515],
    ), BodyIndex(38): RigidTransform(
    R=RotationMatrix([
        [0.031234071177496825, -0.9960779673732557, 0.0827835473463322],
        [-0.18214792594831486, -0.08710948419599901, -0.9794049575307622],
        [0.9827749314417856, 0.015511952703330556, -0.18415431966946472],
    ]),
    p=[2.766683358163046, 0.2180805233140119, 0.5209213955766314],
    ), BodyIndex(39): RigidTransform(
    R=RotationMatrix([
        [-0.6293322935235526, -0.047041954991401236, -0.7757112341580283],
        [-0.4812268212176171, 0.8073586712023503, 0.34145676530882035],
        [0.6102143974613938, 0.5881828206475961, -0.530734734708277],
    ]),
    p=[3.322957543534521, 0.17278340714639637, 0.9802375833625083],
    ), BodyIndex(40): RigidTransform(
    R=RotationMatrix([
        [-0.7855411032762786, 0.23677994197269456, 0.571717092750334],
        [0.29322670610904, 0.9560176452899584, 0.006954187133376893],
        [-0.5449250167572869, 0.1731055197665596, -0.8204213582900066],
    ]),
    p=[2.948429639145896, 0.6481564395517053, 0.6282715880112066],
    ), BodyIndex(41): RigidTransform(
    R=RotationMatrix([
        [0.054428864218120786, -0.010363018023291745, 0.9984638734563079],
        [0.003065891719041778, 0.9999431638094186, 0.010211241784290096],
        [-0.9985129438559585, 0.002505395828816546, 0.054457542579962825],
    ]),
    p=[3.276499268298285, -1.2022228128647536, 1.0395929408156372],
    ), BodyIndex(42): RigidTransform(
    R=RotationMatrix([
        [0.9995703939660746, 0.01613213883531897, 0.024470014366653903],
        [-0.014974216257441105, -0.43661656799121745, 0.8995230655203045],
        [0.025195244670009487, -0.899503044270651, -0.43618742874348154],
    ]),
    p=[3.4921104289450144, 0.022793758258257375, 0.46943777887977695],
    ), BodyIndex(43): RigidTransform(
    R=RotationMatrix([
        [0.08535035930967405, -0.8500273564733326, -0.5197776538219658],
        [0.9930686096554572, 0.11488675867019793, -0.02481469724241786],
        [0.0808087413754644, -0.514056928685154, 0.8539411111945013],
    ]),
    p=[2.3627387117738285, -1.2718087903849935, 1.1035934880447682],
    ), BodyIndex(44): RigidTransform(
    R=RotationMatrix([
        [0.0007858850727897604, -0.9707112391792738, 0.24024793967002311],
        [-0.0005565850712406228, -0.24024840124849933, -0.9707112834981362],
        [0.999999536298748, 0.0006291490910733111, -0.0007290910165155817],
    ]),
    p=[2.1618259876256802, -1.0643399874644561, 0.025532667721623655],
    ), BodyIndex(45): RigidTransform(
    R=RotationMatrix([
        [0.5219766498595195, -0.8323698741934558, -0.18628142563498729],
        [-0.06054721351032344, 0.18168522110699137, -0.9814909655047462],
        [0.8508080934838173, 0.5235941872924817, 0.04443776655006115],
    ]),
    p=[3.5840018710959316, -1.240163388940937, 1.2238155529870303],
    ), BodyIndex(46): RigidTransform(
    R=RotationMatrix([
        [0.5199467893150228, -0.8541985861165214, 0.00033430156331069535],
        [0.8541835326191943, 0.5199352181985206, -0.00615316821244144],
        [0.005082212430940547, 0.0034848749464830337, 0.9999810132014578],
    ]),
    p=[2.6374908127164023, -0.6988652402806506, 0.5246891151860461],
    ), BodyIndex(47): RigidTransform(
    R=RotationMatrix([
        [0.007215800151295526, -0.6832574313577283, 0.7301419140979482],
        [0.0059868113627425945, -0.7301183192836653, -0.6832945177126055],
        [0.9999560441928852, 0.009301738592026154, -0.0011778545250197003],
    ]),
    p=[1.3919882603262008, -0.5455016667384315, 0.025256833995291553],
    ), BodyIndex(48): RigidTransform(
    R=RotationMatrix([
        [-0.5174862284997326, 0.04785663063872464, 0.8543522377901469],
        [0.8547247611290338, -0.018538801430840968, 0.5187503210162132],
        [0.040664308994978915, 0.9986821595209443, -0.03131067275777322],
    ]),
    p=[1.7054181426776405, -0.18320831912059057, 0.023610282762503692],
    ), BodyIndex(49): RigidTransform(
    R=RotationMatrix([
        [0.8069688733479689, -0.05109915565829089, 0.5883792261276054],
        [0.06076875672502838, 0.9981462789020562, 0.003341275216573821],
        [-0.5874592714849297, 0.033058768957544844, 0.8085782102811128],
    ]),
    p=[3.428113420738816, 0.7426479712753069, 1.0438746127715521],
    ), BodyIndex(50): RigidTransform(
    R=RotationMatrix([
        [0.5648357203040141, -0.7849663310952768, 0.25453578945890054],
        [-0.00691600149460142, -0.31294440345773555, -0.9497462657298572],
        [0.8251743924722653, 0.534690246209244, -0.18219100586122675],
    ]),
    p=[3.1697977881620134, -1.060970739283022, 1.4882785196675854],
    ), BodyIndex(51): RigidTransform(
    R=RotationMatrix([
        [0.6392473421360046, -0.7689656637987066, -0.007392122203279339],
        [-0.5186476258757593, -0.42401884356565983, -0.742436974075698],
        [0.5677741414906917, 0.4784347690123021, -0.669875134674107],
    ]),
    p=[3.680375184595719, -0.7746776623789434, 0.37818534519630775],
    ), BodyIndex(52): RigidTransform(
    R=RotationMatrix([
        [0.9353161024596756, -0.35379803143360794, -0.0032774125991891353],
        [0.3537491048139943, 0.9352828398821379, -0.010372090690925642],
        [0.006734933031535458, 0.008541801666325592, 0.9999408374005703],
    ]),
    p=[3.532659658825348, 0.19556255259500382, 0.5240384515274412],
    ), BodyIndex(53): RigidTransform(
    R=RotationMatrix([
        [0.4240222431225529, -0.7625174855471513, 0.4886432457040295],
        [-0.04194071232915436, -0.5555028400949389, -0.8304562428543601],
        [0.9046801169400385, 0.33163787311057635, -0.267526086825288],
    ]),
    p=[2.202897868333349, -0.09186948290570038, 0.016645408900757133],
    ), BodyIndex(54): RigidTransform(
    R=RotationMatrix([
        [0.5408710756882906, -0.8325212192823759, -0.11986199951768645],
        [-0.4358398937310062, -0.1555243483876776, -0.8864850614032554],
        [0.7193761648236177, 0.5317147698749535, -0.446964581349002],
    ]),
    p=[3.053161099113765, 0.8342795045731867, 0.8186377971448809],
    ), BodyIndex(55): RigidTransform(
    R=RotationMatrix([
        [-0.16815106608635422, -0.017809396814850414, -0.9856003471788726],
        [-0.04107858703238406, 0.9990948701765971, -0.01104491168961043],
        [0.98490495412575, 0.03862985596529763, -0.16873045239865558],
    ]),
    p=[2.99694581176524, -0.68463482136439, 0.021844102594615663],
    ), BodyIndex(56): RigidTransform(
    R=RotationMatrix([
        [0.18668099273959798, -0.9823933872490804, -0.0073102420641552035],
        [0.15662154009317225, 0.0371065112868873, -0.9869613974208714],
        [0.9698556078760391, 0.18310199209564748, 0.16079104565243824],
    ]),
    p=[3.8984037360636967, -0.7432050842814731, 1.0658621776500632],
    ), BodyIndex(57): RigidTransform(
    R=RotationMatrix([
        [0.9976327560793382, -0.0031890831296258054, 0.06869289443847965],
        [0.06865676390139679, -0.010316904398871167, -0.9975869938277125],
        [0.003890085877153622, 0.9999416939164089, -0.010073529642160243],
    ]),
    p=[3.49193838603759, 0.2516126856828918, 0.5244082760627368],
    ), BodyIndex(58): RigidTransform(
    R=RotationMatrix([
        [0.997487096070097, -0.036937735949430714, -0.06045739687230471],
        [0.059548716416914685, 0.8994438744603241, 0.4329603527678932],
        [0.038385460095498425, -0.43547252537758424, 0.8993832531766011],
    ]),
    p=[3.472945466265526, -0.2006813335862031, 1.1316782250454338],
    ), BodyIndex(59): RigidTransform(
    R=RotationMatrix([
        [0.07058725138111585, -0.5193355124396604, -0.8516502013511735],
        [-0.9973854478682365, -0.02349399266711763, -0.06833959825192554],
        [0.015482516692590487, 0.8542474219028071, -0.5196360590348506],
    ]),
    p=[2.483031722891834, -0.7337030787208404, 0.021145850306302196],
    ), BodyIndex(60): RigidTransform(
    R=RotationMatrix([
        [0.9988343742499888, -0.01424200664960192, -0.046120039714055386],
        [0.009792192630318585, 0.995399463170361, -0.0953101342127255],
        [0.04726527033799856, 0.09474742195305214, 0.9943786603970975],
    ]),
    p=[3.4791176715618284, -1.2713066021731687, 2.354113744341829],
    ), BodyIndex(61): RigidTransform(
    R=RotationMatrix([
        [0.04081999890327781, -0.9991664822859636, 0.0002614686019306589],
        [-0.3871278339883035, -0.01605698584356985, -0.9218862258202711],
        [0.9211220157183635, 0.03753017295340905, -0.3874606022256021],
    ]),
    p=[3.979978276136306, 0.8115010363645038, 1.5256838655409035],
    ), BodyIndex(62): RigidTransform(
    R=RotationMatrix([
        [0.44103748742657045, -0.896052519293395, 0.050752510799148165],
        [-0.5099834727688668, -0.29674411026356373, -0.8073783441029937],
        [0.7385139079171523, 0.33020117457672027, -0.5878472523718129],
    ]),
    p=[3.3003239840861456, 0.5984875230821538, 1.2450309579362417],
    ), BodyIndex(63): RigidTransform(
    R=RotationMatrix([
        [0.5635741714737867, -0.8260568231560232, -0.003778645923244728],
        [-0.00446006846369934, 0.0015314013331437226, -0.9999888812378146],
        [0.8260534250500593, 0.5635847582461182, -0.0028212117486615396],
    ]),
    p=[2.968980666966756, -1.2684558764481033, 1.0192602352295455],
    ), BodyIndex(64): RigidTransform(
    R=RotationMatrix([
        [-0.028609133461320324, -0.0032971286941236855, 0.9995852371983928],
        [0.9989444732047008, -0.03604567241483869, 0.02847189761756641],
        [0.03593684650015094, 0.9993447045151822, 0.004324883939258561],
    ]),
    p=[3.49108668904645, -0.17055542453929814, 0.02474874616630205],
    ), BodyIndex(65): RigidTransform(
    R=RotationMatrix([
        [0.2961708224728054, -0.9052850154552283, 0.3045355229000536],
        [-0.22794267400122065, -0.37661975857613467, -0.8978862371254097],
        [0.9275370511655884, 0.19651106391824696, -0.3178968708131044],
    ]),
    p=[2.8765490105453275, 0.17693992520555063, 0.985752031156955],
    ), BodyIndex(66): RigidTransform(
    R=RotationMatrix([
        [0.9919832426776406, -0.12579682439785836, -0.012016872228976924],
        [0.12581616011166943, 0.9920531979942733, 0.0008638310540279962],
        [0.011812709321232151, -0.0023688226505027706, 0.9999274216550632],
    ]),
    p=[3.0206794214673107, -1.2673689692148866, 0.5254622790436679],
    ), BodyIndex(67): RigidTransform(
    R=RotationMatrix([
        [0.9509564127189757, 0.028374018400038198, 0.308020804798136],
        [0.3082271168934394, -0.003100558645244597, -0.9513077477597064],
        [-0.026037386969800708, 0.999592567807465, -0.01169413801283178],
    ]),
    p=[3.5122070384550574, 0.46250629432266466, 1.0414921022430428],
    ), BodyIndex(68): RigidTransform(
    R=RotationMatrix([
        [-0.034521089573960884, 0.7626151845660407, 0.6459306268044042],
        [0.8359979629452579, -0.33212812382881857, 0.4368046649399802],
        [0.54764559737663, 0.5550756611771525, -0.6260793160956059],
    ]),
    p=[2.498629029739537, -0.4273635454701759, 0.5116335890223804],
    ), BodyIndex(69): RigidTransform(
    R=RotationMatrix([
        [0.16027637277797557, -0.8229334783922633, 0.5450614410049072],
        [-0.027017210937048064, -0.5556507384103172, -0.8309767308518647],
        [0.9867023637563247, 0.11845989635855597, -0.11129105225669192],
    ]),
    p=[2.5022953066324325, 0.5742806249222091, 0.021044478032981928],
    ), BodyIndex(70): RigidTransform(
    R=RotationMatrix([
        [-0.05646763612882777, -0.9984041650029364, -0.0007275814810643655],
        [0.9984032866269875, -0.056466453520647275, -0.0015546315414793487],
        [0.0015110666601749123, -0.0008142061101822484, 0.9999985268718943],
    ]),
    p=[2.9845470011600685, 0.007699868564930984, 0.5257880610083309],
    )}
    RigidTransform(
    R=RotationMatrix([
        [-0.8536098718270233, -0.04772102920326726, -0.5187223632071727],
        [-0.3095507258311142, 0.8473705832034533, 0.4314411232820417],
        [0.41896125698790265, 0.5288532859813017, -0.7380959741455688],
    ]),
    p=[3.186922219812689, 0.447422426268203, 0.49705190263460913],
    )
    RigidTransform(
    R=RotationMatrix([
        [0.04020062383227477, -0.9979837912708588, 0.04911478600318947],
        [-0.5328833851329288, -0.0629943063314353, -0.8438406337805097],
        [0.8452332268035652, 0.0077504664675396295, -0.5343413914126145],
    ]),
    p=[3.3710956372267638, -0.14463947369813762, 0.5182351187386193],
    )
    RigidTransform(
    R=RotationMatrix([
        [0.8709871962229057, -0.43340099331009224, 0.23139767287850377],
        [-0.15035182197295632, -0.6835181731257793, -0.7142809227721326],
        [0.4677345760633632, 0.5873384765232245, -0.6604982061672866],
    ]),
    p=[2.909719138276534, -0.1085275877363519, 1.3636859298684254],
    )
    RigidTransform(
    R=RotationMatrix([
        [0.9998873545013918, -0.01464308446686603, -0.0032952064433542424],
        [0.014634688446047006, 0.9998896356867972, -0.002557799877045424],
        [0.0033322968498071762, 0.0025092874327390384, 0.9999912995992939],
    ]),
    p=[2.7964021858983066, 0.6469848298639723, 0.5241991540567049],
    )
    RigidTransform(
    R=RotationMatrix([
        [0.18883108774441426, 0.5841022017797527, 0.7894095503458939],
        [0.8254142202283158, -0.5299089218947923, 0.1946481429173682],
        [0.5320095726076203, 0.6148342478850245, -0.5821895415425403],
    ]),
    p=[1.3811788807678114, -0.9141458565206424, 0.2138115464295215],
    )
    RigidTransform(
    R=RotationMatrix([
        [0.25582367942067336, -0.9617389993713175, -0.09804255267956641],
        [-0.6894302501993204, -0.11040946968333859, -0.7158880353200117],
        [0.6776726165028146, 0.25073461287285576, -0.6912973157402883],
    ]),
    p=[3.8253368153961493, -0.24759576315488405, 1.5084928955798425],
    )
    RigidTransform(
    R=RotationMatrix([
        [0.23679800071336043, 0.040827180426142085, 0.9707006995962282],
        [0.8256183579734161, -0.5351162834440467, -0.17889910611937365],
        [0.5121338046208583, 0.8437912683427591, -0.16042213573282996],
    ]),
    p=[3.317187285865186, -0.5464432973061067, 0.8580909650986392],
    )
    RigidTransform(
    R=RotationMatrix([
        [0.031239449889045368, -0.996077568254033, 0.0827863200762195],
        [-0.18214944827166835, -0.0871132255542495, -0.9794043416423381],
        [0.9827744783341481, 0.015516570325524393, -0.18415634873763254],
    ]),
    p=[2.766682906104442, 0.21808082933612497, 0.5209209921380021],
    )
    RigidTransform(
    R=RotationMatrix([
        [-0.6293228623213392, -0.04704072966060094, -0.7757189598770131],
        [-0.48122988681488466, 0.8073591784525769, 0.3414512454282653],
        [0.6102217064279263, 0.5881822223778771, -0.5307269941154411],
    ]),
    p=[3.322955327844624, 0.1727829907397551, 0.9802342348180504],
    )
    RigidTransform(
    R=RotationMatrix([
        [-0.7855420352730781, 0.23678078542870792, 0.5717154628578754],
        [0.2932270042611794, 0.9560175572345313, 0.006953720682757861],
        [-0.5449235127896802, 0.1731048523614625, -0.8204224980446794],
    ]),
    p=[2.948428809409267, 0.6481561075523267, 0.6282708996457611],
    )
    RigidTransform(
    R=RotationMatrix([
        [0.05442913854961711, -0.010363915047973932, 0.9984638491911588],
        [0.0030673500449179417, 0.9999431509821111, 0.010212059913102405],
        [-0.998512924423339, 0.0025068045087781985, 0.05445783406183391],
    ]),
    p=[3.2764991938936974, -1.2022227677903345, 1.039592608890841],
    )
    RigidTransform(
    R=RotationMatrix([
        [0.9995704265878524, 0.016135749585970888, 0.02446630082945661],
        [-0.01496929208053649, -0.43661680548133963, 0.8995230322042219],
        [0.02519687650456335, -0.899502864229219, -0.4361877057619173],
    ]),
    p=[3.492108986959193, 0.022793268167588546, 0.4694375454390706],
    )
    RigidTransform(
    R=RotationMatrix([
        [0.0853287773606477, -0.8500284484279849, -0.5197794114980406],
        [0.9930703186422537, 0.11486927905788502, -0.024827222166285468],
        [0.0808105314046961, -0.5140590292873322, 0.853939677273668],
    ]),
    p=[2.362739901647918, -1.271809502827564, 1.103589183325209],
    )
    RigidTransform(
    R=RotationMatrix([
        [0.0007860080700236027, -0.9707294666725093, 0.2401742798990344],
        [-0.0005562114486773462, -0.240174741328672, -0.9707295113761247],
        [0.9999995364099618, 0.0006294135455941485, -0.000728710127831933],
    ]),
    p=[2.161821799100168, -1.0643482383598766, 0.025532681676665264],
    )
    RigidTransform(
    R=RotationMatrix([
        [0.5219780445859591, -0.8323691974909627, -0.18628054122336335],
        [-0.06054900168848798, 0.18168316368369075, -0.9814912360425907],
        [0.8508071105513552, 0.5235959769728333, 0.04443549856947793],
    ]),
    p=[3.5840016333762192, -1.2401622539892492, 1.2238139084446826],
    )
    RigidTransform(
    R=RotationMatrix([
        [0.5199458727260959, -0.8541991438177108, 0.00033486741917500545],
        [0.8541840952502999, 0.5199343007871422, -0.006152583718422557],
        [0.005081422687098184, 0.0034850489344723907, 0.9999810166085151],
    ]),
    p=[2.637491243523391, -0.698865046195784, 0.5246891019185214],
    )
    RigidTransform(
    R=RotationMatrix([
        [0.007223495023751969, -0.6833796373470582, 0.730027460017118],
        [0.005993287918496681, -0.7300038077687263, -0.6834167989909863],
        [0.9999559498395755, 0.009311922602951328, -0.0011774879484784673],
    ]),
    p=[1.3919695367758405, -0.5455185277014618, 0.025256755559075795],
    )
    RigidTransform(
    R=RotationMatrix([
        [-0.5174809090012157, 0.047858836612052646, 0.8543553362491605],
        [0.854727774428437, -0.018542781272349096, 0.5187452138413392],
        [0.04066866656144952, 0.9986819799219501, -0.03131074159592784],
    ]),
    p=[1.7054185977264125, -0.18320806361083683, 0.023610194178165234],
    )
    RigidTransform(
    R=RotationMatrix([
        [0.8069685306293344, -0.05109877472654867, 0.5883797292526132],
        [0.06076825735129816, 0.9981463090574646, 0.0033413490459271306],
        [-0.5874597939199069, 0.033058447277598386, 0.8085778438661146],
    ]),
    p=[3.428114626639301, 0.742648429615116, 1.0438733344894189],
    )
    RigidTransform(
    R=RotationMatrix([
        [0.5648339142926659, -0.7849657372314763, 0.2545416284961467],
        [-0.006917020016501962, -0.3129516457279955, -0.9497438719308648],
        [0.8251756201554411, 0.534686879218842, -0.18219532675757377],
    ]),
    p=[3.169798969916165, -1.0609707366219574, 1.4882772548435728],
    )
    RigidTransform(
    R=RotationMatrix([
        [0.6392443161834531, -0.7689681674469794, -0.007393353798086033],
        [-0.5186496342003738, -0.42401589984734556, -0.7424372523119138],
        [0.5677757137910777, 0.4784333539061425, -0.6698748127069356],
    ]),
    p=[3.6803763495488075, -0.7746774611385822, 0.37818453862439205],
    )
    RigidTransform(
    R=RotationMatrix([
        [0.9353159934218447, -0.35379835973079776, -0.00327308739646651],
        [0.3537494840826432, 0.9352827195523017, -0.010370005695007598],
        [0.006730153086792279, 0.008541379200558787, 0.9999408731924007],
    ]),
    p=[3.532660841748098, 0.19556235342255648, 0.524038599058401],
    )
    RigidTransform(
    R=RotationMatrix([
        [0.4240223500032679, -0.7625181197922154, 0.4886421632301605],
        [-0.041940649150678966, -0.5555016724368589, -0.8304570271053601],
        [0.9046800697741306, 0.33163837068167734, -0.26752562951066317],
    ]),
    p=[2.202897863851858, -0.09186949574917934, 0.016645370947861945],
    )
    RigidTransform(
    R=RotationMatrix([
        [0.5408746709440747, -0.8325189914270306, -0.11986124997036679],
        [-0.4358405176879571, -0.15552798633148213, -0.8864841163885384],
        [0.7193730836448029, 0.5317171939866099, -0.4469666566382478],
    ]),
    p=[3.0531612053667065, 0.834280644027419, 0.8186363840240759],
    )
    RigidTransform(
    R=RotationMatrix([
        [-0.1681514700856831, -0.017808573230160413, -0.985600293134864],
        [-0.04107801144645178, 0.9990949021586165, -0.011044159373606071],
        [0.9849049091580511, 0.03862940848848774, -0.1687308173280766],
    ]),
    p=[2.996945756509604, -0.6846348775284448, 0.02184444836989125],
    )
    RigidTransform(
    R=RotationMatrix([
        [0.18669177691465688, -0.982391311518236, -0.007313787386044268],
        [0.15663000619562262, 0.03711346774649771, -0.9869597923274226],
        [0.9698521648125591, 0.18311171880933774, 0.1608007364506855],
    ]),
    p=[3.89840434987018, -0.7432060873250637, 1.0658618747308086],
    )
    RigidTransform(
    R=RotationMatrix([
        [0.9976326224250419, -0.0031841170628317053, 0.06869506584802026],
        [0.06865899120407717, -0.010315369007775162, -0.9975868564135516],
        [0.0038850482843987635, 0.999941725582428, -0.010072330369099023],
    ]),
    p=[3.4919398889935978, 0.25161205862135, 0.5244085297625887],
    )
    RigidTransform(
    R=RotationMatrix([
        [0.9974875930193244, -0.03693875350210192, -0.06044857535315082],
        [0.05954575817911048, 0.899444344575558, 0.4329597829982824],
        [0.03837713453854387, -0.43547146806737624, 0.8993841204089911],
    ]),
    p=[3.4729494319507754, -0.20068088364652614, 1.1316779032754296],
    )
    RigidTransform(
    R=RotationMatrix([
        [0.07059750205317786, -0.5193364211742157, -0.8516487975367599],
        [-0.9973847947324224, -0.02350341666959796, -0.06834588971854072],
        [0.015477853223726079, 0.8542466102050467, -0.5196375323365047],
    ]),
    p=[2.4830310516241285, -0.7337029864361154, 0.02114630736312074],
    )
    RigidTransform(
    R=RotationMatrix([
        [0.9988343436014616, -0.014240442809805352, -0.04612118635527611],
        [0.009790490873445397, 0.9953994105842296, -0.0953108582325501],
        [0.04726627053930135, 0.09474820946671735, 0.9943785378175439],
    ]),
    p=[3.479117009503441, -1.2713065052542951, 2.3541131069321652],
    )
    RigidTransform(
    R=RotationMatrix([
        [0.04081644434863263, -0.9991666271049645, 0.0002629646811895814],
        [-0.3871260559523365, -0.016056914229917285, -0.9218869737165187],
        [0.9211229205016686, 0.03752634787854581, -0.3874588217364199],
    ]),
    p=[3.979982455496992, 0.8115000313511036, 1.5256833616810523],
    )
    RigidTransform(
    R=RotationMatrix([
        [0.44103839101897563, -0.8960520550002966, 0.05075285585204553],
        [-0.5099801734186622, -0.29674359404551537, -0.8073806178704203],
        [0.7385156466638092, 0.33020289841655803, -0.5878440996642221],
    ]),
    p=[3.3003262579262387, 0.5984854023121416, 1.2450290576803194],
    )
    RigidTransform(
    R=RotationMatrix([
        [0.5635735847328527, -0.8260572394368618, -0.0037751510318297377],
        [-0.004460222840172889, 0.001527067670709037, -0.999988887176525],
        [0.8260538245196725, 0.5635841598539477, -0.002823784365315274],
    ]),
    p=[2.9689809883423863, -1.2684557998963824, 1.0192595095028232],
    )
    RigidTransform(
    R=RotationMatrix([
        [-0.02860785327551263, -0.003296811570431546, 0.9995852748837568],
        [0.9989444403502721, -0.03604759095157617, 0.028470621346741887],
        [0.03593877883635521, 0.9993446363592777, 0.004324575782043372],
    ]),
    p=[3.491085973379234, -0.17055544676076428, 0.02474805587614118],
    )
    RigidTransform(
    R=RotationMatrix([
        [0.2961709699882758, -0.90528426481494, 0.3045376108376063],
        [-0.22794287043931316, -0.3766219212121051, -0.8978852801323726],
        [0.927536955787795, 0.19651037718394504, -0.3178975736096978],
    ]),
    p=[2.8765511526501304, 0.17693964339961252, 0.9857514688950346],
    )
    RigidTransform(
    R=RotationMatrix([
        [0.9919829088025355, -0.12579973883149165, -0.012013923322091438],
        [0.12581906456335085, 0.9920528298717531, 0.00086355998618063],
        [0.011809811008816213, -0.0023682173411383074, 0.9999274573240606],
    ]),
    p=[3.0206810055202564, -1.267369089631013, 0.5254621745764841],
    )
    RigidTransform(
    R=RotationMatrix([
        [0.9509593339135489, 0.028371554424339945, 0.30801201298365977],
        [0.3082185991027496, -0.0031123121200210928, -0.9513104691321367],
        [-0.026031527228305623, 0.9995926012195239, -0.011704323019623795],
    ]),
    p=[3.5122090924996545, 0.46251388259754783, 1.0414858134402283],
    )
    RigidTransform(
    R=RotationMatrix([
        [-0.03454746012403037, 0.762590271730196, 0.6459586290633821],
        [0.8360008388942857, -0.33212398037177765, 0.4368023111546655],
        [0.5476395441958413, 0.5551123662146905, -0.626052066930937],
    ]),
    p=[2.4986296879322243, -0.42736355115916835, 0.5116335710895338],
    )
    RigidTransform(
    R=RotationMatrix([
        [0.16027649500713148, -0.8229311666204564, 0.5450648953591886],
        [-0.027016220675734004, -0.55565412156106, -0.8309745008197341],
        [0.9867023710160165, 0.11846008693606791, -0.11129078503843048],
    ]),
    p=[2.502295289906343, 0.57428068613587, 0.02104487445082461],
    )
    RigidTransform(
    R=RotationMatrix([
        [-0.05646665156081254, -0.9984042206860406, -0.0007275835413905294],
        [0.9984033446343212, -0.05646547020304116, -0.0015530929625715147],
        [0.0015095312221725771, -0.0008141198003842642, 0.9999985292611384],
    ]),
    p=[2.9845467537864856, 0.007700122276121421, 0.5257881866040494],
    )}
    """

    # Regex pattern to capture the body index, rotation matrix, and translation vector
    # pattern = re.compile(
    #     r'RigidTransform\(\s*R=RotationMatrix\(\[\s*\[\s*([^]]+)\s*\],\s*\[\s*([^]]+)\s*\],\s*\[\s*([^]]+)\s*\]\s*\]\s*\),\s*p=\[\s*([^\]]+)\s*\]\s*\)'
    # )
    pattern = re.compile(
        r'RigidTransform\(\s*R=RotationMatrix\(\[\s*\[\s*([^]]+)\s*\],\s*\[\s*([^]]+)\s*\],\s*\[\s*([^]]+)\s*\],\s*\]\s*\),\s*p=\[\s*([^\]]+)'
    )

    # Find all matches in the string
    matches = pattern.findall(box_poses_string)
    
    box_poses = []
    
    for match in matches:
        # Extract the rotation matrix rows and translation vector
        r1 = list(map(float, match[0].split(',')))
        r2 = list(map(float, match[1].split(',')))
        r3 = list(map(float, match[2].split(',')))
        p = list(map(float, match[3].split(',')))
        
        # Create the RotationMatrix and RigidTransform objects
        R = RotationMatrix([r1, r2, r3])
        T = RigidTransform(R=R, p=p)
        
        # Append the RigidTransform to the list
        box_poses.append(T)
    
    return box_poses


def get_W_X_eef(plant, plant_context):
    eef_model_idx = plant.GetModelInstanceByName("kuka")  # ModelInstanceIndex
    eef_body_idx = plant.GetBodyIndices(eef_model_idx)[-1]  # BodyIndex
    eef_frame = plant.get_body(eef_body_idx).body_frame()  # RigidBodyFrame
    W_X_eef = plant.CalcRelativeTransform(plant_context, eef_frame, plant.world_frame())
    return W_X_eef


def set_up_scene(station, station_context, plant, plant_context, simulator, randomize_boxes, box_fall_runtime, box_randomization_runtime):
    fast_box_poses = get_fast_box_poses()  # Get pre-computed box poses

    # 'Remove' Top of truck trailer
    trailer_roof_model_idx = plant.GetModelInstanceByName("Truck_Trailer_Roof")  # ModelInstanceIndex
    trailer_roof_body_idx = plant.GetBodyIndices(trailer_roof_model_idx)[0]  # BodyIndex
    if randomize_boxes:
        plant.SetFreeBodyPose(plant_context, plant.get_body(trailer_roof_body_idx), RigidTransform([0,0,100]))

    # Move Robot to start position
    robot_model_idx = plant.GetModelInstanceByName("robot_base")  # ModelInstanceIndex
    robot_body_idx = plant.GetBodyIndices(robot_model_idx)[0]  # BodyIndex
    plant.SetFreeBodyPose(plant_context, plant.get_body(robot_body_idx), robot_pose)
    for joint_idx in plant.GetJointIndices(robot_model_idx):
        robot_joint = plant.get_joint(joint_idx)  # Joint object
        robot_joint.Lock(plant_context)

    # Set poses for all boxes
    # Because of added floating joints between boxes and eef, we must express free body pose relative to eef
    W_X_eef = get_W_X_eef(plant, plant_context)
    
    all_box_positions = []
    for i in range(NUM_BOXES):
        box_model_idx = plant.GetModelInstanceByName(f"Boxes/Box_{i}")  # ModelInstanceIndex
        box_body_idx = plant.GetBodyIndices(box_model_idx)[0]  # BodyIndex

        if randomize_boxes:
            i=0
            while True:
                box_pos_x = np.random.uniform(2.0, 3.5, 1)
                box_pos_y = np.random.uniform(-1.2, 0.7, 1)
                box_pos_z = np.random.uniform(0.5, 5, 1)

                in_collision = False
                for pos in all_box_positions:
                    if abs(box_pos_x - pos[0]) < BOX_DIM and abs(box_pos_y - pos[1]) < BOX_DIM and abs(box_pos_z - pos[2]) < BOX_DIM:
                        in_collision = True
                        break
                
                if not in_collision:   
                    all_box_positions.append((box_pos_x, box_pos_y, box_pos_z))
                    break

                i+=1
                if (i > 100):
                    raise Exception("Could not find box randomization configuration that does not result in collision.")

            plant.SetFreeBodyPose(plant_context, plant.get_body(box_body_idx), W_X_eef @ RigidTransform([box_pos_x[0], box_pos_y[0], box_pos_z[0]]))
        else:
            plant.SetFreeBodyPose(plant_context, plant.get_body(box_body_idx), W_X_eef @ fast_box_poses[i])

    if randomize_boxes:
        simulator.AdvanceTo(box_fall_runtime)

    # Put Top of truck trailer back and lock it
    plant.SetFreeBodyPose(plant_context, plant.get_body(trailer_roof_body_idx), RigidTransform([0,0,0]))
    trailer_roof_joint_idx = plant.GetJointIndices(trailer_roof_model_idx)[0]  # JointIndex object
    trailer_roof_joint = plant.get_joint(trailer_roof_joint_idx)  # Joint object
    trailer_roof_joint.Lock(plant_context)

    if randomize_boxes:
        # Applied external forces on the box to shove them to the back of the truck trailer
        box_forces = []
        zero_box_forces = []
        for i in range(NUM_BOXES):
            force = ExternallyAppliedSpatialForce()
            zero_force = ExternallyAppliedSpatialForce()

            box_model_idx = plant.GetModelInstanceByName(f"Boxes/Box_{i}")  # ModelInstanceIndex
            box_body_idx = plant.GetBodyIndices(box_model_idx)[0]  # BodyIndex

            force.body_index = box_body_idx
            force.p_BoBq_B = [0,0,0]
            force.F_Bq_W = SpatialForce(tau=[0,0,0], f=[1000,0,0])
            box_forces.append(force)

            zero_force.body_index = box_body_idx
            zero_force.p_BoBq_B = [0,0,0]
            zero_force.F_Bq_W = SpatialForce(tau=[0,0,0], f=[0,0,0])
            zero_box_forces.append(zero_force)

        # Apply pushing force to back of truck trailer
        station.GetInputPort("applied_spatial_force").FixValue(station_context, box_forces)
        simulator.AdvanceTo(box_fall_runtime+1.0)

        # Remove pushing force to back of truck trailer
        station.GetInputPort("applied_spatial_force").FixValue(station_context, zero_box_forces)
        simulator.AdvanceTo(box_randomization_runtime)
    else:
        simulator.AdvanceTo(0.001)
        # pass