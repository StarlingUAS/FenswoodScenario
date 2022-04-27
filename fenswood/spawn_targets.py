import subprocess
import os
import json
import argparse
import xml.etree.cElementTree as ET
import random
import math

def default_environ(key, default, prefix="SPAWN_TARGET"):
    _key = f"{prefix}_{key}"
    return (
        {'default': os.environ.get(_key) if os.environ.get(_key) else default }
    )

def parse_arguments():
    parser = argparse.ArgumentParser(description="Spawn Fenswood Targets")
    parser.add_argument('-name', type=str, default='area_of_interest')
    parser.add_argument('-num_landing_targets', type=int, **default_environ("NUM_TARGETS", 2))
    parser.add_argument('-num_hotspots', type=int, **default_environ("NUM_HOTSPOTS", 5))
    parser.add_argument('-generation_theta_min', type=float, **default_environ("GEN_TARGET_LOC_ANGLE_MIN", -math.pi/2))
    parser.add_argument('-generation_theta_max', type=float, **default_environ("GEN_TARGET_LOC_ANGLE_MAX", math.pi/2))
    parser.add_argument('-generation_target_theta_variance', type=float, **default_environ("GEN_HOTSPOT_ANGLE_VARIANCE", 0.3))
    parser.add_argument('-generation_hotspot_radius_variance', type=float, **default_environ("GEN_HOTSPOT_RADIUS_VARIANCE", 0.5))
    parser.add_argument('-generation_min_theta_between_targets', type=float, **default_environ("GEN_BETWEEN_TARGET_LOC_ANGLE_MIN", math.pi/3.0))
    parser.add_argument('-generation_min_dist_between_hotspots', type=float, **default_environ("GEN_BETWEEN_TARGET_LOC_ANGLE_MAX", 4.0))
    parser.add_argument('-annulus_x_relative_to_takeoff', type=float, **default_environ("ANNULUS_LOC_X", -195))
    parser.add_argument('-annulus_y_relative_to_takeoff', type=float, **default_environ("ANNULUS_LOC_Y", -163))
    parser.add_argument('-annulus_z_relative_to_takeoff', type=float, **default_environ("ANNULUS_LOC_Z", 0.1))
    parser.add_argument('-annulus_radius', type=float, **default_environ("ANNULUS_RADIUS", 40))
    parser.add_argument('-hotspot_radius', type=float, **default_environ("HOTSPOT_RADIUS", 3))
    parser.add_argument('-target_length', type=float, **default_environ("TARGET_LENGTH", 20))
    parser.add_argument('-target_width', type=float, **default_environ("TARGET_WIDTH", 5))
    parser.add_argument('-random_seed', type=int, **default_environ("RANDOM_SEED", argparse.SUPPRESS))
    parser.add_argument('-locations_from_file', type=str, **default_environ("FILE_PATH", ''))
    parser.add_argument('-temporary_sdf_location', type=str, **default_environ("TEMP_SDF_PATH", "/tmp/targets.sdf"))

    return parser.parse_args()

def generate_volcano(root, name, radius, z = 0.1, colour="Gazebo/Orange"):
    link = ET.SubElement(root, "link", name=f"{name}")
    ET.SubElement(link, "pose").text = f"0 0 {z} 0 0 0"

    # Visual
    visual = ET.SubElement(link, "visual", name="visual")
    ET.SubElement(visual, "pose").text = "0 0 0 0 0"
    vis_geometry = ET.SubElement(visual, "geometry")
    vis_cylinder = ET.SubElement(vis_geometry, "sphere")
    ET.SubElement(vis_cylinder, "radius").text = str(radius)

    vis_material = ET.SubElement(visual, "material")
    vis_material_script = ET.SubElement(vis_material, "script")
    ET.SubElement(vis_material_script, "name").text = colour
    ET.SubElement(vis_material_script, "uri").text = "file://media/materials/scripts/gazebo.material"

    # Collision
    collision = ET.SubElement(link, "collision", name="collision")
    col_geometry = ET.SubElement(collision, "geometry")
    col_cylinder = ET.SubElement(col_geometry, "sphere")
    ET.SubElement(col_cylinder, "radius").text = str(radius)


def generate_hotspot_link(root, name, radius, pos, z = 0.1, colour="Gazebo/Red", model_height=0.2):
    link = ET.SubElement(root, "link", name=f"hotspot_{name}")
    ET.SubElement(link, "pose").text = f"{pos[0]} {pos[1]} {z} 0 0 0"

    # Visual
    visual = ET.SubElement(link, "visual", name="visual")
    ET.SubElement(visual, "pose").text = "0 0 0 0 0"
    vis_geometry = ET.SubElement(visual, "geometry")
    vis_cylinder = ET.SubElement(vis_geometry, "cylinder")
    ET.SubElement(vis_cylinder, "radius").text = str(radius)
    ET.SubElement(vis_cylinder, "length").text = str(model_height)

    vis_material = ET.SubElement(visual, "material")
    vis_material_script = ET.SubElement(vis_material, "script")
    ET.SubElement(vis_material_script, "name").text = colour
    ET.SubElement(vis_material_script, "uri").text = "file://media/materials/scripts/gazebo.material"

    # Collision
    collision = ET.SubElement(link, "collision", name="collision")
    col_geometry = ET.SubElement(collision, "geometry")
    col_cylinder = ET.SubElement(col_geometry, "cylinder")
    ET.SubElement(col_cylinder, "radius").text = str(radius)
    ET.SubElement(col_cylinder, "length").text = str(model_height)

def generate_target_link(root, name, height, width, pos, yaw, z = 0.1, colour="Gazebo/Yellow", model_height=0.15):
    link = ET.SubElement(root, "link", name=f"target_{name}")
    ET.SubElement(link, "pose").text = f"{pos[0]} {pos[1]} {z} 0 0 {yaw}"

    # Visual
    visual = ET.SubElement(link, "visual", name="visual")
    ET.SubElement(visual, "pose").text = "0 0 0 0 0"
    vis_geometry = ET.SubElement(visual, "geometry")
    vis_box = ET.SubElement(vis_geometry, "box")
    ET.SubElement(vis_box, "size").text = f"{height} {width} {model_height}"

    vis_material = ET.SubElement(visual, "material")
    vis_material_script = ET.SubElement(vis_material, "script")
    ET.SubElement(vis_material_script, "name").text = colour
    ET.SubElement(vis_material_script, "uri").text = "file://media/materials/scripts/gazebo.material"

    # Collision
    collision = ET.SubElement(link, "collision", name="collision")
    col_geometry = ET.SubElement(collision, "geometry")
    col_box = ET.SubElement(col_geometry, "box")
    ET.SubElement(col_box, "size").text = f"{height} {width} {model_height}"

def spawn_entity(xml_file_location, location, target_name="target", spawn_timeout=30.0):
    command = ["ros2", "run", "gazebo_ros", "spawn_entity.py",
                "-entity", str(target_name),
                "-spawn_service_timeout", str(spawn_timeout),
                "-x", str(location[0]), "-y", str(location[1]), "-z", str(location[2]),
                "-file", xml_file_location
            ]
    ret = subprocess.run(command)
    ret.check_returncode() # If ret.returncode is zero, raise a CalledProcessError

def generate_target_hotstpot_from_file(filename):
    with open(filename, 'r') as file:
        data = json.load(file)
        if 'hotspots' in data and 'targets' in data:
            return data['targets'], data['hotspots']

def generate_random_target_hotspot_locations(nt, nh, radius, hotspot_theta_variance, hotspot_radius_variance, min_theta_between_targets, min_dist_between_hotspots, theta_range):
    # Generate target locs on radius
    random_thetas = []
    while len(random_thetas) != nt:
        theta = random.uniform(theta_range[0], theta_range[1])

        min_theta = 100000
        for rt in random_thetas:
            dist = abs(rt - theta)
            if dist < min_theta:
                min_theta = dist

        if min_theta > min_theta_between_targets:
            random_thetas.append(theta)

    locs = [ (radius*math.cos(theta), radius*math.sin(theta), theta) for theta in random_thetas]

    # Generate hotspot locations
    # Random number of hotspots assigned to each target location
    # Hotspot locations sampled on gaussian based on target location
    hotspot_locs = []
    num_hs = [math.floor(nh/nt) for _ in range(nt-1)] + [nh - math.floor(nh/nt)]
    print(f"Assigning {num_hs} hotspots to each target")
    for i, ((x, y, t), num_h) in enumerate(zip(locs, num_hs)):
        # Generate points which aren't too close together
        for i in range(num_h):
            loc = (0, 0, 0)
            while len(hotspot_locs) < sum(num_hs[:i]):
                tloc = random.gauss(t, hotspot_theta_variance)
                rloc = random.gauss(radius, hotspot_radius_variance)
                loc = (rloc * math.cos(tloc), rloc * math.sin(tloc), tloc)

                # Check if hotspots are too close together
                min_dist = 1000000000
                for hloc in hotspot_locs:
                    dist = math.sqrt((loc[0] - hloc[0])**2 + (loc[1] - hloc[1])**2)
                    if dist < min_dist:
                        min_dist = dist

                # If min distance is greater than min, then append
                if min_dist > min_dist_between_hotspots:
                    hotspot_locs.append(loc)

    return locs, hotspot_locs


def main():
    args = parse_arguments()

    # Set random seed
    if "random_seed" in args:
        print(f"Random seed given: {args.random_seed}")
        random.seed(args.random_seed)

    # Create name
    randname = random.randrange(5000)
    target_name = f"{args.name}_{randname}"
    print(f"Generating {target_name} ")

    # Generate locations:
    if args.locations_from_file != '':
        print(f"Using File: {args.locations_from_file}")
        target_locs, hotspot_locs = generate_target_hotstpot_from_file(args.locations_from_file)
    else:
        print("Using Random Generation")
        target_locs, hotspot_locs = generate_random_target_hotspot_locations(
            args.num_landing_targets, args.num_hotspots,
            args.annulus_radius,
            args.generation_target_theta_variance,
            args.generation_hotspot_radius_variance,
            args.generation_min_theta_between_targets,
            args.generation_min_dist_between_hotspots,
            (args.generation_theta_min, args.generation_theta_max)
        )
    print(f"Generate {len(target_locs)} targets and {len(hotspot_locs)} hotspots")
    print(f"Targets are here: {target_locs}")
    print(f"Hotspots are here: {hotspot_locs}")

    # Generate SDF XML base
    sdf = ET.Element("sdf", version="1.7")
    model = ET.SubElement(sdf, "model", name=target_name)
    ET.SubElement(model, "static").text = "1"

    # generate_volcano(model, 'volcano', radius=args.annulus_radius*0.6)

    # Generate Targets XML
    for i, (x, y, yaw) in enumerate(target_locs):
        generate_target_link(model, str(i), args.target_width, args.target_length, pos=(x, y), yaw=yaw)

    # Generate Hotspots XML
    for i, (x, y, yaw) in enumerate(hotspot_locs):
        generate_hotspot_link(model, str(i), radius = args.hotspot_radius, pos = (x, y))


    # Write Tree to temporary location
    tree = ET.ElementTree(sdf)
    tree.write(args.temporary_sdf_location, encoding="UTF-8", xml_declaration=True)

    # Get target position from arguments
    target_pos = (
        args.annulus_x_relative_to_takeoff,
        args.annulus_y_relative_to_takeoff,
        args.annulus_z_relative_to_takeoff
    )

    # Spawn the targets and hotspots into the gazebo world
    spawn_entity(args.temporary_sdf_location, target_pos, target_name=target_name)


if __name__=="__main__":
    main()
