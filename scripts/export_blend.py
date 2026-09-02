# SPDX-License-Identifier: MIT

from __future__ import annotations

import shutil
import subprocess
import sys
import tomllib
from math import cos, pi, radians
from pathlib import Path

import bpy
from mathutils import Vector


def command_arguments() -> list[str]:
    if "--" not in sys.argv:
        raise SystemExit(
            "expected: blender --background --python export_blend.py -- "
            "INPUT.obj OUTPUT.blend "
            "[ASSEMBLY.png [COMPOUNDS.png [CASE.png [HANDLE.png [MOTION.mp4]]]]]"
        )
    return sys.argv[sys.argv.index("--") + 1 :]


def aim_at(obj: bpy.types.Object, target: tuple[float, float, float]) -> None:
    direction = Vector(target) - obj.location
    obj.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()


def add_area_light(
    name: str,
    location: tuple[float, float, float],
    energy: float,
    size: float,
    target: tuple[float, float, float],
) -> None:
    light_data = bpy.data.lights.new(name=name, type="AREA")
    light_data.energy = energy
    light_data.shape = "DISK"
    light_data.size = size
    light = bpy.data.objects.new(name=name, object_data=light_data)
    bpy.context.scene.collection.objects.link(light)
    light.location = location
    aim_at(light, target)


def bounds_center(obj: bpy.types.Object) -> Vector:
    corners = [obj.matrix_world @ Vector(corner) for corner in obj.bound_box]
    lower = Vector(tuple(min(point[index] for point in corners) for index in range(3)))
    upper = Vector(tuple(max(point[index] for point in corners) for index in range(3)))
    return (lower + upper) * 0.5


def set_origin_at(obj: bpy.types.Object, pivot: Vector) -> None:
    bpy.ops.object.select_all(action="DESELECT")
    bpy.context.scene.cursor.location = pivot
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.origin_set(type="ORIGIN_CURSOR", center="MEDIAN")
    obj.select_set(False)


def configure_motion(
    meshes: list[bpy.types.Object], metadata_path: Path
) -> dict[str, object]:
    if not metadata_path.is_file():
        raise SystemExit(f"animation metadata does not exist: {metadata_path}")
    with metadata_path.open("rb") as source:
        metadata = tomllib.load(source)
    if metadata.get("version") != 1:
        raise SystemExit(f"unsupported animation metadata version: {metadata.get('version')}")

    timeline = metadata["timeline"]
    motion = metadata["motion"]
    frame_start = int(timeline["frame_start"])
    frame_mid = int(timeline["frame_mid"])
    frame_end = int(timeline["frame_end"])
    fps = int(timeline["fps"])
    samples = int(timeline["samples"])
    if not (frame_start < frame_mid < frame_end and fps > 0 and samples >= 3):
        raise SystemExit("invalid animation timeline")

    by_name = {obj.name: obj for obj in meshes}
    required = {
        "handle-shaft",
        "handle-spur",
        "handle-crank",
        "handle-knob",
        "reduction-d-large-plus-small",
        "driven-b-output-plus-pinion",
        "driven-c-output-plus-pinion",
        "idler-20t",
        "rack",
    }
    missing = sorted(required - by_name.keys())
    if missing:
        raise SystemExit(f"animation objects are missing: {', '.join(missing)}")

    handle_center = bounds_center(by_name["handle-shaft"])
    animated_rotations = {
        "handle-shaft": float(motion["handle_delta_deg"]),
        "handle-spur": float(motion["handle_delta_deg"]),
        "handle-crank": float(motion["handle_delta_deg"]),
        "reduction-d-large-plus-small": float(motion["reduction_delta_deg"]),
        "driven-b-output-plus-pinion": float(motion["driven_delta_deg"]),
        "driven-c-output-plus-pinion": float(motion["driven_delta_deg"]),
        "idler-20t": float(motion["idler_delta_deg"]),
    }
    for name in animated_rotations:
        obj = by_name[name]
        center = bounds_center(obj)
        pivot = (
            Vector((handle_center.x, handle_center.y, center.z))
            if name == "handle-crank"
            else center
        )
        set_origin_at(obj, pivot)
        obj.rotation_mode = "XYZ"

    crank = by_name["handle-crank"]
    knob = by_name["handle-knob"]
    knob_world = knob.matrix_world.copy()
    knob.parent = crank
    knob.matrix_world = knob_world

    initial_rotations = {
        name: by_name[name].rotation_euler.z for name in animated_rotations
    }
    rack = by_name["rack"]
    initial_rack_x = rack.location.x
    rack_delta = float(motion["rack_delta_x_mm"])
    for index in range(samples):
        phase = index / (samples - 1)
        frame = round(frame_start + phase * (frame_end - frame_start))
        travel_fraction = (1.0 - cos(2.0 * pi * phase)) * 0.5
        for name, delta_deg in animated_rotations.items():
            obj = by_name[name]
            obj.rotation_euler.z = initial_rotations[name] + radians(delta_deg * travel_fraction)
            obj.keyframe_insert(data_path="rotation_euler", index=2, frame=frame)
        rack.location.x = initial_rack_x + rack_delta * travel_fraction
        rack.keyframe_insert(data_path="location", index=0, frame=frame)

    scene = bpy.context.scene
    scene.frame_start = frame_start
    scene.frame_end = frame_end
    scene.render.fps = fps
    scene["animation_description"] = "handle drives D, B/C, rack, and passive A out and back"
    scene["animation_handle_delta_deg"] = float(motion["handle_delta_deg"])
    scene["animation_rack_delta_x_mm"] = rack_delta
    scene.timeline_markers.new("CENTER", frame=frame_start)
    scene.timeline_markers.new("EXTENDED", frame=frame_mid)
    scene.timeline_markers.new("LOOP", frame=frame_end)
    scene.frame_set(frame_start)
    bpy.context.view_layer.update()
    return metadata


def render_motion(scene: bpy.types.Scene, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    frames_directory = destination.parent / "prototype-motion-frames"
    frames_directory.mkdir(parents=True, exist_ok=True)
    for old_frame in frames_directory.glob("frame_*.png"):
        old_frame.unlink()
    scene.render.resolution_x = 720
    scene.render.resolution_y = 540
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    source_frames = list(range(scene.frame_start, scene.frame_end + 1, 2))
    if source_frames[-1] != scene.frame_end:
        source_frames.append(scene.frame_end)
    for output_index, source_frame in enumerate(source_frames):
        scene.frame_set(source_frame)
        scene.render.filepath = str(frames_directory / f"frame_{output_index:04d}.png")
        bpy.ops.render.render(write_still=True)
        print(f"animation frame {output_index + 1}/{len(source_frames)}")

    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise SystemExit("FFmpeg is required to encode prototype-motion.mp4")
    subprocess.run(
        [
            ffmpeg,
            "-y",
            "-framerate",
            str(max(scene.render.fps // 2, 1)),
            "-i",
            str(frames_directory / "frame_%04d.png"),
            "-c:v",
            "libx264",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(destination),
        ],
        check=True,
    )
    scene.frame_set(scene.frame_start)
    print(f"rendered {destination}")


def main() -> None:
    arguments = command_arguments()
    if len(arguments) not in (2, 3, 4, 5, 6, 7):
        raise SystemExit(
            "expected INPUT.obj OUTPUT.blend "
            "[ASSEMBLY.png [COMPOUNDS.png [CASE.png [HANDLE.png [MOTION.mp4]]]]]"
        )

    obj_path = Path(arguments[0]).resolve()
    blend_path = Path(arguments[1]).resolve()
    render_path = Path(arguments[2]).resolve() if len(arguments) >= 3 else None
    compounds_path = Path(arguments[3]).resolve() if len(arguments) >= 4 else None
    case_path = Path(arguments[4]).resolve() if len(arguments) >= 5 else None
    handle_path = Path(arguments[5]).resolve() if len(arguments) >= 6 else None
    motion_path = Path(arguments[6]).resolve() if len(arguments) == 7 else None
    if not obj_path.is_file():
        raise SystemExit(f"OBJ does not exist: {obj_path}")

    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.obj_import(
        filepath=str(obj_path),
        forward_axis="Y",
        up_axis="Z",
        use_split_objects=True,
        validate_meshes=True,
    )

    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.length_unit = "MILLIMETERS"
    scene.unit_settings.scale_length = 0.001
    scene.render.engine = "BLENDER_WORKBENCH"
    scene.render.resolution_x = 1600
    scene.render.resolution_y = 1100
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.film_transparent = False
    if scene.world is None:
        scene.world = bpy.data.worlds.new("Assembly World")
    scene.world.color = (0.055, 0.065, 0.085)

    for material in bpy.data.materials:
        material.diffuse_color[3] = max(material.diffuse_color[3], 0.32)
        material.roughness = 0.62

    scene.display.shading.light = "STUDIO"
    scene.display.shading.color_type = "MATERIAL"
    scene.display.shading.show_shadows = True
    scene.display.shading.show_cavity = True
    scene.display.shading.cavity_type = "WORLD"

    target = (0.0, -12.0, -4.0)
    camera_data = bpy.data.cameras.new("Assembly Camera")
    camera_data.lens = 52.0
    camera_data.clip_start = 0.5
    camera_data.clip_end = 2500.0
    camera = bpy.data.objects.new("Assembly Camera", camera_data)
    scene.collection.objects.link(camera)
    camera.location = (245.0, -335.0, 225.0)
    aim_at(camera, target)
    scene.camera = camera

    add_area_light("Key Light", (-90.0, -210.0, 330.0), 1650.0, 210.0, target)
    add_area_light("Fill Light", (260.0, 80.0, 180.0), 1100.0, 170.0, target)
    add_area_light("Rim Light", (-250.0, 150.0, 90.0), 850.0, 130.0, target)

    imported_meshes = [obj for obj in scene.objects if obj.type == "MESH"]
    if not imported_meshes:
        raise SystemExit("OBJ import produced no mesh objects")
    for obj in imported_meshes:
        obj.select_set(False)
        if obj.name == "top-plate":
            obj.hide_render = True
            obj.display_type = "WIRE"

    motion_metadata_path = obj_path.with_name("prototype-animation.toml")
    motion_metadata = configure_motion(imported_meshes, motion_metadata_path)

    blend_path.parent.mkdir(parents=True, exist_ok=True)
    if render_path is not None:
        render_path.parent.mkdir(parents=True, exist_ok=True)
        scene.render.filepath = str(render_path)
        bpy.ops.render.render(write_still=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(blend_path), check_existing=False)
    if motion_path is not None:
        render_motion(scene, motion_path)

    if compounds_path is not None:
        compound_names = {
            "reduction-d-large-plus-small",
            "driven-b-output-plus-pinion",
            "driven-c-output-plus-pinion",
        }
        compounds = [obj for obj in imported_meshes if obj.name in compound_names]
        if len(compounds) != len(compound_names):
            found = ", ".join(sorted(obj.name for obj in compounds))
            raise SystemExit(f"expected three compound gears, found: {found}")
        for obj in imported_meshes:
            obj.hide_render = obj not in compounds

        corners = [obj.matrix_world @ Vector(corner) for obj in compounds for corner in obj.bound_box]
        lower = Vector(tuple(min(point[index] for point in corners) for index in range(3)))
        upper = Vector(tuple(max(point[index] for point in corners) for index in range(3)))
        center = (lower + upper) * 0.5
        camera.data.type = "ORTHO"
        camera.data.ortho_scale = max((upper.x - lower.x) * 0.72, (upper.z - lower.z) * 1.35)
        camera.location = (center.x, center.y - 300.0, center.z)
        aim_at(camera, tuple(center))
        scene.render.resolution_x = 1600
        scene.render.resolution_y = 700
        compounds_path.parent.mkdir(parents=True, exist_ok=True)
        scene.render.filepath = str(compounds_path)
        bpy.ops.render.render(write_still=True)

    if case_path is not None:
        plates = [obj for obj in imported_meshes if obj.name in {"top-plate", "bottom-plate"}]
        if len(plates) != 2:
            found = ", ".join(sorted(obj.name for obj in plates))
            raise SystemExit(f"expected top and bottom plates, found: {found}")
        for obj in imported_meshes:
            obj.hide_render = obj not in plates
        top_plate = next(obj for obj in plates if obj.name == "top-plate")
        bottom_plate = next(obj for obj in plates if obj.name == "bottom-plate")
        top_plate.hide_render = False
        top_plate.display_type = "TEXTURED"
        for plate in plates:
            bpy.context.view_layer.objects.active = plate
            plate.select_set(True)
            bpy.ops.object.origin_set(type="ORIGIN_GEOMETRY", center="BOUNDS")
            plate.select_set(False)

        top_plate.rotation_euler.x = pi
        top_plate.location.x += 150.0
        bottom_corners = [
            bottom_plate.matrix_world @ Vector(corner) for corner in bottom_plate.bound_box
        ]
        top_corners = [top_plate.matrix_world @ Vector(corner) for corner in top_plate.bound_box]
        bottom_low_z = min(point.z for point in bottom_corners)
        top_center_z = (min(point.z for point in top_corners) + max(point.z for point in top_corners)) * 0.5
        top_plate.location.z += bottom_low_z + 2.0 - top_center_z
        bpy.context.view_layer.update()

        corners = [obj.matrix_world @ Vector(corner) for obj in plates for corner in obj.bound_box]
        lower = Vector(tuple(min(point[index] for point in corners) for index in range(3)))
        upper = Vector(tuple(max(point[index] for point in corners) for index in range(3)))
        center = (lower + upper) * 0.5
        camera.data.type = "ORTHO"
        camera.data.ortho_scale = 360.0
        center.x = 75.0
        camera.location = (center.x, center.y - 260.0, center.z + 240.0)
        aim_at(camera, tuple(center))
        scene.render.resolution_x = 1600
        scene.render.resolution_y = 900
        case_path.parent.mkdir(parents=True, exist_ok=True)
        scene.render.filepath = str(case_path)
        bpy.ops.render.render(write_still=True)

    if handle_path is not None:
        handle_names = {"handle-spur", "handle-shaft"}
        handles = [obj for obj in imported_meshes if obj.name in handle_names]
        if {obj.name for obj in handles} != handle_names:
            found = ", ".join(sorted(obj.name for obj in handles))
            raise SystemExit(f"expected handle-spur and handle-shaft, found: {found}")
        for obj in imported_meshes:
            obj.hide_render = obj not in handles
        handle_spur = next(obj for obj in handles if obj.name == "handle-spur")
        handle_shaft = next(obj for obj in handles if obj.name == "handle-shaft")
        handle_spur.location.x -= 20.0
        handle_shaft.location.x += 20.0
        bpy.context.view_layer.update()
        corners = [
            obj.matrix_world @ Vector(corner)
            for obj in handles
            for corner in obj.bound_box
        ]
        lower = Vector(tuple(min(point[index] for point in corners) for index in range(3)))
        upper = Vector(tuple(max(point[index] for point in corners) for index in range(3)))
        center = (lower + upper) * 0.5
        camera.data.type = "ORTHO"
        camera.data.ortho_scale = max((upper.x - lower.x) * 1.25, (upper.z - lower.z) * 1.25)
        camera.location = (center.x - 100.0, center.y - 180.0, center.z + 100.0)
        aim_at(camera, tuple(center))
        scene.render.resolution_x = 1000
        scene.render.resolution_y = 1000
        handle_path.parent.mkdir(parents=True, exist_ok=True)
        scene.render.filepath = str(handle_path)
        bpy.ops.render.render(write_still=True)

    print(
        f"saved {blend_path} with {len(imported_meshes)} mesh objects"
        + f", animation metadata {motion_metadata_path}"
        + f", rack travel {motion_metadata['motion']['rack_delta_x_mm']:.6f} mm"
        + (f", rendered {render_path}" if render_path is not None else "")
        + (f", rendered {compounds_path}" if compounds_path is not None else "")
        + (f", rendered {case_path}" if case_path is not None else "")
        + (f", and rendered {handle_path}" if handle_path is not None else "")
        + (f", rendered {motion_path}" if motion_path is not None else "")
    )


if __name__ == "__main__":
    main()
