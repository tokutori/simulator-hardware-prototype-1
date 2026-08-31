# SPDX-License-Identifier: MIT
"""Import the generated glTF into Blender and render inspection artifacts.

This is a platform adapter. The parametric model and its motion remain authored by
the Rust core/exporter; Blender only provides inspection rendering and a .blend file.
"""

from __future__ import annotations

import argparse
import math
import shutil
import subprocess
import sys
from pathlib import Path

import bpy
from mathutils import Matrix, Vector


def parse_args() -> argparse.Namespace:
    arguments = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--gltf", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--animation-frames", type=int, default=72)
    parser.add_argument("--animation-fps", type=int, default=12)
    parser.add_argument("--still-width", type=int, default=1200)
    parser.add_argument("--still-height", type=int, default=900)
    parser.add_argument("--video-width", type=int, default=720)
    parser.add_argument("--video-height", type=int, default=540)
    parser.add_argument("--skip-animation", action="store_true")
    return parser.parse_args(arguments)


def clear_scene() -> None:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for datablocks in (bpy.data.meshes, bpy.data.curves, bpy.data.materials, bpy.data.cameras, bpy.data.lights):
        for datablock in list(datablocks):
            if datablock.users == 0:
                datablocks.remove(datablock)


def mesh_bounds(objects: list[bpy.types.Object]) -> tuple[Vector, Vector]:
    corners = [obj.matrix_world @ Vector(corner) for obj in objects for corner in obj.bound_box]
    return (
        Vector(tuple(min(point[axis] for point in corners) for axis in range(3))),
        Vector(tuple(max(point[axis] for point in corners) for axis in range(3))),
    )


def look_at(camera: bpy.types.Object, target: Vector, image_up: Vector = Vector((0.0, 0.0, 1.0))) -> None:
    forward = (target - camera.location).normalized()
    local_z = -forward
    local_y = (image_up - image_up.dot(local_z) * local_z).normalized()
    local_x = local_y.cross(local_z).normalized()
    rotation = Matrix((local_x, local_y, local_z)).transposed().to_quaternion()
    camera.rotation_euler = rotation.to_euler()


def add_camera(name: str) -> bpy.types.Object:
    camera_data = bpy.data.cameras.new(name)
    camera_data.type = "ORTHO"
    camera = bpy.data.objects.new(name, camera_data)
    bpy.context.scene.collection.objects.link(camera)
    bpy.context.scene.camera = camera
    return camera


def add_area_light(name: str, location: Vector, energy: float, size: float, target: Vector) -> None:
    light_data = bpy.data.lights.new(name=name, type="AREA")
    light_data.energy = energy
    light_data.shape = "DISK"
    light_data.size = size
    light = bpy.data.objects.new(name=name, object_data=light_data)
    bpy.context.scene.collection.objects.link(light)
    light.location = location
    light.rotation_euler = (target - location).to_track_quat("-Z", "Y").to_euler()


def configure_scene(center: Vector, size: Vector) -> bpy.types.Object:
    scene = bpy.context.scene
    scene.render.engine = "BLENDER_EEVEE"
    scene.render.image_settings.file_format = "PNG"
    scene.render.image_settings.color_mode = "RGBA"
    scene.render.film_transparent = False
    scene.render.resolution_percentage = 100
    scene.render.image_settings.color_depth = "8"
    scene.view_settings.look = "AgX - Medium High Contrast"

    world = bpy.data.worlds.new("Inspection World") if scene.world is None else scene.world
    scene.world = world
    world.use_nodes = True
    background = world.node_tree.nodes.get("Background")
    background.inputs["Color"].default_value = (0.035, 0.045, 0.060, 1.0)
    background.inputs["Strength"].default_value = 0.52

    span = max(size)
    # glTF uses metres. Keep lighting useful for this approximately 0.3 m model.
    light_scale = max(span * span * 7.0, 0.45)
    add_area_light(
        "Key Light",
        center + Vector((span * 1.15, -span * 1.10, span * 1.40)),
        light_scale,
        span * 0.70,
        center,
    )
    add_area_light(
        "Fill Light",
        center + Vector((-span * 0.90, -span * 0.25, span * 0.65)),
        light_scale * 0.5,
        span * 0.55,
        center,
    )
    add_area_light(
        "Rim Light",
        center + Vector((0.25 * span, span * 0.95, span * 0.85)),
        light_scale * 0.65,
        span * 0.50,
        center,
    )

    camera = add_camera("Inspection Camera")
    camera.data.lens = 52.0
    camera.data.dof.use_dof = False
    camera.data.clip_start = max(span * 0.0001, 0.01)
    camera.data.clip_end = span * 10.0
    return camera


def render_view(
    camera: bpy.types.Object,
    destination: Path,
    center: Vector,
    direction: Vector,
    image_up: Vector,
    span: float,
    width: int,
    height: int,
) -> None:
    scene = bpy.context.scene
    camera.location = center + direction.normalized() * span * 2.2
    look_at(camera, center, image_up)
    camera.data.ortho_scale = span * 1.16
    scene.render.resolution_x = width
    scene.render.resolution_y = height
    scene.render.filepath = str(destination)
    scene.render.image_settings.file_format = "PNG"
    bpy.ops.render.render(write_still=True)
    print(f"rendered {destination}")


def configure_animation_range(scene: bpy.types.Scene) -> None:
    actions = list(bpy.data.actions)
    if not actions:
        scene.frame_start = 1
        scene.frame_end = 1
        return
    imported_start = min(action.frame_range[0] for action in actions)
    imported_end = max(action.frame_range[1] for action in actions)
    scene.frame_start = max(1, math.floor(imported_start))
    scene.frame_end = math.ceil(imported_end)
    print(
        "using imported animation timeline: "
        f"frames {scene.frame_start}..{scene.frame_end}"
    )


def render_animation(
    output: Path,
    width: int,
    height: int,
    frame_count: int,
    fps: int,
) -> None:
    scene = bpy.context.scene
    scene.render.resolution_x = width
    scene.render.resolution_y = height
    scene.render.resolution_percentage = 100
    frames_directory = output.parent / "frames"
    frames_directory.mkdir(parents=True, exist_ok=True)
    for old_frame in frames_directory.glob("frame_*.png"):
        old_frame.unlink()
    imported_start = scene.frame_start
    imported_end = scene.frame_end
    for output_index in range(frame_count):
        alpha = output_index / max(frame_count - 1, 1)
        imported_frame = round(imported_start + alpha * (imported_end - imported_start))
        scene.frame_set(imported_frame)
        scene.render.filepath = str(frames_directory / f"frame_{output_index:04d}.png")
        scene.render.image_settings.file_format = "PNG"
        bpy.ops.render.render(write_still=True)
        print(f"animation frame {output_index + 1}/{frame_count}")

    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise RuntimeError("FFmpeg is required to encode gimbal-motion.mp4")
    subprocess.run(
        [
            ffmpeg,
            "-y",
            "-framerate",
            str(fps),
            "-i",
            str(frames_directory / "frame_%04d.png"),
            "-c:v",
            "libx264",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(output),
        ],
        check=True,
    )
    print(f"rendered {output}")


def main() -> None:
    args = parse_args()
    gltf = args.gltf.resolve()
    output = args.output.resolve()
    preview = output / "preview"
    model = output / "model"
    preview.mkdir(parents=True, exist_ok=True)
    model.mkdir(parents=True, exist_ok=True)

    clear_scene()
    bpy.ops.import_scene.gltf(filepath=str(gltf), import_shading="NORMALS")
    meshes = [obj for obj in bpy.context.scene.objects if obj.type == "MESH"]
    if not meshes:
        raise RuntimeError(f"glTF contains no mesh objects: {gltf}")

    lower, upper = mesh_bounds(meshes)
    center = (lower + upper) * 0.5
    size = upper - lower
    span = max(size)
    camera = configure_scene(center, size)
    scene = bpy.context.scene
    configure_animation_range(scene)
    scene.frame_set(scene.frame_start)
    detail_object = next(
        (obj for obj in meshes if obj.name.startswith("pitch_drive_right_front_1")),
        None,
    )
    detail_target = (
        detail_object.matrix_world.translation.copy()
        if detail_object is not None
        else Vector((upper.x - size.x * 0.10, center.y, center.z))
    )

    # After glTF import Blender axes match the core: +X nose/front, +Y right,
    # +Z up. Names include the viewing direction to avoid ambiguous "side".
    views = (
        ("isometric", Vector((-1.25, -1.60, 1.05)), Vector((0.0, 0.0, 1.0)), center, span),
        ("top-z", Vector((0.0, 0.0, 1.0)), Vector((1.0, 0.0, 0.0)), center, span),
        ("left-side-minus-y", Vector((0.0, -1.0, 0.0)), Vector((0.0, 0.0, 1.0)), center, span),
        ("front-plus-x", Vector((1.0, 0.0, 0.0)), Vector((0.0, 0.0, 1.0)), center, span),
        (
            "drive-unit-detail",
            Vector((1.0, -1.25, 0.70)),
            Vector((0.0, 0.0, 1.0)),
            detail_target,
            span * 0.30,
        ),
    )
    for name, direction, image_up, target, view_span in views:
        render_view(
            camera,
            preview / f"{name}.png",
            target,
            direction,
            image_up,
            view_span,
            args.still_width,
            args.still_height,
        )

    # The animation uses the inspection view and the motion already embedded in glTF.
    camera.location = center + Vector((-1.25, -1.60, 1.05)).normalized() * span * 2.2
    look_at(camera, center)
    camera.data.ortho_scale = span * 1.16
    bpy.ops.wm.save_as_mainfile(filepath=str(model / "gimbal-prototype.blend"), compress=True)
    if not args.skip_animation:
        render_animation(
            preview / "gimbal-motion.mp4",
            args.video_width,
            args.video_height,
            max(args.animation_frames, 2),
            max(args.animation_fps, 1),
        )


if __name__ == "__main__":
    main()
