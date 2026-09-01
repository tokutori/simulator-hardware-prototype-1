# SPDX-License-Identifier: MIT

from __future__ import annotations

import sys
from math import pi
from pathlib import Path

import bpy
from mathutils import Vector


def command_arguments() -> list[str]:
    if "--" not in sys.argv:
        raise SystemExit(
            "expected: blender --background --python export_blend.py -- "
            "INPUT.obj OUTPUT.blend [ASSEMBLY.png [COMPOUNDS.png [CASE.png]]]"
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


def main() -> None:
    arguments = command_arguments()
    if len(arguments) not in (2, 3, 4, 5):
        raise SystemExit(
            "expected INPUT.obj OUTPUT.blend [ASSEMBLY.png [COMPOUNDS.png [CASE.png]]]"
        )

    obj_path = Path(arguments[0]).resolve()
    blend_path = Path(arguments[1]).resolve()
    render_path = Path(arguments[2]).resolve() if len(arguments) >= 3 else None
    compounds_path = Path(arguments[3]).resolve() if len(arguments) >= 4 else None
    case_path = Path(arguments[4]).resolve() if len(arguments) == 5 else None
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

    blend_path.parent.mkdir(parents=True, exist_ok=True)
    if render_path is not None:
        render_path.parent.mkdir(parents=True, exist_ok=True)
        scene.render.filepath = str(render_path)
        bpy.ops.render.render(write_still=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(blend_path), check_existing=False)

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

    print(
        f"saved {blend_path} with {len(imported_meshes)} mesh objects"
        + (f", rendered {render_path}" if render_path is not None else "")
        + (f", rendered {compounds_path}" if compounds_path is not None else "")
        + (f", and rendered {case_path}" if case_path is not None else "")
    )


if __name__ == "__main__":
    main()
