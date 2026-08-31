# SPDX-License-Identifier: MIT
"""Small platform adapter for deterministic, dependency-light preview renders."""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


@dataclass(frozen=True)
class Mesh:
    vertices: np.ndarray
    triangles: np.ndarray
    triangle_materials: np.ndarray
    colors: np.ndarray


@dataclass(frozen=True)
class View:
    name: str
    camera: tuple[float, float, float]
    target: tuple[float, float, float]
    up: tuple[float, float, float]
    span: float | None = None


VIEWS = (
    View("isometric", (1.25, -1.55, 1.05), (0.0, 0.0, 55.0), (0.0, 0.0, 1.0)),
    View("top", (0.0, 0.0, 1.0), (0.0, 0.0, 30.0), (0.0, 1.0, 0.0)),
    View("side", (0.0, -1.0, 0.22), (0.0, 0.0, 55.0), (0.0, 0.0, 1.0)),
    View("drive-unit-detail", (1.2, -1.4, 0.8), (145.0, 0.0, -8.0), (0.0, 0.0, 1.0), 190.0),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--obj", type=Path, required=True)
    parser.add_argument("--mtl", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--width", type=int, default=1400)
    parser.add_argument("--height", type=int, default=1000)
    return parser.parse_args()


def load_mtl(path: Path) -> dict[str, tuple[float, float, float]]:
    colors: dict[str, tuple[float, float, float]] = {}
    current: str | None = None
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        fields = raw_line.split()
        if not fields:
            continue
        if fields[0] == "newmtl" and len(fields) >= 2:
            current = fields[1]
        elif fields[0] == "Kd" and current and len(fields) >= 4:
            colors[current] = tuple(float(value) for value in fields[1:4])
    return colors


def load_obj(obj_path: Path, mtl_path: Path) -> Mesh:
    material_colors = load_mtl(mtl_path)
    material_names: list[str] = []
    material_indices: dict[str, int] = {}
    vertices: list[list[float]] = []
    triangles: list[list[int]] = []
    triangle_materials: list[int] = []
    current_material = "default"

    def material_index(name: str) -> int:
        if name not in material_indices:
            material_indices[name] = len(material_names)
            material_names.append(name)
        return material_indices[name]

    for raw_line in obj_path.read_text(encoding="utf-8").splitlines():
        fields = raw_line.split()
        if not fields:
            continue
        if fields[0] == "v" and len(fields) >= 4:
            vertices.append([float(value) for value in fields[1:4]])
        elif fields[0] == "usemtl" and len(fields) >= 2:
            current_material = fields[1]
        elif fields[0] == "f" and len(fields) == 4:
            triangles.append([int(value.split("/")[0]) - 1 for value in fields[1:4]])
            triangle_materials.append(material_index(current_material))

    colors = np.asarray(
        [material_colors.get(name, (0.65, 0.65, 0.65)) for name in material_names],
        dtype=np.float64,
    )
    return Mesh(
        vertices=np.asarray(vertices, dtype=np.float64),
        triangles=np.asarray(triangles, dtype=np.int32),
        triangle_materials=np.asarray(triangle_materials, dtype=np.int32),
        colors=colors,
    )


def camera_basis(view: View) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    direction = np.asarray(view.target) - np.asarray(view.camera)
    forward = direction / np.linalg.norm(direction)
    up = np.asarray(view.up, dtype=np.float64)
    right = np.cross(forward, up)
    right /= np.linalg.norm(right)
    camera_up = np.cross(right, forward)
    return right, camera_up, forward


def render(mesh: Mesh, view: View, width: int, height: int, output: Path | None = None) -> Image.Image:
    right, camera_up, forward = camera_basis(view)
    target = np.asarray(view.target, dtype=np.float64)
    relative = mesh.vertices - target
    projected = np.column_stack(
        (relative @ right, relative @ camera_up, relative @ forward)
    )
    points = projected[mesh.triangles]
    centroids = points.mean(axis=1)
    if view.span is None:
        min_xy = projected[:, :2].min(axis=0)
        max_xy = projected[:, :2].max(axis=0)
        span = max(*(max_xy - min_xy)) * 1.10
        center = (min_xy + max_xy) * 0.5
    else:
        span = view.span
        center = np.zeros(2)
    pixels_per_unit = min(width, height) / span
    screen = np.empty_like(points[:, :, :2])
    screen[:, :, 0] = (points[:, :, 0] - center[0]) * pixels_per_unit + width * 0.5
    screen[:, :, 1] = height * 0.5 - (points[:, :, 1] - center[1]) * pixels_per_unit

    world_triangles = mesh.vertices[mesh.triangles]
    normals = np.cross(
        world_triangles[:, 1] - world_triangles[:, 0],
        world_triangles[:, 2] - world_triangles[:, 0],
    )
    lengths = np.linalg.norm(normals, axis=1)
    valid = lengths > 1.0e-12
    normals[valid] /= lengths[valid, None]
    light = np.asarray((0.35, -0.45, 0.82), dtype=np.float64)
    light /= np.linalg.norm(light)
    illumination = np.clip(np.abs(normals @ light) * 0.68 + 0.32, 0.22, 1.0)
    base_colors = mesh.colors[mesh.triangle_materials]
    shaded = np.clip(base_colors * illumination[:, None], 0.0, 1.0)

    image = Image.new("RGB", (width, height), (239, 243, 247))
    draw = ImageDraw.Draw(image)
    order = np.argsort(centroids[:, 2])[::-1]
    for triangle_index in order:
        polygon = [tuple(point) for point in screen[triangle_index]]
        color = tuple(int(channel * 255) for channel in shaded[triangle_index])
        edge = tuple(max(0, int(channel * 0.58)) for channel in color)
        draw.polygon(polygon, fill=color, outline=edge)

    draw.rectangle((18, 18, 470, 70), fill=(250, 252, 254), outline=(88, 98, 108), width=2)
    draw.text((34, 30), f"Geared gimbal prototype - {view.name}", fill=(24, 30, 36), font=ImageFont.load_default(18))
    draw.text((34, height - 42), "Concept geometry only - not load-rated", fill=(110, 36, 32), font=ImageFont.load_default(15))
    if output is not None:
        output.parent.mkdir(parents=True, exist_ok=True)
        image.save(output, optimize=True)
    return image


def main() -> None:
    args = parse_args()
    mesh = load_obj(args.obj, args.mtl)
    args.output.mkdir(parents=True, exist_ok=True)
    for view in VIEWS:
        destination = args.output / f"{view.name}.png"
        render(mesh, view, args.width, args.height, destination)
        print(destination)


if __name__ == "__main__":
    main()
