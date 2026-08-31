# SPDX-License-Identifier: MIT
"""Render the generated glTF keyframes to an animated GIF without CAD dependencies."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from render import Mesh, VIEWS, render


COMPONENT_DTYPES = {
    5125: np.dtype("<u4"),
    5126: np.dtype("<f4"),
}
TYPE_WIDTHS = {"SCALAR": 1, "VEC3": 3, "VEC4": 4}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--gltf", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=32)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=720)
    return parser.parse_args()


class GltfScene:
    def __init__(self, path: Path):
        self.path = path
        self.document = json.loads(path.read_text(encoding="utf-8"))
        binary_path = path.with_name(self.document["buffers"][0]["uri"])
        self.binary = binary_path.read_bytes()
        self.node_channels: dict[int, dict[str, tuple[np.ndarray, np.ndarray]]] = {}
        animation = self.document["animations"][0]
        for channel in animation["channels"]:
            sampler = animation["samplers"][channel["sampler"]]
            node = channel["target"]["node"]
            path_name = channel["target"]["path"]
            self.node_channels.setdefault(node, {})[path_name] = (
                self.accessor(sampler["input"]),
                self.accessor(sampler["output"]),
            )

    def accessor(self, index: int) -> np.ndarray:
        accessor = self.document["accessors"][index]
        view = self.document["bufferViews"][accessor["bufferView"]]
        dtype = COMPONENT_DTYPES[accessor["componentType"]]
        width = TYPE_WIDTHS[accessor["type"]]
        offset = view.get("byteOffset", 0) + accessor.get("byteOffset", 0)
        count = accessor["count"] * width
        array = np.frombuffer(self.binary, dtype=dtype, count=count, offset=offset)
        return array.reshape(accessor["count"], width).astype(np.float64)

    def duration(self) -> float:
        animation = self.document["animations"][0]
        sampler = animation["samplers"][0]
        return float(self.accessor(sampler["input"])[-1, 0])

    def mesh_at(self, time: float) -> Mesh:
        vertices: list[np.ndarray] = []
        triangles: list[np.ndarray] = []
        triangle_materials: list[np.ndarray] = []
        colors = np.asarray(
            [
                material["pbrMetallicRoughness"]["baseColorFactor"][:3]
                for material in self.document["materials"]
            ],
            dtype=np.float64,
        )
        vertex_offset = 0
        for node_index, node in enumerate(self.document["nodes"]):
            primitive = self.document["meshes"][node["mesh"]]["primitives"][0]
            positions = self.accessor(primitive["attributes"]["POSITION"])
            indices = self.accessor(primitive["indices"]).astype(np.int32).reshape(-1, 3)
            translation = np.asarray(node.get("translation", [0.0, 0.0, 0.0]), dtype=np.float64)
            rotation = np.asarray(node.get("rotation", [0.0, 0.0, 0.0, 1.0]), dtype=np.float64)
            channels = self.node_channels.get(node_index, {})
            if "translation" in channels:
                translation = sample(*channels["translation"], time, quaternion=False)
            if "rotation" in channels:
                rotation = sample(*channels["rotation"], time, quaternion=True)
            transformed = rotate_by_quaternion(positions, rotation) + translation
            vertices.append(transformed)
            triangles.append(indices + vertex_offset)
            triangle_materials.append(
                np.full(len(indices), primitive["material"], dtype=np.int32)
            )
            vertex_offset += len(positions)
        return Mesh(
            vertices=np.vstack(vertices),
            triangles=np.vstack(triangles),
            triangle_materials=np.concatenate(triangle_materials),
            colors=colors,
        )


def sample(
    times: np.ndarray,
    values: np.ndarray,
    time: float,
    *,
    quaternion: bool,
) -> np.ndarray:
    flat_times = times[:, 0]
    upper = int(np.searchsorted(flat_times, time, side="right"))
    if upper <= 0:
        return values[0]
    if upper >= len(flat_times):
        return values[-1]
    lower = upper - 1
    alpha = (time - flat_times[lower]) / (flat_times[upper] - flat_times[lower])
    start = values[lower]
    end = values[upper]
    if quaternion and np.dot(start, end) < 0.0:
        end = -end
    result = start * (1.0 - alpha) + end * alpha
    if quaternion:
        result /= np.linalg.norm(result)
    return result


def rotate_by_quaternion(points: np.ndarray, quaternion: np.ndarray) -> np.ndarray:
    vector = quaternion[:3]
    scalar = quaternion[3]
    cross = np.cross(np.broadcast_to(vector, points.shape), points)
    return points + 2.0 * (scalar * cross + np.cross(vector, cross))


def main() -> None:
    args = parse_args()
    scene = GltfScene(args.gltf)
    frames = []
    for index, time in enumerate(np.linspace(0.0, scene.duration(), args.frames, endpoint=False)):
        mesh = scene.mesh_at(float(time))
        image = render(mesh, VIEWS[0], args.width, args.height)
        frames.append(image)
        print(f"frame {index + 1}/{args.frames}")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(
        args.output,
        save_all=True,
        append_images=frames[1:],
        duration=100,
        loop=0,
        optimize=False,
    )


if __name__ == "__main__":
    main()
