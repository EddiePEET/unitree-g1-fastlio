#!/usr/bin/env python3
"""
多高度点云投影到地面，生成 Nav2 PGM/YAML。

处理流程：
1. 读取 binary PCD；
2. 根据地面平面计算“离地高度”；
3. 将多个高度层分别进行半径滤波；
4. 所有高度层投影到 XY 平面并取并集；
5. 进行轻微闭运算和膨胀，让墙线更连续；
6. 输出 PGM 和 YAML。

说明：
- 默认去除地面：低于 0.20 m 不参与投影；
- 默认去除天花板：高于 1.75 m 不参与投影；
- 默认参数针对当前 map.pcd / map1(1).pcd；
- 兼容 Ubuntu 20.04 上较旧的 SciPy，不使用 workers 参数。
"""

import argparse
from pathlib import Path
import sys

import numpy as np

try:
    from scipy.spatial import cKDTree
except ImportError:
    print("错误：缺少 scipy，请先安装或确认 scipy 可用。", file=sys.stderr)
    raise SystemExit(2)


def read_binary_pcd(path: Path):
    header = {}

    with path.open("rb") as f:
        while True:
            line = f.readline()
            if not line:
                raise RuntimeError("没有找到 PCD DATA 字段")

            text = line.decode("ascii", errors="ignore").strip()
            if not text or text.startswith("#"):
                continue

            parts = text.split()
            key = parts[0].upper()
            header[key] = parts[1:]

            if key == "DATA":
                data_offset = f.tell()
                break

    mode = header["DATA"][0].lower()
    if mode != "binary":
        raise RuntimeError(f"当前脚本只支持 DATA binary，实际为：{mode}")

    fields = header["FIELDS"]
    sizes = list(map(int, header["SIZE"]))
    types = header["TYPE"]
    counts = list(map(int, header.get("COUNT", ["1"] * len(fields))))
    points = int(header["POINTS"][0])

    type_map = {
        ("F", 4): "<f4",
        ("F", 8): "<f8",
        ("I", 1): "<i1",
        ("I", 2): "<i2",
        ("I", 4): "<i4",
        ("U", 1): "<u1",
        ("U", 2): "<u2",
        ("U", 4): "<u4",
    }

    dtype_fields = []

    for name, size, field_type, count in zip(
        fields,
        sizes,
        types,
        counts,
    ):
        dtype = type_map.get((field_type, size))
        if dtype is None:
            raise RuntimeError(
                f"不支持字段 {name}: TYPE={field_type}, SIZE={size}"
            )

        if count == 1:
            dtype_fields.append((name, dtype))
        else:
            dtype_fields.append((name, dtype, (count,)))

    with path.open("rb") as f:
        f.seek(data_offset)
        cloud = np.fromfile(
            f,
            dtype=np.dtype(dtype_fields),
            count=points,
        )

    return cloud


def parse_layers(text: str):
    layers = []

    for item in text.split(","):
        item = item.strip()
        if not item:
            continue

        try:
            low_text, high_text = item.split(":")
            low = float(low_text)
            high = float(high_text)
        except ValueError as error:
            raise argparse.ArgumentTypeError(
                "高度层格式应为：0.20:0.50,0.45:0.80"
            ) from error

        if low >= high:
            raise argparse.ArgumentTypeError(
                f"高度层无效：{low}:{high}"
            )

        layers.append((low, high))

    if not layers:
        raise argparse.ArgumentTypeError("至少需要一个高度层")

    return layers


def disk_offsets(radius: int):
    offsets = []

    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy <= radius * radius:
                offsets.append((dy, dx))

    return offsets


def binary_dilate(mask: np.ndarray, radius: int):
    if radius <= 0:
        return mask.copy()

    height, width = mask.shape
    padded = np.pad(
        mask,
        radius,
        mode="constant",
        constant_values=False,
    )

    output = np.zeros_like(mask, dtype=bool)

    for dy, dx in disk_offsets(radius):
        y0 = radius + dy
        x0 = radius + dx
        output |= padded[y0:y0 + height, x0:x0 + width]

    return output


def binary_erode(mask: np.ndarray, radius: int):
    if radius <= 0:
        return mask.copy()

    height, width = mask.shape
    padded = np.pad(
        mask,
        radius,
        mode="constant",
        constant_values=False,
    )

    output = np.ones_like(mask, dtype=bool)

    for dy, dx in disk_offsets(radius):
        y0 = radius + dy
        x0 = radius + dx
        output &= padded[y0:y0 + height, x0:x0 + width]

    return output


def binary_close(mask: np.ndarray, radius: int):
    return binary_erode(
        binary_dilate(mask, radius),
        radius,
    )


def write_pgm(path: Path, image_xy: np.ndarray):
    # 内部数组第 0 行代表最小 Y；
    # ROS 地图 PGM 第 0 行应代表最大 Y，因此上下翻转。
    image_file = np.flipud(image_xy).astype(np.uint8)

    with path.open("wb") as f:
        f.write(
            f"P5\n{image_file.shape[1]} {image_file.shape[0]}\n255\n"
            .encode("ascii")
        )
        f.write(image_file.tobytes())


def main():
    parser = argparse.ArgumentParser(
        description="多高度点云投影为二维 Nav2 地图"
    )

    parser.add_argument("input_pcd")
    parser.add_argument(
        "--output-prefix",
        default="map_multilayer",
    )

    parser.add_argument(
        "--layers",
        type=parse_layers,
        default=parse_layers(
            "0.20:0.50,"
            "0.45:0.80,"
            "0.75:1.10,"
            "1.05:1.40,"
            "1.35:1.75"
        ),
        help=(
            "相对地面的高度层，例如："
            "0.20:0.50,0.45:0.80,0.75:1.10"
        ),
    )

    parser.add_argument("--radius", type=float, default=0.15)
    parser.add_argument("--min-neighbors", type=int, default=2)
    parser.add_argument("--resolution", type=float, default=0.05)

    # 当前点云对应的地面平面：
    # ground_z(x,y) = a*x + b*y + c
    parser.add_argument("--plane-a", type=float, default=-0.06203)
    parser.add_argument("--plane-b", type=float, default=-0.02687)
    parser.add_argument("--plane-c", type=float, default=-1.51000)

    parser.add_argument("--xmin", type=float, default=-6.5)
    parser.add_argument("--xmax", type=float, default=23.5)
    parser.add_argument("--ymin", type=float, default=-20.0)
    parser.add_argument("--ymax", type=float, default=28.5)

    parser.add_argument("--close-radius", type=int, default=1)
    parser.add_argument("--dilate-radius", type=int, default=1)
    parser.add_argument(
        "--min-layers",
        type=int,
        default=1,
        help="至少多少个高度层出现才判为障碍，默认 1",
    )

    args = parser.parse_args()

    input_path = Path(args.input_pcd).resolve()
    output_prefix = Path(args.output_prefix)

    if not output_prefix.is_absolute():
        output_prefix = input_path.parent / output_prefix

    if not input_path.exists():
        raise RuntimeError(f"找不到输入点云：{input_path}")

    cloud = read_binary_pcd(input_path)

    x = cloud["x"].astype(np.float64)
    y = cloud["y"].astype(np.float64)
    z = cloud["z"].astype(np.float64)

    finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    x = x[finite]
    y = y[finite]
    z = z[finite]

    ground_z = (
        args.plane_a * x
        + args.plane_b * y
        + args.plane_c
    )

    relative_height = z - ground_z

    width = int(
        np.ceil((args.xmax - args.xmin) / args.resolution)
    ) + 1

    height = int(
        np.ceil((args.ymax - args.ymin) / args.resolution)
    ) + 1

    layer_masks = []

    print("=" * 68)
    print(f"输入点云：{input_path}")
    print(f"有效点数：{len(x)}")
    print(
        "地面平面："
        f"z={args.plane_a:.5f}*x"
        f"{args.plane_b:+.5f}*y"
        f"{args.plane_c:+.5f}"
    )

    for low, high in args.layers:
        selected = (
            (relative_height >= low)
            & (relative_height <= high)
            & (x >= args.xmin)
            & (x <= args.xmax)
            & (y >= args.ymin)
            & (y <= args.ymax)
        )

        points = np.column_stack([
            x[selected],
            y[selected],
            z[selected],
        ])

        layer_mask = np.zeros(
            (height, width),
            dtype=bool,
        )

        if len(points) == 0:
            layer_masks.append(layer_mask)
            print(f"高度层 {low:.2f}～{high:.2f} m：0 点")
            continue

        tree = cKDTree(points)

        # 使用 query 而不是 query_ball_point(workers=...)
        # 以兼容 Ubuntu 20.04 的旧版 SciPy。
        distances, _ = tree.query(
            points,
            k=args.min_neighbors,
            distance_upper_bound=args.radius,
        )

        if args.min_neighbors == 1:
            keep = np.isfinite(distances)
        else:
            keep = np.isfinite(distances[:, -1])

        filtered = points[keep]

        ix = np.floor(
            (filtered[:, 0] - args.xmin) / args.resolution
        ).astype(np.int64)

        iy = np.floor(
            (filtered[:, 1] - args.ymin) / args.resolution
        ).astype(np.int64)

        valid = (
            (ix >= 0)
            & (ix < width)
            & (iy >= 0)
            & (iy < height)
        )

        layer_mask[iy[valid], ix[valid]] = True
        layer_masks.append(layer_mask)

        print(
            f"高度层 {low:.2f}～{high:.2f} m："
            f"{len(points)} -> {len(filtered)} 点"
        )

    layer_count = np.sum(layer_masks, axis=0)
    obstacles = layer_count >= args.min_layers

    obstacles = binary_close(
        obstacles,
        args.close_radius,
    )

    obstacles = binary_dilate(
        obstacles,
        args.dilate_radius,
    )

    # 文档式地图：没有障碍点的格子全部设为自由。
    image = np.full(
        (height, width),
        254,
        dtype=np.uint8,
    )

    image[obstacles] = 0

    pgm_path = output_prefix.with_suffix(".pgm")
    yaml_path = output_prefix.with_suffix(".yaml")

    write_pgm(pgm_path, image)

    yaml_path.write_text(
        f"""image: {pgm_path.name}
mode: trinary
resolution: {args.resolution:.6f}
origin: [{args.xmin:.6f}, {args.ymin:.6f}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
""",
        encoding="utf-8",
    )

    print(f"地图尺寸：{width} × {height}")
    print(f"障碍栅格：{int(np.count_nonzero(obstacles))}")
    print(f"已生成：{pgm_path}")
    print(f"已生成：{yaml_path}")
    print("=" * 68)


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(f"错误：{error}", file=sys.stderr)
        raise SystemExit(1)
