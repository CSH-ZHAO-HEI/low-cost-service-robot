#!/usr/bin/env python3
"""
get_scene.py — 從 Gazebo 抓取所有物件位置，生成 gazebo_scene.yaml
用法：
  python3 get_scene.py               # 輸出到 ./gazebo_scene.yaml
  python3 get_scene.py --deploy      # 同時複製到 ~/.ros/semantic_map.yaml（Big Brain 直接用）
  python3 get_scene.py --approach 0.4  # 指定接近距離（預設 0.35m）

需要 Gazebo 正在運行（run_gazebo.sh）

接近點計算說明：
  base_link +X = 車頭（相機 / 機械臂方向），底盤半長 0.143m。
  若能取得 bbox，接近點會落在 bbox 外側，而不是物件中心附近。
  approach_yaw  = 從接近點指向物件的方向（讓車頭朝向物件）。
"""

import sys
import os
import math
import xml.etree.ElementTree as ET
import yaml
import rospy
import numpy as np
from datetime import datetime
from gazebo_msgs.srv import GetWorldProperties, GetModelState
from nav_msgs.msg import OccupancyGrid

# ── 設定 ──────────────────────────────────────────────────────
APPROACH_DIST    = 0.55   # bbox 失敗時，底盤中心距物件中心距離（公尺）
APPROACH_MARGIN  = 0.30   # bbox 邊界之外額外保留距離（公尺）
MIN_APPROACH_DIST = 0.55  # bbox 很小時也至少保留此距離
PLACE_INSET      = 0.18   # 桌面放置點離 bbox 邊緣往內縮，避免碰邊穿模

# 包根目录（scripts/ 的上级 = 解压后的 code/）
_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# mesh 搜尋路徑（model:// 解析用）。优先包内路径，再回退原项目绝对路径。
MODEL_DIRS = [
    os.path.join(_PKG_ROOT, "scene", "aws-robomaker-small-house-world", "models"),
    os.path.join(_PKG_ROOT, "small_brain", "models"),
    os.path.join(_PKG_ROOT, "small_brain", "aws-robomaker-small-warehouse-world", "models"),
    os.path.join(_PKG_ROOT, "scene"),  # coke 等独立模型
    os.path.expanduser("~/.gazebo/models"),
]


def _resolve_mesh(uri):
    """把 model://ModelName/meshes/xxx.DAE 解析成絕對路徑"""
    if not uri.startswith("model://"):
        return None
    rel = uri[len("model://"):]  # ModelName/meshes/xxx.DAE
    for base in MODEL_DIRS:
        full = os.path.join(base, rel)
        if os.path.exists(full):
            return full
    return None


def get_model_bbox(model_name):
    """
    從模型 SDF 找 collision mesh，用 trimesh 算 bounding box。
    回傳 (half_x, half_y, surface_z) 或 None。
    half_x/y = 模型 local frame 的 XY 半寬，用來算 oriented bbox approach
    surface_z = bbox max Z（頂面高度）
    """
    try:
        import trimesh
    except ImportError:
        return None

    # 找 SDF 文件
    sdf_path = None
    for base in MODEL_DIRS:
        candidate = os.path.join(base, model_name, "model.sdf")
        if os.path.exists(candidate):
            sdf_path = candidate
            break
    if sdf_path is None:
        return None

    try:
        tree = ET.parse(sdf_path)
        root = tree.getroot()
        def _pose_z(el):
            """從 <pose>x y z r p y</pose> 取 z，找不到回傳 0"""
            p = el.find("pose")
            if p is None or not p.text:
                return 0.0
            parts = p.text.strip().split()
            return float(parts[2]) if len(parts) >= 3 else 0.0

        # 找第一個 collision mesh uri 和 scale，同時累加 link/collision pose z offset
        for link in root.iter("link"):
            link_z = _pose_z(link)
            for col in link.iter("collision"):
                col_z = _pose_z(col)
                mesh_el = col.find(".//mesh")
                if mesh_el is None:
                    continue
                uri_el = mesh_el.find("uri")
                if uri_el is None:
                    continue
                mesh_path = _resolve_mesh(uri_el.text.strip())
                if mesh_path is None:
                    continue

                scale = [1.0, 1.0, 1.0]
                scale_el = mesh_el.find("scale")
                if scale_el is not None:
                    scale = [float(v) for v in scale_el.text.strip().split()]

                mesh = trimesh.load(mesh_path, force="mesh")
                # DAE 單位公分（meter=0.01），先換算成公尺
                mesh.apply_scale(0.01)
                # 再套用 SDF scale
                mesh.apply_scale(scale)
                bounds = mesh.bounds  # [[min_x,min_y,min_z],[max_x,max_y,max_z]]
                half_x = (bounds[1][0] - bounds[0][0]) / 2.0
                half_y = (bounds[1][1] - bounds[0][1]) / 2.0
                # local surface z + link pose z + collision pose z
                surface_z = float(bounds[1][2]) + link_z + col_z
                return float(half_x), float(half_y), surface_z

    except Exception as e:
        print(f"    [bbox err] {model_name}: {e}")
    return None
OUTPUT_PATH   = os.environ.get("GAZEBO_SCENE_PATH") or os.path.join(_PKG_ROOT, "gazebo_scene.yaml")
DEPLOY_PATH   = os.path.expanduser("~/.ros/semantic_map.yaml")

SKIP_MODELS = {
    "ground_plane",
    "sun",
    "mini_mec_six_arm",
    "mini_mec_six_arm_sim",
    "Tablet",
}


_costmap = None  # OccupancyGrid 快取

def _on_costmap(msg):
    global _costmap
    _costmap = msg

def _costmap_value(x, y):
    """回傳 costmap 值（0=free, 100=occupied, -1=unknown），範圍外回傳 999"""
    if _costmap is None:
        return 0
    res = _costmap.info.resolution
    ox  = _costmap.info.origin.position.x
    oy  = _costmap.info.origin.position.y
    w   = _costmap.info.width
    h   = _costmap.info.height
    cx  = int((x - ox) / res)
    cy  = int((y - oy) / res)
    if cx < 0 or cy < 0 or cx >= w or cy >= h:
        return 999
    cost = _costmap.data[cy * w + cx]
    return cost if cost >= 0 else 999

def _is_free(x, y, threshold=80):
    return _costmap_value(x, y) < threshold


def get_yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def compute_approach(obj_x, obj_y, obj_yaw, dist, robot_x=0.0, robot_y=0.0,
                     bbox_half_x=None, bbox_half_y=None):
    """
    試四個世界方向（±X, ±Y），優先選 free 且離機器人最近的接近點。
    有 bbox 時，按物件 yaw 算該方向到 oriented bbox 邊界的距離，
    使 approach 落在物件外側，而不是寬桌子的中心附近。
    approach_yaw：使 base_link +X（車頭）朝向物件。
    """
    candidates = []
    for angle in [math.pi, 0.0, math.pi / 2, -math.pi / 2]:
        if bbox_half_x is not None and bbox_half_y is not None:
            rel = angle - obj_yaw
            edge_dist = (
                abs(math.cos(rel)) * bbox_half_x +
                abs(math.sin(rel)) * bbox_half_y
            )
            approach_dist = max(edge_dist + APPROACH_MARGIN, MIN_APPROACH_DIST)
        else:
            approach_dist = dist

        ax = obj_x + approach_dist * math.cos(angle)
        ay = obj_y + approach_dist * math.sin(angle)
        ayaw = math.atan2(obj_y - ay, obj_x - ax)
        free = _is_free(ax, ay)
        robot_dist = math.sqrt((ax - robot_x)**2 + (ay - robot_y)**2)
        candidates.append((ax, ay, ayaw, free, robot_dist, approach_dist, angle, edge_dist if bbox_half_x is not None else None))

    # 按離機器人距離排序，選第一個 free 的
    candidates_sorted = sorted(candidates, key=lambda c: c[4])
    for ax, ay, ayaw, free, _, _, angle, edge_dist in candidates_sorted:
        if free:
            return ax, ay, ayaw, angle, edge_dist

    # 全部被佔用時選離機器人最近的
    return (
        candidates_sorted[0][0],
        candidates_sorted[0][1],
        candidates_sorted[0][2],
        candidates_sorted[0][6],
        candidates_sorted[0][7],
    )


def compute_all_approaches(obj_x, obj_y, obj_yaw,
                           bbox_half_x=None, bbox_half_y=None):
    """回傳 4 個世界方向（E, W, N, S）的所有 approach candidate 列表。
    跟 compute_approach 同樣演算法（按 yaw 算 oriented bbox 邊界距離），
    但保留全部 4 個方向供下游 LLM 自選。

    回傳：list of dict
        [{x, y, yaw, side, edge_dist, is_free}, ...]
    side 標籤：'E'=+X, 'W'=-X, 'N'=+Y, 'S'=-Y
    """
    sides = [
        (0.0,        "E"),
        (math.pi,    "W"),
        (math.pi/2,  "N"),
        (-math.pi/2, "S"),
    ]
    out = []
    for angle, side in sides:
        if bbox_half_x is not None and bbox_half_y is not None:
            rel = angle - obj_yaw
            edge_dist = (
                abs(math.cos(rel)) * bbox_half_x +
                abs(math.sin(rel)) * bbox_half_y
            )
            approach_dist = max(edge_dist + APPROACH_MARGIN, MIN_APPROACH_DIST)
        else:
            edge_dist = None
            approach_dist = APPROACH_DIST
        ax = obj_x + approach_dist * math.cos(angle)
        ay = obj_y + approach_dist * math.sin(angle)
        ayaw = math.atan2(obj_y - ay, obj_x - ax)
        out.append({
            "x":         round(ax, 3),
            "y":         round(ay, 3),
            "yaw":       round(ayaw, 4),
            "side":      side,
            "is_free":   bool(_is_free(ax, ay)),
            "edge_dist": round(edge_dist, 3) if edge_dist is not None else None,
        })
    return out


def compute_place_point(name, obj_x, obj_y, approach_angle, edge_dist):
    """
    給機械臂的放置點。導航用 approach 點在 bbox 外；放置點應在桌面/桶口上。
    - table/desk 等大平面：放在靠近機器人那側的桌面內側，降低伸手距離與穿模機率
    - trash/bin：放在中心桶口
    """
    lower = name.lower()
    if "trash" in lower or "bin" in lower:
        return obj_x, obj_y
    if edge_dist is None:
        return obj_x, obj_y

    place_dist = max(0.0, edge_dist - PLACE_INSET)
    return (
        obj_x + place_dist * math.cos(approach_angle),
        obj_y + place_dist * math.sin(approach_angle),
    )


def compute_coffeetable_pose(obj_x, obj_y, bbox_half_y=None):
    """CoffeeTable 專用接近點：從 north 正對桌面長邊。"""
    edge_dist = bbox_half_y if bbox_half_y is not None else 0.334
    approach_dist = edge_dist + 0.30
    ax = obj_x
    ay = obj_y + approach_dist
    ayaw = -math.pi / 2.0
    place_x = obj_x
    place_y = obj_y + edge_dist * 0.7
    return ax, ay, ayaw, place_x, place_y


def main():
    deploy = "--deploy" in sys.argv
    approach_dist = APPROACH_DIST
    for i, arg in enumerate(sys.argv):
        if arg == "--approach" and i + 1 < len(sys.argv):
            approach_dist = float(sys.argv[i + 1])

    rospy.init_node("get_scene", anonymous=True)

    # 訂閱 global costmap（若導航已啟動則可用，否則跳過）
    rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, _on_costmap, queue_size=1)
    print("[get_scene] 等待 costmap（3 秒）...")
    rospy.sleep(3.0)
    if _costmap is None:
        print("[get_scene] 警告：costmap 未收到，approach 方向可能不準（請先啟動 run_ego.sh 或 run_teb_compare.sh）")
    else:
        print("[get_scene] costmap 已載入")

    print("[get_scene] 等待 Gazebo 服務...")
    rospy.wait_for_service("/gazebo/get_world_properties",   timeout=10.0)
    rospy.wait_for_service("/gazebo/get_model_state",        timeout=10.0)

    get_world = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
    get_state = rospy.ServiceProxy("/gazebo/get_model_state",      GetModelState)

    # 取機器人當前位置
    robot_x, robot_y = 0.0, 0.0
    try:
        robot_state = get_state("mini_mec_six_arm", "world")
        if robot_state.success:
            robot_x = robot_state.pose.position.x
            robot_y = robot_state.pose.position.y
            print(f"[get_scene] 機器人位置：({robot_x:.2f}, {robot_y:.2f})")
    except Exception:
        print("[get_scene] 無法取得機器人位置，使用原點")

    world  = get_world()
    models = world.model_names
    print(f"[get_scene] 共找到 {len(models)} 個模型：{models}")

    scene   = {}
    skipped = []

    for name in models:
        skip = False
        for s in SKIP_MODELS:
            if name == s or name.startswith(s):
                skip = True
                break
        if skip:
            skipped.append(name)
            continue

        try:
            state = get_state(name, "world")
            if not state.success:
                print(f"  [跳過] {name}：get_model_state 失敗")
                continue

            pos     = state.pose.position
            ori     = state.pose.orientation
            obj_yaw = get_yaw_from_quaternion(ori)

            # 從 SDF + trimesh 算真實 bbox
            dist        = approach_dist
            surface_z   = None
            bbox_half_x = None
            bbox_half_y = None
            # model 名稱去掉末尾 _001/_002 等，找對應 SDF 目錄
            # 例：BalconyTable_01_001 → aws_robomaker_residential_BalconyTable_01
            sdf_model_name = None
            for base in MODEL_DIRS:
                if not os.path.isdir(base):
                    continue
                for entry_name in os.listdir(base):
                    # 比對：去掉 aws_robomaker_residential_ 前綴後的名字
                    stripped = entry_name.replace("aws_robomaker_residential_", "")
                    # name 可能是 BalconyTable_01_001，stripped 是 BalconyTable_01
                    if name.lower().startswith(stripped.lower()):
                        sdf_model_name = entry_name
                        break
                if sdf_model_name:
                    break

            if sdf_model_name:
                bbox = get_model_bbox(sdf_model_name)
                if bbox:
                    half_x, half_y, surface_z = bbox
                    bbox_half_x = float(half_x)
                    bbox_half_y = float(half_y)
                    # 加上模型世界 z（應對非地面擺放的物件）
                    surface_z = float(surface_z) + float(pos.z)
                    # trash 是桶，放在桶口內部，降低 surface_z
                    if 'trash' in name.lower():
                        surface_z = round(surface_z * 0.5, 3)
                    print(
                        f"    [bbox] half=({bbox_half_x:.3f},{bbox_half_y:.3f})m"
                        f"  margin={APPROACH_MARGIN:.2f}m"
                        f"  surface_z={surface_z:.3f}m"
                    )
                else:
                    print(f"    [bbox] 無法取得，使用預設 dist={dist:.2f}m")
            else:
                print(f"    [bbox] 找不到 SDF：{name}，使用預設 dist={dist:.2f}m")

            if name.lower().startswith("coffeetable"):
                ax, ay, ayaw, place_x, place_y = compute_coffeetable_pose(
                    pos.x, pos.y, bbox_half_y=bbox_half_y
                )
            else:
                ax, ay, ayaw, approach_angle, edge_dist = compute_approach(
                    pos.x, pos.y, obj_yaw, dist, robot_x, robot_y,
                    bbox_half_x=bbox_half_x,
                    bbox_half_y=bbox_half_y,
                )
                place_x, place_y = compute_place_point(name, pos.x, pos.y, approach_angle, edge_dist)

            entry = {
                "object_x":     round(pos.x, 3),
                "object_y":     round(pos.y, 3),
                "place_x":      round(place_x, 3),
                "place_y":      round(place_y, 3),
                "approach_x":   round(ax,    3),
                "approach_y":   round(ay,    3),
                "approach_yaw": round(ayaw,  4),
            }
            if surface_z is not None:
                entry["surface_z"] = surface_z
            if bbox_half_x is not None and bbox_half_y is not None:
                entry["bbox_half_x"] = round(bbox_half_x, 3)
                entry["bbox_half_y"] = round(bbox_half_y, 3)

            # 只對 Trash_01_001 額外存 4 個方向的 approach 候選清單。
            # 對稱物件無預設「正面」，LLM 可從這 4 個自選最近可達側。
            if name == "Trash_01_001":
                entry["approach_candidates"] = compute_all_approaches(
                    pos.x, pos.y, obj_yaw,
                    bbox_half_x=bbox_half_x, bbox_half_y=bbox_half_y,
                )

            scene[name] = entry
            print(f"  ✓ {name:35s}  obj=({pos.x:.2f},{pos.y:.2f})"
                  f"  approach=({ax:.2f},{ay:.2f}, yaw={math.degrees(ayaw):.1f}°)"
                  f"  place=({place_x:.2f},{place_y:.2f})")

        except Exception as e:
            print(f"  [錯誤] {name}：{e}")

    if skipped:
        print(f"\n[get_scene] 已跳過（非場景物件）：{skipped}")

    header = (
        f"# gazebo_scene.yaml — Gazebo 場景物件位置（自動生成）\n"
        f"# 生成時間：{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n"
        f"# 接近距離：{approach_dist}m（底盤半長 0.143m → 車頭距物件 ≈ {approach_dist - 0.143:.3f}m）\n"
        f"# 欄位說明：\n"
        f"#   object_x/y   — 物件 XY 座標（公尺）\n"
        f"#   place_x/y    — 機械臂放置點；大桌子取靠近邊緣的桌面點，垃圾桶取中心\n"
        f"#   approach_x/y — 機器人底盤中心停靠位置\n"
        f"#   approach_yaw — 機器人朝向（弧度，車頭 +X 朝向物件）\n"
        f"#   bbox_half_x/y — SDF collision bbox 的 local XY 半寬（若可取得）\n\n"
    )

    yaml_str = yaml.dump(scene, default_flow_style=False, allow_unicode=True, sort_keys=True)

    with open(OUTPUT_PATH, "w") as f:
        f.write(header + yaml_str)
    print(f"\n[get_scene] 已儲存：{OUTPUT_PATH}（共 {len(scene)} 個物件）")

    if deploy:
        import shutil
        os.makedirs(os.path.dirname(DEPLOY_PATH), exist_ok=True)
        shutil.copy(OUTPUT_PATH, DEPLOY_PATH)
        print(f"[get_scene] 已部署至：{DEPLOY_PATH}")

    print("\n完成！")


if __name__ == "__main__":
    main()
