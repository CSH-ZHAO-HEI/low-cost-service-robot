#!/usr/bin/env python3
"""
Lightweight RViz semantic annotation tool.

Workflow:
  1. Start RViz with an existing map.
  2. Select "Publish Point" in RViz.
  3. Click the object center.
  4. Type the object name in this terminal.
  5. Click the robot approach point.

The tool writes a task-level semantic YAML file. It does not modify the
RTAB-Map database or occupancy grid.
"""

from __future__ import annotations

import argparse
import math
import os
import shutil
import sys
from datetime import datetime
from queue import Empty, Queue
from typing import Dict, Optional, Tuple

import rospy
import yaml
from geometry_msgs.msg import PointStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


Point2 = Tuple[float, float]


def _expand(path: str) -> str:
    return os.path.abspath(os.path.expanduser(path))


def _load_yaml(path: str) -> Dict[str, dict]:
    if not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return data


def _atomic_write_yaml(path: str, data: Dict[str, dict]) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp_path = path + ".tmp"
    with open(tmp_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=True)
    os.replace(tmp_path, path)


def _yaw_from_approach_to_object(approach: Point2, obj: Point2) -> float:
    return math.atan2(obj[1] - approach[1], obj[0] - approach[0])


def _color(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    c = ColorRGBA()
    c.r = r
    c.g = g
    c.b = b
    c.a = a
    return c


class ManualSemanticAnnotator:
    def __init__(self, output_path: str, frame_id: str, backup: bool) -> None:
        self.output_path = _expand(output_path)
        self.frame_id = frame_id
        self.backup = backup
        self.points: "Queue[PointStamped]" = Queue()
        self.data = _load_yaml(self.output_path)
        self.marker_pub = rospy.Publisher(
            "/semantic_annotation_markers", MarkerArray, queue_size=1, latch=True
        )
        rospy.Subscriber("/clicked_point", PointStamped, self._on_clicked_point, queue_size=20)
        self._publish_markers()

    def _on_clicked_point(self, msg: PointStamped) -> None:
        self.points.put(msg)

    def _wait_point(self, prompt: str) -> PointStamped:
        print(prompt, flush=True)
        while not rospy.is_shutdown():
            try:
                msg = self.points.get(timeout=0.2)
            except Empty:
                continue
            x = msg.point.x
            y = msg.point.y
            frame = msg.header.frame_id or self.frame_id
            print(f"  got point: frame={frame}, x={x:.3f}, y={y:.3f}", flush=True)
            return msg
        raise rospy.ROSInterruptException("shutdown while waiting for point")

    def _save(self) -> None:
        if self.backup and os.path.exists(self.output_path):
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            shutil.copy2(self.output_path, f"{self.output_path}.bak_{stamp}")
        _atomic_write_yaml(self.output_path, self.data)
        print(f"[saved] {self.output_path}", flush=True)

    def _publish_markers(self) -> None:
        markers = MarkerArray()
        marker_id = 0
        for name, item in sorted(self.data.items()):
            try:
                ox = float(item["object_x"])
                oy = float(item["object_y"])
                ax = float(item.get("approach_x", ox))
                ay = float(item.get("approach_y", oy))
                yaw = float(item.get("approach_yaw", _yaw_from_approach_to_object((ax, ay), (ox, oy))))
                frame = str(item.get("frame_id", self.frame_id))
            except Exception:
                continue

            obj_marker = Marker()
            obj_marker.header.frame_id = frame
            obj_marker.header.stamp = rospy.Time.now()
            obj_marker.ns = "semantic_objects"
            obj_marker.id = marker_id
            marker_id += 1
            obj_marker.type = Marker.SPHERE
            obj_marker.action = Marker.ADD
            obj_marker.pose.position.x = ox
            obj_marker.pose.position.y = oy
            obj_marker.pose.position.z = 0.12
            obj_marker.pose.orientation.w = 1.0
            obj_marker.scale.x = 0.22
            obj_marker.scale.y = 0.22
            obj_marker.scale.z = 0.22
            obj_marker.color = _color(0.1, 0.65, 1.0, 0.95)
            markers.markers.append(obj_marker)

            arrow = Marker()
            arrow.header.frame_id = frame
            arrow.header.stamp = rospy.Time.now()
            arrow.ns = "semantic_approach"
            arrow.id = marker_id
            marker_id += 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = ax
            arrow.pose.position.y = ay
            arrow.pose.position.z = 0.08
            arrow.pose.orientation.z = math.sin(yaw / 2.0)
            arrow.pose.orientation.w = math.cos(yaw / 2.0)
            arrow.scale.x = 0.45
            arrow.scale.y = 0.06
            arrow.scale.z = 0.06
            arrow.color = _color(0.1, 0.9, 0.35, 0.95)
            markers.markers.append(arrow)

            text = Marker()
            text.header.frame_id = frame
            text.header.stamp = rospy.Time.now()
            text.ns = "semantic_labels"
            text.id = marker_id
            marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = ox
            text.pose.position.y = oy
            text.pose.position.z = 0.45
            text.pose.orientation.w = 1.0
            text.scale.z = 0.28
            text.color = _color(1.0, 1.0, 1.0, 1.0)
            text.text = name
            markers.markers.append(text)

        self.marker_pub.publish(markers)

    def annotate_one(self) -> Optional[str]:
        obj_msg = self._wait_point("\n[1/2] Click the OBJECT CENTER in RViz.")
        obj = (float(obj_msg.point.x), float(obj_msg.point.y))
        frame = obj_msg.header.frame_id or self.frame_id

        while not rospy.is_shutdown():
            name = input("Object name (empty to cancel, q to quit): ").strip()
            if not name:
                print("[cancelled] object point ignored", flush=True)
                return None
            if name.lower() in {"q", "quit", "exit"}:
                raise KeyboardInterrupt
            if name:
                break

        app_msg = self._wait_point("[2/2] Click the ROBOT APPROACH POINT in RViz.")
        approach = (float(app_msg.point.x), float(app_msg.point.y))
        yaw = _yaw_from_approach_to_object(approach, obj)

        old = self.data.get(name, {})
        self.data[name] = {
            **old,
            "object_x": round(obj[0], 4),
            "object_y": round(obj[1], 4),
            "approach_x": round(approach[0], 4),
            "approach_y": round(approach[1], 4),
            "approach_yaw": round(yaw, 6),
            "frame_id": frame,
            "source": "manual_rviz_annotation",
            "updated_at": datetime.now().isoformat(timespec="seconds"),
        }
        self._save()
        self._publish_markers()
        print(
            f"[ok] {name}: object=({obj[0]:.3f},{obj[1]:.3f}), "
            f"approach=({approach[0]:.3f},{approach[1]:.3f}), yaw={yaw:.3f}",
            flush=True,
        )
        return name

    def run(self) -> None:
        print("\nManual semantic annotator")
        print(f"  output: {self.output_path}")
        print("  RViz tool: Publish Point")
        print("  marker topic: /semantic_annotation_markers")
        print("  Ctrl+C to exit\n")
        while not rospy.is_shutdown():
            try:
                self.annotate_one()
            except KeyboardInterrupt:
                print("\n[exit]", flush=True)
                return
            except EOFError:
                print("\n[exit]", flush=True)
                return


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RViz manual semantic map annotator")
    parser.add_argument(
        "--output",
        default="~/.ros/semantic_map.yaml",
        help="YAML output path (default: ~/.ros/semantic_map.yaml)",
    )
    parser.add_argument("--frame-id", default="map", help="default frame id")
    parser.add_argument("--no-backup", action="store_true", help="do not backup existing YAML before saving")
    return parser.parse_args(argv)


def main() -> int:
    args = parse_args(rospy.myargv(argv=sys.argv)[1:])
    rospy.init_node("manual_semantic_annotator", anonymous=False)
    annotator = ManualSemanticAnnotator(
        output_path=args.output,
        frame_id=args.frame_id,
        backup=not args.no_backup,
    )
    annotator.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
