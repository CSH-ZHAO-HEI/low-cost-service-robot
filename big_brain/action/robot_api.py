import math
import os
import subprocess
import sys

from model.llm import JudgeLLM
from std_srvs.srv import Trigger

judge_llm = JudgeLLM()


def _is_manual_mode() -> bool:
    """偵測是否在 E4 manual baseline 模式（judge 被 CompositeTester 替換成 lambda）。

    LLM 模式（E2/E3）下 judge 是 JudgeLLM.judge 的 bound method，有 __self__。
    Manual 模式下 judge 被換成 lambda，沒有 __self__。
    回 True 時保留老行為：中繼點 + nav-fail 直接 raise。
    """
    j = getattr(judge_llm, 'judge', None)
    return not (callable(j) and getattr(j, '__self__', None) is judge_llm)

from config import PROJECT_ROOT
COFFEETABLE_NAME = "CoffeeTable_01_001"
COKE_NAME = "Coke"
BALCONY_TABLE_NAME = "BalconyTable_01_001"
NIGHTSTAND_NAME = "NightStand_01_001"

CUP_SNAP_DOWN = 0.058
BLOCK_HALF = 0.025
ARM_CLAMP_POSE = [0.0, -1.1, 0.66, 1.0, 0.0]
ARM_PUT_POSE = [0.0, -0.5, 0.3, 0.3, 0.0]
BALCONY_PICK_POSE = [0.0, -0.65, 0.60, 1.0, 0.0]
GRIP_OPEN = [0.45, -0.45, 0.45, 0.45, 0.45, 0.45]
GRIP_CLOSE = [-0.45, 0.45, -0.45, -0.45, -0.45, -0.45]
GRIP_JOINTS = ['joint6', 'joint7', 'joint8', 'joint9', 'joint10', 'joint11']

_coke_io_ready = False
_coke_link_pos = {}
_coke_model_pub = None
_coke_arm_pub = None
_coke_gripper_pub = None
_coke_attach_pub = None
_coke_detach_pub = None

# 由 put_down_* 設 True；下一個 nav primitive 啟動前會跑 _flush_post_put_if_pending()
# 做 double-clear（清 → 等 laser 補真實障礙 → 再清），消掉 put-arm 殘影；之後 reset。
_post_put_pending = False


def _flush_post_put_if_pending():
    """If the previous action was a put_down, do a double-clear before this nav.

    Pattern: sleep (wait laser flush arm shadow) → clear → sleep (laser rebuild
    real obstacles) → clear (kill any transitional phantoms) → short settle.
    Total ~1.8s, only paid on put→nav transition.
    """
    global _post_put_pending
    if not _post_put_pending:
        return
    import rospy
    import ros_bridge
    print("[robot_api] pre-nav double-clear (post-put) — flushing arm phantoms")
    rospy.sleep(0.6)
    ros_bridge.clear_costmaps()
    rospy.sleep(0.8)
    ros_bridge.clear_costmaps()
    rospy.sleep(0.4)
    _post_put_pending = False


def move_to_xy(x: float, y: float):
    """移动到指定 map 坐标 (x, y)，通过 move_base 执行。

    nav 失敗（move_base state≠SUCCEEDED）改為走 Judge 流程：
    rule_judge 會以實際 robot XY 與 target 比距離 → 規則失敗 → VLM → AdjustLLM。
    """
    import ros_bridge
    _flush_post_put_if_pending()
    print(f"[robot_api] Moving to ({x:.2f}, {y:.2f})")
    nav_exc = None
    try:
        ros_bridge.move_to_goal(x, y)
    except RuntimeError as e:
        if _is_manual_mode():
            print(f"[robot_api] Navigation error: {e}")
            raise
        print(f"[robot_api] Navigation error: {e} — handing off to Judge")
        nav_exc = e

    judged = judge_llm.judge(
        action_id=1,
        x=x,
        y=y,
        task_text=f"Move to ({x:.2f}, {y:.2f})",
    )
    if nav_exc is not None and not judged:
        raise nav_exc


def _ensure_coke_io():
    global _coke_io_ready, _coke_model_pub, _coke_arm_pub, _coke_gripper_pub
    global _coke_attach_pub, _coke_detach_pub
    if _coke_io_ready:
        return
    import rospy
    from gazebo_msgs.msg import LinkStates, ModelState
    from std_msgs.msg import String
    from trajectory_msgs.msg import JointTrajectory

    def _on_link_states(msg):
        for name, pose in zip(msg.name, msg.pose):
            _coke_link_pos[name] = pose.position

    _coke_model_pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)
    _coke_arm_pub = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=1)
    _coke_gripper_pub = rospy.Publisher("/hand_controller/command", JointTrajectory, queue_size=1)
    _coke_attach_pub = rospy.Publisher("/block_follower/attach", String, queue_size=1)
    _coke_detach_pub = rospy.Publisher("/block_follower/detach", String, queue_size=1)
    rospy.Subscriber("/gazebo/link_states", LinkStates, _on_link_states, queue_size=1)
    rospy.sleep(0.3)
    _coke_io_ready = True


def _set_arm_direct(pose, secs: float = 2.0):
    import rospy
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    _ensure_coke_io()
    traj = JointTrajectory()
    traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
    traj.header.stamp = rospy.Time.now()
    pt = JointTrajectoryPoint()
    pt.positions = pose
    pt.time_from_start = rospy.Duration(secs)
    traj.points = [pt]
    _coke_arm_pub.publish(traj)
    rospy.sleep(secs + 0.2)


def _set_gripper_direct(pos, secs: float = 1.0):
    import rospy
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    _ensure_coke_io()
    traj = JointTrajectory()
    traj.joint_names = GRIP_JOINTS
    traj.header.stamp = rospy.Time.now()
    pt = JointTrajectoryPoint()
    pt.positions = pos
    pt.time_from_start = rospy.Duration(secs)
    traj.points = [pt]
    _coke_gripper_pub.publish(traj)
    rospy.sleep(secs + 0.2)


def _get_gripper_center():
    p7 = _coke_link_pos.get("mini_mec_six_arm::link7")
    p9 = _coke_link_pos.get("mini_mec_six_arm::link9")
    if p7 is None or p9 is None:
        return None
    return ((p7.x + p9.x) / 2.0, (p7.y + p9.y) / 2.0, (p7.z + p9.z) / 2.0 - 0.03)


def _teleport_model(name: str, x: float, y: float, z: float, duration: float = 0.4):
    import time
    import rospy
    from gazebo_msgs.msg import ModelState
    _ensure_coke_io()
    msg = ModelState()
    msg.model_name = name
    msg.reference_frame = "world"
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.position.z = float(z)
    msg.pose.orientation.w = 1.0
    t0 = time.time()
    while time.time() - t0 < duration and not rospy.is_shutdown():
        _coke_model_pub.publish(msg)
        time.sleep(0.005)


def _publish_coke_attach(x: float, y: float, z: float):
    import rospy
    from std_msgs.msg import String
    _ensure_coke_io()
    _coke_attach_pub.publish(String(
        data=f"mini_mec_six_arm::link7,{COKE_NAME},{x:.4f},{y:.4f},{z:.4f}"
    ))
    rospy.sleep(0.3)


def _publish_coke_detach():
    import rospy
    from std_msgs.msg import String
    _ensure_coke_io()
    _coke_detach_pub.publish(String(data="detach"))
    rospy.sleep(0.4)


def pick_up_coke_from_balconytable():
    """C1-tuned Coke pickup copied from the successful E4 manual baseline."""
    import rospy
    import ros_bridge

    print("[robot_api] pick_up_coke_from_balconytable")
    _ensure_coke_io()
    _set_gripper_direct(GRIP_OPEN, secs=0.5)
    _set_arm_direct(ARM_CLAMP_POSE, secs=1.5)

    ax, ay, _ = ros_bridge.get_obj_approach_pos(BALCONY_TABLE_NAME)
    print(f"[robot_api] C1 Coke pick midpoint ({ax:.2f},{ay - 0.25:.2f})")
    ros_bridge.move_to_goal(ax, ay - 0.25)
    _set_arm_direct(BALCONY_PICK_POSE, secs=2.0)
    info = ros_bridge.get_obj_info(BALCONY_TABLE_NAME)
    table_x, table_y = float(info["object_x"]), float(info["object_y"])
    ros_bridge.rotate_to_face(table_x, table_y)
    print("[robot_api] C1 Coke pick drive_forward 0.20m")
    ros_bridge.drive_forward(0.20)
    ros_bridge.rotate_to_face(table_x, table_y)

    c = _get_gripper_center()
    if c is None:
        rx, ry = ros_bridge.get_current_pos()
        ryaw = ros_bridge.get_current_orientation()
        c = (rx + 0.40 * math.cos(ryaw), ry + 0.40 * math.sin(ryaw), 0.35)
    snap = (c[0], c[1], c[2] - CUP_SNAP_DOWN)
    _teleport_model(COKE_NAME, *snap, duration=0.3)
    _set_gripper_direct(GRIP_CLOSE, secs=0.8)
    _publish_coke_attach(*snap)

    ns = ros_bridge.get_obj_info(NIGHTSTAND_NAME)
    ros_bridge.rotate_to_face(float(ns["object_x"]), float(ns["object_y"]))
    _set_arm_direct(ARM_CLAMP_POSE, secs=2.0)
    _publish_coke_detach()
    c2 = _get_gripper_center() or c
    snap2 = (c2[0], c2[1], c2[2] - CUP_SNAP_DOWN)
    _teleport_model(COKE_NAME, *snap2, duration=0.3)
    _publish_coke_attach(*snap2)

    judge_llm.judge(action_id=4, target=COKE_NAME, task_text=f"Pick up {COKE_NAME}")


def put_down_coke_on_nightstand():
    """C1-tuned Coke placement copied from the successful E4 manual baseline."""
    import ros_bridge

    print("[robot_api] put_down_coke_on_nightstand")
    _ensure_coke_io()
    ns = ros_bridge.get_obj_info(NIGHTSTAND_NAME)
    ns_ax = float(ns["approach_x"])
    ns_ay = float(ns["approach_y"])
    ns_ox = float(ns["object_x"])
    ns_oy = float(ns["object_y"])
    ns_px = float(ns.get("place_x", ns_ox))
    ns_py = float(ns.get("place_y", ns_oy))
    ns_sz = float(ns.get("surface_z", 0.3694))

    print(f"[robot_api] C1 Coke place midpoint ({ns_ax:.2f},{ns_ay - 0.15:.2f})")
    ros_bridge.move_to_goal(ns_ax, ns_ay - 0.15)
    ros_bridge.rotate_to_face(ns_ox, ns_oy)
    _set_arm_direct(ARM_PUT_POSE, secs=2.0)
    print("[robot_api] C1 Coke place drive_forward 0.15m")
    ros_bridge.drive_forward(0.15)
    ros_bridge.rotate_to_face(ns_ox, ns_oy)

    _publish_coke_detach()
    _set_gripper_direct(GRIP_OPEN, secs=0.8)
    # Keep the final Coke pose inside the NightStand bbox. The gripper center can
    # drift just outside the front edge after the last drive-forward.
    drop_x = ns_px
    drop_y = ns_py
    _teleport_model(COKE_NAME, drop_x, drop_y, ns_sz + BLOCK_HALF, duration=0.4)

    judge_llm.judge(
        action_id=6,
        target=NIGHTSTAND_NAME,
        dx=0.0,
        dy=0.0,
        task_text=f"Put down on {NIGHTSTAND_NAME} with offset (0.00, 0.00)",
    )


def _run_composite_task(task_id: str):
    """Run a C1-C4 tuned composite helper from the verified manual baseline."""
    import importlib
    c_dir = os.path.join(PROJECT_ROOT, "C1-C4")
    if c_dir not in sys.path:
        sys.path.insert(0, c_dir)

    original_judge = judge_llm.judge
    try:
        comp = importlib.import_module("composite_capability_test")
        tester = comp.CompositeTester(pause_after_reset=0.0)
        runner = getattr(tester, f"run_{task_id.lower()}")
        ok = bool(runner())
        if not ok:
            raise RuntimeError(f"{task_id} tuned composite helper failed")
        return True
    finally:
        judge_llm.judge = original_judge


def run_c1_coke_to_nightstand():
    """C1 tuned helper: Coke from BalconyTable_01_001 -> NightStand_01_001."""
    print("[robot_api] run_c1_coke_to_nightstand")
    pick_up_coke_from_balconytable()
    put_down_coke_on_nightstand()
    return True


def run_c2_red_block_between_blue_yellow():
    """C2 tuned helper: red block from NightStand_01_002 -> between blue/yellow."""
    print("[robot_api] run_c2_red_block_between_blue_yellow")
    return _run_composite_task("C2")


def run_c3_all_ground_blocks_to_trash():
    """C3 tuned helper: collect red/blue/yellow ground blocks into Trash_01_001."""
    print("[robot_api] run_c3_all_ground_blocks_to_trash")
    return _run_composite_task("C3")


def run_c4_sofa_patrol_red_to_trash():
    """C4 tuned helper: N3-style SofaC patrol, pick detected red, drop to trash, resume."""
    print("[robot_api] run_c4_sofa_patrol_red_to_trash")
    return _run_composite_task("C4")


def move_to_obj_by_offset(obj: str, dx: float, dy: float, extra_forward: float = 0.0):
    """移动到指定物体旁边，保持相对偏移 (dx, dy)，對齊朝向物件。

    extra_forward：nav + 對齊完之後額外往前推的距離（公尺），跟 pick/put 同義，
    讓 AdjustLLM 可以用統一介面生成「再靠近一點」的恢復代碼。
    """
    import math
    import ros_bridge
    _flush_post_put_if_pending()
    try:
        ax, ay, ayaw = ros_bridge.get_obj_approach_pos(obj)
        info = ros_bridge.get_obj_info(obj)
        obj_x, obj_y = float(info["object_x"]), float(info["object_y"])
    except KeyError as e:
        print(f"[robot_api] {e}")
        raise

    target_x = ax + dx
    target_y = ay + dy
    print(f"[robot_api] Moving to '{obj}' approach ({ax:.2f},{ay:.2f}) + offset ({dx},{dy})"
          f" → ({target_x:.2f},{target_y:.2f})")
    nav_exc = None
    try:
        ros_bridge.move_to_goal(target_x, target_y, ayaw)
    except RuntimeError as e:
        if _is_manual_mode():
            print(f"[robot_api] Navigation error: {e}")
            raise
        print(f"[robot_api] Navigation error: {e} — handing off to Judge")
        nav_exc = e

    # nav 成功才做對齊 + nudge；失敗則跳過，交由 Judge/AdjustLLM 處理
    if nav_exc is None:
        # 對齊：車頭朝向物件本身（不是 approach 點）
        print(f"[robot_api] aligning to face '{obj}' center ({obj_x:.2f}, {obj_y:.2f})")
        ros_bridge.rotate_to_face(obj_x, obj_y)

        # 補殘差：navigate 因 xy_goal_tolerance 可能停遠 — 直線前進補上
        # 當有 offset 時不補（使用者明確要 offset 位置）
        # 動態物件（red_block）：ros_bridge 算的 approach 在 ARM_REACH_M。
        # 靜態家具（桌子）：yaml 的 approach 在桌邊外（譬如 0.58m），不應該再衝近。
        if info.get("__dynamic__") and abs(dx) < 1e-3 and abs(dy) < 1e-3:
            rx, ry = ros_bridge.get_current_pos()
            cur_dist    = math.hypot(obj_x - rx, obj_y - ry)
            target_dist = math.hypot(obj_x - ax, obj_y - ay)   # approach 到物件的正確距離
            nudge = cur_dist - target_dist
            if nudge > 0.05:                    # 殘差 > 5cm 才補（且只往前補，不會推超過 approach）
                print(f"[robot_api] nudging forward {nudge:.2f}m to close gap "
                      f"(current {cur_dist:.2f} → target {target_dist:.2f})")
                ros_bridge.drive_forward(nudge)

        if extra_forward > 0.0:
            print(f"[robot_api] move_to_obj_by_offset extra_forward {extra_forward:.2f}m")
            ros_bridge.drive_forward(float(extra_forward))

    judged = judge_llm.judge(
        action_id=2,
        target=obj,
        dx=dx,
        dy=dy,
        task_text=f"Move to {obj} with offset ({dx}, {dy})",
    )
    if nav_exc is not None and not judged:
        raise nav_exc


def _call_arm_service(service_name: str, target_name: str):
    """設定 /arm_task/target_name 參數後呼叫指定 Trigger service"""
    import rospy
    rospy.set_param('/arm_task/target_name', target_name)
    rospy.wait_for_service(service_name, timeout=5.0)
    srv  = rospy.ServiceProxy(service_name, Trigger)
    resp = srv()
    if not resp.success:
        raise RuntimeError(f"{service_name} failed: {resp.message}")


def pick_up_xy(x: float, y: float):
    """（无机械臂）接近指定坐标，无法实际拾取。"""
    print(f"[robot_api] pick_up_xy: No arm. Approaching ({x:.2f},{y:.2f}) only.")
    move_to_xy(x, y)

    judge_llm.judge(
        action_id=3,
        x=x, y=y,
        task_text=f"Pick up at ({x:.2f}, {y:.2f})",
    )


def pick_up_obj(item: str, extra_forward: float = 0.0):
    """導航靠近物件（內含對齊朝向）→ 呼叫 /arm/pick 抓取。

    傳 /arm_task/source_z 給 arm_task_server 決定 pick 姿態：
      地面物件 z=0.025 → ARM_CLAMP/PICK_JOINTS（向下）
      桌面物件 z=0.4+  → ARM_PICK_HIGH/PICK_HIGH_LOWER（向前上）
    """
    import rospy
    import ros_bridge
    print(f"[robot_api] pick_up_obj: '{item}'")
    if item == COKE_NAME:
        try:
            info = ros_bridge.get_obj_info(item)
            coke_x = float(info["object_x"])
            coke_y = float(info["object_y"])
            if -1.2 <= coke_x <= 0.2 and coke_y >= 3.2:
                print("[robot_api] Coke is on/near BalconyTable; using C1 tuned pickup")
                pick_up_coke_from_balconytable()
                return
        except Exception as e:
            print(f"[robot_api] Coke tuned pickup check skipped: {e}")

    # 設定 source_z（物件目前所在表面高度）— arm_task_server 讀此 param 選姿態
    source_z = 0.025
    try:
        info = ros_bridge.get_obj_info(item)
        # 動態物件 (red_block 等) → info['surface_z'] 是當前 z（地上 0.025 / 桌上 0.45+）
        # 靜態物件 → 用 yaml 的 surface_z（物件本身的高度，譬如杯子在桌上）
        source_z = float(info.get('surface_z', 0.025))
        rospy.set_param('/arm_task/source_z', source_z)
        print(f"[robot_api]   source_z = {source_z:.3f} ({'table-top' if source_z > 0.15 else 'floor'})")
    except Exception as e:
        print(f"[robot_api]   source_z lookup failed: {e}")

    move_to_obj_by_offset(item, 0.0, 0.0)   # 已含 rotate_to_face + nudge
    forward_d = float(extra_forward)
    if forward_d <= 0.0 and source_z > 0.15:
        forward_d = 0.08
        print(f"[robot_api] table-top pick auto_forward {forward_d:.2f}m")
    elif forward_d > 0.0:
        print(f"[robot_api] pick_up_obj extra_forward {forward_d:.2f}m")
    if forward_d > 0.0:
        ros_bridge.drive_forward(forward_d)
    _call_arm_service('/arm/pick', item)

    judge_llm.judge(
        action_id=4,
        target=item,
        task_text=f"Pick up {item}",
    )


def pick_up_from_coffeetable(item: str = "red_block"):
    """從 CoffeeTable 桌面拿起物件。

    red_block 走已調好的 put_on_table_test.py coffeetable --pick-only 流程；
    其他物件保留通用桌面抓取 fallback，方便 C1 之後接杯子模型。
    """
    import rospy
    import ros_bridge

    print(f"[robot_api] pick_up_from_coffeetable: '{item}'")

    if item == "red_block":
        script = os.path.join(PROJECT_ROOT, "put_on_table_test.py")
        cmd = [sys.executable, script, "coffeetable", "--pick-only"]
        print("[robot_api]   using tuned CoffeeTable pick script")
        subprocess.run(cmd, cwd=PROJECT_ROOT, check=True)
    else:
        print("[robot_api]   generic table-top fallback via CoffeeTable approach")
        move_to_obj_by_offset(COFFEETABLE_NAME, 0.0, 0.0)
        info = ros_bridge.get_obj_info(item)
        rospy.set_param('/arm_task/source_z', float(info.get('surface_z', 0.30)))
        _call_arm_service('/arm/pick', item)

    judge_llm.judge(
        action_id=4,
        target=item,
        task_text=f"Pick up {item} from CoffeeTable",
    )


def put_down_xy(x: float, y: float, surface_z: float = 0.0):
    """把手中物件放到世界座標 (x, y)。

    surface_z 預設 0.0 表示地面；桌面/櫃面放置可傳入對應高度。

    做法：
      1. 計算 approach 點 = (x, y) 沿機器人當下方向往回退 ARM_REACH_M
         （否則 chassis 會壓在 drop 點上，方塊會掉車底）
      2. 導航到 approach 點
      3. 對齊朝向 (x, y)
      4. /arm/put → 手臂伸過去放下，方塊落在 (x, y)
    """
    import math
    import rospy
    import ros_bridge

    print(f"[robot_api] put_down_xy: target ({x:.2f},{y:.2f}), surface_z={surface_z:.3f}")

    # 1. 計算 approach 點（讓手臂正好伸到 (x, y)）
    rx, ry = ros_bridge.get_current_pos()
    dx_dir, dy_dir = x - rx, y - ry
    dist = math.hypot(dx_dir, dy_dir)
    if dist < 0.10:
        # 機器人已經在目標上 → 預設往 -X 退（避免 0/0 除）
        approach_x, approach_y = x - ros_bridge.ARM_REACH_M, y
    else:
        ux, uy = dx_dir / dist, dy_dir / dist
        approach_x = x - ros_bridge.ARM_REACH_M * ux
        approach_y = y - ros_bridge.ARM_REACH_M * uy
    print(f"[robot_api]   approach ({approach_x:.2f}, {approach_y:.2f})  drop ({x:.2f}, {y:.2f})")

    # 2. 導航到 approach
    move_to_xy(approach_x, approach_y)

    # 3. 對齊朝向 drop 點
    ros_bridge.rotate_to_face(x, y)

    # 4. 設 drop 座標 + /arm/put
    rospy.set_param('/arm_task/use_target_xy', True)
    rospy.set_param('/arm_task/target_x', float(x))
    rospy.set_param('/arm_task/target_y', float(y))
    rospy.set_param('/arm_task/surface_z', float(surface_z))
    try:
        _call_arm_service('/arm/put', 'ground')         # prepare_put + drop
    finally:
        rospy.set_param('/arm_task/use_target_xy', False)

    judge_llm.judge(
        action_id=5,
        x=x, y=y,
        task_text=f"Put down at ({x:.2f}, {y:.2f})",
    )


def _surface_between(info_a: dict, info_b: dict) -> float:
    """根據兩個參照物高度推估中間點的放置高度。"""
    za = float(info_a.get("surface_z", 0.025))
    zb = float(info_b.get("surface_z", 0.025))
    if min(za, zb) > 0.10:
        return max(0.0, (za + zb) / 2.0 - 0.025)
    return 0.0


def put_down_between_objs(obj_a: str, obj_b: str):
    """把手中物件放到 obj_a 和 obj_b 的中點。
    跟 put_down_obj_by_offset 同流程：中繼點展臂 → 導航到接近點 → 依夾爪真實 XY 下落。
    """
    import math
    import rospy
    import ros_bridge

    info_a = ros_bridge.get_obj_info(obj_a)
    info_b = ros_bridge.get_obj_info(obj_b)
    mid_x = (float(info_a["object_x"]) + float(info_b["object_x"])) / 2.0
    mid_y = (float(info_a["object_y"]) + float(info_b["object_y"])) / 2.0
    surface_z = _surface_between(info_a, info_b)

    print(f"[robot_api] put_down_between_objs: '{obj_a}' <-> '{obj_b}'")
    print(f"[robot_api]   midpoint ({mid_x:.2f}, {mid_y:.2f}), surface_z={surface_z:.3f}")

    rospy.set_param('/arm_task/surface_z', float(surface_z))
    rospy.set_param('/arm_task/use_target_xy', False)

    # 從當前位置算出接近方向（固定後整段導航都沿同一軸）
    rx, ry = ros_bridge.get_current_pos()
    dx, dy = mid_x - rx, mid_y - ry
    dist = math.hypot(dx, dy)
    ux, uy = (dx / dist, dy / dist) if dist > 1e-3 else (1.0, 0.0)
    ayaw = math.atan2(uy, ux)

    # 1. Manual 模式（E4）保留中繼點：距中點 ARM_REACH_M + 1.2m 處先展臂
    if _is_manual_mode():
        _PRE_EXTRA = 1.2
        pre_x = mid_x - ux * (ros_bridge.ARM_REACH_M + _PRE_EXTRA)
        pre_y = mid_y - uy * (ros_bridge.ARM_REACH_M + _PRE_EXTRA)
        print(f"[robot_api] 中繼點 ({pre_x:.2f}, {pre_y:.2f}) → ARM_PUT")
        ros_bridge.move_to_goal(pre_x, pre_y, ayaw)
        _call_arm_service('/arm/prepare_put', obj_a)

    # 2. 導航到接近點（距中點 ARM_REACH_M）
    approach_x = mid_x - ux * ros_bridge.ARM_REACH_M
    approach_y = mid_y - uy * ros_bridge.ARM_REACH_M
    ros_bridge.move_to_goal(approach_x, approach_y, ayaw)
    ros_bridge.rotate_to_face(mid_x, mid_y)

    # 3. LLM 模式才在 approach 點展臂（manual 已在中繼點展過）
    if not _is_manual_mode():
        _call_arm_service('/arm/prepare_put', obj_a)

    # 3. drop：依夾爪真實 XY，fall animation 在 arm_task_server 裡
    try:
        _call_arm_service('/arm/drop', obj_a)
    finally:
        rospy.set_param('/arm_task/use_target_xy', False)

    # 3.5. drop 完把手臂收回 ARM_CLAMP — 否則手臂維持 ARM_PUT 伸出去，
    # laser 會掃到自己手臂寫進 costmap 變幻影障礙，下一次 nav 就 TEB state=4。
    print("[robot_api] restoring arm to ARM_CLAMP after put")
    _set_arm_direct(ARM_CLAMP_POSE, secs=1.2)

    # put 完只做一次 clear 並標記 flag。實際 double-clear 延遲到下一個 nav primitive 啟動前
    # 才執行（見 move_to_xy / move_to_obj_by_offset 開頭的 _flush_post_put_if_pending），這樣
    # 不會延遲 Judge，也只在「put → nav」轉場時才付那 ~1.8s。
    print("[robot_api] post-put: clear + mark for pre-nav double-clear")
    ros_bridge.clear_costmaps()
    global _post_put_pending
    _post_put_pending = True

    judge_llm.judge(
        action_id=6,
        target=obj_a, dx=0.0, dy=0.0,
        task_text=f"Put down between {obj_a} and {obj_b}",
    )


def put_down_obj_by_offset(target: str, dx: float, dy: float, extra_forward: float = 0.0):
    """放下手中物件到 target 旁。

    序列（抄 pick_deliver_teb.py 的 Step A/B/C）：
      1. 設 /arm_task/* 參數（surface_z, target_x/y）
      2. move_to_obj_by_offset（含對齊 + nudge）
      3. /arm/prepare_put → 手臂抬到 ARM_PUT 姿態（block_follower 仍跟著）
      4. rotate_to_face → 抬完臂再對齊一次（臂的重心改變後更穩）
      5. trash 類額外 drive_forward → 貼桶口
      6. /arm/drop → detach follower + 開夾爪 + teleport 方塊到目標
    """
    import rospy
    import ros_bridge
    print(f"[robot_api] put_down_obj_by_offset: '{target}' offset ({dx},{dy})")
    if target == NIGHTSTAND_NAME and abs(dx) < 1e-3 and abs(dy) < 1e-3:
        try:
            coke_info = ros_bridge.get_obj_info(COKE_NAME)
            coke_z = float(coke_info.get("surface_z", coke_info.get("object_z", 0.0)))
            if coke_z > 0.10:
                print("[robot_api] NightStand target with Coke active; using C1 tuned placement")
                put_down_coke_on_nightstand()
                return
        except Exception as e:
            print(f"[robot_api] Coke tuned placement check skipped: {e}")

    # 1. 設定 surface_z + target_x/y（arm_task_server.handle_drop 會用）
    try:
        info = ros_bridge.get_obj_info(target)
        obj_x = float(info["object_x"])
        obj_y = float(info["object_y"])
        place_x = float(info.get("place_x", obj_x))   # yaml 可能有 place_x 區分（譬如桌面內側）
        place_y = float(info.get("place_y", obj_y))
        if "surface_z" in info:
            rospy.set_param("/arm_task/surface_z", float(info["surface_z"]))
            print(f"[robot_api] surface_z set to {info['surface_z']:.3f}")
        rospy.set_param("/arm_task/target_x", place_x)
        rospy.set_param("/arm_task/target_y", place_y)
    except Exception as e:
        print(f"[robot_api] 無法設定 surface_z/target_xy：{e}")
        obj_x = obj_y = None

    # 2. Manual 模式（E4）保留中繼點：往後退 1.2m 在空曠處展臂避免穿模
    #    LLM 模式（E2/E3）：跳過中繼點，approach 點直接展臂，由 LLM 負責擺位
    if _is_manual_mode():
        _PRE_EXTRA = 1.2  # m
        try:
            ax, ay, ayaw = ros_bridge.get_obj_approach_pos(target)
            if obj_x is not None:
                app_dist = math.hypot(ax - obj_x, ay - obj_y)
                if app_dist > 1e-3:
                    ux = (ax - obj_x) / app_dist
                    uy = (ay - obj_y) / app_dist
                    pre_x = obj_x + ux * (app_dist + _PRE_EXTRA)
                    pre_y = obj_y + uy * (app_dist + _PRE_EXTRA)
                    print(f"[robot_api] 中繼點 ({pre_x:.2f}, {pre_y:.2f}) → face target → ARM_PUT")
                    ros_bridge.move_to_goal(pre_x, pre_y, ayaw)
                    print(f"[robot_api] 中繼點先朝向 '{target}' ({obj_x:.2f}, {obj_y:.2f})")
                    ros_bridge.rotate_to_face(obj_x, obj_y)
                    _call_arm_service('/arm/prepare_put', target)
        except Exception as _e:
            print(f"[robot_api] 中繼點失敗，跳過：{_e}")

    # 3. 導航到最終 approach（內含對齊 + nudge）
    move_to_obj_by_offset(target, dx, dy)

    # 3.5. LLM 模式才在 approach 點展臂（manual 已在中繼點展過）
    if not _is_manual_mode():
        _call_arm_service('/arm/prepare_put', target)

    # 桌類：approach 點由 yaml 決定可能不夠近，補推讓夾爪伸到桌面上方
    is_table = any(k in target.lower() for k in ('table', 'desk', 'bench', 'nightstand'))
    if is_table:
        table_forward = 0.12 if 'nightstand' in target.lower() else 0.08
        print(f"[robot_api] table — drive_forward {table_forward:.2f}m to reach over surface")
        ros_bridge.drive_forward(table_forward)

    if extra_forward > 0.0:
        print(f"[robot_api] put_down_obj_by_offset extra_forward {extra_forward:.2f}m")
        ros_bridge.drive_forward(float(extra_forward))

    # 4. 抬完臂後重新對齊（重心改變了，可能微偏）
    if obj_x is not None and obj_y is not None:
        print(f"[robot_api] re-aligning after arm raise to face '{target}' ({obj_x:.2f}, {obj_y:.2f})")
        ros_bridge.rotate_to_face(obj_x, obj_y)

    # 5. 落點 XY 選擇
    #    Manual 模式（E4）：trash 用 yaml target_x/y（瞬移到桶心，保留 baseline 100% 命中）
    #    LLM 模式（E2/E3）：一律夾爪實際 XY，靠 LLM 自行規劃車身擺位
    is_trash = ('trash' in target.lower()) or ('bin' in target.lower())
    if _is_manual_mode() and is_trash:
        print("[robot_api] trash detected (manual mode) — driving forward 0.20m + use_target_xy")
        ros_bridge.drive_forward(0.20)
        rospy.set_param("/arm_task/use_target_xy", True)
    else:
        rospy.set_param("/arm_task/use_target_xy", False)

    # 6. 開夾爪 + drop
    try:
        _call_arm_service('/arm/drop', target)
    finally:
        rospy.set_param("/arm_task/use_target_xy", False)

    # 6.5. Manual 模式 trash：drop 完倒車 0.40m 離開桶緣（配對 step 5 的前進 + 留間距）
    if _is_manual_mode() and is_trash:
        print("[robot_api] trash detected (manual mode) — backing off 0.40m to clear bin edge")
        ros_bridge.drive_forward(-0.40)

    # 6.55. drop 完把手臂收回 ARM_CLAMP — 否則手臂維持 ARM_PUT 伸出去，
    # laser 會掃到自己手臂寫進 costmap 變幻影障礙，下一次 nav 就 TEB state=4。
    print("[robot_api] restoring arm to ARM_CLAMP after put")
    _set_arm_direct(ARM_CLAMP_POSE, secs=1.2)

    # 6.6. put 完只做一次 clear 並標記 flag。實際 double-clear 延遲到下一個 nav primitive
    # 啟動前才執行（見 _flush_post_put_if_pending），這樣不會延遲 Judge，也只在
    # 「put → nav」轉場時才付那 ~1.8s 等待，patrol 連續 move_to_xy 不受影響。
    print("[robot_api] post-put: clear + mark for pre-nav double-clear")
    ros_bridge.clear_costmaps()
    global _post_put_pending
    _post_put_pending = True

    judge_llm.judge(
        action_id=6,
        target=target, dx=dx, dy=dy,
        task_text=f"Put down on {target} with offset ({dx:.2f}, {dy:.2f})",
    )
