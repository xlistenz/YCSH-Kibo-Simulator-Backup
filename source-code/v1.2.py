#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""7th Kibo-RPC  –  optimized user program (Int-Ball2).

Strategy
  1. Read all CheckPoints (position + target attitude + margin bands) and the
     goalpoint_id from /mission/get_checkpoints.
  2. The goal is itself a checkpoint (JudgeReport: "goal checkpoint previously
     passed"), so it is forced to be the LAST node of the tour.
  3. Exact optimal tour (Held-Karp DP, <=12 nodes) with a time-like cost
     = translation [m] + ROT_WEIGHT * rotation [rad]  (start -> ... -> goal).
  4. At every checkpoint: move with the exact target quaternion, verify the
     real pose from /sensor_fusion/navigation against the Lv3 bands (re-aim if
     needed), hold >= 1 s, then send checkpoint_passed.
  5. Report mission_finished immediately after the goal is passed (max time
     bonus).  If time runs short, remaining optional checkpoints are skipped
     so that the goal + finish report are never lost.
"""

import json
import math
import time

import actionlib
import rospy
from geometry_msgs.msg import PoseStamped
from platform_msgs.msg import UserLogic
from platform_msgs.srv import StopProcessingUserNode, StopProcessingUserNodeResponse
from krpc_ib2_msgs.srv import MissionStart, MissionFinished, CheckPoints
from krpc_ib2_msgs.msg import CheckpointPassedAction, CheckpointPassedGoal
from ib2_msgs.msg import CtlCommandAction, CtlCommandGoal, CtlCommandResult, CtlStatusType

try:
    from ib2_msgs.msg import Navigation
except Exception:  # pragma: no cover - verification is optional
    Navigation = None


# ============================ pure helper functions ============================
def dist3(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def quat_angle(q1, q2):
    """Smallest rotation angle [rad] between two unit quaternions (x,y,z,w)."""
    d = abs(sum(q1[i] * q2[i] for i in range(4)))
    return 2.0 * math.acos(min(1.0, d))


def normalize_q(q):
    n = math.sqrt(sum(v * v for v in q))
    if n < 1e-9:
        raise ValueError('zero quaternion')
    return tuple(v / n for v in q)


def leg_cost(a, b, rot_weight):
    return dist3(a['position'], b['position']) + rot_weight * quat_angle(a['quaternion'], b['quaternion'])


def plan_route(start, cps, goal_idx, rot_weight):
    """Exact shortest open tour start -> (all cps) -> cps[goal_idx] (if given).

    Returns list of indices into cps.
    """
    n = len(cps)
    if n == 0:
        return []
    free = [i for i in range(n) if i != goal_idx]
    m = len(free)
    if m == 0:
        return [goal_idx]
    c0 = [leg_cost(start, cps[i], rot_weight) for i in free]
    cc = [[leg_cost(cps[i], cps[j], rot_weight) for j in free] for i in free]
    cg = [leg_cost(cps[i], cps[goal_idx], rot_weight) for i in free] if goal_idx is not None else [0.0] * m
    INF = float('inf')
    full = (1 << m) - 1
    dp = [[INF] * m for _ in range(1 << m)]
    par = [[-1] * m for _ in range(1 << m)]
    for i in range(m):
        dp[1 << i][i] = c0[i]
    for mask in range(1, full + 1):
        for last in range(m):
            base = dp[mask][last]
            if base == INF:
                continue
            for nx in range(m):
                if mask & (1 << nx):
                    continue
                nm = mask | (1 << nx)
                v = base + cc[last][nx]
                if v < dp[nm][nx]:
                    dp[nm][nx] = v
                    par[nm][nx] = last
    best = min(range(m), key=lambda i: dp[full][i] + cg[i])
    order, mask, last = [], full, best
    while last != -1:
        order.append(free[last])
        p = par[mask][last]
        mask ^= (1 << last)
        last = p
    order.reverse()
    if goal_idx is not None:
        order.append(goal_idx)
    return order


# ================================= user node ==================================
class UserTemplate:
    INTERNAL_DEADLINE_SEC = 288.0     # hard stop (limit is 300 s)
    SERVICE_WAIT_SEC = 10.0
    POLL_SEC = 0.05
    DWELL_SEC = 1.0                   # rulebook: keep conditions for 1 s
    DWELL_MARGIN = 0.15
    MAX_REAIM = 2                     # extra move commands per checkpoint
    ROT_WEIGHT = 0.5                  # metre-equivalent of 1 rad of rotation
    SAFETY_FACTOR = 0.8               # use 80 % of the Lv3 band for verification
    START_POSITION = (0.2, 0.0, 0.0)
    START_QUATERNION = (0.0, 0.0, 0.0, 1.0)
    PRIOR_SEC_PER_COST = 10.0         # pessimistic prior, replaced by measurement

    def __init__(self):
        rospy.init_node('krpc_user')
        self._command = actionlib.SimpleActionClient('/ctl/command', CtlCommandAction)
        self._cp_client = actionlib.SimpleActionClient('/event/checkpoint_passed', CheckpointPassedAction)
        self._mission_start = rospy.ServiceProxy('/event/mission_start', MissionStart)
        self._mission_finished = rospy.ServiceProxy('/event/mission_finished', MissionFinished)
        self._checkpoints = rospy.ServiceProxy('/mission/get_checkpoints', CheckPoints)

        # DO NOT REMOVE: required by the simulator runtime.
        self._subscriber_start = rospy.Subscriber('/ib2_user/start', UserLogic, self._callback_start)
        self._stop_srv = rospy.Service('/ib2_user/stop', StopProcessingUserNode, self._stop_processing)

        self._nav = None
        if Navigation is not None:
            rospy.Subscriber('/sensor_fusion/navigation', Navigation, self._cb_nav, queue_size=1)

        self._started = False
        self._action_active = False
        self._cp_active = False
        self._stop_requested = False
        self._t0 = None
        self._sec_per_cost = self.PRIOR_SEC_PER_COST
        rospy.set_param('/ib2_user/ready', True)  # DO NOT REMOVE
        rospy.loginfo('Kibo-RPC optimized user node ready')

    # ------------------------------- utilities --------------------------------
    def _cb_nav(self, msg):
        self._nav = msg

    def _elapsed(self):
        return 0.0 if self._t0 is None else time.monotonic() - self._t0

    def _time_left(self):
        return self.INTERNAL_DEADLINE_SEC - self._elapsed()

    def _check(self, what):
        if self._stop_requested or rospy.is_shutdown():
            raise RuntimeError('stopped during ' + what)
        if self._time_left() <= 0:
            raise TimeoutError('deadline reached during ' + what)

    # ------------------------------ data loading ------------------------------
    def _load(self):
        rospy.wait_for_service('/mission/get_checkpoints', timeout=self.SERVICE_WAIT_SEC)
        res = self._checkpoints()
        if not res.success:
            raise RuntimeError('get_checkpoints failed: ' + res.message)
        payload = json.loads(res.message)
        cps = []
        for it in payload.get('checkpoints', []):
            att = it.get('attitude') or {}
            q = att.get('quaternion') if isinstance(att, dict) else None
            if q is None:
                q = it.get('quaternion', [0, 0, 0, 1])
            pos_r = 0.1
            bands = it.get('score_bands') or []
            if bands:
                pos_r = min(float(b['radius']) for b in bands)
            cen = rol = 10.0
            if isinstance(att, dict):
                if att.get('centering_bands'):
                    cen = min(float(b['angle']) for b in att['centering_bands'])
                if att.get('roll_bands'):
                    rol = min(float(b['angle']) for b in att['roll_bands'])
            cps.append({
                'id': int(it['checkpoint_id']),
                'position': tuple(float(v) for v in it['position'][:3]),
                'quaternion': normalize_q([float(v) for v in q[:4]]),
                'pos_tol': pos_r * self.SAFETY_FACTOR,
                'ang_tol': math.radians(min(cen, rol)) * self.SAFETY_FACTOR,
            })
        goal_id = payload.get('goalpoint_id')
        return cps, goal_id

    # ------------------------------- ROS actions ------------------------------
    def _move(self, cp, label):
        self._check('move ' + label)
        if not self._command.wait_for_server(rospy.Duration(self.SERVICE_WAIT_SEC)):
            raise RuntimeError('/ctl/command unavailable')
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = cp['position']
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = cp['quaternion']
        goal = CtlCommandGoal(target=pose, type=CtlStatusType(type=CtlStatusType.MOVE_TO_ABSOLUTE_TARGET))
        self._action_active = True
        try:
            self._command.send_goal(goal)
            while not rospy.is_shutdown():
                self._check('move ' + label)
                if self._command.wait_for_result(rospy.Duration(self.POLL_SEC * 4)):
                    r = self._command.get_result()
                    return r is not None and r.type == CtlCommandResult.TERMINATE_SUCCESS
            return False
        finally:
            self._action_active = False

    def _errors(self, cp):
        """(position error [m], attitude error [rad]) from navigation, or None."""
        nav = self._nav
        if nav is None:
            return None
        p = nav.pose.pose.position
        o = nav.pose.pose.orientation
        try:
            q = normalize_q([o.x, o.y, o.z, o.w])
        except ValueError:
            return None
        return dist3((p.x, p.y, p.z), cp['position']), quat_angle(q, cp['quaternion'])

    def _settle(self, cp):
        """Wait until pose is inside Lv3 margins continuously for DWELL_SEC."""
        need = self.DWELL_SEC + self.DWELL_MARGIN
        if self._nav is None:                      # no feedback -> plain dwell
            end = time.monotonic() + need
            while time.monotonic() < end:
                self._check('dwell')
                time.sleep(self.POLL_SEC)
            return True
        since, give_up = None, time.monotonic() + 4.0
        while time.monotonic() < give_up:
            self._check('settle')
            e = self._errors(cp)
            ok = e is not None and e[0] <= cp['pos_tol'] and e[1] <= cp['ang_tol']
            now = time.monotonic()
            if ok:
                since = since or now
                if now - since >= need:
                    return True
            else:
                since = None
            time.sleep(self.POLL_SEC)
        return False

    def _notify_passed(self, cid):
        self._check('checkpoint notify')
        if not self._cp_client.wait_for_server(rospy.Duration(self.SERVICE_WAIT_SEC)):
            raise RuntimeError('checkpoint action unavailable')
        goal = CheckpointPassedGoal()
        goal.checkpoint_id = int(cid)
        self._cp_active = True
        try:
            self._cp_client.send_goal(goal)
            while not rospy.is_shutdown():
                self._check('checkpoint notify')
                if self._cp_client.wait_for_result(rospy.Duration(0.1)):
                    r = self._cp_client.get_result()
                    rospy.loginfo('checkpoint %s -> %s', cid, r)
                    return bool(r is not None and r.success)
            return False
        finally:
            self._cp_active = False

    def _visit(self, cp):
        """Move, re-aim if needed, hold 1 s, notify.  Returns True if accepted."""
        cid = cp['id']
        t_start = time.monotonic()
        for attempt in range(self.MAX_REAIM + 1):
            if not self._move(cp, 'cp%d' % cid):
                rospy.logwarn('move to cp%d failed (attempt %d)', cid, attempt)
                continue
            if self._settle(cp):
                break
            rospy.logwarn('cp%d not within Lv3 margins %s, re-aiming', cid, self._errors(cp))
        accepted = self._notify_passed(cid)
        if not accepted and self._time_left() > 20:
            rospy.logwarn('cp%d rejected, one retry', cid)
            self._move(cp, 'cp%d retry' % cid)
            self._settle(cp)
            accepted = self._notify_passed(cid)
        return accepted, time.monotonic() - t_start

    # --------------------------------- mission --------------------------------
    def _run(self, msg):
        self._t0 = time.monotonic()
        finished_sent = False
        try:
            rospy.wait_for_service('/event/mission_start', timeout=self.SERVICE_WAIT_SEC)
            if not self._mission_start().success:
                raise RuntimeError('mission_start rejected')
            cps, goal_id = self._load()
            ids = [c['id'] for c in cps]
            goal_idx = ids.index(goal_id) if goal_id in ids else None
            if goal_idx is None:
                rospy.logwarn('goalpoint_id=%s is not in the checkpoint list %s', goal_id, ids)
            start = {'position': self.START_POSITION, 'quaternion': self.START_QUATERNION}
            order = plan_route(start, cps, goal_idx, self.ROT_WEIGHT)
            rospy.loginfo('route: %s', [cps[i]['id'] for i in order])

            prev = start
            for k, i in enumerate(order):
                cp = cps[i]
                is_goal = (i == goal_idx)
                # ---- time guard: keep enough time for the goal + report ----
                if not is_goal and goal_idx is not None:
                    need = (leg_cost(prev, cp, self.ROT_WEIGHT) + leg_cost(cp, cps[goal_idx], self.ROT_WEIGHT)
                            ) * self._sec_per_cost + 2 * (self.DWELL_SEC + 1.0) + 6.0
                    if self._time_left() < need:
                        rospy.logwarn('time short (%.0fs left): skip cp%d', self._time_left(), cp['id'])
                        continue
                cost = leg_cost(prev, cp, self.ROT_WEIGHT)
                ok, dt = self._visit(cp)
                if cost > 0.3:   # learn the real speed (exclude dwell time)
                    meas = max(0.1, dt - self.DWELL_SEC - 0.5) / cost
                    self._sec_per_cost = meas * 1.2 if k == 0 else max(meas * 1.2, 0.5 * self._sec_per_cost)
                prev = cp
                if is_goal:
                    break
            rospy.wait_for_service('/event/mission_finished', timeout=self.SERVICE_WAIT_SEC)
            finished_sent = bool(self._mission_finished().success)
            rospy.loginfo('mission_finished sent=%s at %.1fs', finished_sent, self._elapsed())
        except Exception as exc:
            rospy.logerr('mission flow stopped: %s', exc)
            if not finished_sent and not rospy.is_shutdown() and not isinstance(exc, TimeoutError):
                try:
                    self._mission_finished()
                except Exception:
                    pass

    def _callback_start(self, msg):
        if self._started:
            return
        self._started = True
        self._run(msg)

    def _stop_processing(self, request):
        try:
            self._stop_requested = True
            if self._action_active:
                self._command.cancel_goal()
            if self._cp_active:
                self._cp_client.cancel_goal()
            return StopProcessingUserNodeResponse(result=StopProcessingUserNodeResponse.SUCCESS)
        except Exception as exc:
            rospy.logerr('stop failed: %s', exc)
            return StopProcessingUserNodeResponse(result=StopProcessingUserNodeResponse.ERROR)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    UserTemplate().run()
