#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""7th Kibo-RPC user program.

The program uses only the public ROS interfaces included in the Beta sample.
Checkpoint positions/attitudes are taken from /mission/get_checkpoints.
GoalPoint pose is configurable because the Beta API currently returns only
Goal.checkpoint_id, not the GoalPoint position/attitude.
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


class UserTemplate:
    # Rulebook: 5-minute mission. Keep a small reserve for the finish report.
    MISSION_LIMIT_SEC = 300.0
    INTERNAL_DEADLINE_SEC = 285.0
    SERVICE_WAIT_SEC = 10.0
    ACTION_WAIT_SEC = 10.0
    POLL_SEC = 0.2
    START_POSITION = (0.2, 0.0, 0.0)  # Rulebook, Docking Station
    START_QUATERNION = (0.0, 0.0, 0.0, 1.0)

    def __init__(self):
        rospy.init_node('krpc_user')

        self._command = actionlib.SimpleActionClient('/ctl/command', CtlCommandAction)
        self._checkpoint_client = actionlib.SimpleActionClient(
            '/event/checkpoint_passed', CheckpointPassedAction
        )

        self._mission_start = rospy.ServiceProxy('/event/mission_start', MissionStart)
        self._mission_finished = rospy.ServiceProxy('/event/mission_finished', MissionFinished)
        self._checkpoints = rospy.ServiceProxy('/mission/get_checkpoints', CheckPoints)

        # DO NOT REMOVE: required by the simulator runtime.
        self._subscriber_start = rospy.Subscriber(
            '/ib2_user/start', UserLogic, self._callback_start
        )
        self._stop_processing_server = rospy.Service(
            '/ib2_user/stop', StopProcessingUserNode, self._stop_processing
        )

        self._started = False
        self._action_active = False
        self._checkpoint_active = False
        self._stop_requested = False
        self._mission_started_at = None
        rospy.set_param('/ib2_user/ready', True)  # DO NOT REMOVE
        rospy.loginfo('Kibo-RPC user node ready')

    # ----------------------------- data parsing -----------------------------
    @staticmethod
    def _checkpoint_id(item):
        for key in ('checkpoint_id', 'checkpointid', 'id'):
            if key in item:
                return int(item[key])
        raise ValueError('checkpoint has no checkpoint_id')

    @staticmethod
    def _as_vector(value, length, field_name):
        if isinstance(value, dict):
            # Accept ROS-like {x,y,z[,w]} objects if supplied by a future API.
            keys = ('x', 'y', 'z', 'w')[:length]
            if all(k in value for k in keys):
                value = [value[k] for k in keys]
        if not isinstance(value, (list, tuple)) or len(value) < length:
            raise ValueError('invalid %s: expected %d values' % (field_name, length))
        result = tuple(float(value[i]) for i in range(length))
        if not all(math.isfinite(v) for v in result):
            raise ValueError('non-finite value in %s' % field_name)
        return result

    @classmethod
    def _normalize_quaternion(cls, value, field_name):
        q = cls._as_vector(value, 4, field_name)
        norm = math.sqrt(sum(v * v for v in q))
        if norm < 1e-8:
            raise ValueError('%s has zero norm' % field_name)
        return tuple(v / norm for v in q)

    @classmethod
    def _parse_target(cls, item, label):
        if not isinstance(item, dict):
            raise ValueError('%s must be a JSON object' % label)

        position = item.get('position')
        if position is None:
            raise ValueError('%s has no position' % label)
        position = cls._as_vector(position, 3, label + '.position')

        attitude = item.get('attitude')
        quaternion = None
        if isinstance(attitude, dict):
            quaternion = attitude.get('quaternion')
        elif isinstance(attitude, (list, tuple)):
            quaternion = attitude
        if quaternion is None:
            # Some API revisions may expose the target orientation directly.
            quaternion = item.get('quaternion')
        if quaternion is None:
            raise ValueError('%s has no target attitude quaternion' % label)
        quaternion = cls._normalize_quaternion(quaternion, label + '.quaternion')

        return {
            'checkpoint_id': cls._checkpoint_id(item) if label.startswith('checkpoint') else None,
            'position': position,
            'quaternion': quaternion,
        }

    def _get_task_data(self):
        rospy.wait_for_service('/mission/get_checkpoints', timeout=self.SERVICE_WAIT_SEC)
        response = self._checkpoints()
        if not response.success:
            raise RuntimeError('get_checkpoints failed: %s' % response.message)

        payload = json.loads(response.message)
        if not isinstance(payload, dict):
            raise ValueError('checkpoint response root must be an object')
        raw_checkpoints = payload.get('checkpoints', payload.get('Checkpoints', []))
        if not isinstance(raw_checkpoints, list) or not raw_checkpoints:
            raise ValueError('no checkpoints were returned')

        checkpoints = []
        seen_ids = set()
        for index, item in enumerate(raw_checkpoints, start=1):
            target = self._parse_target(item, 'checkpoint_%d' % index)
            if target['checkpoint_id'] in seen_ids:
                raise ValueError('duplicate checkpoint id: %s' % target['checkpoint_id'])
            seen_ids.add(target['checkpoint_id'])
            checkpoints.append(target)

        # Beta API currently omits Goal position/attitude. Accept a future `goal`
        # object if present, otherwise read explicitly configured ROS parameters.
        goal_raw = payload.get('goal', payload.get('Goal'))
        goal = None
        if isinstance(goal_raw, dict) and goal_raw.get('position') is not None:
            goal = self._parse_target(goal_raw, 'goal')

        if goal is None and rospy.has_param('~goal_position'):
            goal_position = rospy.get_param('~goal_position')
            goal_quaternion = rospy.get_param('~goal_quaternion', None)
            if goal_quaternion is None:
                raise ValueError('~goal_position is set but ~goal_quaternion is missing')
            goal = {
                'checkpoint_id': None,
                'position': self._as_vector(goal_position, 3, '~goal_position'),
                'quaternion': self._normalize_quaternion(goal_quaternion, '~goal_quaternion'),
            }

        return checkpoints, goal

    # ----------------------------- route planning ----------------------------
    @staticmethod
    def _distance(a, b):
        return math.sqrt(sum((a['position'][i] - b['position'][i]) ** 2 for i in range(3)))

    @classmethod
    def _plan_route(cls, checkpoints, start_position, goal=None):
        """Exact shortest open route for up to 12 checkpoints (Held-Karp DP).

        If a GoalPoint pose is known, its final leg is included in route cost.
        """
        n = len(checkpoints)
        if n == 0:
            return []
        start = {'position': tuple(start_position)}
        start_cost = [cls._distance(start, cp) for cp in checkpoints]
        between = [[cls._distance(checkpoints[i], checkpoints[j]) for j in range(n)]
                   for i in range(n)]
        goal_cost = [cls._distance(cp, goal) for cp in checkpoints] if goal else [0.0] * n

        # dp[(mask,last)] = (cost, previous_index)
        dp = {}
        for i in range(n):
            dp[(1 << i, i)] = (start_cost[i], -1)

        full_mask = (1 << n) - 1
        for mask in range(1, full_mask + 1):
            for last in range(n):
                state = dp.get((mask, last))
                if state is None:
                    continue
                base_cost = state[0]
                remaining = full_mask ^ mask
                while remaining:
                    bit = remaining & -remaining
                    nxt = bit.bit_length() - 1
                    new_mask = mask | bit
                    new_cost = base_cost + between[last][nxt]
                    old = dp.get((new_mask, nxt))
                    if old is None or new_cost < old[0]:
                        dp[(new_mask, nxt)] = (new_cost, last)
                    remaining ^= bit

        best_last = min(
            range(n),
            key=lambda i: dp[(full_mask, i)][0] + goal_cost[i]
        )
        route_indices = []
        mask, last = full_mask, best_last
        while last != -1:
            route_indices.append(last)
            cost, previous = dp[(mask, last)]
            mask ^= (1 << last)
            last = previous
        route_indices.reverse()
        total = dp[(full_mask, best_last)][0] + goal_cost[best_last]
        rospy.loginfo('planned route ids=%s estimated_distance=%.3f',
                      [checkpoints[i]['checkpoint_id'] for i in route_indices], total)
        return [checkpoints[i] for i in route_indices]

    # ------------------------------ ROS actions ------------------------------
    def _time_left(self):
        if self._mission_started_at is None:
            return self.INTERNAL_DEADLINE_SEC
        return self.INTERNAL_DEADLINE_SEC - (time.monotonic() - self._mission_started_at)

    def _check_time(self, operation):
        if self._stop_requested or rospy.is_shutdown():
            raise RuntimeError('stopped during %s' % operation)
        if self._time_left() <= 0:
            raise TimeoutError('internal 5-minute mission deadline reached during %s' % operation)

    def _notify_mission_start(self):
        rospy.wait_for_service('/event/mission_start', timeout=self.SERVICE_WAIT_SEC)
        response = self._mission_start()
        rospy.loginfo('mission_start: success=%s message=%s', response.success, response.message)
        return bool(response.success)

    def _hold_at_checkpoint(self, seconds=1.1):
        """Remain at the commanded target for the rulebook's 1-second window.

        This dwell does not independently verify camera centering; the simulator's
        checkpoint judge remains authoritative for the actual score.
        """
        end_time = time.monotonic() + seconds
        while time.monotonic() < end_time:
            self._check_time('checkpoint dwell')
            time.sleep(min(0.1, max(0.0, end_time - time.monotonic())))

    def _notify_checkpoint_passed(self, checkpoint_id):
        self._check_time('checkpoint notification')
        if not self._checkpoint_client.wait_for_server(rospy.Duration(self.SERVICE_WAIT_SEC)):
            raise RuntimeError('checkpoint action server is unavailable')
        goal = CheckpointPassedGoal()
        goal.checkpoint_id = int(checkpoint_id)
        self._checkpoint_active = True
        try:
            self._checkpoint_client.send_goal(goal)
            while not rospy.is_shutdown():
                self._check_time('checkpoint notification')
                if self._checkpoint_client.wait_for_result(rospy.Duration(self.POLL_SEC)):
                    result = self._checkpoint_client.get_result()
                    if result is None or not result.success:
                        raise RuntimeError('checkpoint %s notification failed: %s' %
                                           (checkpoint_id, getattr(result, 'message', 'no result')))
                    rospy.loginfo('checkpoint %s notification accepted: %s',
                                  checkpoint_id, result.message)
                    return True
            raise RuntimeError('ROS shutdown during checkpoint notification')
        finally:
            self._checkpoint_active = False

    def _move_to_target(self, target, label):
        self._check_time('move to ' + label)
        if not self._command.wait_for_server(rospy.Duration(self.SERVICE_WAIT_SEC)):
            raise RuntimeError('movement action server is unavailable')

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = target['position']
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = target['quaternion']

        command_type = CtlStatusType(type=CtlStatusType.MOVE_TO_ABSOLUTE_TARGET)
        goal = CtlCommandGoal(target=pose, type=command_type)
        rospy.loginfo('moving to %s: position=%s quaternion=%s',
                      label, target['position'], target['quaternion'])

        self._action_active = True
        try:
            self._command.send_goal(goal)
            while not rospy.is_shutdown():
                self._check_time('move to ' + label)
                if self._command.wait_for_result(rospy.Duration(self.POLL_SEC)):
                    result = self._command.get_result()
                    if result is None or result.type != CtlCommandResult.TERMINATE_SUCCESS:
                        raise RuntimeError('movement to %s did not terminate successfully: %s' %
                                           (label, result))
                    rospy.loginfo('arrived at %s', label)
                    return True
            raise RuntimeError('ROS shutdown during move to ' + label)
        finally:
            self._action_active = False

    def _notify_mission_finished(self):
        self._check_time('mission finish report')
        rospy.wait_for_service('/event/mission_finished', timeout=self.SERVICE_WAIT_SEC)
        response = self._mission_finished()
        rospy.loginfo('mission_finished: success=%s message=%s',
                      response.success, response.message)
        return bool(response.success)

    # ------------------------------- main flow -------------------------------
    def _run_user_flow(self, msg):
        self._mission_started_at = time.monotonic()
        try:
            if not self._notify_mission_start():
                raise RuntimeError('mission start notification was rejected')

            checkpoints, goal = self._get_task_data()
            rospy.loginfo('loaded %d checkpoints: %s', len(checkpoints),
                          [cp['checkpoint_id'] for cp in checkpoints])

            route = self._plan_route(checkpoints, self.START_POSITION, goal)
            for index, checkpoint in enumerate(route, start=1):
                self._check_time('checkpoint %d/%d' % (index, len(route)))
                cp_id = checkpoint['checkpoint_id']
                if not self._move_to_target(checkpoint, 'checkpoint %d' % cp_id):
                    raise RuntimeError('move failed for checkpoint %d' % cp_id)
                self._hold_at_checkpoint(1.1)
                if not self._notify_checkpoint_passed(cp_id):
                    raise RuntimeError('checkpoint %d was not accepted' % cp_id)

            # Never claim mission completion without a known GoalPoint pose.
            if goal is None:
                raise RuntimeError(
                    'all checkpoints processed, but GoalPoint pose is unavailable. '
                    'The Beta API returns goalpoint_id only; configure ~goal_position '
                    'and ~goal_quaternion from the official Programming Manual/mission data.'
                )

            self._move_to_target(goal, 'GoalPoint')
            if not self._notify_mission_finished():
                raise RuntimeError('mission finish report was rejected')
            rospy.loginfo('MISSION FLOW COMPLETED')

        except Exception as exc:
            rospy.logerr('MISSION FLOW STOPPED: %s', exc)
            if isinstance(exc, TimeoutError):
                rospy.logerr('Mission time reserve was exhausted; finish report was not sent.')

    def _callback_start(self, msg):
        if self._started:
            rospy.logwarn('start callback already processed; ignoring duplicate')
            return
        self._started = True
        rospy.loginfo('mission start signal received: %s', msg)
        self._run_user_flow(msg)

    def _stop_processing(self, request):
        try:
            self._stop_requested = True
            if self._action_active:
                self._command.cancel_goal()
            if self._checkpoint_active:
                self._checkpoint_client.cancel_goal()
            rospy.loginfo('stop processing requested')
            return StopProcessingUserNodeResponse(
                result=StopProcessingUserNodeResponse.SUCCESS
            )
        except Exception as exc:
            rospy.logerr('stop processing failed: %s', exc)
            return StopProcessingUserNodeResponse(
                result=StopProcessingUserNodeResponse.ERROR
            )

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    UserTemplate().run()
