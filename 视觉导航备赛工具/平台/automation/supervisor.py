"""Combine candidate, task, safety, emergency and odometry into one decision."""


class MotionSupervisor:
    def __init__(self, gate, feedback, emergency):
        self.gate, self.feedback, self.emergency = gate, feedback, emergency
        self.last_reason = 'waiting'

    def output(self, now, ros_now_ns):
        values = self.gate.output(now, ros_now_ns)
        if values is None:
            self.last_reason = 'disabled'
            return None
        zero = (0.0, 0.0, 0.0)
        if not self.emergency.allowed(now):
            self.last_reason = 'estop_latched_or_stale'
            self.feedback.evaluate(now, ros_now_ns, True)
            return zero
        ready, reason = self.feedback.evaluate(now, ros_now_ns, values == zero)
        if not ready:
            self.last_reason = reason
            return zero
        self.last_reason = 'ready' if values != zero else 'safety_mission_or_command_stop'
        return values

    def reset(self, now, ros_now_ns):
        gate = self.gate
        if gate.safety is not True or gate.safety_at is None or not 0 <= now-gate.safety_at <= gate.timeout:
            return False, '安全监测信号无效或已过期'
        if not self.feedback.can_reset(now, ros_now_ns):
            return False, '里程计无效或车辆尚未静止'
        if not self.emergency.can_reset(now):
            return False, '急停仍按下或急停信号已过期'
        self.feedback.reset(now, ros_now_ns)
        self.emergency.reset(now)
        # A newly observed candidate is required after operator reset.
        gate.clear_command()
        return True, '已复位，等待新的控制指令'

    def diagnostics(self, now, ros_now_ns=None):
        feedback = self.feedback
        def elapsed(at):
            return None if at is None else round(max(0.0, now-at), 3)
        stamp = self.gate.frame_stamp_ns
        frame_age = None
        if stamp is not None and isinstance(ros_now_ns, int) and not isinstance(ros_now_ns, bool):
            frame_age = round(max(0.0, (ros_now_ns-stamp)/1_000_000_000), 3)
        return {'source':'control_gate', 'active':self.gate.enabled, 'reason':self.last_reason,
                'estop_latched':self.emergency.latched, 'feedback_fault':feedback.latched_reason,
                'feedback_age_s':elapsed(feedback.at), 'safety_age_s':elapsed(self.gate.safety_at),
                'command_age_s':elapsed(self.gate.command_at), 'frame_age_s':frame_age}
