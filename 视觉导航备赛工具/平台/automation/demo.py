"""Deterministic demonstration using simulated perception and odometry."""
from automation.race import Race


def demo():
    race = Race(frames=3)
    step_number = 0
    timeline = []
    def collect():
        timeline.extend(race.pop_events())
    def steps(count, line=False, crosswalk=False):
        nonlocal step_number
        for _ in range(count):
            step_number += 1
            t = step_number / 10
            race.frame(t, line, crosswalk)
            race.odom(t, 0.0, 0.0, 0.0)
            race.tick(t)
            collect()
    steps(3)
    race.arm(step_number / 10)
    collect()
    steps(3, line=True)
    for _ in range(2):
        steps(3)
        steps(3, crosswalk=True)
        race.acknowledge(step_number / 10, race.announcement_id, True)
        collect()
        steps(32, crosswalk=True)
        steps(3)
        steps(3, line=True)
    return {'timeline': timeline, 'final': race.status(step_number / 10)}


if __name__ == '__main__':
    import json
    print(json.dumps(demo(), ensure_ascii=False, indent=2))
