"""Keep the isolated replays compatible with ROS 2 topic naming rules."""
import ast
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class SmokeTopics(unittest.TestCase):
    def test_isolated_topic_names_are_valid(self):
        for name in ('ros2_model_smoke.py', 'ros2_barrier_smoke.py', 'steering_gate_smoke.py'):
            tree = ast.parse((ROOT / 'tools' / name).read_text(encoding='utf-8'))
            topics = [node.value for node in ast.walk(tree)
                      if isinstance(node, ast.Constant) and isinstance(node.value, str)
                      and node.value.startswith(('/trial/', '/visual_race/trial/'))]
            self.assertTrue(topics, name)
            for topic in topics:
                for component in topic.strip('/').split('/'):
                    self.assertRegex(component, r'^[A-Za-z_][A-Za-z0-9_]*$', name + ': ' + topic)


if __name__ == '__main__':
    unittest.main()
