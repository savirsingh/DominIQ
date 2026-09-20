import unittest
from unittest.mock import patch

from mission import State, mark_boat_found


class MissionEventTest(unittest.TestCase):
    def test_first_detection_notifies_once(self):
        state = State()

        with patch("mission.urllib.request.urlopen") as send:
            mark_boat_found(state, (71.0, -95.0))
            mark_boat_found(state, (70.0, -96.0))

        self.assertTrue(state.boat_found)
        self.assertEqual((state.boat_lat, state.boat_lon), (70.0, -96.0))
        self.assertEqual(send.call_count, 1)


if __name__ == "__main__":
    unittest.main()
