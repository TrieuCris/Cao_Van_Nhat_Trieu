import unittest
from unittest.mock import MagicMock
import sys
import os

# Add parent directory to path to import robot_controller
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from robot_controller import RobotController
import constants as C

class TestStatusParsing(unittest.TestCase):
    def setUp(self):
        self.mock_app = MagicMock()
        # Mock after to execute callback immediately
        self.mock_app.after = MagicMock(side_effect=lambda delay, callback, *args, **kwargs: callback(*args, **kwargs))
        
        self.mock_conn = MagicMock()
        self.controller = RobotController(self.mock_app, self.mock_conn)
        
    def test_status_parsing_with_tray(self):
        # Initial state
        self.controller.is_tray_present = False
        
        # Simulate STATUS response
        # STATUS:RS:ESTOP:BTN_START:BTN_STOP:CONV:CONV_SPEED:PUMP:SERVO:TRAY:S1,S2,S3
        # TRAY=1 (Present), SPEED=0
        message = "STATUS:0:0:0:0:0:0:0:-9000:1:0,0,0"
        self.controller.handle_stm_message(message)
        
        # Verify tray is present
        self.assertTrue(self.controller.is_tray_present)
        # Verify GUI update called
        self.mock_app.update_indicator.assert_any_call(C.INDICATOR_TRAY_SENSOR, True)

    def test_status_parsing_without_tray(self):
        # Initial state
        self.controller.is_tray_present = True
        self.controller.conveyor_speed_mm_s = 0.0
        
        # Simulate STATUS response
        # TRAY=0 (Absent), SPEED=45
        message = "STATUS:0:0:0:0:1:45:0:-9000:0:0,0,0"
        self.controller.handle_stm_message(message)
        
        # Verify tray is NOT present
        self.assertFalse(self.controller.is_tray_present)
        # Verify Conveyor Speed update
        self.assertEqual(self.controller.conveyor_speed_mm_s, 45.0)
        # Verify GUI update called
        self.mock_app.update_indicator.assert_any_call(C.INDICATOR_TRAY_SENSOR, False)
        
    def test_status_parsing_legacy_format(self):
        # Initial state
        self.controller.is_tray_present = False
        
        # Simulate invalid/old STATUS response (too few parts)
        message = "STATUS:0:1"
        self.controller.handle_stm_message(message)
        
        # Verify tray state UNCHANGED (Parsing should fail gracefully)
        self.assertFalse(self.controller.is_tray_present)

if __name__ == '__main__':
    unittest.main()
