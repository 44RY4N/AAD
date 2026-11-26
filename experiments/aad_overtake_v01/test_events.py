import unittest
from pathlib import Path
from .events import reconstruct


class OvertakeTests(unittest.TestCase):
    def test_clear_pass_and_return(self):
        r=reconstruct(Path('test_output/overtake_matrix_01/clear'))
        self.assertTrue(r['completed_under_definition']);self.assertEqual(r['return_confirmation_step'],151)
    def test_negative_is_not_pass(self):
        r=reconstruct(Path('test_output/overtake_matrix_01/negative'))
        self.assertFalse(r['observed_full_pass']);self.assertIsNone(r['return_confirmation_step'])
    def test_delayed_return(self):
        r=reconstruct(Path('test_output/overtake_matrix_01/delayed'))
        self.assertEqual(r['return_confirmation_step'],199)
    def test_incomplete_rejected(self):
        with self.assertRaises(ValueError):reconstruct(Path('test_output/overtake_blocked_probe_01'))


if __name__=='__main__':unittest.main()
