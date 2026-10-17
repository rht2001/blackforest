import csv
import json
import tempfile
import unittest
from pathlib import Path
from typing import TypedDict

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from logic.robot_dataset import RobotDataset


class RobotDatasetTest(unittest.TestCase):
    def test_batches_restart_and_schema_changes(self):
        class Outputs(TypedDict):
            angle: float
            tactile: list

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "main_dataset.csv"
            dataset = RobotDataset(Outputs, path)
            matrix = [[1, 2], [3, 4]]
            for step in range(2000):
                self.assertEqual(dataset.loop({"angle": step, "tactile": matrix}), step)
                self.assertLess(len(dataset.dataframe), 1000)
            self.assertEqual(len(dataset.dataframe), 200)
            with path.open(newline="", encoding="utf-8") as file:
                rows = list(csv.DictReader(file, delimiter=";"))
            self.assertEqual(len(rows), 1800)
            self.assertEqual(json.loads(rows[0]["tactile"]), matrix)
            matrix[0][0] = 99
            dataset.close()
            dataset.close()
            changed = RobotDataset({"angle": float, "force": float}, path)
            self.assertEqual(changed.next_index, 2000)
            changed.loop({"angle": 5, "force": 6})
            changed.close()
            with path.open(newline="", encoding="utf-8") as file:
                rows = list(csv.DictReader(file, delimiter=";"))
            self.assertEqual(len(rows), 2001)
            self.assertEqual(rows[-1]["index"], "2000")
            self.assertEqual(rows[0]["force"], "")
            self.assertEqual(rows[-1]["tactile"], "")
            self.assertEqual(json.loads(rows[-2]["tactile"])[0][0], 1)
            with self.assertRaises(RuntimeError):
                changed.loop({"angle": 0, "force": 0})


if __name__ == "__main__":
    unittest.main()
