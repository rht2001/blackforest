"""Buffered robot telemetry storage. One RobotDataset instance/writer per file."""

import csv
import json
import os
import tempfile
from collections.abc import Mapping
from pathlib import Path

import pandas as pd


class RobotDataset:
    """Use dictionary keys or TypedDict annotations as a fixed run schema.

    Call loop(outputs) once per robot loop and close() from on_close.
    Indices start at zero. Buffered rows are not persisted until a batch or close.
    """

    def __init__(self, robot_outputs, path="main_dataset.csv"):
        schema = (robot_outputs if isinstance(robot_outputs, Mapping)
                  else getattr(robot_outputs, "__annotations__", None))
        if schema is None:
            raise TypeError("robot_outputs must be a dictionary or TypedDict definition")
        self.fields = tuple(schema)
        if any(not isinstance(key, str) or not key or key == "index" for key in self.fields):
            raise ValueError("Schema keys must be nonempty strings; 'index' is reserved")
        self.path = Path(path)
        self.columns = ["index", *self.fields]
        self.next_index = 0
        self._closed = False
        self._initialize()
        self.dataframe = pd.DataFrame(columns=self.columns)

    def _initialize(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if not self.path.exists() or self.path.stat().st_size == 0:
            with self.path.open("w", newline="", encoding="utf-8") as file:
                csv.writer(file, delimiter=";").writerow(self.columns)
            return
        with self.path.open(newline="", encoding="utf-8") as file:
            reader = csv.DictReader(file, delimiter=";")
            old_columns = reader.fieldnames
            if not old_columns or "index" not in old_columns or len(set(old_columns)) != len(old_columns):
                raise ValueError("Existing dataset needs unique columns and an index column")
            for row in reader:
                self.next_index = max(self.next_index, int(row["index"]) + 1)
        self.columns = old_columns + [key for key in self.fields if key not in old_columns]
        if self.columns == old_columns:
            return
        # Stream the migration, and replace only after the new file is complete.
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode="w", newline="", encoding="utf-8",
                                             dir=self.path.parent, delete=False) as target:
                temporary = Path(target.name)
                writer = csv.DictWriter(target, fieldnames=self.columns, delimiter=";")
                writer.writeheader()
                with self.path.open(newline="", encoding="utf-8") as source:
                    writer.writerows(csv.DictReader(source, delimiter=";"))
                target.flush()
                os.fsync(target.fileno())
            os.replace(temporary, self.path)
        finally:
            if temporary is not None and temporary.exists():
                temporary.unlink()

    @staticmethod
    def _cell(value):
        if hasattr(value, "tolist"):
            value = value.tolist()
        if isinstance(value, (Mapping, list, tuple)):
            return json.dumps(value, default=lambda item: item.tolist(), separators=(",", ":"))
        return value

    def loop(self, outputs):
        """Snapshot one output dictionary; return its assigned index."""
        if self._closed:
            raise RuntimeError("Dataset is closed")
        if set(outputs) != set(self.fields):
            raise ValueError("Output keys must match the schema for this run")
        index = self.next_index
        row = {key: self._cell(outputs[key]) for key in self.fields}
        row["index"] = index
        self.dataframe.loc[len(self.dataframe)] = [row.get(key) for key in self.columns]
        self.next_index += 1
        if len(self.dataframe) >= 1000:
            self._write(900)
        return index

    def _write(self, count):
        if count == 0:
            return
        # Roll back a failed append so a retry cannot duplicate a partial batch.
        with self.path.open("r+b") as file:
            file.seek(0, os.SEEK_END)
            start = file.tell()
            try:
                batch = self.dataframe.iloc[:count].copy()
                batch["index"] = batch["index"].astype("int64")
                payload = batch.to_csv(
                    sep=";", index=False, header=False, lineterminator="\n")
                file.write(payload.encode("utf-8"))
                file.flush()
                os.fsync(file.fileno())
            except BaseException:
                file.seek(start)
                file.truncate()
                raise
        self.dataframe = self.dataframe.iloc[count:].reset_index(drop=True).copy()

    def close(self):
        """Write all remaining rows. Safe to call more than once."""
        if not self._closed:
            self._write(len(self.dataframe))
            self._closed = True

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
