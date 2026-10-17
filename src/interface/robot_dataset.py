import os
import json
from typing import TypedDict, Type, get_type_hints, Any
import numpy as np
import pandas as pd
from config.config_parameters import DataDict



class RobotDatabase:
    def __init__(
        self,
        schema: Type[Any] = DataDict,
        filepath: str = "main_dataset.csv",
        sep: str = ";",
        flush_threshold: int = 1000,
        flush_count: int = 900,
    ):
        if flush_count >= flush_threshold:
            raise ValueError("flush_count must be smaller than flush_threshold.")

        self.filepath = filepath
        self.sep = sep
        self.flush_threshold = flush_threshold
        self.flush_count = flush_count

        # Extract column names defined in the TypedDict schema
        schema_cols = list(get_type_hints(schema).keys())

        # Load existing file metadata, migrate schema if needed, and get starting index
        self.columns, self.current_index = self._initialize_storage(schema_cols)

        # In-memory DataFrame indexed by step index
        self.df = pd.DataFrame(columns=self.columns)
        self.df.index.name = "index"

    def _initialize_storage(self, schema_cols: list[str]) -> tuple[list[str], int]:
        """Handles CSV creation, schema migration, and index recovery."""
        if not os.path.exists(self.filepath) or os.path.getsize(self.filepath) == 0:
            # Fresh start: create CSV with header only
            empty_df = pd.DataFrame(columns=schema_cols)
            empty_df.index.name = "index"
            empty_df.to_csv(self.filepath, sep=self.sep, index=True)
            return schema_cols, 0

        # Read only the header first to check for schema changes
        existing_header_df = pd.read_csv(self.filepath, sep=self.sep, index_col="index", nrows=0)
        existing_cols = list(existing_header_df.columns)

        new_cols = [col for col in schema_cols if col not in existing_cols]

        if new_cols:
            # Schema changed: load existing dataset, add new columns, and rewrite
            full_df = pd.read_csv(self.filepath, sep=self.sep, index_col="index")
            for col in new_cols:
                full_df[col] = np.nan
            full_df.to_csv(self.filepath, sep=self.sep, index=True)

            all_cols = list(full_df.columns)
            next_index = int(full_df.index.max()) + 1 if not full_df.empty else 0
            return all_cols, next_index
        else:
            # Schema unchanged: read only the index column to avoid loading huge files into RAM
            index_series = pd.read_csv(self.filepath, sep=self.sep, usecols=["index"])["index"]
            next_index = int(index_series.max()) + 1 if not index_series.empty else 0
            return existing_cols, next_index

    @staticmethod
    def _serialize_value(val: Any) -> Any:
        """Serializes numpy arrays to single-line JSON strings so CSV rows don't break or truncate."""
        if isinstance(val, np.ndarray):
            return json.dumps(val.tolist())
        return val

    def main(self, data: DataDict) -> None:
        """Call this once per robot loop iteration to store a datapoint."""
        # Build row aligned with the CSV's exact column order
        serialized_row = {
            col: self._serialize_value(data.get(col, np.nan))
            for col in self.columns
        }

        # Insert into the in-memory DataFrame at current_index
        self.df.loc[self.current_index] = serialized_row
        self.current_index += 1

        # When we hit 1000 entries, write and flush the oldest 900
        if len(self.df) >= self.flush_threshold:
            self._flush(self.flush_count)

    def _flush(self, n_rows: int) -> None:
        """Writes the first n_rows to CSV and removes them from memory."""
        if self.df.empty or n_rows <= 0:
            return

        to_write = self.df.iloc[:n_rows]
        to_write.to_csv(self.filepath, sep=self.sep, mode="a", header=False, index=True)

        # Keep only the remaining tail in memory
        self.df = self.df.iloc[n_rows:].copy()

    def on_close(self) -> None:
        """Flushes all remaining rows in the DataFrame to CSV on app shutdown."""
        if not self.df.empty:
            self._flush(len(self.df))