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
        state_changes_filepath: str = "state_changes.csv",
        sep: str = ";",
        flush_threshold: int = 1000,
        flush_count: int = 900,
    ):
        if flush_count >= flush_threshold:
            raise ValueError("flush_count must be smaller than flush_threshold.")

        self.filepath = filepath
        self.state_changes_filepath = state_changes_filepath
        self.sep = sep
        self.flush_threshold = flush_threshold
        self.flush_count = flush_count

        # Extract column names defined in the TypedDict schema
        schema_cols = list(get_type_hints(schema).keys())

        # Load existing file metadata, migrate schema if needed, and get starting index
        self.columns, self.current_index = self._initialize_storage(schema_cols)

        # In-memory DataFrame indexed by step index for regular flushed data
        self.df = pd.DataFrame(columns=self.columns)
        self.df.index.name = "index"

        # Persistent DataFrame for state change events (retained in memory until on_close)
        self.state_changes_df = pd.DataFrame(columns=self.columns)
        self.state_changes_df.index.name = "index"

    def _initialize_storage(self, schema_cols: list[str]) -> tuple[list[str], int]:
        """Handles CSV creation, schema migration, and index recovery."""
        if not os.path.exists(self.filepath) or os.path.getsize(self.filepath) == 0:
            empty_df = pd.DataFrame(columns=schema_cols)
            empty_df.index.name = "index"
            empty_df.to_csv(self.filepath, sep=self.sep, index=True)
            return schema_cols, 0

        existing_header_df = pd.read_csv(self.filepath, sep=self.sep, index_col="index", nrows=0)
        existing_cols = list(existing_header_df.columns)

        new_cols = [col for col in schema_cols if col not in existing_cols]

        if new_cols:
            full_df = pd.read_csv(self.filepath, sep=self.sep, index_col="index")
            for col in new_cols:
                full_df[col] = np.nan
            full_df.to_csv(self.filepath, sep=self.sep, index=True)

            all_cols = list(full_df.columns)
            next_index = int(full_df.index.max()) + 1 if not full_df.empty else 0
            return all_cols, next_index
        else:
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
        serialized_row = {
            col: self._serialize_value(data.get(col, np.nan))
            for col in self.columns
        }

        # Check for state change condition and store in persistent DataFrame
        if data.get("state_change") is True and data.get("state") == "steady_state":
            self.state_changes_df.loc[self.current_index] = serialized_row

        # Insert into regular rolling buffer
        self.df.loc[self.current_index] = serialized_row
        self.current_index += 1

        # Flush main data buffer when threshold is met
        if len(self.df) >= self.flush_threshold:
            self._flush(self.flush_count)

    def _flush(self, n_rows: int) -> None:
        """Writes the first n_rows to CSV and removes them from memory."""
        if self.df.empty or n_rows <= 0:
            return

        to_write = self.df.iloc[:n_rows]
        to_write.to_csv(self.filepath, sep=self.sep, mode="a", header=False, index=True)

        self.df = self.df.iloc[n_rows:].copy()

    def on_close(self) -> None:
        """Flushes remaining telemetry and dumps state changes to CSV."""
        # 1. Flush remaining rolling telemetry
        if not self.df.empty:
            self._flush(len(self.df))

        # 2. Write accumulated state changes
        if not self.state_changes_df.empty:
            file_exists = (
                os.path.exists(self.state_changes_filepath)
                and os.path.getsize(self.state_changes_filepath) > 0
            )
            self.state_changes_df.to_csv(
                self.state_changes_filepath,
                sep=self.sep,
                mode="a" if file_exists else "w",
                header=not file_exists,
                index=True,
            )