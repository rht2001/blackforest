
import csv
from pathlib import Path

allowed_states = {
    "thumb_probing"
}

repo_dir = Path(__file__).resolve().parent

input_file = repo_dir / "main_dataset.csv"
output_file = repo_dir / "dataset_filtered.csv"

with open(input_file, "r", encoding="utf-8-sig", newline="") as f:
    reader = csv.DictReader(f, delimiter=";")

    print("Columns:", reader.fieldnames)

    if "state" not in reader.fieldnames:
        raise ValueError("Column 'state' not found")

    with open(output_file, "w", encoding="utf-8", newline="") as out:
        writer = csv.DictWriter(
            out,
            fieldnames=reader.fieldnames,
            delimiter=";"
        )
        writer.writeheader()

        total = 0
        kept = 0

        for row in reader:
            total += 1
            state = (row.get("state") or "").strip().lower()

            if state in allowed_states:
                writer.writerow(row)
                kept += 1

print(f"Total rows: {total}")
print(f"Kept rows: {kept}")
print(f"Removed rows: {total - kept}")
print(f"Saved to: {output_file}")
