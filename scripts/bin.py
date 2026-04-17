# -----------------------------
# Binary format (little endian)
# -----------------------------
#
# Header:
#   magic        uint32
#   ra_bins      uint16
#   dec_bins     uint16
#   total_bins   uint32
#
# BinIndex:
#   offset       uint32   # byte offset to first object in this bin
#   count        uint32   # number of objects in this bin
#
# ObjectRecord:
#   id           uint32
#   ra_deg       float32
#   dec_deg      float32
#   mag          float32
#
# File layout:
#   [Header]
#   [BinIndex * TOTAL_BINS]
#   [ObjectRecord ... grouped by bin]
# TODO: THIS SHOULD NOT BE FLASHED TO THE MCU TO REDUCE SPACE

import struct
import csv
import math
import os
import struct
from collections import defaultdict


CSV_PATH = 'messier.csv'
OUT_PATH = "catalogue.bin"

RA_BINS = 36 
DEC_BINS = 18
TOTAL_BINS = RA_BINS * DEC_BINS

HEADER_FORMAT = "<IHHI"
BIN_INDEX_FORMAT = "<II"
OBJECT_FORMAT = "<Ifff"

HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
BIN_INDEX_SIZE = struct.calcsize(BIN_INDEX_FORMAT)
OBJECT_SIZE = struct.calcsize(OBJECT_FORMAT)

MAGIC_BYTE = 0x44534F31

bins = defaultdict(list)

# wrap RA val into [0, 360)
def normalize_ra_deg(ra_deg: float) -> float:
    return ra_deg % 360.0

# clamp dec to [-90, 90]
def clamp_dec_deg(dec_deg: float) -> float:
    return max(-90.0, min(90.0, dec_deg))

def parse_ra(value: str) -> float:
    s = str(value).strip()

    if not s:
        raise ValueError("Empty RA")

    # hh:mm:ss
    if ":" in s:
        parts = s.split(":")
        if len(parts) != 3:
            raise ValueError(f"Invalid RA format: {s}")
        h = float(parts[0])
        m = float(parts[1])
        sec = float(parts[2])
        hours = h + m / 60.0 + sec / 3600.0
        return normalize_ra_deg(hours * 15.0)

    # decimal hours with trailing h
    if s.lower().endswith("h"):
        hours = float(s[:-1])
        return normalize_ra_deg(hours * 15.0)

    # assume decimal hours (CSV stores RA in hours)
    return normalize_ra_deg(float(s) * 15.0)

def parse_dec(value: str) -> float:
    s = str(value).strip()

    if not s:
        raise ValueError("Empty Dec")

    if ":" in s:
        sign = -1.0 if s.startswith("-") else 1.0
        s_clean = s.lstrip("+-")
        parts = s_clean.split(":")
        if len(parts) != 3:
            raise ValueError(f"Invalid Dec format: {s}")
        d = float(parts[0])
        m = float(parts[1])
        sec = float(parts[2])
        deg = d + m / 60.0 + sec / 3600.0
        return clamp_dec_deg(sign * deg)

    return clamp_dec_deg(float(s))

def parse_mag(value: str) -> float:
    s = str(value).strip()
    if not s:
        return 99.0  # fallback
    return float(s)

# assign sky coordinates to a bin based of RA and dec
def get_bin_id(ra_deg: float, dec_deg: float) -> int:
    ra_bin_width = 360.0 / RA_BINS
    dec_bin_width = 180.0 / DEC_BINS

    ra_bin = int(ra_deg // ra_bin_width)
    dec_bin = int((dec_deg + 90.0) // dec_bin_width)

    # edge-case clamp
    if ra_bin >= RA_BINS:
        ra_bin = RA_BINS - 1
    if dec_bin >= DEC_BINS:
        dec_bin = DEC_BINS - 1
    if dec_bin < 0:
        dec_bin = 0

    return dec_bin * RA_BINS + ra_bin

# specifically grab necessary columns from CSV
def detect_necessary_columns(fieldnames):
    lowered = {name.lower().strip(): name for name in fieldnames}

    def find(*candidates):
        for c in candidates:
            if c in lowered:
                return lowered[c]
        return None

    id_col = find("id", "messier", "messier_id", "catalog_id", "object_id")
    ra_col = find("ra", "ra_deg", "right_ascension")
    dec_col = find("dec", "decl", "declination", "dec_deg")
    mag_col = find("mag", "apparent_mag", "magnitude", "v_mag", "vmag")

    if ra_col is None or dec_col is None or mag_col is None:
        raise ValueError(
            f"Could not detect required columns. Found columns: {fieldnames}"
        )

    return id_col, ra_col, dec_col, mag_col

# read CSV and group all objects into spatial bins
def create_bins():
    with open(CSV_PATH, mode="r", newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)

        if reader.fieldnames is None:
            raise ValueError("CSV has no header row.")

        id_col, ra_col, dec_col, mag_col = detect_necessary_columns(reader.fieldnames)

        generated_id = 1

        for row in reader:
            try:
                obj_id = generated_id if id_col is None else int(row[id_col])
                ra_deg = parse_ra(row[ra_col])
                dec_deg = parse_dec(row[dec_col])
                mag = parse_mag(row[mag_col])

                bin_id = get_bin_id(ra_deg, dec_deg)
                bins[bin_id].append((obj_id, ra_deg, dec_deg, mag))

                generated_id += 1

            except Exception as e:
                print(f"Skipping row due to parse error: {row}\n  Error: {e}")

# sort objects within a bin by brightness
def sort_bins():
    for bin_id in bins:
        bins[bin_id].sort(key=lambda obj: obj[3])  # sort by mag

# calculate offset and count for each bin
def build_index_entries():
    index_entries = []
    current_offset = HEADER_SIZE + TOTAL_BINS * BIN_INDEX_SIZE

    for bin_id in range(TOTAL_BINS):
        obj_list = bins.get(bin_id, [])
        count = len(obj_list)

        index_entries.append((current_offset, count))
        current_offset += count * OBJECT_SIZE

    return index_entries

def write_binary():
    create_bins()
    sort_bins()
    index_entries = build_index_entries()

    total_objects = sum(len(v) for v in bins.values())

    with open(OUT_PATH, "wb") as f:
        # Header
        f.write(struct.pack(
            HEADER_FORMAT,
            MAGIC_BYTE,
            RA_BINS,
            DEC_BINS,
            TOTAL_BINS
        ))

        # Bin index table
        for offset, count in index_entries:
            f.write(struct.pack(BIN_INDEX_FORMAT, offset, count))

        # Object data, grouped by bin
        for bin_id in range(TOTAL_BINS):
            for obj_id, ra_deg, dec_deg, mag in bins.get(bin_id, []):
                f.write(struct.pack(
                    OBJECT_FORMAT,
                    obj_id,
                    ra_deg,
                    dec_deg,
                    mag
                ))

    print(f"Wrote {OUT_PATH}")
    print(f"Total bins: {TOTAL_BINS}")
    print(f"Total objects: {total_objects}")
    print(f"Header size: {HEADER_SIZE} bytes")
    print(f"Bin index table size: {TOTAL_BINS * BIN_INDEX_SIZE} bytes")
    print(f"Object record size: {OBJECT_SIZE} bytes")
        
def main():
    write_binary()
    return 0;

if __name__ == "__main__":
    main()
