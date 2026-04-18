import struct
import csv
import math
import os
from collections import defaultdict

OUT_PATH = "catalogue.bin"

RA_BINS = 36 
DEC_BINS = 18
TOTAL_BINS = RA_BINS * DEC_BINS

HEADER_FORMAT = "<IHHI"
BIN_INDEX_FORMAT = "<II"
# id (uint16), type (uint16, 0=M, 1=N), ra (float), dec (float), mag (float)
OBJECT_FORMAT = "<HHfff"

HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
BIN_INDEX_SIZE = struct.calcsize(BIN_INDEX_FORMAT)
OBJECT_SIZE = struct.calcsize(OBJECT_FORMAT)

MAGIC_BYTE = 0x44534F31

bins = defaultdict(list)

def normalize_ra_deg(ra_deg: float) -> float:
    return ra_deg % 360.0

def clamp_dec_deg(dec_deg: float) -> float:
    return max(-90.0, min(90.0, dec_deg))

def parse_ra(value: str) -> float:
    s = str(value).strip()
    if ":" in s:
        parts = s.split(":")
        h = float(parts[0])
        m = float(parts[1])
        sec = float(parts[2])
        return normalize_ra_deg((h + m / 60.0 + sec / 3600.0) * 15.0)
    return normalize_ra_deg(float(s) * 15.0)

def parse_dec(value: str) -> float:
    s = str(value).strip()
    if ":" in s:
        sign = -1.0 if s.startswith("-") else 1.0
        s_clean = s.lstrip("+-")
        parts = s_clean.split(":")
        d = float(parts[0])
        m = float(parts[1])
        sec = float(parts[2])
        return clamp_dec_deg(sign * (d + m / 60.0 + sec / 3600.0))
    return clamp_dec_deg(float(s))

def get_bin_id(ra_deg: float, dec_deg: float) -> int:
    ra_bin_width = 360.0 / RA_BINS
    dec_bin_width = 180.0 / DEC_BINS
    ra_bin = int(ra_deg // ra_bin_width)
    dec_bin = int((dec_deg + 90.0) // dec_bin_width)
    return min(DEC_BINS - 1, max(0, dec_bin)) * RA_BINS + min(RA_BINS - 1, max(0, ra_bin))

def process_csv(path, id_col_name, obj_type):
    with open(path, mode="r", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                obj_id = int(row[id_col_name])
                ra_deg = parse_ra(row['ra'])
                dec_deg = parse_dec(row['dec'])
                mag = float(row['mag']) if 'mag' in row else float(row['v_mag'])
                
                bin_id = get_bin_id(ra_deg, dec_deg)
                bins[bin_id].append((obj_id, obj_type, ra_deg, dec_deg, mag))
            except Exception as e:
                pass

def main():
    # Types: Messier = 0, NGC = 1
    process_csv('messier.csv', 'messier_id', 0)
    process_csv('ngc.csv', 'id', 1)

    for b in bins:
        bins[b].sort(key=lambda x: x[4]) # sort by mag

    index_entries = []
    current_offset = HEADER_SIZE + TOTAL_BINS * BIN_INDEX_SIZE
    for i in range(TOTAL_BINS):
        count = len(bins[i])
        index_entries.append((current_offset, count))
        current_offset += count * OBJECT_SIZE

    with open(OUT_PATH, "wb") as f:
        f.write(struct.pack(HEADER_FORMAT, MAGIC_BYTE, RA_BINS, DEC_BINS, TOTAL_BINS))
        for offset, count in index_entries:
            f.write(struct.pack(BIN_INDEX_FORMAT, offset, count))
        for i in range(TOTAL_BINS):
            for obj in bins[i]:
                f.write(struct.pack(OBJECT_FORMAT, *obj))
    
    print(f"Combined catalogue written to {OUT_PATH}")
    print(f"Total objects: {sum(len(v) for v in bins.values())}")

if __name__ == "__main__":
    main()
