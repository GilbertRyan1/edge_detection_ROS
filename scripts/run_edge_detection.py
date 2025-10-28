#!/usr/bin/env python3
import os
import cv2
from edge_detector import EdgeDetector

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data")
DATA_DIR = os.path.normpath(DATA_DIR)

def list_candidate_images(folder_path):
    out = []
    for f in os.listdir(folder_path):
        low = f.lower()
        if low.endswith(".png") or low.endswith(".jpg") or low.endswith(".jpeg"):
            out.append(f)
    out.sort()
    return out

def run_on_image(det, img_path, save_dir):
    img_bgr = cv2.imread(img_path)
    if img_bgr is None:
        print("skip (cant read):", img_path)
        return

    edge_mask = det.get_edges(img_bgr)
    segs = det.get_line_segments(edge_mask)
    overlay = det.draw_segments(img_bgr, segs)

    # show once for this img
    cv2.imshow("result (green lines)", overlay)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    base_name = os.path.basename(img_path)
    stem = base_name.rsplit(".", 1)[0]
    out_name = stem + "_result.png"
    out_path = os.path.join(save_dir, out_name)
    cv2.imwrite(out_path, overlay)
    print("Saved:", out_path)

def parse_selection(inp, total_count):
    inp = inp.strip().lower()
    if inp == "all":
        return list(range(total_count))

    idx_list = []
    for token in inp.split():
        try:
            k = int(token)
        except ValueError:
            print("not a number:", token)
            continue
        if 0 <= k < total_count:
            idx_list.append(k)
        else:
            print("out of range:", k)
    # remove duplicates while keeping order
    seen = set()
    uniq = []
    for k in idx_list:
        if k not in seen:
            uniq.append(k)
            seen.add(k)
    return uniq

# main logic  
print("Image folder:", DATA_DIR)
imgs = list_candidate_images(DATA_DIR)

if len(imgs) == 0:
    print("No images in data/. Put them in:", DATA_DIR) 
    exit()

print("\nAvailable images:")
for idx, name in enumerate(imgs):
    print(f"[{idx}] {name}")

print("\nEnter index num (e.g. 0)")
print("for many images (e.g. 0 2 4)")
print("enter 'all' for everything")
raw_sel = input("Selection: ")

sel_indices = parse_selection(raw_sel, len(imgs))
if len(sel_indices) == 0:
    print("Nothing selected.")
    exit()

det = EdgeDetector()

for i in sel_indices:
    fname = imgs[i]
    in_path = os.path.join(DATA_DIR, fname)
    print("\nProcessing:", fname)
    run_on_image(det, in_path, DATA_DIR)
