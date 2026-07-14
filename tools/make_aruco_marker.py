#!/usr/bin/env python3
"""Generate a print-ready ArUco marker for yaw ground-truth.

Produces:
  - aruco_4x4_id0_40mm.png : high-res raster of the marker (black border only)
  - aruco_4x4_id0_40mm.pdf : the marker at EXACTLY 40 mm on paper, with a
                             white quiet-zone border so detection is reliable.

The 40 mm refers to the outer edge of the BLACK square -- that is the value you
pass to OpenCV as `markerLength` when estimating pose.
"""
import cv2
from reportlab.pdfgen import canvas
from reportlab.lib.units import mm

# --- configuration -----------------------------------------------------------
DICT = cv2.aruco.DICT_4X4_50   # 4x4 -> big cells, robust when small/printed
MARKER_ID = 0
MARKER_MM = 40.0               # outer black-square edge length (physical)
QUIET_MM = 8.0                 # white border around the marker (>= 1 cell)
PX = 1200                      # raster resolution of the marker itself
OUT_PNG = "aruco_4x4_id0_40mm.png"
OUT_PDF = "aruco_4x4_id0_40mm.pdf"

# --- generate raster ---------------------------------------------------------
dictionary = cv2.aruco.getPredefinedDictionary(DICT)
img = cv2.aruco.generateImageMarker(dictionary, MARKER_ID, PX)
cv2.imwrite(OUT_PNG, img)

# --- lay it out on paper at an exact physical size ---------------------------
page_w, page_h = 210 * mm, 297 * mm  # A4
c = canvas.Canvas(OUT_PDF, pagesize=(page_w, page_h))

# center the marker on the page
x = (page_w - MARKER_MM * mm) / 2
y = (page_h - MARKER_MM * mm) / 2

# white quiet-zone rectangle behind the marker
c.setFillColorRGB(1, 1, 1)
c.rect(x - QUIET_MM * mm, y - QUIET_MM * mm,
       (MARKER_MM + 2 * QUIET_MM) * mm, (MARKER_MM + 2 * QUIET_MM) * mm,
       stroke=0, fill=1)

# the marker itself at exactly MARKER_MM
c.drawImage(OUT_PNG, x, y, width=MARKER_MM * mm, height=MARKER_MM * mm)

# small tick marks at the black-square corners + a label to verify print scale
c.setStrokeColorRGB(0.6, 0.6, 0.6)
c.setLineWidth(0.3)
tick = 4 * mm
for cx, cy in [(x, y), (x + MARKER_MM * mm, y),
               (x, y + MARKER_MM * mm), (x + MARKER_MM * mm, y + MARKER_MM * mm)]:
    c.line(cx - tick, cy, cx - tick / 2, cy)
c.setFillColorRGB(0, 0, 0)
c.setFont("Helvetica", 9)
c.drawString(x - QUIET_MM * mm, y - QUIET_MM * mm - 12,
             "ArUco DICT_4X4_50  id=0  markerLength=40 mm  "
             "(measure the black square edge = 40.0 mm after printing)")
c.showPage()
c.save()

print("wrote", OUT_PNG, "and", OUT_PDF)
