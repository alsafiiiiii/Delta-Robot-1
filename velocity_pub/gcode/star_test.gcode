; Star Test (Pentagram, 5-point, Radius 0.05m)
; Safe Z: -0.22, Draw Z: -0.25, Speed F50

; 1. Start Center
G1 X0.0 Y0.0 Z-0.22 F200

; 2. Point 1 (Top, 90 deg)
G1 X0.0 Y0.05 Z-0.22
G1 X0.0 Y0.05 Z-0.228

; 3. Point 3 (Bottom Right, 306 deg)
; -18 deg (342 deg in standard circle?)
; Pentagram skip 2 points: 90 -> (90+144) = 234 -> (234+144) = 378/18 -> 162 -> 306
G1 X0.029 Y-0.040 Z-0.228

; 4. Point 5 (Top Left, 162 deg)
G1 X-0.047 Y0.015 Z-0.228

; 5. Point 2 (Top Right, 18 deg)
G1 X0.047 Y0.015 Z-0.228

; 6. Point 4 (Bottom Left, 234 deg)
G1 X-0.029 Y-0.040 Z-0.228

; 7. Close to Point 1
G1 X0.0 Y0.05 Z-0.228

; 8. Lift and Center
G0 X0.0 Y0.05 Z-0.22
G0 X0.0 Y0.0 Z-0.22
