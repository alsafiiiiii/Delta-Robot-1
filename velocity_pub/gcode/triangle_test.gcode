; Triangle Test (Equilateral, Radius 0.05m)
; Safe Z: -0.22, Draw Z: -0.25, Speed F12

; 1. Start Center
G1 X0.0 Y0.0 Z-0.22 F12

; 2. Move to Vertex 1 (Top, 90 deg) -> X0, Y0.05
G1 X0.0 Y0.05 Z-0.22
G1 X0.0 Y0.05 Z-0.25

; 3. Vertex 2 (Bottom Left, 210 deg)
; X = 0.05 * cos(210) = -0.0433
; Y = 0.05 * sin(210) = -0.025
G1 X-0.043 Y-0.025 Z-0.25

; 4. Vertex 3 (Bottom Right, 330 deg)
; X = 0.05 * cos(330) = 0.0433
; Y = 0.05 * sin(330) = -0.025
G1 X0.043 Y-0.025 Z-0.25

; 5. Close loop to Vertex 1
G1 X0.0 Y0.05 Z-0.25

; 6. Lift and Center
G1 X0.0 Y0.05 Z-0.22
G1 X0.0 Y0.0 Z-0.22
