; Delta Robot Square Test
; Units: Meters for Position, Radians for Angles
; A = Tilt (X-axis), C = Spin (Z-axis)

; 1. Go to Start Height (Center)
G1 X0.0 Y0.0 Z-0.25 A0.0 C0.0 F500

; 2. Move to Corner 1 (Top Right)
G1 X0.05 Y0.05 Z-0.3

; 3. Tilt Tool Outward (Tilt +0.2 rad)
G1 X0.05 Y0.05 Z-0.3 

; 4. Move to Corner 2 (Bottom Right) with Tilt
G1 X0.05 Y-0.05 Z-0.3 

; 5. Spin Tool 90 degrees (1.57 rad) while moving to Corner 3
G1 X-0.05 Y-0.05 Z-0.3 

; 6. Remove Tilt (A0) at Corner 3
G1 X-0.05 Y-0.05 Z-0.3 

; 7. Move to Corner 4 (Top Left)
G1 X-0.05 Y0.05 Z-0.3

G1 X0.05 Y0.05 Z-0.3
; 8. Return to Center and Reset Rotation
G1 X0.0 Y0.0 Z-0.25
