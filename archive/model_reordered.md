# Link Order Reference from Old Model

## Old Model Link Order

1. world_link
2. base_link (Frame in new model)
3. base_hex (base in new model)
4. eye_to_hand_cam (NOT in new model)
5. forearm1
6. forearm2
7. forearm3
8. arm1
9. arm2
10. arm3
11. arm4
12. arm5
13. arm6
14. dummy_arm1-6 (NOT in new model)
15. EEBase
16. camera_holder, camera_stick, camera_bar, camera (NOT in new model)
17. Bevel1
18. Bevel2
19. T
20. EE
21. vacuum_gripper (NOT in new model)

## New Model Should Have (in this order)

1. Frame (was base_link)
2. base (was base_hex)
3. forearm1
4. forearm2
5. forearm3
6. arm1-6
7. EEBase
8. Bevel1
9. Bevel2
10. T
11. EE

## Joint Order from Old Model

1. virtual_joint (world_link -> base_link) - NOT NEEDED
2. base_joint (base_link -> base_hex) - becomes Frame -> base
3. jbf1, jbf2, jbf3 (base_hex -> forearms) - becomes base -> forearms
4. ball1-6 (forearms -> arms)
5. ball7-12 (arms -> EEBase or dummy arms)
6. camera joints - NOT NEEDED
7. Bevelj1, Bevelj2 (EEBase -> Bevels)
8. Tj1 (EEBase -> T)
9. BeveljEE (T -> EE)
10. vacuum_gripper_joint - NOT NEEDED
