# g1_nav2_bringup

Planning-only Nav2 bringup for the current G1 FAST-LIO project.

- global frame: `map`
- odom frame: `camera_init`
- FAST-LIO frame: `body`
- navigation base: `nav_base` (static 180-degree X correction from `body`)
- odometry topic: `/Odometry`
- local obstacle source: `/cloud_registered_body`
- isolated velocity output: `/cmd_vel_nav`

This package intentionally does not connect velocity output to Unitree SDK2.
