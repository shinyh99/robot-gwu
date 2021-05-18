- lidar on
    - in range
        - intensity: 15.0, range: x.xx
    - too far
        - intensity: 0.0, range: inf
    - noise
        - intensity: x.xx, range: x.xx

    - laser_f/scan
    - laser_r/scan

- lidar duplicate angle filter
    - usage: remove duplication

    - in range
        - intensity: 15.0, range: x.xx
    - too far
        - intensity: 0.0, range: 13, inf
    - noise
        - intensity: x.xx, range: x.xx

    - laser_f/scan -> laser_f/scan_filtered
    - laser_r/scan -> laser_r/scan_filtered

- lidar intensity filter
    - usage: remove ambiguous intensity by valid

    - laser_f/scan_filtered -> laser_f/scan_intensity_valid
        - in range(valid)
            - intensity: 15.0,  range: x.xx
        - too far
            - intensity: 0.0,   range: nan
        - noise
            - intensity: x.xx,  range: nan

    - laser_r/scan_filtered -> laser_r/scan_intensity_inf
        - in range(inf)
            - intensity: 15.0,  range: nan
        - too far
            - intensity: 0.0, range: 13, inf
        - noise
            - intensity: x.xx, range: nan   

- lidar intensity merge
    - usage: merge two intensity(reading, no reading)

    - laser_f/scan_intensity_valid, inf, noise_inf -> laser_f/scan_intensity
        - in range
            - range: x.xx
        - too far
            - range: 13
        - noise
            - range: 13

    - laser_r/scan_intensity_valid, inf, noise_inf -> laser_r/scan_intensity

- lidar merge
    - usage: merge two lidars

    - in range
            - range: x.xx
        - too far
            - range: 13
        - noise
            - range: 13

    - laser_f/scan_intensity, laser_r/scan_intensity -> scan_all

- lidar range filter
    - usage: max_range to inf

    - scan_all -> scan_range

- lidar back filter
    - usage: used for localization

    - in range
            - range: x.xx
        - too far
            - range: inf
        - noise
            - range: inf

    - scan_range -> scan

