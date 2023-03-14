type LaserScan = {
    angle_min: number,
    angle_max: number,
    angle_increment: number,
    time_increment: number,
    scan_time: number,
    range_min: number,
    range_max: number,
    ranges: number[],
    intensities: number[]
}

export default LaserScan;
