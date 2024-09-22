## Rectangle
First iteration

| Rectangle Part | Gyro Angle (Degrees) | Left Motor Distance (mm) | Right Motor Distance (mm) | Absolute value of Calculated Angle (Degrees) |
| ----- | ----- | ----- | ----- | ----- |
| 1, go straight | 2 | 248.4 | 249.0 | 0.18 |
| 1, turn 90 deg | 90 | 405.0 | 72.6 | 97.67 |
| 2, go straight | 92 | 654.6 | 324.0 | 97.14 |
| 2, turn 90 deg | 180 | 805.8 | 150.0 | 192.69 |
| 3, go straight | 181 | 1054.2 | 400.8 | 191.98 |
| 3, turn 90 deg | 269 | 1207.8 | 225.0 | 288.77 |
| 4, go straight | 269 | 1458.0 | 475.8 | 288.59 |
| 4, turn 90 deg | 357 | 1606.8 | 303.0 | 383.09 |

Second iteration:

| Rectangle Part | Gyro Angle (Degrees) | Left Motor Distance (mm) | Right Motor Distance (mm) | Absolute value of Calculated Angle (Degrees) |
| ----- | ----- | ----- | ----- | ----- |
| 1, go straight | \-1 | 1857.0 | 555.0 | 22.56 |
| 1, turn 90 deg | 87 | 2004.6 | 382.8 | 116.52 |
| 2, go straight | 88 | 2254.2 | 634.2 | 116.00 |
| 2, turn 90 deg | 177 | 2401.2 | 464.4 | 209.08 |
| 3, go straight | 178 | 2647.2 | 713.4 | 208.20 |
| 3, turn 90 deg | 266 | 2792.2 | 546.0 | 299.99 |
| 4, go straight | 267 | 3048.0 | 796.2 | 301.63 |
| 4, turn 90 deg | 355 | 3196.8 | 624.6 | 395.78 |

Third iteration:

| Rectangle Part | Gyro Angle (Degrees) | Left Motor Distance (mm) | Right Motor Distance (mm) | Absolute value of Calculated Angle (Degrees) |
| ----- | ----- | ----- | ----- | ----- |
| 1, go straight | \-3 | 3447.0 | 874.8 | 35.78 |
| 1, turn 90 deg | 85 | 3595.8 | 702.6 | 130.09 |
| 2, go straight | 86 | 3838.8 | 950.4 | 128.68 |
| 2, turn 90 deg | 174 | 3993.0 | 782.4 | 223.35 |
| 3, go straight | 174 | 4243.2 | 1034.4 | 222.82 |
| 3, turn 90 deg | 264 | 4396.8 | 856.8 | 320.14 |
| 4, go straight | 265 | 4646.4 | 1107.6 | 319.79 |
| 4, turn 90 deg | 353 | 4798.2 | 933.6 | 415.51 |

measuring first line of first iteration we get 246 which somewhat coincides with what we found in part 2: e=0.09(0.6)+0.2=0.254.
The distance traveled from the wheel encoders tell us that the robot went 248.7 mm but not by much