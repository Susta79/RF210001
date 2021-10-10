# RF210001
FK and IK for 6DOF industrial robots.

## Define robot dimensions
```language
The middle point at the end of the axis 6 (J6) is called MP (mounting point).
The direction of the frame @ MP is the same as the robot base: X forward, Z upward.
Attached to MP is also a frame called UT0 (or tool0). The origin of the UT0 is @ MP. The direction can change between different robot brand.

ABB: tool0: Z forward, X downward = rotation of 90° around Y of the base frame.

```

### Industrial robot
| a1z | a2x | a2z | a3z | a4x | a4z | a5x | a6x |
|-----|-----|-----|-----|-----|-----|-----|-----|
| 650 | 400 | 680 | 1100| 766 | 230 | 345 | 244 |

#define a1z 650.0
#define a2x 400.0
#define a2z 680.0
#define a3z 1100.0
#define a4x 766.0
#define a4z 230.0
#define a5x 345.0
#define a6x 244.0

### ABB 4600-20/2.50
| a1z | a2x | a2z | a3z | a4x | a4z | a5x | a6x |
|-----|-----|-----|-----|-----|-----|-----|-----|
|  0  | 175 | 495 | 1095|1230.5|175 |  0  |  85 |

#define a1z 0.0
#define a2x 175.0
#define a2z 495.0
#define a3z 1095.0
#define a4x 1230.5
#define a4z 175.0
#define a5x 0.0
#define a6x 85.0

## Test dataset

| jpos  |  j1 |  j2 |  j3 |  j4 |  j5 |  j6 |
| ----- |:---:|:---:|:---:|:---:|:---:|:---:|
| jpos0 |   0 |   0 |   0 |   0 |   0 |   0 |
| jpos1 |  90 |   0 |   0 |   0 |   0 |   0 |
| jpos2 |  45 |  45 |  45 |  45 |  45 |  45 |
| jpos3 | -45 | -45 | -45 | -45 | -45 | -45 |
| jpos4 | 130 | -60 |  30 |  60 | -90 |  60 |
| jpos5 | -46 |  46 |  46 |  46 |  46 |  46 |

UT1 : User Tool 1
|  X  |  Y  |  Z  |  RX |  RY |  RZ |
|:---:|:---:|:---:|:---:|:---:|:---:|
| 200 | 100 | 300 |   0 |  60 |   0 |

UF1 = User Frame 1
|  X  |  Y  |  Z  |  RX |  RY |  RZ |
|:---:|:---:|:---:|:---:|:---:|:---:|
| 1000| -500| 750 |   0 |  60 |  45 |

### Industrial Robot

| jpos  |      X |      Y |      Z |     RX |     RY |     RZ |
| ----- |:------:|:------:|:------:|:------:|:------:|:------:|
| jpos0 |    1755|      0 |    2660|      0 |      0 |      0 |
| jpos1 |      0 |    1755|    2660|      0 |      0 |     90 |
| jpos4 |    11,8|   314,7|  2740,3|  -176,3|   -25,7|    23,9|
| jpos5 |   962,3|  -814,8|   810,6|    -132|    42,6|    89,3|

### ABB 4600-20/2.50

Quaternion: q1 = w; q2 = x; q3 = y; q4 = z;

| jpos  |      X |      Y |      Z |     RX |     RY |     RZ |
| ----- |:------:|:------:|:------:|:------:|:------:|:------:|
| jpos0 | 1490,50|     0,0| 1765,00|    0,00|   90,00|    0,00|
| jpos1 |    0,00| 1490,50| 1765,00|    0,00|   90,00|  -90,00|
| jpos2 |  734,88|  794,99|  -21,32| -144,74|  -30,00|  125,26|
| jpos3 | -547,50|  607,60| 2559,89|   35,26|  -30,00| -144,74|
