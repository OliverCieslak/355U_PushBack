## Vex Robotics Heading Convention

In this codebase, all angles and headings are expressed in compass degrees (0° = North, 90° = East, 180° = South, 270° = West), following the Vex Robotics convention. Always use compass heading for orientation and sensor calculations.
## Dev build of units install

```bash
pros conduct fetch units@0.7.2-dev.zip
pros conduct apply units@0.7.2-dev
```

## Dev build of hardware install (local build)

```bash
#pros conduct fetch hardware@0.4.2.zip
#pros conduct apply hardware@0.4.2
pros conduct fetch hardware@0.4.3-dev.zip
pros conduct apply hardware@0.4.3-dev

```

## Install robodash

```bash
pros c add-depot robodash https://raw.githubusercontent.com/unwieldycat/robodash/depot/stable.json
pros c apply robodash
```

## Performance notes:

Changing the ParticleFilter from using `std::exp` to `utils::fastExp` reduced the
`updateSensor` time from about 13.7ms to about 8.5ms for 300 particles

Before:
```
===== PARTICLE FILTER PERFORMANCE (300 particles, 2 sensors) =====
  Motion update:  3.59 ms (19.52%)
  Sensor update:  13.65 ms (74.23%)
  Resample:       0.06 ms (0.31%)
  Total update:   18.38 ms
============================================================
```

After:
```
===== PARTICLE FILTER PERFORMANCE (300 particles, 2 sensors) =====
  Motion update:  3.72 ms (27.75%)
  Sensor update:  8.49 ms (63.33%)
  Resample:       0.06 ms (0.42%)
  Total update:   13.40 ms
============================================================
```

Updating to use `utils::fastSin` and `utils::fastCos` improved the `updateMotion`
from about 3.7ms to 2.6ms

Before:
```
===== PARTICLE FILTER PERFORMANCE (300 particles, 2 sensors) =====
  Motion update:  3.72 ms (27.75%)
  Sensor update:  8.49 ms (63.33%)
  Resample:       0.06 ms (0.42%)
  Total update:   13.40 ms
============================================================
```
After:
```
===== PARTICLE FILTER PERFORMANCE (300 particles, 2 sensors) =====
  Motion update:  2.57 ms (22.66%)
  Sensor update:  8.49 ms (74.93%)
  Resample:       0.06 ms (0.54%)
  Total update:   11.34 ms
============================================================
```

After changing some missed some calls in calculateExpectedDistance
and notice going from 300 to 1000 particles:
```
===== PARTICLE FILTER PERFORMANCE (1000 particles, 2 sensors) =====
  Motion update:  8.20 ms (62.59%)
  Sensor update:  4.19 ms (31.99%)
  Resample:       0.03 ms (0.23%)
  Total update:   13.10 ms
============================================================
```

Next, changing motionUpdate to use fastNorm instead of m_normalDist:
```
===== PARTICLE FILTER PERFORMANCE (1000 particles, 2 sensors) =====
  Motion update:  3.82 ms (44.26%)
  Sensor update:  4.15 ms (48.11%)
  Resample:       0.01 ms (0.15%)
  Total update:   8.64 ms
============================================================
```

For 500 particles, well under 10ms
```
===== PARTICLE FILTER PERFORMANCE (500 particles, 2 sensors) =====
  Motion update:  1.92 ms (44.45%)
  Sensor update:  2.07 ms (47.87%)
  Resample:       0.01 ms (0.14%)
  Total update:   4.32 ms
  Updates:        58 since last log
============================================================
```