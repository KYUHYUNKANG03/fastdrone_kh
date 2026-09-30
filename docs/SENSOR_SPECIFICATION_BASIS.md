# Basis for sensor specifications before tuning

Reviewed 2026-09-30. The active v9 profile is an **engineering simulation
profile**, not a calibrated model of the team's BMI270, QMC5883L or ESC.
This note records the distinction without changing the running experiment.
Hardware names come from the team's supplied state/sensor table; board settings
and flight logs have not been supplied.

## What the active profile actually represents

Source: `configs/sensors/rotor_projected_development_v9.json` and the measurement
generator in `control/arena_sensors.py`.

| Quantity | Active value | Meaning / unresolved hardware mapping |
|---|---:|---|
| IMU sample rate | 500 Hz | Simulated packet rate on 2 ms plant ticks; not a documented chip-register setting |
| Gyro noise parameter | 0.002 rad/s/sqrt(Hz) | Generator uses sample sigma = parameter times sqrt(500) |
| Acceleration noise parameter | 0.03 m/s²/sqrt(Hz) | Same discrete convention |
| Gyro clipping | 50 rad/s | About 2865 deg/s; not a BMI270 selectable full-scale range |
| Acceleration clipping | 100 m/s² | About 10.2 g; not a BMI270 selectable full-scale range |
| GNSS noise | 0.8 m position; 0.15 m/s velocity | Per-axis Gaussian standard deviations, not unspecified CEP/95% accuracy figures |
| GNSS transport | 10 Hz, 120 ms | Fixed delay; this profile has no random arrival jitter |
| Barometer | 50 Hz, 0.25 m sigma, 20 ms delay | Height-domain approximation; pressure errors at speed need separate justification |
| Magnetic field / noise | [20, 0, 40]; sigma 0.04 | Common field units are internally consistent but not explicitly labeled in the schema |
| Rotor telemetry | 500 Hz, 8 rad/s sigma, 4 ms delay | Mechanical angular speed; per-rotor rate and actual telemetry protocol remain unverified |

At 500 Hz, the implemented independent sample standard deviations are
0.04472 rad/s for gyro and 0.67082 m/s² for acceleration. These are calculations
from the generator, not measured device performance. ESKF process covariance,
measurement covariance, bias random walk, and generated measurement noise are
separate settings even when their defaults share a parameter.

## Manufacturer anchors

Bosch's BMI270 product page lists typical acceleration density 160 micro-g/sqrt(Hz)
and gyro density 0.007 deg/s/sqrt(Hz), with configurable bandwidth, output rate
and ranges up to ±16 g and ±2000 deg/s. The linked datasheet retrieved for this
review is revision 1.6, March 2026, BST-BMI270-DS000-08. Record the operating mode
when using these specifications. Sources:
[Bosch BMI270 product specifications](https://www.bosch-sensortec.com/en/products/motion-sensors/imus/bmi270),
[Bosch datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf).

Converting units alone gives 0.001569064 m/s²/sqrt(Hz) and
0.000122173 rad/s/sqrt(Hz). The simulation parameters are respectively about
19.1 and 16.4 times these numbers. **These are numerical density ratios, not
ratios of in-flight error or evidence that the simulated profile bounds all
hardware errors.** Their PSD convention and effective bandwidth have not been
matched, and vibration, installation, temperature and calibration can matter.

QST's QMC5883L revision B specification, table 2, gives a typical 2 mGauss
field-resolution standard deviation over 100 data points at ±2 Gauss full
scale, and selectable 10/50/100/200 Hz output rates. It also lists offset and
cross-axis effects separately. Source:
[QST QMC5883L datasheet](https://www.qstcorp.com/upload/pdf/202512/13-52-04%20QMC5883L%20Datasheet%20Rev.%20B.pdf).

Two mGauss is 0.2 microtesla. If the simulation's field unit is declared to be
microtesla, its 0.04 noise sigma would be one fifth of that typical figure.
That unit interpretation must be stated explicitly before a hardware match.
A compass heading-accuracy headline cannot be substituted directly for a
three-axis field covariance, especially with motor-current magnetic effects.

## Convert bandwidth as well as units

For a flat **one-sided** amplitude density `N1` and a unity-DC-gain sensor
filter `H(f)`, define its one-sided equivalent noise bandwidth as

```text
B_ENBW = integral from 0 to infinity of |H(f)|² df
sigma_output² = N1² B_ENBW
```

The current generator instead specifies independent samples with
`sigma_sim² = D_sim² f_sample`. Matching only the variance therefore requires

```text
D_sim = N1 sqrt(B_ENBW / f_sample)
```

For the illustrative ideal brick-wall bandwidth `f_sample/2`, this becomes
`D_sim = N1/sqrt(2)`. That idealization is not an identified BMI270 filter.
Variance matching alone does not reproduce colored noise, filter group delay,
aliasing or the joint accelerometer/gyro timing. A filtered hardware profile
needs either a specified transfer function and sampling chain or measured
sample statistics with an explicit approximation. Do not silently change the
existing convention and reinterpret archived results.

## Information needed for a hardware-specific profile

1. Actual board/sensor revisions, range registers, chip ODR, onboard filtering,
   driver readout/decimation and controller-consumed rate. Characterize elapsed
   time from physical sensing to availability, including filter delay.
2. Bias after calibration, drift with time/temperature, axis alignment and
   measured stationary/propeller-running noise spectra. White noise and bias
   random walk describe different processes; a maximum offset is not a random
   walk coefficient.
3. GNSS receiver/mode, position and velocity accuracy definitions, latency,
   update timing, outages and common errors. Avoid converting CEP or 95%
   bounds into independent Gaussian sigma without stating a distribution.
4. Barometer pressure-to-height convention, placement/airspeed pressure effects
   and preflight reference uncertainty. The current 100-sample calibration
   assumes a known stationary height reference.
5. Magnetic units, installation calibration and current-dependent interference;
   ESC firmware/protocol, electrical-versus-mechanical speed, motor pole pairs,
   actual per-rotor sample rate, dropouts and latency. The observer's assumed
   20 ms motor constant is separate from these telemetry properties.

Hardware calibration is not required to report a bounded simulation study
under declared engineering assumptions. It is required to identify a profile
as a measured hardware specification. The existing profile remains fixed for
the current integration/GNSS study; a future hardware-matched profile must be
a separately named, source-bound comparison with its own validation.
