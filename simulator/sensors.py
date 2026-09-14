"""
Force-sensor signal chain: noise, moving average and first-order low-pass.

What
----
:class:ForceSensor reproduces the firmware's conditioning of the load-cell
channel: an optional Gaussian noise source (the stochastic part of ADC
quantisation and amplifier noise), a moving average of
`FILTER_FORCE_MA_WINDOW` samples and a first-order low-pass with
`FILTER_FORCE_LPF_TAU_S`.

Why it is modelled at all
-------------------------
Scenario C exists to show what the raw and the filtered channels look like with
`sigma = 0.08 N` of noise, and to quantify how much of the force loop's margin
that filtering buys.  Without a sensor model the controller would be tested
against a signal no real load cell ever produces.

What happens without it
-----------------------
The force loop would appear far better damped than it is on hardware, and a gain
that is stable in simulation would chatter on the real gripper.

Units
-----
Force in N, time in s, `adc_counts` in 12-bit counts.
"""

from __future__ import annotations

from typing import Optional

from . import config as C
from .pid import LowPassFilter, MovingAverage, _finite


class ForceSensor:
    """Force-sensor signal chain: noise, moving average and first-order LPF.

    What
   ----
    Reproduces the firmware's conditioning of the load-cell channel: an optional
    Gaussian noise source (the stochastic part of ADC quantisation and amplifier
    noise), a moving average of :data:`config.FILTER_FORCE_MA_WINDOW` samples and
    a first-order low-pass with ``TP_FILTER_FORCE_LPF_TAU_S``.

    Why
    ---
    Scenario C exists to show what the raw and the filtered channels look like
    with ``sigma = 0.08 N`` of noise, and to quantify how much of the force loop's
    stability margin that filtering buys.  Without a sensor model the controller
    would be tested against a signal no real load cell ever produces.

    What happens without it
    -----------------------
    The force loop would appear far better damped than it is on hardware, and a
    gain that is stable in simulation would chatter on the real gripper.
    """

    def __init__(
        self,
        dt: float = C.CONTROL_DT_S,
        noise_sigma_n: float = 0.0,
        enable_filter: bool = True,
    ) -> None:
        """Create the sensor chain.

        Parameters
        ----------
        dt:
            Sample time, seconds.
        noise_sigma_n:
            Standard deviation of the additive Gaussian noise, N.  ``0.0``
            disables the noise source entirely (bit-exact determinism).
        enable_filter:
            When ``False`` only the moving average is kept, which gives
            Scenario C its "unfiltered" comparison channel.
        """
        self.dt: float = max(C.EPS, _finite(dt, C.CONTROL_DT_S))
        self.noise_sigma_n: float = max(0.0, _finite(noise_sigma_n))
        self.enable_filter: bool = bool(enable_filter)
        self.ma = MovingAverage(C.FILTER_FORCE_MA_WINDOW)
        self.lpf = LowPassFilter(
            C.FILTER_FORCE_LPF_TAU_S if self.enable_filter else 0.0,
            self.dt,
        )

    def reset(self) -> None:
        """Clear the filter states."""
        self.ma.reset()
        self.lpf.reset(0.0)

    def sample(self, force_n: float, rng: Optional[object] = None) -> Dict[str, float]:
        """Read the sensor once.

        Parameters
        ----------
        force_n:
            True force acting on the jaw, N.
        rng:
            Optional :class:`numpy.random.Generator` used for the noise.  When
            ``None`` or when the noise sigma is zero, no randomness is consumed.

        Returns
        -------
        dict
            ``raw``, ``filtered``, ``adc_counts`` -- ``raw`` includes the noise,
            ``filtered`` is the conditioned channel the controller consumes.
        """
        true_f = _finite(force_n)
        noisy = true_f
        if self.noise_sigma_n > 0.0 and rng is not None:
            noisy = true_f + float(rng.normal(0.0, self.noise_sigma_n))
        averaged = self.ma.update(noisy)
        filtered = self.lpf.update(averaged)
        counts = C.FORCE_CAL_ZERO_COUNTS + true_f / max(C.FORCE_CAL_SCALE_N_PER_COUNT, C.EPS)
        return {
            "raw": _finite(noisy),
            "filtered": _finite(filtered),
            "true": true_f,
            "adc_counts": _finite(counts),
        }
