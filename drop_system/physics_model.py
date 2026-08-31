"""Ballistic / aerodynamic physics model for payload trajectory.

MASS EFFECT REVIEW (spec section 17A/153, required before use):

 1. F = m*a                       — Newton's second law, translational.
 2. F_gravity = m*g                — weight force.
 3. a_gravity = F_gravity/m = g    — mass cancels; free-fall acceleration
    is independent of mass. This is why the ballistic-only model below
    produces IDENTICAL trajectories for 0.5 kg and 1.0 kg payloads
    (verified in tests/test_mass_physics.py).
 4. F_drag = -0.5*rho*Cd*A*|Vrel|*Vrel — quadratic drag force.
 5. a_drag = F_drag/m               — mass does NOT cancel here; heavier
    payload -> smaller |a_drag| for identical rho/Cd/A/Vrel.
 6. beta = m/(Cd*A)                 — ballistic coefficient, only defined
    when Cd and A are known (never invented).
 7. I_axis, I_transverse are functions of m and geometry, used only as
    payload properties (reported for traceability) — never substituted
    into the translational F=m*a equation.
 8. Mass affects trajectory ONLY when the drag model is enabled.
 9. Mass has NO effect on trajectory in the ballistic-only (drag
    disabled) model.
10. Monte Carlo may sample mass around a mean with a sigma ONLY when
    that sigma has a documented source (measured weighings or an
    explicit CONFIGURED/ASSUMED test value) — never an arbitrary
    fraction of the nominal mass.

No formula here is adjusted to force impact error under any threshold;
accuracy performance requirements are evaluated downstream, not by
altering these equations.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from config import GRAVITY_MPS2


@dataclass
class MassProperties:
    mass_kg: float
    mass_source: str  # MEASURED / CONFIGURED / ASSUMED / UNKNOWN
    diameter_m: Optional[float] = None
    cylinder_length_m: Optional[float] = None
    nose_length_m: Optional[float] = None

    @property
    def radius_m(self) -> Optional[float]:
        if self.diameter_m is None:
            return None
        return self.diameter_m / 2.0

    def frontal_area_m2(self) -> Optional[float]:
        r = self.radius_m
        if r is None:
            return None
        return math.pi * r * r

    def volume_m3(self) -> Optional[float]:
        r = self.radius_m
        if r is None or self.cylinder_length_m is None or self.nose_length_m is None:
            return None
        v_cyl = math.pi * r * r * self.cylinder_length_m
        v_cone = (1.0 / 3.0) * math.pi * r * r * self.nose_length_m
        return v_cyl + v_cone

    def moment_of_inertia_axis(self) -> Optional[float]:
        """I_axis = 0.5*m*r^2 (solid-cylinder approximation). Property
        only — never used in the translational F=m*a equation.
        """
        r = self.radius_m
        if r is None:
            return None
        return 0.5 * self.mass_kg * r * r

    def moment_of_inertia_transverse(self) -> Optional[float]:
        """I_transverse = (1/12)*m*(3r^2 + L^2)."""
        r = self.radius_m
        if r is None or self.cylinder_length_m is None:
            return None
        length = self.cylinder_length_m + (self.nose_length_m or 0.0)
        return (1.0 / 12.0) * self.mass_kg * (3.0 * r * r + length * length)


def gravity_force_n(mass_kg: float, g: float = GRAVITY_MPS2) -> float:
    """F_gravity = m*g."""
    return mass_kg * g


def gravity_acceleration(g: float = GRAVITY_MPS2) -> float:
    """a_gravity = F_gravity/m = g. Mass-independent by construction —
    do not pass mass into this function; that would be the anti-pattern
    the spec explicitly forbids (section 17A-N).
    """
    return g


def time_of_flight(height_m: float, vz0_mps: float = 0.0, g: float = GRAVITY_MPS2) -> float:
    """Solve z(t) = h + Vz0*t - 0.5*g*t^2 = 0 for the positive root.

    height_m : release altitude above ground (m), must be >= 0.
    vz0_mps  : initial vertical velocity at release (m/s), + = upward.
    Returns t_flight in seconds. Raises ValueError for non-physical input.
    """
    if height_m < 0:
        raise ValueError(f"height_m must be >= 0, got {height_m}")
    if g <= 0:
        raise ValueError(f"g must be > 0, got {g}")

    if vz0_mps == 0.0:
        return math.sqrt(2.0 * height_m / g)

    discriminant = vz0_mps ** 2 + 2.0 * g * height_m
    if discriminant < 0:
        raise ValueError("No real solution for time of flight (negative discriminant)")
    t = (vz0_mps + math.sqrt(discriminant)) / g
    if t < 0:
        raise ValueError(f"Computed negative time of flight ({t}); check vz0/height inputs")
    return t


def ballistic_displacement(v_horizontal_mps: float, t_flight_s: float) -> float:
    """Horizontal drop distance under ballistic (no-drag) assumption:
    D = V_horizontal * t_flight. Valid because horizontal velocity is
    unaffected by gravity (acts vertically only) when drag is disabled.
    """
    return v_horizontal_mps * t_flight_s


def ballistic_coefficient(mass_kg: float, cd: Optional[float],
                           area_m2: Optional[float]) -> Optional[float]:
    """beta = m/(Cd*A). Returns None (UNKNOWN) if Cd or A is unavailable
    — never fabricated.
    """
    if cd is None or area_m2 is None or cd <= 0 or area_m2 <= 0:
        return None
    return mass_kg / (cd * area_m2)


def drag_acceleration(rho: float, cd: float, area_m2: float, mass_kg: float,
                       v_rel_n: float, v_rel_e: float, v_rel_u: float) -> tuple:
    """a_drag = -(rho*Cd*A)/(2*m) * |Vrel| * Vrel, per axis.

    All three velocity-relative components must be expressed in the
    same coordinate frame as the payload's ground/air motion (spec
    section 149). Returns (a_N, a_E, a_U).
    """
    v_rel_mag = math.sqrt(v_rel_n ** 2 + v_rel_e ** 2 + v_rel_u ** 2)
    coeff = -(rho * cd * area_m2) / (2.0 * mass_kg)
    return (
        coeff * v_rel_mag * v_rel_n,
        coeff * v_rel_mag * v_rel_e,
        coeff * v_rel_mag * v_rel_u,
    )


def integrate_trajectory_with_drag(
    mass_kg: float,
    rho: float,
    cd: float,
    area_m2: float,
    v0_n: float, v0_e: float, v0_u: float,
    wind_n: float, wind_e: float, wind_u: float,
    height_m: float,
    g: float = GRAVITY_MPS2,
    dt: float = 0.005,
    max_time_s: float = 60.0,
) -> dict:
    """Simple, stable semi-implicit (symplectic) Euler integrator for the
    drag-enabled 3-DOF translational trajectory. Only invoked when Cd/A
    are known (drag_enabled and both non-None) — never with fabricated
    values.

    Returns dict with keys: t_flight, north, east, drop_distance,
    trajectory (list of (t, n, e, u) samples, coarse).
    """
    n, e, u = 0.0, 0.0, height_m
    vn, ve, vu = v0_n, v0_e, v0_u
    t = 0.0
    trajectory = [(t, n, e, u)]

    while u > 0.0 and t < max_time_s:
        v_rel_n = vn - wind_n
        v_rel_e = ve - wind_e
        v_rel_u = vu - wind_u
        a_n, a_e, a_u = drag_acceleration(rho, cd, area_m2, mass_kg,
                                           v_rel_n, v_rel_e, v_rel_u)
        a_u -= g  # gravity acts downward on vertical axis

        vn += a_n * dt
        ve += a_e * dt
        vu += a_u * dt

        n += vn * dt
        e += ve * dt
        u += vu * dt

        t += dt
        trajectory.append((t, n, e, u))

    return {
        "t_flight": t,
        "north": n,
        "east": e,
        "drop_distance": math.hypot(n, e),
        "trajectory": trajectory,
    }
