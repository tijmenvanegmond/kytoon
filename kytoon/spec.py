"""KytoonSpec — pydantic models mirroring the YAML design specs.

Units: SI throughout (m, m², m³, kg, Pa, N) unless the field name says otherwise.
"""
from __future__ import annotations

import math
from enum import Enum
from pathlib import Path
from typing import ClassVar

import yaml
from pydantic import BaseModel, Field, model_validator


class Archetype(str, Enum):
    LEI = "lei"              # Mk I  — leading-edge-inflatable traction wing
    HELIKITE = "helikite"    # Mk II — buoyant lobe + keel wing
    SPINE = "spine"          # Mk III — semi-rigid spar + twin-skin canopy
    TORUS = "torus"          # Mk IV — annular lifting body
    FATWING = "fatwing"      # Mk V  — multi-tube inflatable fat wing
    BLIMP = "blimp"          # alternate (specs/alternates/): hull + side wings


class InflatableTube(BaseModel):
    """A pressurized fabric tube (LE tube, strut, or keel spar)."""
    diameter: float = Field(gt=0, description="tube diameter [m]")
    length: float = Field(gt=0, description="developed length [m]")
    pressure_bar: float = Field(gt=0, description="gauge pressure [bar]")
    fabric_areal_density: float = Field(0.30, gt=0, description="kg/m² incl. bladder")
    fabric_strength_n_per_m: float = Field(
        60_000, gt=0, description="fabric tensile strength [N/m] (warp)"
    )
    gas: str = Field("helium", description="'helium' or 'air'")

    @property
    def radius(self) -> float:
        return self.diameter / 2

    @property
    def pressure_pa(self) -> float:
        return self.pressure_bar * 1e5

    @property
    def volume(self) -> float:
        return math.pi * self.radius**2 * self.length

    @property
    def skin_area(self) -> float:
        return math.pi * self.diameter * self.length

    @property
    def mass(self) -> float:
        return self.skin_area * self.fabric_areal_density


class TorusEnvelope(BaseModel):
    """Toroidal envelope: ring (major) diameter and tube (minor) diameter."""
    ring_diameter: float = Field(gt=0, description="major diameter, centerline [m]")
    tube_diameter: float = Field(gt=0, description="minor (tube) diameter [m]")
    pressure_bar: float = Field(0.02, gt=0)
    fabric_areal_density: float = Field(0.25, gt=0)
    fabric_strength_n_per_m: float = Field(60_000, gt=0)

    @property
    def volume(self) -> float:
        R = self.ring_diameter / 2
        r = self.tube_diameter / 2
        return 2 * math.pi**2 * R * r**2

    @property
    def skin_area(self) -> float:
        R = self.ring_diameter / 2
        r = self.tube_diameter / 2
        return 4 * math.pi**2 * R * r

    @property
    def mass(self) -> float:
        return self.skin_area * self.fabric_areal_density


class Canopy(BaseModel):
    area: float = Field(gt=0, description="projected wing area [m²]")
    span: float = Field(gt=0, description="projected span [m]")
    areal_density: float = Field(0.15, gt=0, description="kg/m² canopy fabric")
    cl_max: float = Field(1.0, gt=0, description="max lift coefficient")
    cl_op: float = Field(0.8, gt=0, description="operating lift coefficient")
    cd_op: float = Field(0.15, gt=0, description="operating drag coefficient")
    twin_skin: bool = Field(False, description="double-surface canopy (2× fabric)")

    @property
    def aspect_ratio(self) -> float:
        return self.span**2 / self.area

    @property
    def mass(self) -> float:
        mult = 2.0 if self.twin_skin else 1.0
        return self.area * self.areal_density * mult

    @property
    def cr_op(self) -> float:
        """Resultant aerodynamic force coefficient."""
        return math.hypot(self.cl_op, self.cd_op)


class Lobe(BaseModel):
    """Oblate spheroid He lobe (Helikite-style)."""
    diameter: float = Field(gt=0, description="equatorial diameter [m]")
    height: float = Field(gt=0, description="polar height [m]")
    fabric_areal_density: float = Field(0.20, gt=0)

    @property
    def volume(self) -> float:
        a = self.diameter / 2
        c = self.height / 2
        return 4 / 3 * math.pi * a * a * c

    @property
    def skin_area(self) -> float:
        # Thomsen approximation for oblate spheroid
        a = self.diameter / 2
        c = self.height / 2
        p = 1.6075
        return 4 * math.pi * ((a**p * a**p + 2 * (a**p * c**p)) / 3) ** (1 / p)

    @property
    def mass(self) -> float:
        return self.skin_area * self.fabric_areal_density


class Hull(BaseModel):
    """Prolate spheroid He hull (blimp-style), long axis into the wind."""
    length: float = Field(gt=0, description="hull length [m]")
    diameter: float = Field(gt=0, description="max hull diameter [m]")
    fabric_areal_density: float = Field(0.20, gt=0)
    fabric_strength_n_per_m: float = Field(60_000, gt=0)

    @property
    def volume(self) -> float:
        a = self.diameter / 2
        c = self.length / 2
        return 4 / 3 * math.pi * a * a * c

    @property
    def skin_area(self) -> float:
        # Thomsen approximation for prolate spheroid
        a = self.diameter / 2
        c = self.length / 2
        p = 1.6075
        return 4 * math.pi * ((a**p * a**p + 2 * (a**p * c**p)) / 3) ** (1 / p)

    @property
    def mass(self) -> float:
        return self.skin_area * self.fabric_areal_density


class FatWing(BaseModel):
    """Lofted inflatable fat wing (manta-style): closed airfoil sections
    scaled by a linearly tapering local chord c(y) = c₀·(1 − (1−λ)·|2y/b|)
    — fat in the middle, thin at the tips. One pressurized body divided
    into n_cells chordwise cells by internal webs; the web pitch sets the
    skin-bulge hoop check, the section depth sets the equivalent-beam
    wrinkle check. The wing IS the buoyant volume and the structure; the
    spec's `canopy` carries only the aero coefficients (+ doubler mass).

    Closed forms (NACA-4 thickness law, K_A = 0.681, perimeter ≈ 2.1·c):
      section area A(y) = K_A · t(y) · c(y),  t = thickness_ratio · c
      volume = K_A·(t/c)·c₀²·b·(λ²+λ+1)/3
      skin   = 2.1·c₀·b·(1+λ)/2   (+20% mass for internal webs)
    """
    span: float = Field(gt=0, description="wing span [m]")
    chord: float = Field(gt=0, description="CENTER chord c₀ [m]")
    taper: float = Field(0.35, gt=0, le=1, description="tip/center chord λ")
    thickness_ratio: float = Field(0.28, gt=0, lt=0.5, description="t/c")
    n_cells: int = Field(5, ge=2, description="chordwise pressure cells")
    pressure_bar: float = Field(0.10, gt=0)
    fabric_areal_density: float = Field(0.20, gt=0)
    fabric_strength_n_per_m: float = Field(120_000, gt=0)  # load-taped
    gas: str = Field("helium", description="'helium' or 'air'")

    K_A: ClassVar[float] = 0.681   # ∫ NACA-4 closed-TE thickness dx / (t·c)
    K_P: ClassVar[float] = 2.1     # airfoil perimeter / chord (fat section)
    WEB_MASS_FACTOR: ClassVar[float] = 1.2

    @property
    def planform_area(self) -> float:
        """∫c dy = b·c₀·(1+λ)/2 for linear taper."""
        return self.span * self.chord * (1 + self.taper) / 2

    def chord_at(self, y_frac: float) -> float:
        """Local chord at |2y/b| = y_frac ∈ [0, 1]."""
        return self.chord * (1 - (1 - self.taper) * y_frac)

    @property
    def t_max(self) -> float:
        return self.thickness_ratio * self.chord

    @property
    def cell_pitch(self) -> float:
        return self.chord / self.n_cells

    @property
    def equivalent_tube(self) -> InflatableTube:
        """Center-section pressurized beam of diameter t_max — a
        conservative stand-in for the wing box's wrinkle capacity."""
        return InflatableTube(
            diameter=self.t_max, length=self.span,
            pressure_bar=self.pressure_bar,
            fabric_areal_density=self.fabric_areal_density,
            fabric_strength_n_per_m=self.fabric_strength_n_per_m,
            gas=self.gas,
        )

    @property
    def volume(self) -> float:
        factor = self.span * (self.taper**2 + self.taper + 1) / 3
        return self.K_A * self.t_max * self.chord * factor

    @property
    def skin_area(self) -> float:
        return self.K_P * self.chord * self.span * (1 + self.taper) / 2

    @property
    def mass(self) -> float:
        return self.skin_area * self.fabric_areal_density * self.WEB_MASS_FACTOR


class Tether(BaseModel):
    length: float = Field(400, gt=0, description="deployed length [m]")
    diameter_mm: float = Field(14, gt=0)
    linear_density: float = Field(0.11, gt=0, description="kg/m (Dyneema + jacket)")
    mbl_kn: float = Field(150, gt=0, description="minimum breaking load [kN]")
    safety_factor: float = Field(3.0, gt=1)
    elevation_deg: float = Field(40, gt=0, lt=90, description="nominal tether elevation")

    @property
    def mass(self) -> float:
        return self.length * self.linear_density

    @property
    def wll_n(self) -> float:
        return self.mbl_kn * 1e3 / self.safety_factor


class BridleAttachment(BaseModel):
    """Bridle support positions along the spar, as span fractions [0..1]."""
    positions: list[float] = Field(default_factory=lambda: [0.25, 0.75])


class KytoonSpec(BaseModel):
    name: str
    mk: str
    archetype: Archetype
    canopy: Canopy | None = None
    le_tube: InflatableTube | None = None
    struts: InflatableTube | None = None
    n_struts: int = 0
    spar: InflatableTube | None = None
    lobe: Lobe | None = None
    torus: TorusEnvelope | None = None
    fat_wing: FatWing | None = None
    hull: Hull | None = None
    tether: Tether = Field(default_factory=Tether)
    bridle: BridleAttachment = Field(default_factory=BridleAttachment)
    payload_mass: float = Field(0, ge=0, description="avionics/pod/dock hardware [kg]")
    dock_capacity: float = Field(0, ge=0, description="perched-drone mass rating [kg]")
    rigging_mass: float = Field(0, ge=0, description="bridle lines, fittings [kg]")
    notes: str = ""

    @model_validator(mode="after")
    def _check_archetype(self) -> "KytoonSpec":
        need = {
            Archetype.LEI: ("canopy", "le_tube"),
            Archetype.HELIKITE: ("canopy", "lobe"),
            Archetype.SPINE: ("canopy", "spar"),
            Archetype.TORUS: ("torus",),
            Archetype.FATWING: ("canopy", "fat_wing"),
            Archetype.BLIMP: ("canopy", "hull"),
        }[self.archetype]
        for field in need:
            if getattr(self, field) is None:
                raise ValueError(f"{self.archetype.value} spec requires '{field}'")
        if self.fat_wing is not None and self.canopy is not None:
            # one wing, two views of it: planforms must agree
            planform = self.fat_wing.planform_area
            if abs(planform - self.canopy.area) > 0.05 * self.canopy.area:
                raise ValueError(
                    f"fatwing planform {planform:.0f} m² disagrees with "
                    f"canopy area {self.canopy.area:.0f} m²")
        return self

    # ---- aggregates -------------------------------------------------------
    @property
    def helium_volume(self) -> float:
        v = 0.0
        for tube in (self.le_tube, self.spar):
            if tube is not None and tube.gas == "helium":
                v += tube.volume
        if self.struts is not None and self.struts.gas == "helium":
            v += self.n_struts * self.struts.volume
        if self.lobe is not None:
            v += self.lobe.volume
        if self.torus is not None:
            v += self.torus.volume
        if self.fat_wing is not None and self.fat_wing.gas == "helium":
            v += self.fat_wing.volume
        if self.hull is not None:
            v += self.hull.volume
        return v

    @property
    def structure_mass(self) -> float:
        m = self.rigging_mass
        if self.canopy:
            m += self.canopy.mass
        for tube in (self.le_tube, self.spar):
            if tube is not None:
                m += tube.mass
        if self.struts is not None:
            m += self.n_struts * self.struts.mass
        if self.lobe is not None:
            m += self.lobe.mass
        if self.torus is not None:
            m += self.torus.mass
        if self.fat_wing is not None:
            m += self.fat_wing.mass
        if self.hull is not None:
            m += self.hull.mass
        return m

    @property
    def total_mass(self) -> float:
        """Kytoon flying mass, excluding tether (tether handled separately)."""
        return self.structure_mass + self.payload_mass

    @property
    def wing_area(self) -> float:
        return self.canopy.area if self.canopy else 0.0


def load_spec(path: str | Path) -> KytoonSpec:
    # explicit utf-8: spec names carry «» — Windows' cp1252 default mangles them
    with open(path, encoding="utf-8") as f:
        return KytoonSpec.model_validate(yaml.safe_load(f))


def load_all(directory: str | Path) -> list[KytoonSpec]:
    specs = [load_spec(p) for p in sorted(Path(directory).glob("*.yaml"))]
    return specs
