"""KytoonSpec — pydantic models mirroring the YAML design specs.

Units: SI throughout (m, m², m³, kg, Pa, N) unless the field name says otherwise.
"""
from __future__ import annotations

import math
from enum import Enum
from pathlib import Path

import yaml
from pydantic import BaseModel, Field, model_validator


class Archetype(str, Enum):
    LEI = "lei"              # Mk I  — leading-edge-inflatable traction wing
    HELIKITE = "helikite"    # Mk II — buoyant lobe + keel wing
    SPINE = "spine"          # Mk III — semi-rigid spar + twin-skin canopy
    TORUS = "torus"          # Mk IV — annular lifting body


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
        }[self.archetype]
        for field in need:
            if getattr(self, field) is None:
                raise ValueError(f"{self.archetype.value} spec requires '{field}'")
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
        return m

    @property
    def total_mass(self) -> float:
        """Kytoon flying mass, excluding tether (tether handled separately)."""
        return self.structure_mass + self.payload_mass

    @property
    def wing_area(self) -> float:
        return self.canopy.area if self.canopy else 0.0


def load_spec(path: str | Path) -> KytoonSpec:
    with open(path) as f:
        return KytoonSpec.model_validate(yaml.safe_load(f))


def load_all(directory: str | Path) -> list[KytoonSpec]:
    specs = [load_spec(p) for p in sorted(Path(directory).glob("*.yaml"))]
    return specs
