"""Unit tests for GPS jammer."""

import math
import time
import pytest
from swarm.navigation.gps_jammer import (
    DenialZoneShape,
    GPSDenialZone,
    GPSJammerConfig,
    GPSJammer,
    create_urban_denial_zones,
    create_canyon_denial_zone,
)


class TestGPSDenialZone:
    """Tests for GPSDenialZone."""

    def test_sphere_containment(self):
        """Sphere zone should contain points within radius."""
        zone = GPSDenialZone(
            zone_id=1,
            center=(50.0, 50.0, -10.0),
            radius=20.0,
            shape=DenialZoneShape.SPHERE,
        )

        # Point at center
        assert zone.contains((50.0, 50.0, -10.0))

        # Point inside
        assert zone.contains((60.0, 50.0, -10.0))  # 10m away

        # Point outside
        assert not zone.contains((80.0, 50.0, -10.0))  # 30m away

    def test_cylinder_containment(self):
        """Cylinder zone should contain points within horizontal radius."""
        zone = GPSDenialZone(
            zone_id=1,
            center=(50.0, 50.0, -10.0),
            radius=15.0,
            shape=DenialZoneShape.CYLINDER,
            height=30.0,
        )

        # Inside horizontal radius and height
        assert zone.contains((55.0, 50.0, -10.0))

        # Inside radius but outside height
        assert not zone.contains((55.0, 50.0, -50.0))

        # Outside radius
        assert not zone.contains((70.0, 50.0, -10.0))

    def test_cylinder_infinite_height(self):
        """Cylinder with no height should have infinite height."""
        zone = GPSDenialZone(
            zone_id=1,
            center=(50.0, 50.0, -10.0),
            radius=15.0,
            shape=DenialZoneShape.CYLINDER,
            height=None,  # Infinite
        )

        # Any altitude should be inside if horizontal is
        assert zone.contains((55.0, 50.0, -100.0))
        assert zone.contains((55.0, 50.0, -1.0))

    def test_box_containment(self):
        """Box zone should contain points within dimensions."""
        zone = GPSDenialZone(
            zone_id=1,
            center=(50.0, 50.0, -10.0),
            radius=0,  # Not used for box
            shape=DenialZoneShape.BOX,
            dimensions=(20.0, 30.0, 15.0),  # N, E, D
        )

        # Inside
        assert zone.contains((55.0, 60.0, -12.0))

        # Outside north
        assert not zone.contains((65.0, 50.0, -10.0))

        # Outside east
        assert not zone.contains((50.0, 70.0, -10.0))

    def test_inactive_zone(self):
        """Inactive zone should not contain any point."""
        zone = GPSDenialZone(
            zone_id=1,
            center=(0.0, 0.0, 0.0),
            radius=1000.0,
            active=False,
        )

        assert not zone.contains((0.0, 0.0, 0.0))

    def test_probability(self):
        """Probability should affect denial."""
        zone = GPSDenialZone(
            zone_id=1,
            center=(0.0, 0.0, 0.0),
            radius=50.0,
            probability=0.5,
        )

        # Inside zone should return probability
        prob = zone.get_denial_probability((10.0, 10.0, 0.0))
        assert prob == 0.5

        # Outside zone should return 0
        prob = zone.get_denial_probability((100.0, 100.0, 0.0))
        assert prob == 0.0


class TestGPSJammer:
    """Tests for GPSJammer."""

    def test_initialization(self):
        """Jammer should initialize correctly."""
        config = GPSJammerConfig(
            zones=[
                GPSDenialZone(zone_id=1, center=(0, 0, 0), radius=20.0),
            ],
        )

        jammer = GPSJammer(config)
        assert len(jammer.zones) == 1
        assert len(jammer.jammed_drones) == 0

    def test_zone_detection(self):
        """Jammer should detect positions in zones."""
        config = GPSJammerConfig(
            zones=[
                GPSDenialZone(
                    zone_id=1,
                    center=(50.0, 50.0, -10.0),
                    radius=20.0,
                ),
            ],
        )

        jammer = GPSJammer(config)

        assert jammer.is_in_denial_zone((50.0, 50.0, -10.0))
        assert not jammer.is_in_denial_zone((0.0, 0.0, -10.0))

    def test_add_zone(self):
        """Should be able to add zones dynamically."""
        jammer = GPSJammer(GPSJammerConfig())
        assert len(jammer.zones) == 0

        jammer.add_zone(GPSDenialZone(
            zone_id=1,
            center=(0, 0, 0),
            radius=10.0,
        ))

        assert len(jammer.zones) == 1

    def test_remove_zone(self):
        """Should be able to remove zones."""
        config = GPSJammerConfig(
            zones=[
                GPSDenialZone(zone_id=1, center=(0, 0, 0), radius=10.0),
                GPSDenialZone(zone_id=2, center=(50, 0, 0), radius=10.0),
            ],
        )

        jammer = GPSJammer(config)
        assert len(jammer.zones) == 2

        removed = jammer.remove_zone(1)
        assert removed
        assert len(jammer.zones) == 1

    def test_zone_activation(self):
        """Should be able to activate/deactivate zones."""
        config = GPSJammerConfig(
            zones=[
                GPSDenialZone(zone_id=1, center=(0, 0, 0), radius=50.0),
            ],
        )

        jammer = GPSJammer(config)
        assert jammer.is_in_denial_zone((10.0, 10.0, 0.0))

        jammer.set_zone_active(1, False)
        assert not jammer.is_in_denial_zone((10.0, 10.0, 0.0))

        jammer.set_zone_active(1, True)
        assert jammer.is_in_denial_zone((10.0, 10.0, 0.0))

    def test_multiple_zones(self):
        """Should detect position in any active zone."""
        config = GPSJammerConfig(
            zones=[
                GPSDenialZone(zone_id=1, center=(0, 0, 0), radius=20.0),
                GPSDenialZone(zone_id=2, center=(100, 0, 0), radius=20.0),
            ],
        )

        jammer = GPSJammer(config)

        assert jammer.is_in_denial_zone((10.0, 0.0, 0.0))  # In zone 1
        assert jammer.is_in_denial_zone((90.0, 0.0, 0.0))  # In zone 2
        assert not jammer.is_in_denial_zone((50.0, 0.0, 0.0))  # In neither

    def test_get_zones_at_position(self):
        """Should return all zones containing position."""
        config = GPSJammerConfig(
            zones=[
                GPSDenialZone(zone_id=1, center=(50, 50, 0), radius=30.0),
                GPSDenialZone(zone_id=2, center=(60, 50, 0), radius=30.0),
            ],
        )

        jammer = GPSJammer(config)

        # In both zones
        zones = jammer.get_denial_zones_at((55.0, 50.0, 0.0))
        assert len(zones) == 2

        # In zone 1 only
        zones = jammer.get_denial_zones_at((30.0, 50.0, 0.0))
        assert len(zones) == 1
        assert zones[0].zone_id == 1

    def test_stats(self):
        """Should track statistics."""
        config = GPSJammerConfig(
            zones=[GPSDenialZone(zone_id=1, center=(0, 0, 0), radius=10.0)],
        )

        jammer = GPSJammer(config)

        stats = jammer.get_stats()
        assert stats["total_zones"] == 1
        assert stats["active_zones"] == 1
        assert stats["currently_jammed"] == 0


class TestUrbanZoneGeneration:
    """Tests for urban zone generation."""

    def test_creates_zones(self):
        """Should create specified number of zones."""
        zones = create_urban_denial_zones(
            city_center=(100.0, 100.0, -20.0),
            num_buildings=5,
            seed=42,
        )

        assert len(zones) == 5

    def test_zones_near_center(self):
        """Zones should be within spread of center."""
        center = (100.0, 100.0, -20.0)
        spread = 50.0

        zones = create_urban_denial_zones(
            city_center=center,
            num_buildings=10,
            spread=spread,
            seed=42,
        )

        for zone in zones:
            dist = math.sqrt(
                (zone.center[0] - center[0])**2 +
                (zone.center[1] - center[1])**2
            )
            assert dist <= spread * 1.5  # Allow some margin

    def test_reproducible(self):
        """Same seed should produce same zones."""
        zones1 = create_urban_denial_zones(
            city_center=(0, 0, 0),
            num_buildings=3,
            seed=123,
        )

        zones2 = create_urban_denial_zones(
            city_center=(0, 0, 0),
            num_buildings=3,
            seed=123,
        )

        for z1, z2 in zip(zones1, zones2):
            assert z1.center == z2.center
            assert z1.radius == z2.radius


class TestCanyonZoneGeneration:
    """Tests for canyon zone generation."""

    def test_creates_zones(self):
        """Should create zones along canyon."""
        zones = create_canyon_denial_zone(
            start=(0.0, 0.0, -10.0),
            end=(100.0, 0.0, -10.0),
            width=30.0,
        )

        assert len(zones) >= 2

    def test_zones_cover_canyon(self):
        """Zones should cover the canyon path."""
        start = (0.0, 0.0, -10.0)
        end = (100.0, 0.0, -10.0)

        zones = create_canyon_denial_zone(
            start=start,
            end=end,
            width=30.0,
        )

        # Check zones span the distance
        all_x = [z.center[0] for z in zones]
        assert min(all_x) <= start[0] + 10
        assert max(all_x) >= end[0] - 10
