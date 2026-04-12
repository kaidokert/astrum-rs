# Roadmap

Planned work, roughly in priority order. Nothing here is committed to a
timeline.

## Next

- **Ethernet demo** — networked partition communicating over an Ethernet
  MAC, demonstrating IPC between a network-facing partition and
  application partitions.
- **defmt** - full defmt build that does not bring in any core::fmt deps.
- **C APEX API** — ARINC 653 Part 1 compatible C bindings so partitions
  can be written in C against the standard APEX interface.

## Medium-term

- **ARMv8-M (Cortex-M33+)** — take advantage of TrustZone-M and the
  SAU for hardware-enforced secure/non-secure partitioning, in addition
  to or instead of the current MPU-only model.
- **Cortex-R port** — work in progress. Extends the kernel to ARMv7-R
  (no extensive MPU changes needed for the scheduling core; memory
  protection uses the Cortex-R PMSAv7 model instead of the M-profile MPU).
- **CAN/CAN-FD demo** - After USB and Ethernet
- **Config generator** - Offline config generation from declarative format.
  Possibly AADL backend.
- **Nordic SoftDevice** - SD / kernel coexistence.

## Exploratory

- **Bootloader** — very tentative. A minimal secure bootloader that
  validates partition images before handing off to the kernel.
- **Flight demo** - run something that flies on it, either with wings
  or rotors.
