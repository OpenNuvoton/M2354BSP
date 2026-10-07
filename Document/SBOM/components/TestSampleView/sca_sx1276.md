# Semtech SX1276 LoRa Radio Driver Third-Party Component Description (for SCA / SBOM)

This document provides SBOM-ready metadata for the vendored Semtech SX1276 radio driver under `ThirdParty/SX1276`. The code is a subset of Semtech's LoRaMac-node project (radio driver, board and system utilities).

## 1) Component Identity

- Component name (`name`): `Semtech SX1276 LoRa radio driver (LoRaMac-node subset)`
- Component type (`type`): `library`
- Manufacturer (`manufacturer.name`): `Semtech Corporation`
- Supplier (`supplier.name`): `Semtech Corporation`
- Author (`author`): `Semtech Corporation`
- Upstream project: <https://github.com/Lora-net/LoRaMac-node>
- Version: `2017` (see Section 2; no upstream release tag in sources)
- License: `BSD-3-Clause` (SPDX; "Revised BSD License")
- Copyright: `Copyright Semtech Corporation 2013. All rights reserved.`, `(C)2013-2017 Semtech`
- Evidence path: `ThirdParty/SX1276`
- Key files: `radio/sx1276/sx1276.c`, `radio/radio.h`, `boards/sx1276mb1mas-board.c`, `boards/mcu/utilities.c`, `system/{gpio,timer,delay,uart,systime,fifo}.c`

## 2) Evidence for Version and License

- No explicit LoRaMac-node release version or tag is present in the vendored sources.
- Source headers (e.g. `radio/sx1276/sx1276.c`, `boards/board.h`) state `(C)2013-2017 Semtech`; version `2017` reflects this snapshot period.
- `boards/utilities.h` references `LMN (LoRaMac-node)`.
- `ThirdParty/SX1276/LICENSE` and `ThirdParty/SX1276/radio/LICENSE`: Revised BSD License (BSD-3-Clause).
- Introduced into this BSP by commit `e4c2e559` (2021-11-08, "Add LoRa_Master sample code for NuMaker IOT board").

> Action: if the exact upstream LoRaMac-node release can be confirmed, update `version`, `purl` and `bom-ref` in `sca_sx1276.json`.

## 3) Usage in This BSP (Test Sample scope)

- `SampleCode/NuMakerIoT/LoRa_Master/KEIL/LoRa_Master.uvprojx`
- `SampleCode/NuMakerIoT/LoRa_Slave/KEIL/LoRa_Slave.uvprojx`

## 4) Suggested BOM-Ref and purl

- `bom-ref`: `pkg:github/lora-net/loramac-node@2017?source=vendored&path=ThirdParty/SX1276`
- `purl`: `pkg:github/lora-net/loramac-node@2017`

## 5) CycloneDX JSON Component Example

```json
{
  "type": "library",
  "bom-ref": "pkg:github/lora-net/loramac-node@2017?source=vendored&path=ThirdParty/SX1276",
  "name": "Semtech SX1276 LoRa radio driver (LoRaMac-node subset)",
  "version": "2017",
  "scope": "required",
  "author": "Semtech Corporation",
  "manufacturer": {
    "name": "Semtech Corporation"
  },
  "supplier": {
    "name": "Semtech Corporation",
    "url": [
      "https://github.com/Lora-net/LoRaMac-node"
    ]
  },
  "purl": "pkg:github/lora-net/loramac-node@2017",
  "description": "Subset of Semtech LoRaMac-node (SX1276 radio driver, board and system utilities) vendored in ThirdParty/SX1276. No release tag is recorded in the sources; version reflects the '(C)2013-2017 Semtech' source headers.",
  "licenses": [
    {
      "license": {
        "id": "BSD-3-Clause"
      }
    }
  ],
  "properties": [
    {
      "name": "src_path",
      "value": "ThirdParty/SX1276"
    },
    {
      "name": "integration",
      "value": "vendored_source"
    },
    {
      "name": "bsp:version-evidence",
      "value": "ThirdParty/SX1276/radio/sx1276/sx1276.c header '(C)2013-2017 Semtech'; no explicit release version in sources"
    },
    {
      "name": "bsp:license-evidence",
      "value": "ThirdParty/SX1276/LICENSE; ThirdParty/SX1276/radio/LICENSE (Revised BSD License)"
    },
    {
      "name": "bsp:component-origin",
      "value": "third-party"
    },
    {
      "name": "bsp:component-source",
      "value": "Semtech LoRaMac-node"
    },
    {
      "name": "bsp:evidence-file",
      "value": "Document/SBOM/components/TestSampleView/sca_sx1276.json"
    },
    {
      "name": "bsp:evidence-path",
      "value": "ThirdParty/SX1276"
    }
  ]
}
```

## 6) Compliance Notes

- `manufacturer` identifies Semtech Corporation as the organization that created this component; `supplier` records Semtech Corporation as the upstream supplier in this component description.
- Keep the original Semtech copyright and Revised BSD License text.
- `boards/lpm-board.h` also credits STMicroelectronics (MCD Application Team); preserve those notices.
