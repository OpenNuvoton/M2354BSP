# BME680 Sensor API Third-Party Component Description (for SCA / SBOM)

This document provides SBOM-ready metadata for the vendored Bosch Sensortec BME680 driver under `ThirdParty/BME680`.

## 1) Component Identity

- Component name (`name`): `BME680 Sensor API`
- Component type (`type`): `library`
- Manufacturer (`manufacturer.name`): `Bosch Sensortec GmbH`
- Supplier (`supplier.name`): `Bosch Sensortec GmbH`
- Author (`author`): `Bosch Sensortec GmbH`
- Version: `3.5.10` (dated 23 Jan 2020)
- License: `BSD-3-Clause` (SPDX)
- Copyright: `Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.`
- Evidence path: `ThirdParty/BME680`
- Key files: `bme680.c`, `bme680.h`, `bme680_defs.h`, `LICENSE`, `README.md`, `Self test/`

## 2) Evidence for Version and License

- `ThirdParty/BME680/bme680.h`, `bme680.c`, `bme680_defs.h`: `@version 3.5.10`, `@date 23 Jan 2020`
- `ThirdParty/BME680/README.md`: version table
- `ThirdParty/BME680/LICENSE`: `BSD-3-Clause`

## 3) Usage in This BSP (Test Sample scope)

- `SampleCode/NuMakerIoT/BME680_and_LCD/Keil/BME680_and_LCD.uvprojx`

## 4) Suggested BOM-Ref and purl

- `bom-ref`: `pkg:github/boschsensortec/bme680_driver@3.5.10?source=vendored&path=ThirdParty/BME680`
- `purl`: `pkg:github/boschsensortec/bme680_driver@3.5.10`

## 5) CycloneDX JSON Component Example

```json
{
  "type": "library",
  "bom-ref": "pkg:github/boschsensortec/bme680_driver@3.5.10?source=vendored&path=ThirdParty/BME680",
  "name": "BME680 Sensor API",
  "version": "3.5.10",
  "scope": "required",
  "author": "Bosch Sensortec GmbH",
  "manufacturer": {
    "name": "Bosch Sensortec GmbH"
  },
  "supplier": {
    "name": "Bosch Sensortec GmbH",
    "url": [
      "https://github.com/BoschSensortec/BME680_driver"
    ]
  },
  "purl": "pkg:github/boschsensortec/bme680_driver@3.5.10",
  "description": "Bosch Sensortec BME680 gas sensor driver API vendored in ThirdParty/BME680.",
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
      "value": "ThirdParty/BME680"
    },
    {
      "name": "integration",
      "value": "vendored_source"
    },
    {
      "name": "bsp:version-evidence",
      "value": "ThirdParty/BME680/bme680.h (@version 3.5.10, @date 23 Jan 2020); ThirdParty/BME680/README.md"
    },
    {
      "name": "bsp:license-evidence",
      "value": "ThirdParty/BME680/LICENSE"
    },
    {
      "name": "bsp:component-origin",
      "value": "third-party"
    },
    {
      "name": "bsp:component-source",
      "value": "Bosch Sensortec BME680 sensor API"
    },
    {
      "name": "bsp:evidence-file",
      "value": "Document/SBOM/components/TestSampleView/sca_bme680.json"
    },
    {
      "name": "bsp:evidence-path",
      "value": "ThirdParty/BME680"
    }
  ]
}
```

## 6) Compliance Notes

- `manufacturer` identifies Bosch Sensortec GmbH as the organization that created this component; `supplier` records Bosch Sensortec GmbH as the upstream supplier in this component description.
- Keep the original copyright and BSD-3-Clause license text.
