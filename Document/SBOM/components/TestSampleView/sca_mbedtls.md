# Mbed TLS Third-Party Component Description (for SCA / SBOM)

This document provides SBOM-ready metadata for the two vendored Mbed TLS copies under `ThirdParty/mbedTLS` and `ThirdParty/mbedtls-3.1.0`. They are distinct versions and are recorded as two separate components in `sca_mbedtls.json`.

## 1) Component Identity

| Field | ThirdParty/mbedTLS | ThirdParty/mbedtls-3.1.0 |
|---|---|---|
| `name` | `Mbed TLS` | `Mbed TLS` |
| `type` | `library` | `library` |
| `version` | `2.17.0` | `3.1.0` |
| Packaging | CMSIS pack `ARM.mbedTLS` 1.6.0 (2019-04-02) | Upstream source tree |
| Author | Mbed TLS Contributors | Mbed TLS Contributors |
| Manufacturer | Arm Limited | Arm Limited |
| Supplier | Arm Limited | Arm Limited |
| License | `Apache-2.0` (SPDX) | `Apache-2.0` (SPDX) |
| Copyright | `Copyright (C) 2006-2015, ARM Limited, All Rights Reserved` | `Copyright The Mbed TLS Contributors` |
| CPE | `cpe:2.3:a:arm:mbed_tls:2.17.0:*:*:*:*:*:*:*` | `cpe:2.3:a:arm:mbed_tls:3.1.0:*:*:*:*:*:*:*` |

## 2) Evidence for Version and License

`ThirdParty/mbedTLS` (2.17.0):

- `include/mbedtls/version.h`: `MBEDTLS_VERSION_STRING "2.17.0"`, `MBEDTLS_VERSION_NUMBER 0x02110000`, `SPDX-License-Identifier: Apache-2.0`
- `ChangeLog`: `= mbed TLS 2.17.0 branch released 2019-03-19`
- `ARM.mbedTLS.pdsc`: `<release version="1.6.0" date="2019-04-02">`
- `LICENSE`: Apache License 2.0

`ThirdParty/mbedtls-3.1.0` (3.1.0):

- `include/mbedtls/build_info.h`: `MBEDTLS_VERSION_STRING "3.1.0"`, `MBEDTLS_VERSION_NUMBER 0x03010000`, `SPDX-License-Identifier: Apache-2.0`
- `LICENSE`: Apache License 2.0

## 3) Usage in This BSP (Test Sample scope)

- `ThirdParty/mbedTLS` (2.17.0): `SampleCode/Crypto/mbedTLS_AES/Keil/AES.uvprojx`
- `ThirdParty/mbedtls-3.1.0` (3.1.0):
  - `SampleCode/Crypto/mbedTLS_AES` (Keil, IAR)
  - `SampleCode/Crypto/mbedTLS_ECDH` (Keil, IAR, CMake)
  - `SampleCode/Crypto/mbedTLS_ECDSA` (Keil, IAR, CMake)
  - `SampleCode/Crypto/mbedTLS_RSA` (Keil, IAR, CMake)
  - `SampleCode/Crypto/mbedTLS_SHA256` (Keil, IAR, CMake)

## 4) Suggested BOM-Ref and purl

- `pkg:generic/mbedtls@2.17.0?source=vendored&path=ThirdParty/mbedTLS` / `pkg:generic/mbedtls@2.17.0`
- `pkg:generic/mbedtls@3.1.0?source=vendored&path=ThirdParty/mbedtls-3.1.0` / `pkg:generic/mbedtls@3.1.0`

## 5) CycloneDX JSON Component Example

```json
{
  "type": "library",
  "bom-ref": "pkg:generic/mbedtls@2.17.0?source=vendored&path=ThirdParty/mbedTLS",
  "name": "Mbed TLS",
  "version": "2.17.0",
  "scope": "required",
  "author": "Mbed TLS Contributors",
  "manufacturer": {
    "name": "Arm Limited"
  },
  "supplier": {
    "name": "Arm Limited",
    "url": [
      "https://www.trustedfirmware.org/projects/mbed-tls/"
    ]
  },
  "purl": "pkg:generic/mbedtls@2.17.0",
  "cpe": "cpe:2.3:a:arm:mbed_tls:2.17.0:*:*:*:*:*:*:*",
  "description": "Mbed TLS cryptographic and TLS library (CMSIS pack ARM.mbedTLS 1.6.0) vendored in ThirdParty/mbedTLS.",
  "licenses": [
    {
      "license": {
        "id": "Apache-2.0"
      }
    }
  ],
  "properties": [
    {
      "name": "src_path",
      "value": "ThirdParty/mbedTLS"
    },
    {
      "name": "integration",
      "value": "vendored_source"
    },
    {
      "name": "mbedtls_version_number",
      "value": "0x02110000"
    },
    {
      "name": "bsp:version-evidence",
      "value": "ThirdParty/mbedTLS/include/mbedtls/version.h; ThirdParty/mbedTLS/ChangeLog; ThirdParty/mbedTLS/ARM.mbedTLS.pdsc"
    },
    {
      "name": "bsp:license-evidence",
      "value": "ThirdParty/mbedTLS/LICENSE"
    },
    {
      "name": "bsp:component-origin",
      "value": "third-party"
    },
    {
      "name": "bsp:component-source",
      "value": "Mbed TLS"
    },
    {
      "name": "bsp:evidence-file",
      "value": "Document/SBOM/components/TestSampleView/sca_mbedtls.json"
    },
    {
      "name": "bsp:evidence-path",
      "value": "ThirdParty/mbedTLS"
    }
  ]
}
```

```json
{
  "type": "library",
  "bom-ref": "pkg:generic/mbedtls@3.1.0?source=vendored&path=ThirdParty/mbedtls-3.1.0",
  "name": "Mbed TLS",
  "version": "3.1.0",
  "scope": "required",
  "author": "Mbed TLS Contributors",
  "manufacturer": {
    "name": "Arm Limited"
  },
  "supplier": {
    "name": "Arm Limited",
    "url": [
      "https://www.trustedfirmware.org/projects/mbed-tls/"
    ]
  },
  "purl": "pkg:generic/mbedtls@3.1.0",
  "cpe": "cpe:2.3:a:arm:mbed_tls:3.1.0:*:*:*:*:*:*:*",
  "description": "Mbed TLS cryptographic and TLS library vendored in ThirdParty/mbedtls-3.1.0.",
  "licenses": [
    {
      "license": {
        "id": "Apache-2.0"
      }
    }
  ],
  "properties": [
    {
      "name": "src_path",
      "value": "ThirdParty/mbedtls-3.1.0"
    },
    {
      "name": "integration",
      "value": "vendored_source"
    },
    {
      "name": "mbedtls_version_number",
      "value": "0x03010000"
    },
    {
      "name": "bsp:version-evidence",
      "value": "ThirdParty/mbedtls-3.1.0/include/mbedtls/build_info.h; ThirdParty/mbedtls-3.1.0/ChangeLog"
    },
    {
      "name": "bsp:license-evidence",
      "value": "ThirdParty/mbedtls-3.1.0/LICENSE"
    },
    {
      "name": "bsp:component-origin",
      "value": "third-party"
    },
    {
      "name": "bsp:component-source",
      "value": "Mbed TLS"
    },
    {
      "name": "bsp:evidence-file",
      "value": "Document/SBOM/components/TestSampleView/sca_mbedtls.json"
    },
    {
      "name": "bsp:evidence-path",
      "value": "ThirdParty/mbedtls-3.1.0"
    }
  ]
}
```

## 6) Compliance Notes

- `manufacturer` identifies Arm Limited as the organization that created this component; `supplier` records Arm Limited as the upstream supplier in this component description.
- Keep the upstream Apache-2.0 `LICENSE` and file headers intact.
- Mbed TLS 2.17.0 is an old release; track CVEs against the CPE above during vulnerability scanning.
