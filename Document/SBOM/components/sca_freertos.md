# FreeRTOS-Kernel Third-Party Component Description (for SCA / SBOM)

This document provides SBOM-ready metadata for the vendored FreeRTOS component under `ThirdParty/FreeRTOS`.

## 1) Component Identity

- Component name (`name`): `FreeRTOS-Kernel`
- Component type (`type`): `library`
- Supplier / project: `FreeRTOS` (Amazon)
- Version: `v10.4.6`
- License: `MIT` (SPDX)
- Evidence path: `ThirdParty/FreeRTOS`

## 2) Evidence for Version and License

Primary evidence in this repository:

- `ThirdParty/FreeRTOS/Source/include/FreeRTOS.h`
  - Header shows `FreeRTOS Kernel V10.4.6`
  - Header includes `SPDX-License-Identifier: MIT`
- `ThirdParty/FreeRTOS/Source/tasks.c`
  - Header shows `FreeRTOS Kernel V10.4.6`
  - Header includes `SPDX-License-Identifier: MIT`
- `ThirdParty/FreeRTOS/Source/queue.c`
  - Header shows `FreeRTOS Kernel V10.4.6`
  - Header includes `SPDX-License-Identifier: MIT`
- `ThirdParty/FreeRTOS/Source/list.c`
  - Header shows `FreeRTOS Kernel V10.4.6`
  - Header includes `SPDX-License-Identifier: MIT`
- `ThirdParty/FreeRTOS/Source/timers.c`
  - Header shows `FreeRTOS Kernel V10.4.6`
  - Header includes `SPDX-License-Identifier: MIT`

## 3) License Handling Guidance

For CycloneDX output, use SPDX license ID directly:

- `licenses[0].license.id = MIT`

Also preserve upstream license headers in vendored source files.

## 4) Suggested CycloneDX Field Mapping

Recommended component fields:

- `type`: `library`
- `name`: `FreeRTOS-Kernel`
- `version`: `10.4.6`
- `scope`: `required`
- `author`: `Amazon.com, Inc. (FreeRTOS project)`
- `description`: `FreeRTOS Kernel vendored in ThirdParty/FreeRTOS, including kernel source, public headers, ARM_CM0 GCC/IAR portable layers, memory managers, and demo/common example files.`
- `licenses[0].license.id`: `MIT`
- `properties` (recommended custom properties):
  - `src_path = ThirdParty/FreeRTOS`
  - `integration = vendored_source`
  - `freertos_kernel_version = 10.4.6`

## 5) Suggested BOM-Ref and purl

Suggested values:

- `bom-ref`: `pkg:generic/freertos-kernel@10.4.6?source=vendored&path=ThirdParty/FreeRTOS`
- `purl`: `pkg:generic/freertos-kernel@10.4.6`

If your internal SBOM naming policy differs, keep naming consistent across all third-party components in this BSP.

## 6) CycloneDX JSON Component Example

```json
{
  "type": "library",
  "bom-ref": "pkg:generic/freertos-kernel@10.4.6?source=vendored&path=ThirdParty/FreeRTOS",
  "name": "FreeRTOS-Kernel",
  "version": "10.4.6",
  "scope": "required",
  "author": "Amazon.com, Inc. (FreeRTOS project)",
  "purl": "pkg:generic/freertos-kernel@10.4.6",
  "description": "FreeRTOS Kernel vendored in ThirdParty/FreeRTOS, including kernel source, public headers, ARM_CM0 GCC/IAR portable layers, memory managers, and demo/common example files.",
  "licenses": [
    {
      "license": {
        "id": "MIT"
      }
    }
  ],
  "properties": [
    { "name": "src_path", "value": "ThirdParty/FreeRTOS" },
    { "name": "integration", "value": "vendored_source" },
    { "name": "freertos_kernel_version", "value": "10.4.6" }
  ]
}
```

## 7) Compliance Notes

- Keep original upstream copyright/license notices.
- version and license evidence is taken from the vendored kernel headers and source files.
- If producing a finer-grained SBOM (for example, app + RTOS + board support), represent each major unit as separate components and declare dependencies between them.
