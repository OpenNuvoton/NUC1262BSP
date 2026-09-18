# HID Transfer Test Tool Component Description (for SCA / SBOM)

This document provides SBOM-ready metadata for the host-side HID Transfer Test Tool located under `Utilities/HIDTransferTest`.

## 1) Component Identity

- Component name (`name`): `HID Transfer Test Tool`
- Component type (`type`): `application`
- Supplier / Author (`supplier.name` / `author`): `Nuvoton Technology Corp.`
- Version: `1.1.0.0`
- License: `Apache-2.0` (SPDX)
- Evidence path: `Utilities/HIDTransferTest`
- Binary path: `Utilities/HIDTransferTest/Debug/HIDTransferTest.exe`
- Component origin: `first-party`
- Usage: `host-side Windows HID transfer test utility`

## 2) Evidence for Version and License

Primary evidence in this repository:

- `Utilities/HIDTransferTest/HIDTransferTest/version.h`
  - Defines `VER_MAJOR` as `1`, `VER_MINOR` as `1`, `VER_REVISION` as `0`, `VER_BUILD` as `0`.
  - Defines `VER_FILE_VERSION_STR` and `VER_PRODUCT_VERSION_STR` as `1.1.0.0`.
  - Defines `TOOL_VERSION_STR` as `1.1.0`.
  - Defines `VER_COMPANY_NAME` as `"Nuvoton Technology Corp."`.
  - Defines `VER_PRODUCT_NAME` as `"HID Transfer Test Tool"`.
  - Defines `VER_LEGAL_COPYRIGHT` as `"Copyright (C) 2010-2026 Nuvoton Technology Corp. All rights reserved."`.
- `Utilities/HIDTransferTest/HIDTransferTest/HIDTransferTest.rc`
  - Incorporates `version.h` to populate `VS_VERSION_INFO`, file/product versions `1,1,0,0`, company and product names into binary metadata.
- `Utilities/HIDTransferTest/LICENSE.md`
  - Explicitly licenses software in this directory under the Apache License, Version 2.0 (`https://www.apache.org/licenses/LICENSE-2.0`).
  - Contains copyright notice: `Copyright (c) 2010-2026 Nuvoton Technology Corporation.`
- `Utilities/HIDTransferTest/HIDTransferTest/HIDTransferTest.cpp`
  - File header copyright: `Copyright (c) 2010 Nuvoton Technology Corp. All rights reserved.`
- `Utilities/HIDTransferTest/Debug/HIDTransferTest.exe`
  - Compiled PE binary matching version `1.1.0.0`.
  - SHA-256: `574f697b673b96f472321fce24cedbd5a7b3d8711067e23c7b065ae6ab074e1a`
  - SHA-1: `38f4ea3e6731367e28b6ad0eb4be8757d510a09d`
  - MD5: `3744e197d6eff9b024e384b3289cd9cd`

## 3) License Handling Guidance

For CycloneDX output, use the SPDX license identifier directly:

- `licenses[0].license.id = Apache-2.0`

The tool is first-party software developed by Nuvoton Technology Corporation and distributed under Apache License 2.0. Third-party Windows DDK/SDK headers (`hidsdi.h`, `hidpi.h`, `hidusage.h`) and `hid.lib` are standard Microsoft build dependencies used for compiling the Windows client utility.

## 4) Suggested CycloneDX Field Mapping

Recommended component fields:

- `type`: `application`
- `name`: `HID Transfer Test Tool`
- `version`: `1.1.0.0`
- `scope`: `required`
- `author`: `Nuvoton Technology Corp.`
- `supplier.name`: `Nuvoton Technology Corp.`
- `description`: `Host-side Windows HID transfer test utility and test tool source code located in Utilities/HIDTransferTest, used for verifying USB HID transfer functionality with Nuvoton MCU sample code.`
- `licenses[0].license.id`: `Apache-2.0`
- `hashes`:
  - `SHA-256`: `574f697b673b96f472321fce24cedbd5a7b3d8711067e23c7b065ae6ab074e1a`
  - `SHA-1`: `38f4ea3e6731367e28b6ad0eb4be8757d510a09d`
  - `MD5`: `3744e197d6eff9b024e384b3289cd9cd`
- `properties` (recommended custom properties):
  - `src_path = Utilities/HIDTransferTest`
  - `integration = first_party_tool`
  - `hidtransfertest_version = 1.1.0.0`
  - `bsp:component-origin = first-party`
  - `bsp:component-source = Nuvoton Technology Corp.`
  - `bsp:license-status = identified`
  - `bsp:license-source = manual-binary-license-mapping`
  - `bsp:license-match-method = sha256`
  - `bsp:included-in-product-firmware = false`
  - `bsp:runtime-dependency = false`
  - `bsp:usage = host-side Windows HID transfer test utility`
  - `bsp:file-path = Utilities/HIDTransferTest/Debug/HIDTransferTest.exe`
  - `bsp:evidence-file = Document/SBOM/components/TestSampleView/sca_hidtransfertest.json`
  - `bsp:evidence-path = Utilities/HIDTransferTest`
  - `bsp:version-evidence = Utilities/HIDTransferTest/HIDTransferTest/version.h; Utilities/HIDTransferTest/HIDTransferTest/HIDTransferTest.rc; Utilities/HIDTransferTest/Debug/HIDTransferTest.exe`
  - `bsp:license-evidence = Utilities/HIDTransferTest/LICENSE.md; Utilities/HIDTransferTest/HIDTransferTest/HIDTransferTest.cpp; Utilities/HIDTransferTest/HIDTransferTest/version.h`

## 5) Suggested BOM-Ref and purl

Suggested values:

- `bom-ref`: `pkg:generic/nuvoton/hid-transfer-test-tool@1.1.0.0?path=Utilities/HIDTransferTest`
- `purl`: `pkg:generic/nuvoton/hid-transfer-test-tool@1.1.0.0`
- `cpe`: `cpe:2.3:a:nuvoton:hid_transfer_test_tool:1.1.0.0:*:*:*:*:*:*:*`

## 6) CycloneDX JSON Component Example

```json
{
  "$schema": "http://cyclonedx.org/schema/bom-1.6.schema.json",
  "bomFormat": "CycloneDX",
  "specVersion": "1.6",
  "serialNumber": "urn:uuid:8b3e8c21-9972-4d8e-90f7-b2f5673ecda2",
  "version": 1,
  "metadata": {
    "timestamp": "2026-09-16T16:40:00+08:00",
    "component": {
      "type": "application",
      "bom-ref": "pkg:generic/nuvoton/hid-transfer-test-tool@1.1.0.0?path=Utilities/HIDTransferTest",
      "name": "HID Transfer Test Tool",
      "version": "1.1.0.0",
      "scope": "required",
      "supplier": {
        "name": "Nuvoton Technology Corp.",
        "url": [
          "https://www.nuvoton.com"
        ]
      },
      "author": "Nuvoton Technology Corp.",
      "purl": "pkg:generic/nuvoton/hid-transfer-test-tool@1.1.0.0",
      "cpe": "cpe:2.3:a:nuvoton:hid_transfer_test_tool:1.1.0.0:*:*:*:*:*:*:*",
      "description": "Host-side Windows HID transfer test utility and test tool source code located in Utilities/HIDTransferTest, used for verifying USB HID transfer functionality with Nuvoton MCU sample code."
    }
  },
  "components": [
    {
      "type": "application",
      "bom-ref": "pkg:generic/nuvoton/hid-transfer-test-tool@1.1.0.0?path=Utilities/HIDTransferTest",
      "name": "HID Transfer Test Tool",
      "version": "1.1.0.0",
      "scope": "required",
      "supplier": {
        "name": "Nuvoton Technology Corp.",
        "url": [
          "https://www.nuvoton.com"
        ]
      },
      "author": "Nuvoton Technology Corp.",
      "purl": "pkg:generic/nuvoton/hid-transfer-test-tool@1.1.0.0",
      "cpe": "cpe:2.3:a:nuvoton:hid_transfer_test_tool:1.1.0.0:*:*:*:*:*:*:*",
      "description": "Host-side Windows HID transfer test utility and test tool source code located in Utilities/HIDTransferTest, used for verifying USB HID transfer functionality with Nuvoton MCU sample code.",
      "licenses": [
        {
          "license": {
            "id": "Apache-2.0"
          }
        }
      ],
      "hashes": [
        {
          "alg": "SHA-256",
          "content": "574f697b673b96f472321fce24cedbd5a7b3d8711067e23c7b065ae6ab074e1a"
        },
        {
          "alg": "SHA-1",
          "content": "38f4ea3e6731367e28b6ad0eb4be8757d510a09d"
        },
        {
          "alg": "MD5",
          "content": "3744e197d6eff9b024e384b3289cd9cd"
        }
      ],
      "properties": [
        { "name": "src_path", "value": "Utilities/HIDTransferTest" },
        { "name": "integration", "value": "first_party_tool" },
        { "name": "hidtransfertest_version", "value": "1.1.0.0" },
        { "name": "bsp:component-origin", "value": "first-party" },
        { "name": "bsp:component-source", "value": "Nuvoton Technology Corp." },
        { "name": "bsp:license-status", "value": "identified" },
        { "name": "bsp:license-source", "value": "manual-binary-license-mapping" },
        { "name": "bsp:license-match-method", "value": "sha256" },
        { "name": "bsp:included-in-product-firmware", "value": "false" },
        { "name": "bsp:runtime-dependency", "value": "false" },
        { "name": "bsp:usage", "value": "host-side Windows HID transfer test utility" },
        { "name": "bsp:file-path", "value": "Utilities/HIDTransferTest/Debug/HIDTransferTest.exe" },
        { "name": "bsp:evidence-file", "value": "Document/SBOM/components/TestSampleView/sca_hidtransfertest.json" },
        { "name": "bsp:evidence-path", "value": "Utilities/HIDTransferTest" },
        { "name": "bsp:version-evidence", "value": "Utilities/HIDTransferTest/HIDTransferTest/version.h; Utilities/HIDTransferTest/HIDTransferTest/HIDTransferTest.rc; Utilities/HIDTransferTest/Debug/HIDTransferTest.exe" },
        { "name": "bsp:license-evidence", "value": "Utilities/HIDTransferTest/LICENSE.md; Utilities/HIDTransferTest/HIDTransferTest/HIDTransferTest.cpp; Utilities/HIDTransferTest/HIDTransferTest/version.h" }
      ]
    }
  ]
}
```

## 7) Compliance Notes

- This component is a host-side Windows test tool used to evaluate USB HID transfer examples under `SampleCode/`.
- It is classified under the **Test Sample SBOM** view (`bsp:included-in-product-firmware = false`, `bsp:runtime-dependency = false`).
- Source code copyright belongs to Nuvoton Technology Corp., licensed under Apache License 2.0.
- Binary hash matches `Utilities/HIDTransferTest/Debug/HIDTransferTest.exe` provided in the repository.
