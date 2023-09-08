# M2354 Series CMSIS BSP

To experience the powerful features of M2354 in few minutes, please refer to NuMaker-PFM-M2354 Board Quick Start Guide. You can select the sample code of your interest to download and execute on the M2354 board. For example, you can try the TrustZone® technology in M2354 that provides system-wide hardware isolation for trusted software with the sample code in the folder BSP\SampleCode\TrustZone. This folder includes the sample code of TrustZone® for collaborative secure software development, Hard Fault handling and a TrustZone® template. You can open the project files to build them with Keil® MDK, IAR or Eclipse, and then download and trace them on the M2354 board to see how the TrustZone® works.This BSP folder

## .\Document\


- CMSIS.html<br>
	Introduction of CMSIS version 5.0. CMSIS components included CMSIS-CORE, CMSIS-Driver, CMSIS-DSP, etc.

- NuMicro M2354 Series CMSIS BSP Revision History.pdf<br>
	The revision history of M2354 Series BSP.

- NuMicro M2354 Series Driver Reference Guide.chm<br>
	The usage of drivers in M2354 Series BSP.

## .\Library\


- CMSIS<br>
	Cortex® Microcontroller Software Interface Standard (CMSIS) V5.0 definitions by ARM® Corp.

- CryptoAccelerator<br>
	A C library that implements cryptographic primitives, X.509 certificate manipulation and the SSL/TLS and DTLS protocols.
	
- Device<br>
	CMSIS compliant device header file.

- LCDLib<br>
	Library for controlling LCD module.

- NuMaker<br>
	Specific libraries for M2354 NuMaker board.

- SmartcardLib<br>
	Library for accessing a smartcard.

- StdDriver<br>
	All peripheral driver header and source files.

- UsbHostLib<br>
	USB host library source code.

## .\Sample Code\

- AttackDetection<br>
	Sample codes for non-invasivephysical attack detection.

- CardReader<br>
	USB CCID Smartcard Readersample code.

- CortexM23<br>
	Cortex®-M23 sample code.

- Crypto<br>
	Crypto sample code using MbedTLS library.

- FreeRTOS<br>
	Simple FreeRTOS™ demo code.

- Hard\_Fault\_Sample<br>
	Show hard fault information when hard fault happened. The hard fault handler show some information included program counter, which is the address where the processor was executing when the hard fault occur. The listing file (or map file) can show what function and instruction that was. It also shows the Link Register (LR), which contains the return address of the last function call. It can show the status where CPU comes from to get to this point.

- ISP<br>
	Sample codes for In-System-Programming.

- NuMaker<br>
	Sample codes for NuMaker-PFM-M2354 board.

- NuMakerIoT<br>
	Sample codes for NuMaker-IoT-M2354 board.

- PowerManagement<br>
	Power management sample code.

- SecureApplication<br>
	Sample codes for secure application.

- Semihost<br>
	Show how to print and get character through IDE console window.

- StdDriver<br>
	Demonstrate the usage of M2354 series MCU peripheral driver APIs.

- TrustZone<br>
	Includes the demo of secure codes and non-secure codes.

- XOM<br>
	Demonstrate how to create XOM library and use it.


## .\ThirdParty\

- BME680<br>
	Sensor driver for BME680 gas sensor.

- FatFs<br>
	An open source FAT/exFAT filesystem library.

- FreeRTOS<br>
	FreeRTOS porting for M2354.

- mbedTLS<br>
	An open source crypto library.

- mbedtls-3.1.0<br>
	An open source crypto library.
	
- SX1276<br>
	The Semtech SX1276 LoRa transceiver library.

# Licesne

**SPDX-License-Identifier: Apache-2.0**

Copyright in some of the content available in this BSP belongs to third parties.
Third parties license is specified in a file header or license file.
M2354 BSP files are provided under the Apache-2.0 license.

