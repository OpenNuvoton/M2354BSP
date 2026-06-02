Windows Test Tool - HIDTransferTest
=====================================

A Windows command-line tool for testing HID Transfer with Control endpoint.

Location
--------
BSP/Utilities/HIDTransferTest

Usage
-----
HIDTransferTest -pagesize 4096 -ctrl

Options
-------
  -pagesize 4096   Set the page size to 4096 bytes (matches M2L31 RRAM page size)
  -ctrl            Use Control endpoint for HID transfer

Description
-----------
This tool communicates with the M2L31 device running the USBD_HID_Transfer_CTRL
firmware. It performs read/write/erase operations over HID using the Control
endpoint, with a page size of 4096 bytes aligned to the M2L31 RRAM page size.
