create_project(USBD_CCID ${CMAKE_CURRENT_LIST_DIR}/USBD_CCID/main.c ${CMAKE_CURRENT_LIST_DIR}/USBD_CCID/Ccid_if.c ${CMAKE_CURRENT_LIST_DIR}/USBD_CCID/ccid.c ${CMAKE_CURRENT_LIST_DIR}/USBD_CCID/descriptors.c ${CMAKE_CURRENT_LIST_DIR}/USBD_CCID/sc_intf.c)
target_link_libraries(USBD_CCID ${BSP_DIR}/Library/SmartcardLib/SmartCardLib_Keil_short_wchar.lib)
