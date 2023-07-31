

set(PRJ CRYPTO_SHA_SM3)
add_executable(${PRJ} main.c)
target_link_libraries(${PRJ} system_startup standard_driver)

target_link_options(${PRJ} 
   PUBLIC 
    --cpu=Cortex-M23
    --library_type=microlib 
    --ro-base 0x00000000
    --rw-base 0x20000000
    --entry Reset_Handler
    --first __Vectors
    --strict
    --map 
    --datacompressor=off 
    --info=inline 
    --entry Reset_Handler 
    --summary_stderr 
    --info totals
    --map 
    --load_addr_map_info 
    --xref 
    --callgraph 
    --symbols
)
 
# Download to board
set(DOWNLOAD_BOARD FALSE)

# Convert to binary and copy to target device
if(${DOWNLOAD_BOARD})

    message("Convert elf to binary file and download to target board .... ")
    add_custom_command(TARGET ${PRJ} POST_BUILD
       COMMAND ${ARM_FROMELF} --bin --output e:/${PRJ}.bin ${PRJ}.elf
       VERBATIM
    )

endif()
