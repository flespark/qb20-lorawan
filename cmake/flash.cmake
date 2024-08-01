option(DUMP_HEX "Create intel hex file of final executable" ON)
option(DUMP_BIN "Create binary file of final executable" ON)
option(DUMP_ASM "Extract executeable assembly of final executable" ON)
option(DUMP_NM "Extract symbol of final executable" ON)

function(generate_extra_output target_name)
    if (${DUMP_HEX})
        add_custom_command(TARGET ${target_name} POST_BUILD
            DEPENDS ${target_name}
            COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${target_name}> ${target_name}.hex)
    endif()
    if (${DUMP_BIN})
        add_custom_command(TARGET ${target_name} POST_BUILD
            DEPENDS ${target_name}
            COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${target_name}> ${target_name}.bin)
    endif()
    if (${DUMP_ASM})
        add_custom_command(TARGET ${target_name} POST_BUILD
            COMMAND ${CMAKE_OBJDUMP} -S $<TARGET_FILE:${target_name}> > ${target_name}.asm)
    endif()
    if (${DUMP_NM})
        add_custom_command(TARGET ${target_name} POST_BUILD
            COMMAND ${CMAKE_NM} -C -l -n $<TARGET_FILE:${target_name}> > ${target_name}.sym)
    endif()
endfunction()
